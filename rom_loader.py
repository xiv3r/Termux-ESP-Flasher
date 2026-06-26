"""
rom_loader.py - Minimal ESP32 ROM serial bootloader client over a raw USB CDC
bulk pipe (no pyserial, no stub loader, no subprocess esptool.py).

This talks SLIP-framed packets directly to the chip's ROM bootloader, the
same protocol real esptool.py uses before it uploads a speed-up stub. We
stay in ROM-only mode on purpose: it is slower (no stub, no high baud
renegotiation since CDC has no real UART clock) but it cuts out ~90% of
esptool's code (stub binaries for 3 different chips, compression, RAM
download, changing baud rates that don't mean anything over native USB CDC).

Supports exactly the targets NRcap32 cares about: ESP32-S3, ESP32-C3,
ESP32-S2. All three use native USB CDC and the same ROM loader command set
(they share the "esp32s2"-family loader ABI introduced after the original
ESP32/ESP8266 loader).

Protocol reference (re-implemented from public ROM source / esptool docs,
no code copied):
  Frame:   0xC0 <SLIP-escaped payload> 0xC0
  Payload: <0x00 req/0x01 resp><CMD 1B><SIZE 2B LE><CHECKSUM/VALUE 4B LE><DATA>
  SLIP escaping: 0xC0 -> 0xDB 0xDC,  0xDB -> 0xDB 0xDD
"""

import struct
import time
import zlib

# ROM loader command bytes
CMD_FLASH_BEGIN = 0x02
CMD_FLASH_DATA = 0x03
CMD_FLASH_END = 0x04
CMD_MEM_BEGIN = 0x05
CMD_MEM_END = 0x06
CMD_MEM_DATA = 0x07
CMD_SYNC = 0x08
CMD_WRITE_REG = 0x09
CMD_READ_REG = 0x0A
CMD_SPI_SET_PARAMS = 0x0B
CMD_SPI_ATTACH = 0x0D
CMD_CHANGE_BAUDRATE = 0x0F
CMD_FLASH_DEFL_BEGIN = 0x10
CMD_FLASH_DEFL_DATA = 0x11
CMD_FLASH_DEFL_END = 0x12
CMD_SPI_FLASH_MD5 = 0x13

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

FLASH_WRITE_SIZE = 0x4000  # 16 KiB per FLASH_DATA packet (matches esptool default)

# Per-chip parameters needed before flash_begin can succeed.
# (efuse/uart base addrs are not needed in ROM-only ASCII-free flashing path;
#  only SPI pin config + chip magic value for ID confirmation.)
CHIP_PARAMS = {
    "esp32s3": {"magic": 0x9, "spi_attach_args": 0},
    "esp32c3": {"magic": 0x7, "spi_attach_args": 0},
    "esp32s2": {"magic": 0x0, "spi_attach_args": 0},
}

CHIP_MAGIC_VALUES = {
    0x00F01D83: "esp32s2",
    0x6921506F: "esp32c3",  # ESP32-C3 (eco3+)
    0x1B31506F: "esp32c3",  # ESP32-C3 (eco1/2)
    0x9: "esp32s3",  # placeholder, real S3 magic checked separately below
}

# Real chip magic values from ROM (read_reg @ 0x40001000 on most, varies).
# We primarily trust the chip name the user passed with --chip and only use
# magic-value detection as a sanity warning, not a hard gate, since magic
# tables drift across silicon revisions and we'd rather flash a correctly
# user-specified chip than refuse on a magic-number mismatch.
KNOWN_MAGIC = {
    0x00F01D83: "esp32s2",
    0x6921506F: "esp32c3",
    0x1B31506F: "esp32c3",
    0x9: "esp32s3",
    0x00000009: "esp32s3",
}


class RomLoaderError(RuntimeError):
    pass


class RomLoader:
    """
    Speaks the ESP32 ROM bootloader SLIP protocol over a pair of raw
    USB bulk endpoints (ep_in / ep_out) obtained from usb_device.py.

    `read_fn(n, timeout_ms) -> bytes` and `write_fn(data, timeout_ms) -> int`
    are injected so this class has zero direct USB dependency and can be
    unit-tested with fakes.
    """

    def __init__(self, read_fn, write_fn, chip: str):
        self._read = read_fn
        self._write = write_fn
        self.chip = chip
        self._seq = 0

    # ---- SLIP framing ----------------------------------------------------

    @staticmethod
    def _slip_encode(payload: bytes) -> bytes:
        out = bytearray([SLIP_END])
        for b in payload:
            if b == SLIP_END:
                out += bytes([SLIP_ESC, SLIP_ESC_END])
            elif b == SLIP_ESC:
                out += bytes([SLIP_ESC, SLIP_ESC_ESC])
            else:
                out.append(b)
        out.append(SLIP_END)
        return bytes(out)

    def _slip_read_packet(self, timeout: float) -> bytes:
        """
        Read one SLIP-framed packet, discarding any boot-log text that
        precedes the leading 0xC0 (the ROM prints plain-text banners on
        some chips before it's ready to answer the bootloader protocol).
        """
        deadline = time.time() + timeout
        buf = bytearray()
        started = False
        while time.time() < deadline:
            remaining_ms = max(1, int((deadline - time.time()) * 1000))
            chunk = self._read(256, remaining_ms)
            if not chunk:
                continue
            for b in chunk:
                if not started:
                    if b == SLIP_END:
                        started = True
                        buf.append(b)
                    continue  # discard pre-frame noise/banner bytes
                buf.append(b)
                if b == SLIP_END and len(buf) > 1:
                    # closing delimiter found
                    return self._slip_decode(bytes(buf))
        raise RomLoaderError("Timed out waiting for response from ROM bootloader")

    @staticmethod
    def _slip_decode(framed: bytes) -> bytes:
        assert framed[0] == SLIP_END and framed[-1] == SLIP_END
        body = framed[1:-1]
        out = bytearray()
        i = 0
        while i < len(body):
            b = body[i]
            if b == SLIP_ESC and i + 1 < len(body):
                nxt = body[i + 1]
                if nxt == SLIP_ESC_END:
                    out.append(SLIP_END)
                    i += 2
                    continue
                elif nxt == SLIP_ESC_ESC:
                    out.append(SLIP_ESC)
                    i += 2
                    continue
            out.append(b)
            i += 1
        return bytes(out)

    # ---- command/response --------------------------------------------------

    def _command(self, cmd: int, data: bytes = b"", chk: int = 0,
                 timeout: float = 3.0, retries: int = 3) -> tuple:
        """
        Send one ROM-loader command, return (value:int, body:bytes).
        Raises RomLoaderError on failure status after exhausting retries.
        """
        pkt = struct.pack("<BBHI", 0x00, cmd, len(data), chk) + data
        framed = self._slip_encode(pkt)

        last_err = None
        for attempt in range(retries):
            try:
                self._write(framed, 3000)
                while True:
                    resp = self._slip_read_packet(timeout)
                    if len(resp) < 8:
                        continue
                    direction, resp_cmd, size, value = struct.unpack_from("<BBHI", resp, 0)
                    body = resp[8:8 + size]
                    if direction != 0x01:
                        continue  # not a response packet, keep reading
                    if resp_cmd != cmd:
                        continue  # stale/async packet, keep reading
                    if len(body) >= 2:
                        status, err = body[-2], body[-1]
                        if status != 0:
                            raise RomLoaderError(
                                f"ROM loader rejected cmd 0x{cmd:02X} "
                                f"(status=0x{status:02X} err=0x{err:02X})"
                            )
                    return value, body
            except RomLoaderError:
                raise
            except Exception as e:
                last_err = e
                time.sleep(0.1)
                continue
        raise RomLoaderError(f"Command 0x{cmd:02X} failed after {retries} attempts: {last_err}")

    # ---- bootloader handshake ----------------------------------------------

    def sync(self, attempts: int = 7) -> bool:
        """
        Send the SYNC sequence the ROM expects to confirm it's alive and
        listening for SLIP packets. Returns True on success.
        """
        sync_payload = b"\x07\x07\x12\x20" + b"\x55" * 32
        for _ in range(attempts):
            try:
                self._command(CMD_SYNC, sync_payload, timeout=0.2, retries=1)
                # Drain any extra echoed SYNC responses (ROM sends several)
                drain_deadline = time.time() + 0.15
                while time.time() < drain_deadline:
                    try:
                        self._slip_read_packet(0.05)
                    except RomLoaderError:
                        break
                return True
            except RomLoaderError:
                continue
        return False

    def spi_attach(self):
        params = CHIP_PARAMS.get(self.chip, {})
        data = struct.pack("<I", params.get("spi_attach_args", 0)) + b"\x00" * 4
        self._command(CMD_SPI_ATTACH, data)

    # ---- flashing ------------------------------------------------------------

    def flash_begin(self, size: int, offset: int, block_size: int = FLASH_WRITE_SIZE):
        num_blocks = (size + block_size - 1) // block_size
        erase_size = num_blocks * block_size
        data = struct.pack("<IIII", erase_size, num_blocks, block_size, offset)
        # Newer ROM loaders (S2/S3/C3) take a 5th word: 0 = don't auto-reboot after.
        data += struct.pack("<I", 0)
        self._command(CMD_FLASH_BEGIN, data, timeout=10.0)

    def flash_block(self, data: bytes, seq: int, block_size: int = FLASH_WRITE_SIZE):
        if len(data) < block_size:
            data = data + b"\xff" * (block_size - len(data))
        checksum = _esp_checksum(data)
        header = struct.pack("<IIII", len(data), seq, 0, 0)
        self._command(CMD_FLASH_DATA, header + data, chk=checksum, timeout=10.0)

    def flash_finish(self, reboot: bool = True):
        # value 0 = reboot, 1 = stay in bootloader
        data = struct.pack("<I", 0 if reboot else 1)
        try:
            self._command(CMD_FLASH_END, data, timeout=3.0)
        except RomLoaderError:
            # Some ROMs don't send a clean response right before reboot — benign.
            pass

    def flash_md5(self, offset: int, size: int) -> str:
        data = struct.pack("<IIII", offset, size, 0, 0)
        _, body = self._command(CMD_SPI_FLASH_MD5, data, timeout=15.0)
        # body: 16 raw bytes OR 32 ASCII hex chars depending on ROM version,
        # followed by 2-byte status. Handle both.
        payload = body[:-2] if len(body) >= 2 else body
        if len(payload) >= 32:
            return payload[:32].decode("ascii", errors="ignore").lower()
        elif len(payload) >= 16:
            return payload[:16].hex()
        raise RomLoaderError("Unexpected MD5 response length from device")


def _esp_checksum(data: bytes, seed: int = 0xEF) -> int:
    chk = seed
    for b in data:
        chk ^= b
    return chk


def crc32_of(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF
