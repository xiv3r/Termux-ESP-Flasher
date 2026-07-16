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

# These two are historically "stub loader only" commands in the original
# ESP32 ROM bootloader. They were promoted into the ROM command set for the
# esp32s2-family loader ABI (S2/S3/C3) - the exact chips this tool targets -
# but availability still isn't guaranteed across every ROM revision. Callers
# must be ready for RomLoaderError (unsupported command) and fall back to
# flash_begin's implicit per-block erase instead of treating this as fatal.
CMD_ERASE_FLASH = 0xD0
CMD_ERASE_REGION = 0xD1

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

FLASH_WRITE_SIZE = 0x400  # 1 KiB per FLASH_DATA packet - required for ROM-only
# loader (no stub uploaded). 0x4000 (16 KiB) is esptool's *stub-loader* block
# size, which relies on the stub's larger RAM receive buffer. The plain ROM
# bootloader's SLIP receive buffer is much smaller; sending 16 KiB blocks to
# it can silently overflow/corrupt the block while the ROM still acks with a
# success status, which is exactly the "100% written, but 0xFF on readback"
# symptom this fixes.

STUB_FLASH_WRITE_SIZE = 0x4000  # 16 KiB - safe once the stub is running,
# since the stub has a much larger RAM receive buffer than the ROM does.
# NOT safe on ESP32-S2's native USB-OTG transport - see USB_OTG_* below.

ESP_RAM_BLOCK = 0x1800  # 6 KiB - chunk size for MEM_DATA uploads while
# pushing the stub itself into RAM (matches real esptool; this is a
# separate, smaller limit than STUB_FLASH_WRITE_SIZE, which only applies
# once the stub is already running and handling FLASH_DATA).

# ESP32-S2's native USB-OTG peripheral can't reliably absorb the standard
# 0x1800/0x4000 block sizes above: its internal buffering silently
# corrupts/drops data past 0x800 bytes per transfer, both during the stub
# RAM upload (MEM_DATA) and the post-stub flash write (FLASH_DATA) phase.
# Real esptool caps both to 0x800 for exactly this case (see
# uses_usb_otg()/_post_connect() in esptool's loader.py - re-derived
# here from public source, no code copied). This silent-corruption-with-
# ACK behavior is why S2 stub uploads/flashes failed with the old fixed
# sizes: the chip acks blocks it never actually buffered correctly.
USB_OTG_RAM_BLOCK = 0x800
USB_OTG_FLASH_WRITE_SIZE = 0x800

# Chips whose transport is the raw USB-OTG peripheral rather than
# USB-Serial-JTAG (which behaves like a real UART with flow control and
# doesn't need the reduction). ESP32-S3 also has a USB-OTG mode, but this
# tool only ever talks to S3 over USB-Serial-JTAG (PID 303A:1001), so
# only S2 needs the reduction in practice.
USB_OTG_CHIPS = {"esp32s2"}


def ram_block_size(chip: str) -> int:
    return USB_OTG_RAM_BLOCK if chip in USB_OTG_CHIPS else ESP_RAM_BLOCK


def stub_flash_write_size(chip: str) -> int:
    return USB_OTG_FLASH_WRITE_SIZE if chip in USB_OTG_CHIPS else STUB_FLASH_WRITE_SIZE

# Per-chip parameters needed before flash_begin can succeed.
# (efuse/uart base addrs are not needed in ROM-only ASCII-free flashing path;
#  only SPI pin config + chip magic value for ID confirmation.)
CHIP_PARAMS = {
    "esp32s3": {"magic": 0x9, "spi_attach_args": 0},
    "esp32c3": {"magic": 0x7, "spi_attach_args": 0},
    "esp32s2": {"magic": 0x000007C6, "spi_attach_args": 0},
    "esp32": {"magic": 0x00F01D83, "spi_attach_args": 0},  # classic ESP32,
    # reached over an external UART bridge (CP2102/CH340/FTDI) - no native
    # USB CDC on this chip.
    "esp8266": {"magic": 0xFFF0C101, "spi_attach_args": 0},  # unused - spi_attach()
    # is skipped entirely for esp8266 pre-stub, see spi_attach()'s comment.
}

# Real chip magic values from ROM (read_reg @ 0x40001000 on most, varies).
# We primarily trust the chip name the user passed with --chip and only use
# magic-value detection as a sanity warning, not a hard gate, since magic
# tables drift across silicon revisions and we'd rather flash a correctly
# user-specified chip than refuse on a magic-number mismatch.
KNOWN_MAGIC = {
    0x00F01D83: "esp32",
    0x000007C6: "esp32s2",
    0x6921506F: "esp32c3",  # ESP32-C3 (eco3+)
    0x1B31506F: "esp32c3",  # ESP32-C3 (eco1/2)
    0x9: "esp32s3",
    0x00000009: "esp32s3",
    0xFFF0C101: "esp8266",
}

CHIP_MAGIC_VALUES = KNOWN_MAGIC

# Register address the ROM's chip-identification magic value lives at, used
# for --chip auto-detection. This matches the classic ESP32 and the
# esp32s2-family loader ABI (S2/S3/C3) that real esptool has relied on for
# years - it's a best-effort heuristic, not a guarantee for every silicon
# revision, which is why detect_chip() below still lets an explicit --chip
# override it and never hard-fails a flash just because the magic value is
# unrecognized (see resolve_chip() in nrflash.py).
CHIP_MAGIC_REG_ADDR = 0x40001000


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
        self.is_stub = False  # set True by upload_stub() on success

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

    def read_reg(self, addr: int) -> int:
        """
        Read a 32-bit register via the ROM's READ_REG command. The read
        value comes back in the response packet's `value` field (not the
        body), same as real esptool's read_reg().
        """
        data = struct.pack("<I", addr)
        value, _ = self._command(CMD_READ_REG, data, timeout=3.0)
        return value

    def spi_attach(self):
        if self.chip == "esp8266" and not self.is_stub:
            # ESP8266's ROM bootloader has its own built-in flash config and
            # doesn't expect SPI_ATTACH before the stub is running - real
            # esptool skips this call for the same reason (see cmds.py's
            # `esp.CHIP_NAME != "ESP8266" and not esp.IS_STUB` guards).
            return
        params = CHIP_PARAMS.get(self.chip, {})
        data = struct.pack("<I", params.get("spi_attach_args", 0)) + b"\x00" * 4
        self._command(CMD_SPI_ATTACH, data)

    # ---- stub flasher upload -------------------------------------------------
    #
    # The ROM bootloader alone only safely accepts small (1 KiB) FLASH_DATA
    # blocks - see FLASH_WRITE_SIZE's comment. Real esptool works around this
    # by uploading a small RAM-resident "stub" program that speaks the same
    # command protocol but with a much bigger receive buffer, then switching
    # to it for the rest of the session. This mirrors that: mem_begin/
    # mem_block push the stub's code+data segments into RAM, mem_finish jumps
    # to its entry point, and we then wait for its raw "OHAI" greeting as
    # confirmation it's alive and listening in place of the ROM.

    def mem_begin(self, size: int, blocks: int, block_size: int, offset: int):
        data = struct.pack("<IIII", size, blocks, block_size, offset)
        self._command(CMD_MEM_BEGIN, data, timeout=5.0)

    def mem_block(self, data: bytes, seq: int):
        checksum = _esp_checksum(data)
        header = struct.pack("<IIII", len(data), seq, 0, 0)
        self._command(CMD_MEM_DATA, header + data, chk=checksum, timeout=5.0)

    def mem_finish(self, entrypoint: int):
        # value: (execute_flag, entrypoint). execute_flag=0 means "jump to
        # entrypoint" (the naming is inverted-looking but matches the ROM's
        # own command definition and real esptool's usage).
        data = struct.pack("<II", 0, entrypoint)
        try:
            self._command(CMD_MEM_END, data, timeout=3.0)
        except RomLoaderError:
            # Some ROMs reset/reconfigure the UART/USB path before the
            # response is fully sent once execution jumps away - benign,
            # matches flash_finish()'s same tolerance.
            pass

    def _upload_segment(self, data: bytes, offset: int):
        if not data:
            return
        block_size = ram_block_size(self.chip)
        length = len(data)
        blocks = (length + block_size - 1) // block_size
        self.mem_begin(length, blocks, block_size, offset)
        for seq in range(blocks):
            start = seq * block_size
            self.mem_block(data[start:start + block_size], seq)

    def upload_stub(self, stub: dict) -> bool:
        """
        Upload the given stub dict (as found in stub_flasher_data.STUBS) and
        confirm it's running. Returns True on success, False on any failure
        - callers should fall back to the plain ROM-only flashing path
        rather than treating a failed stub upload as fatal, since the ROM
        path is slower but already proven to work.
        """
        import base64

        try:
            text = base64.b64decode(stub["text"])
            data = base64.b64decode(stub["data"]) if stub.get("data") else b""

            self._upload_segment(text, stub["text_start"])
            self._upload_segment(data, stub["data_start"])
            self.mem_finish(stub["entry"])

            # The stub's greeting is a raw 4-byte SLIP-framed payload, not a
            # normal (direction, cmd, size, value) response packet - read it
            # directly rather than through _command()'s response parsing.
            greeting = self._slip_read_packet(timeout=3.0)
            if greeting != b"OHAI":
                return False

            # Drain the SYNC-drain style leftover noise, if any, the same
            # way sync() does, so we start the stub session on a clean read
            # buffer rather than risking a stale packet confusing the next
            # command's response matching.
            drain_deadline = time.time() + 0.1
            while time.time() < drain_deadline:
                try:
                    self._slip_read_packet(0.05)
                except RomLoaderError:
                    break

            self.is_stub = True
            return True
        except Exception:
            # Deliberately broad: any failure during stub upload (timeout,
            # malformed stub dict, USB hiccup, wrong OHAI response, etc.)
            # should fall back to the ROM-only path, not crash the flash.
            return False

    def change_baudrate(self, new_baud: int, old_baud: int = 115200):
        """
        Tell the stub (or ROM, on chips where it's supported) to switch its
        read/write baud rate. Only meaningful once a stub is running for
        chips that sit behind a real UART bridge (CH340/CP2102/FTDI) -
        native USB CDC chips ignore actual baud since there's no real UART
        clock underneath. Callers are responsible for reprogramming the
        *host*-side bridge to the same rate (see
        usb_device.set_uart_bridge_baud()) immediately after this call
        returns, since the device stops listening at the old rate the
        moment it applies this.
        """
        data = struct.pack("<II", new_baud, old_baud if self.is_stub else 0)
        self._command(CMD_CHANGE_BAUDRATE, data, timeout=3.0)

    # ---- flashing ------------------------------------------------------------

    def erase_region(self, offset: int, size: int, sector_size: int = 0x1000):
        """
        Erase exactly [offset, offset+size) using the ROM's dedicated
        ERASE_REGION command, rounded up to full erase sectors (required
        by the ROM - both offset and size must be sector-size multiples).

        This is intentionally narrower than a full chip erase: it only
        ever erases bytes this same invocation is about to overwrite, so
        an interruption mid-erase leaves nothing worse off than an
        interrupted write would have anyway - the region still gets
        correctly erased+rewritten by flash_begin/flash_block right after.

        Raises RomLoaderError if the ROM doesn't support this command
        (it was stub-loader-only on the original ESP32 ROM, and isn't
        guaranteed across every S2/S3/C3 ROM revision even though the
        esp32s2-family loader ABI generally promotes it into ROM). The
        caller should catch this and continue without erasing - flash_begin
        always erases its own write range as part of normal operation.
        """
        aligned_offset = offset - (offset % sector_size)
        end = offset + size
        aligned_end = end + ((-end) % sector_size)
        aligned_size = aligned_end - aligned_offset

        data = struct.pack("<II", aligned_offset, aligned_size)
        # Erase of a large region can legitimately take many seconds on
        # real NOR flash; give it a generous timeout rather than a false
        # "unsupported" failure due to a slow chip.
        self._command(CMD_ERASE_REGION, data, timeout=60.0, retries=1)

    # Empirically-generous margin for the ROM's synchronous erase-before-reply
    # inside flash_begin: a plain NOR flash erase runs at roughly 40-80s/MB
    # worst case (bigger/older parts, no stub speed-up), so scale the
    # timeout with erase size rather than using one fixed value that only
    # works for small writes. Matches the spirit of real esptool's
    # ERASE_WRITE_TIMEOUT_PER_MB, re-derived rather than copied.
    ERASE_TIMEOUT_PER_MB = 60.0
    MIN_ERASE_TIMEOUT = 10.0

    def flash_begin(self, size: int, offset: int, block_size: int = FLASH_WRITE_SIZE):
        num_blocks = (size + block_size - 1) // block_size
        erase_size = num_blocks * block_size
        data = struct.pack("<IIII", erase_size, num_blocks, block_size, offset)
        # The 5th word (encrypted-write flag) is only understood by the
        # stub, or by ROMs newer than the classic ESP32/ESP8266 ones - those
        # two chips' plain ROM bootloaders reject/mishandle the extended
        # param format. Matches real esptool's same chip/is_stub check.
        if self.is_stub or self.chip not in ("esp32", "esp8266"):
            data += struct.pack("<I", 0)
        erase_mb = erase_size / (1024 * 1024)
        timeout = max(self.MIN_ERASE_TIMEOUT, erase_mb * self.ERASE_TIMEOUT_PER_MB)
        self._command(CMD_FLASH_BEGIN, data, timeout=timeout)

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