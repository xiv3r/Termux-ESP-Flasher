#!/usr/bin/env python3
"""
nrflash - Termux-native .bin flasher for ESP32 boards, no root required.

Covers two device families, auto-detected from the USB VID:PID:

  native USB CDC   ESP32-S3 / C3 / S2 (303A:1001 / 303A:0002) - talks
                   directly to the chip's USB-Serial-JTAG peripheral;
                   bootloader entry/exit driven by cdc_reset.py.

  UART bridge      Classic ESP32 and ESP8266 devkits behind a CP2102,
                   CH340/CH340G, CH9102, or FTDI FT232 bridge chip - the
                   bridge is opened over raw USB (same fd-wrapping as the
                   native path; Termux never gets a /dev/ttyUSB* node
                   without root, so this is not optional) and its DTR/RTS
                   lines are pulsed in the classic GPIO0+EN pattern by
                   uart_reset.py.

No esptool.py subprocess, no pyserial, no root either way.

Sibling tool to nrcap32 - reuses usb_device.py for all USB backend
detection / fd-wrapping / endpoint discovery, so it inherits the same
no-root Termux support that the capture tool already has.

Scope, on purpose:
  - ROM-loader only, falling back automatically to no stub upload if the
    per-chip stub in stub_flasher_data.py fails to load. Slower per-byte
    than real esptool without a stub, but it means a much smaller attack
    surface to review than shipping every esptool feature.

Usage:
    ./nrflash write --chip esp32c3 --offset 0x0 firmware.bin       # native USB (S3/C3/S2)
    ./nrflash write --chip esp32s3 --offset 0x0 firmware.bin --verify
    ./nrflash write --chip esp32 --offset 0x0 firmware.bin         # CH340/CP2102/FTDI devkit
    ./nrflash write --chip esp8266 --offset 0x0 firmware.bin       # NodeMCU, Wemos D1 mini, etc.
    ./nrflash verify --chip esp32c3 --offset 0x0 firmware.bin
    ./nrflash erase-info --chip esp32c3
    ./nrflash probe --chip esp32
"""

import argparse
import os
import sys
import time
import traceback
import usb.core

import espbridge as usb_device
from . import rom_loader
from . import cdc_reset
from . import uart_reset
from . import stub_flasher_data

__version__ = "1.4.1"
__author__ = "7wp81x"
__url__ = "https://github.com/7wp81x/Termux-ESP-Flasher"

CHUNK = rom_loader.FLASH_WRITE_SIZE  # ROM-only fallback block size; cmd_write
# switches to rom_loader.stub_flash_write_size(chip) for the session if
# stub upload succeeds (0x800 on ESP32-S2's USB-OTG, 0x4000 elsewhere).
CHIP_CHOICES = ("esp32s3", "esp32c3", "esp32s2", "esp32", "esp8266")

# Log/state data lives under the user's home, not next to the installed
# package - site-packages (or Termux's $PREFIX/lib/python*/site-packages)
# isn't guaranteed writable once this is pip-installed rather than run
# from a flat checkout.
DATA_DIR = os.environ.get(
    "NRFLASH_DATA_DIR",
    os.path.join(os.path.expanduser("~"), ".nrflash"),
)
LOG_FILE = os.path.join(DATA_DIR, "nrflash.log")
os.makedirs(DATA_DIR, exist_ok=True)

# Same convention as nrcap32: only the bootstrap (parent) process writes the
# log-file header. The fd-wrapped child just appends to the same file so the
# tail thread in launch_with_fd() can stream its output back to the
# bootstrap's terminal in real time.
IS_CHILD = "TERMUX_USB_FD" in os.environ
if not IS_CHILD:
    with open(LOG_FILE, "w") as f:
        f.write(f"=== nrflash log - {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")


def _log(msg: str):
    # The fd-wrapped child's stdout is already redirected into LOG_FILE by
    # launch_with_fd()'s dup2 dance, so printing here as well as appending
    # below would write every line into the file twice - the tail thread
    # would then dutifully print each line twice too. The child relies
    # entirely on the tail thread (reading LOG_FILE) to reach the real
    # terminal; only the pre-fork parent process prints directly here.
    if not IS_CHILD:
        print(msg, flush=True)
    try:
        with open(LOG_FILE, "a") as f:
            f.write(msg + "\n")
    except OSError:
        pass


def _progress(done: int, total: int, label: str = "Flashing"):
    pct = 0 if total == 0 else int(done * 100 / total)
    bar_len = 24
    filled = int(bar_len * pct / 100)
    bar = "#" * filled + "-" * (bar_len - filled)
    sys.stdout.write(f"\r\033[1;36m[-]\033[0m {label} [\033[1;32m{bar}\033[0m] {pct:3d}%  (\033[1;33m{done}\033[0m/\033[1;32m{total}\033[0m bytes)")
    sys.stdout.flush()
    if done >= total:
        sys.stdout.write("\n")


# ---- device acquisition (mirrors nrcap32's bootstrap dance) ----------------

def acquire_device():
    """
    Detect backend (root vs termux), open the device, and return a ready
    usb.core.Device with the interface already claimed and toggles reset.

    On the 'termux' backend this relies on TERMUX_USB_FD already being set
    in the environment - i.e. this process is the child launch_with_fd()
    spawned after request_permission() succeeded (see bootstrap() below),
    the same two-step flow nrcap32 uses.
    """
    backend = usb_device.detect_backend()

    if backend == "termux":
        fd_str = os.environ.get("TERMUX_USB_FD")
        if not fd_str:
            raise RuntimeError(
                "TERMUX_USB_FD not set - acquire_device() must be called "
                "after the termux-usb permission bootstrap, not directly."
            )
        device = usb_device.wrap_fd(int(fd_str))
        fd_wrapped = True
    else:
        device = usb_device.wrap_direct()
        fd_wrapped = False

    _log(f"\033[1;34m[*]\033[0m Device: \033[1;32m{usb_device.describe_device(device)}\033[0m")
    ep_in, ep_out, iface = usb_device.get_cdc_endpoints(device)
    usb_device.claim_device(device, iface, fd_wrapped=fd_wrapped)
    usb_device.reset_endpoint_toggles(device, ep_in, ep_out)

    if usb_device.is_uart_bridge(device):
        # CH340/CH340G, CP2102, CH9102, FTDI: configure 115200 8N1 line
        # coding before touching DTR/RTS. Bootloader entry/exit itself is
        # handled separately by uart_reset.py, driven off the same
        # set_dtr_rts() primitive, not by the pulse baked into
        # init_uart_bridge() (that pulse assumes "release to normal boot",
        # which isn't always what the caller wants next).
        _log("\033[1;34m[*]\033[0m UART bridge detected - configuring line coding...")
        usb_device.init_uart_bridge(device)
        reset = uart_reset
    else:
        reset = cdc_reset

    return device, ep_in, ep_out, reset


def enter_bootloader_and_sync(device, loader, reset, attempts: int = 3) -> bool:
    """
    Pulse the reset sequence and sync with the ROM bootloader, retrying
    the *whole* reset+sync dance a few times before giving up.

    A single reset pulse can land at the wrong point in the chip's boot
    cycle - e.g. mid-boot, or user firmware that hasn't started polling
    the CDC control lines yet - and just get ignored, causing sync() to
    fail even though the device is otherwise fine. This is the same
    reason PlatformIO/esptool uploads often fail once with a bare
    "Connecting......." timeout and then succeed immediately on a second
    run without you doing anything differently: re-issuing the reset
    pulse a second time usually lands cleanly. Looping here does that
    automatically instead of making you re-run nrflash by hand.
    """
    for attempt in range(1, attempts + 1):
        reset.enter_bootloader(device)
        if loader.sync():
            return True
        if attempt < attempts:
            _log(f"\033[1;31m[!]\033[0m No response on attempt \033[1;33m{attempt}/{attempts}\033[0m - "
                 f"retrying reset pulse...")
    return False


def make_read_write(device, ep_in: int, ep_out: int):
    """
    Build the read_fn/write_fn closures rom_loader.RomLoader expects,
    backed by raw pyusb bulk transfers on the endpoints usb_device.py
    already found for us.
    """
    def read_fn(n: int, timeout_ms: int) -> bytes:
        try:
            data = device.read(ep_in, n, timeout=timeout_ms)
            return bytes(data)
        except Exception:
            return b""

    def write_fn(data: bytes, timeout_ms: int) -> int:
        """
        Bulk-write the full buffer, looping on short writes.

        pyusb's device.write() returns the number of bytes the host
        controller actually transferred in *that call* - it is allowed to
        be less than len(data), especially over a flaky Android OTG link.
        The previous version of this function returned that count straight
        back to rom_loader._command(), which never checks it. The result:
        a SLIP-framed packet (e.g. a FLASH_DATA block) could be silently
        truncated mid-transfer. Depending on exactly where the cut lands,
        the ROM bootloader either desyncs (often masked by retries deeper
        in the protocol) or only part of the intended flash payload for
        that block actually gets written, while the block still reports a
        success status. That reproduces exactly what you saw: a clean
        100% progress bar, no per-block errors, but the on-device MD5
        differs from the local file's MD5 afterward.

        This version keeps writing until either every byte is sent or the
        overall deadline (timeout_ms, treated as a budget for the whole
        buffer) expires, and raises instead of returning a partial count.
        """
        deadline = time.time() + (timeout_ms / 1000.0)
        total = len(data)
        sent = 0
        mv = memoryview(data)
        while sent < total:
            remaining_ms = max(1, int((deadline - time.time()) * 1000))
            try:
                n = device.write(ep_out, mv[sent:], timeout=remaining_ms)
            except Exception as e:
                raise rom_loader.RomLoaderError(
                    f"USB bulk write failed after {sent}/{total} bytes: {e}"
                )
            if not n:
                raise rom_loader.RomLoaderError(
                    f"USB bulk write stalled at {sent}/{total} bytes "
                    "(0-byte transfer - check the OTG cable/connection)"
                )
            sent += n
            if sent < total and time.time() >= deadline:
                raise rom_loader.RomLoaderError(
                    f"USB bulk write timed out after {sent}/{total} bytes"
                )
        return sent

    return read_fn, write_fn


# ---- high level flash operations -------------------------------------------

def resolve_chip(loader, chip_arg, is_bridge: bool) -> str:
    """
    Return the chip name to use for this session: the explicit --chip value
    if the user gave one, otherwise auto-detected from the ROM's magic
    value register. Must be called after loader.sync() succeeds and before
    loader.spi_attach() (spi_attach's esp8266 special-case needs the real
    chip name already set on the loader).

    Sets loader.chip as a side effect either way, since RomLoader is
    constructed before the chip is known in the auto-detect path.
    """
    if chip_arg:
        loader.chip = chip_arg
        return chip_arg

    _log("\033[1;34m[*]\033[0m No --chip given - auto-detecting from ROM magic value...")
    magic = loader.read_reg(rom_loader.CHIP_MAGIC_REG_ADDR)
    detected = rom_loader.KNOWN_MAGIC.get(magic)
    if detected is None:
        raise RuntimeError(
            f"Could not auto-detect chip - unrecognized magic value "
            f"0x{magic:08X}. Pass --chip explicitly (one of: "
            f"{', '.join(CHIP_CHOICES)})."
        )

    # UART-bridge boards (CH340/CP2102/FTDI) are only ever classic ESP32
    # or ESP8266 in this tool's supported set; native USB CDC boards are
    # only ever the S2/S3/C3 family. A mismatch here is usually a stale/
    # unlisted magic value rather than an actual transport mixup, so this
    # is a warning, not a hard failure - the detected name is still used.
    bridge_family = {"esp32", "esp8266"}
    usb_family = {"esp32s2", "esp32s3", "esp32c3"}
    if is_bridge and detected not in bridge_family:
        _log(f"\033[1;31m[!]\033[0m Warning: detected \033[1;33m'{detected}'\033[0m but device is behind a "
             "UART bridge - unexpected combination. Double check with an "
             "explicit --chip if flashing misbehaves.")
    elif not is_bridge and detected not in usb_family:
        _log(f"\033[1;31m[!]\033[0m Warning: detected \033[1;33m'{detected}'\033[0m but device is native "
             "USB CDC - unexpected combination. Double check with an "
             "explicit --chip if flashing misbehaves.")

    loader.chip = detected
    _log(f"\033[1;32m[+]\033[0m Auto-detected chip: \033[1;32m{detected}\033[0m  (magic \033[1;36m0x{magic:08X}\033[0m)")
    return detected


def cmd_probe(chip: str):
    device, ep_in, ep_out, reset = acquire_device()
    read_fn, write_fn = make_read_write(device, ep_in, ep_out)
    loader = rom_loader.RomLoader(read_fn, write_fn, chip or "esp32")

    _log("\033[1;34m[*]\033[0m Resetting into ROM bootloader...")
    if not enter_bootloader_and_sync(device, loader, reset):
        _log("\033[1;36m[-]\033[0m No response from bootloader. Checks:")
        if reset is uart_reset:
            _log("      - Is the CH340/CP2102/FTDI bridge wired to EN+GPIO0 (auto-reset)?")
            _log("      - Some boards need BOOT held + EN tapped manually.")
        else:
            _log("      - Is this actually a native-USB-CDC board (303A:xxxx)?")
            _log("      - Some clones need the BOOT button held manually.")
        _log("      - Try unplugging/replugging the OTG cable once.")
        reset.reset_to_app(device)
        sys.exit(1)
    _log("\033[1;32m[+]\033[0m Bootloader is alive and synced.")

    chip = resolve_chip(loader, chip, reset is uart_reset)

    loader.spi_attach()
    _log("\033[1;32m[+]\033[0m SPI flash attached.")
    _log(f"\033[1;32m[+]\033[0m Chip target: \033[1;32m{chip}\033[0m")

    reset.reset_to_app(device)
    _log("\033[1;34m[*]\033[0m Reset back to application.")


def _read_bin(path: str) -> bytes:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"No such file: {path}")
    with open(path, "rb") as f:
        return f.read()


def cmd_write(chip: str, targets: list, verify: bool, no_reboot: bool, erase: bool, no_stub: bool = False):
    """
    targets: list of (offset:int, path:str) tuples, already sorted by offset.
    Flashes each one in a single sync/spi_attach session - i.e. one USB
    permission prompt / bootloader handshake for the whole batch, instead of
    re-running the full bootstrap dance per file. This is what lets you flash
    bootloader.bin + partitions.bin + firmware.bin directly at their real
    offsets instead of pre-merging them into one image first.
    """
    loaded = []
    for offset, path in targets:
        data = _read_bin(path)
        loaded.append((offset, path, data))
        _log(f"\033[1;34m[*]\033[0m \033[1;33m{path}\033[0m: \033[1;32m{len(data)} bytes\033[0m -> \033[1;36moffset 0x{offset:06X}\033[0m")

    device, ep_in, ep_out, reset = acquire_device()
    read_fn, write_fn = make_read_write(device, ep_in, ep_out)
    loader = rom_loader.RomLoader(read_fn, write_fn, chip or "esp32")

    _log("\033[1;34m[*]\033[0m Resetting into ROM bootloader...")
    if not enter_bootloader_and_sync(device, loader, reset):
        _log("\033[1;36m[-]\033[0m Sync failed - device did not respond as a ROM bootloader.")
        reset.reset_to_app(device)
        sys.exit(1)
    _log("\033[1;32m[+]\033[0m Synced.")

    chip = resolve_chip(loader, chip, reset is uart_reset)

    loader.spi_attach()
    _log("\033[1;32m[+]\033[0m SPI flash attached.")

    chunk_size = CHUNK
    stub = None if no_stub else stub_flasher_data.STUBS.get(chip)
    if no_stub:
        _log("\033[1;34m[*]\033[0m --no-stub set - using the plain ROM loader path.")
    elif stub is not None:
        _log("\033[1;34m[*]\033[0m Uploading stub flasher (faster block writes)...")
        if loader.upload_stub(stub):
            chunk_size = rom_loader.stub_flash_write_size(chip)
            _log(f"\033[1;32m[+]\033[0m Stub running - switching to \033[1;32m{chunk_size // 1024}\033[0m KiB blocks.")

            # Bumping the block size alone doesn't help much if we're still
            # riding the bridge's original 115200 baud - that's ~11.5 KB/s
            # no matter how big the blocks are, since it's a hard serial-
            # line limit, not a protocol-overhead one. Once the stub is up
            # it can handle a much higher rate, so renegotiate both ends
            # now. Native-USB-CDC chips (S3/C3/S2) don't go through
            # is_uart_bridge() at all, so this only fires for classic
            # ESP32/ESP8266 boards behind a real bridge chip.
            #
            # IMPORTANT: change_baudrate() makes the *device* switch rate
            # immediately once it's acked the command - it is not a "try
            # it and revert" operation. If the new rate doesn't actually
            # work on this cable/chip, the device is now transmitting at
            # a speed the host can't usefully listen at, and there is no
            # way to send it a "go back to 115200" command, because
            # reaching it at all requires already talking to it at the
            # rate that just proved broken. Simply setting the host's
            # baud back down (as an earlier version of this code did)
            # leaves the device and host permanently talking past each
            # other for the rest of the process - which is exactly the
            # "lost sync entirely, relaunch the tool" symptom.
            #
            # The only real fix is to physically re-enter the ROM
            # bootloader (a fresh EN/GPIO0 pulse always restarts the chip
            # listening at its default rate) and rebuild the whole
            # session - sync, SPI attach, stub upload - at 115200 before
            # trying the next candidate. That costs a couple seconds per
            # failed candidate but means a bad guess never leaves the
            # session dead.
            if usb_device.is_uart_bridge(device):
                def _rebuild_session_at_115200():
                    usb_device.set_uart_bridge_baud(device, 115200)
                    if not enter_bootloader_and_sync(device, loader, reset):
                        return False
                    loader.spi_attach()
                    return loader.upload_stub(stub)

                for candidate in (921600, 460800, 230400):
                    _log(f"\033[1;34m[*]\033[0m Trying \033[0;33m{candidate}\033[0m baud...")
                    ok = False
                    try:
                        loader.change_baudrate(candidate)
                        usb_device.set_uart_bridge_baud(device, candidate)
                        time.sleep(0.05)
                        ok = loader.sync(attempts=3)
                    except Exception:
                        ok = False

                    if ok:
                        _log(f"\033[1;32m[+]\033[0m Baud rate raised to \033[1;32m{candidate}\033[0m and "
                             "verified - stub was still capped at the "
                             "bridge's original 115200 baud (~11.5 KB/s) "
                             "even after uploading.")
                        break

                    _log(f"\033[1;31m[!]\033[0m \033[0;33m{candidate}\033[0m baud didn't hold - recovering "
                         "session at \033[0;32m115200\033[0m...")
                    if _rebuild_session_at_115200():
                        continue  # session is healthy again at 115200, try next candidate
                    else:
                        _log("\033[1;31m[!]\033[0m Could not recover the stub session after "
                             "a failed baud switch (bridge chip likely wedged talking "
                             "past the device at the new rate). Re-entering the "
                             "bootloader from scratch and falling back to the slower "
                             "ROM-only path rather than aborting the flash...")
                        usb_device.set_uart_bridge_baud(device, 115200)
                        if not enter_bootloader_and_sync(device, loader, reset):
                            _log("\033[1;36m[-]\033[0m Could not re-enter the bootloader "
                                 "after a failed baud recovery. Unplug/replug the board "
                                 "and re-run the command.")
                            sys.exit(1)
                        loader.spi_attach()
                        chunk_size = CHUNK
                        stub = None
                        _log(f"\033[1;31m[!]\033[0m Falling back to the ROM-only path "
                             f"({chunk_size} byte blocks). Flashing will still work, "
                             "just slower.")
                        break
                else:
                    _log("\033[1;31m[!]\033[0m No higher baud rate was usable - staying at "
                         "115200. Flashing will still work, just slower.")
        else:
            _log("\033[1;31m[!]\033[0m Stub upload failed - the chip may have "
                 "already jumped out of the ROM bootloader while trying to "
                 "run it (mem_finish() executes unconditionally, even when "
                 "the stub never answers back). Re-entering the bootloader "
                 "to get a clean session before falling back...")
            if not enter_bootloader_and_sync(device, loader, reset):
                _log("\033[1;36m[-]\033[0m Could not re-enter the bootloader "
                     "after the failed stub upload. Unplug/replug the board "
                     "(hold BOOT while plugging in, if this chip has no "
                     "auto-reset circuit) and re-run the command.")
                sys.exit(1)
            loader.spi_attach()
            _log(f"\033[1;31m[!]\033[0m Falling back to the slower ROM-only "
                 f"path ({chunk_size} byte blocks). Flashing will still "
                 "work, just slower.")
    else:
        _log(f"\033[1;31m[!]\033[0m No stub available for chip \033[1;33m'{chip}'\033[0m - using ROM-only path.")

    for offset, path, data in loaded:
        total = len(data)
        _log(f"\033[1;34m[*]\033[0m --- \033[1;32m{path}\033[0m @ \033[1;36m0x{offset:06X}\033[0m ---")

        if erase:
            _log(f"\033[1;34m[*]\033[0m Erasing \033[1;36m0x{offset:06X}-0x{offset+total:06X}\033[0m via ERASE_REGION...")
            try:
                loader.erase_region(offset, total)
                _log("\033[1;32m[+]\033[0m Erase complete.")
            except rom_loader.RomLoaderError as e:
                _log(f"\033[1;31m[!]\033[0m ERASE_REGION not supported by this ROM ({e}).")
                _log("    Continuing without it - flash_begin erases its own")
                _log("    write range anyway, so this is not fatal.")

        _log("\033[1;34m[*]\033[0m Starting flash_begin...")
        loader.flash_begin(total, offset, block_size=chunk_size)

        seq = 0
        sent = 0
        t0 = time.time()
        try:
            while sent < total:
                block = data[sent:sent + chunk_size]
                loader.flash_block(block, seq, block_size=chunk_size)
                sent += len(block)
                seq += 1
                _progress(sent, total)
        except rom_loader.RomLoaderError as e:
            _log(f"\n\033[1;36m[-]\033[0m Flash write failed at byte \033[1;31m{sent}\033[0m/\033[1;32m{total}\033[0m:\033[1;33m {e}")
            reset.reset_to_app(device)
            sys.exit(1)

        elapsed = time.time() - t0
        rate = (total / 1024) / elapsed if elapsed > 0 else 0
        _log(f"\n\033[1;32m[+]\033[0m Wrote \033[1;33m{total}\033[0m bytes in {elapsed:.1f}s ({rate:.1f} KB/s)")

        if verify:
            _log("\033[1;34m[*]\033[0m Verifying via on-device MD5...")
            import hashlib
            local_md5 = hashlib.md5(data).hexdigest()
            try:
                remote_md5 = loader.flash_md5(offset, total)
            except rom_loader.RomLoaderError as e:
                _log(f"\033[1;36m[-]\033[0m Could not read back MD5: {e}")
                reset.reset_to_app(device)
                sys.exit(1)
            if remote_md5 == local_md5:
                _log(f"\033[1;32m[+]\033[0m Verify OK  (md5 {local_md5})")
            else:
                _log(f"\033[1;36m[-]\033[0m Verify MISMATCH  local={local_md5}  device={remote_md5}")
                reset.reset_to_app(device)
                sys.exit(1)

    loader.flash_finish(reboot=not no_reboot)
    if no_reboot:
        _log("\033[1;34m[*]\033[0m Staying in bootloader (--no-reboot set).")
    else:
        _log("\033[1;34m[*]\033[0m Rebooting into application...")
        time.sleep(0.3)
        try:
            reset.reset_to_app(device)
        except usb.core.USBError:
            # flash_finish(reboot=True) already told the ROM to reboot -
            # on native USB chips (S2/S3/C3) that reboot re-initializes
            # the USB peripheral, which disconnects/re-enumerates the
            # device out from under us. This extra DTR/RTS pulse is only
            # needed as a fallback for ROMs/boards that don't reliably
            # act on flash_finish's own reboot flag - if the device is
            # already gone, the reboot already happened, so there's
            # nothing left to signal and nothing to worry about.
            pass
    _log("\033[1;32m[+]\033[0m Done.")


def cmd_verify(chip: str, offset: int, path: str):
    import hashlib
    data = _read_bin(path)
    total = len(data)
    local_md5 = hashlib.md5(data).hexdigest()

    device, ep_in, ep_out, reset = acquire_device()
    read_fn, write_fn = make_read_write(device, ep_in, ep_out)
    loader = rom_loader.RomLoader(read_fn, write_fn, chip or "esp32")

    _log("\033[1;34m[*]\033[0m Syncing with ROM bootloader...")
    if not enter_bootloader_and_sync(device, loader, reset):
        _log("\033[1;36m[-]\033[0m Sync failed.")
        reset.reset_to_app(device)
        sys.exit(1)

    chip = resolve_chip(loader, chip, reset is uart_reset)
    loader.spi_attach()

    _log(f"\033[1;34m[*]\033[0m Reading back MD5 of {total} bytes @ 0x{offset:06X}...")
    remote_md5 = loader.flash_md5(offset, total)
    reset.reset_to_app(device)

    if remote_md5 == local_md5:
        _log(f"\033[1;32m[+]\033[0m MATCH  (md5 {local_md5})")
    else:
        _log(f"\033[1;36m[-]\033[0m MISMATCH  local={local_md5}  device={remote_md5}")
        sys.exit(1)


def cmd_erase_info(chip: str):
    label = chip if chip else "(no --chip given)"
    _log(f"\033[1;34m[*]\033[0m {label}: this build does not implement full-chip erase.")
    _log("    'write --erase' will erase exactly the bytes about to be")
    _log("    overwritten (via ERASE_REGION) before flashing, if the ROM")
    _log("    supports it - it falls back silently to flash_begin's own")
    _log("    per-block erase if the ROM rejects ERASE_REGION.")
    _log("    Full chip erase is kept out on purpose - an erase covering")
    _log("    flash you are NOT about to immediately rewrite has no")
    _log("    recovery path if it's interrupted mid-operation over an")
    _log("    unreliable OTG link. Flash a full image (bootloader +")
    _log("    partition table + app) at the correct offsets instead of")
    _log("    erasing the whole chip first.")


# ---- argv parsing + termux-usb fd bootstrap --------------------------------

def build_parser():
    p = argparse.ArgumentParser(prog="nrflash",
                                 description="Termux-native .bin flasher for ESP32-S3/C3/S2 (native USB) "
                                              "and classic ESP32/ESP8266 boards behind a CH340/CP2102/FTDI "
                                              "UART bridge - no root, no esptool.py.")
    p.add_argument("--version", action="version",
                    version=f"nrflash {__version__} - {__author__} <{__url__}>")
    sub = p.add_subparsers(dest="action", required=True)

    # --chip is optional for probe/write/verify - if omitted, the tool syncs
    # with the ROM bootloader and auto-detects the chip from its magic
    # value register (see resolve_chip() in this file). erase-info doesn't
    # talk to any hardware at all, so there's nothing to detect - it still
    # requires an explicit --chip.
    common = dict(required=False, default=None, choices=CHIP_CHOICES)
    common_required = dict(required=True, choices=CHIP_CHOICES)

    p_probe = sub.add_parser("probe", help="Check the device answers the ROM bootloader protocol.")
    p_probe.add_argument("--chip", **common)

    p_write = sub.add_parser(
        "write",
        help="Flash one or more .bin files, each at its own offset.",
        description="Single file:  nrflash write --chip esp32c3 --offset 0x0 firmware.bin\n"
                     "Multiple:     nrflash write --chip esp32c3 "
                     "0x0:bootloader.bin 0x8000:partitions.bin 0x10000:firmware.bin\n"
                     "All targets in one invocation share a single USB permission "
                     "prompt and bootloader handshake.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p_write.add_argument("--chip", **common)
    p_write.add_argument("--offset", default=None,
                          help="Flash offset for a single bare FILE argument "
                               "(e.g. 0x0 or 0x10000). Ignored if any argument "
                               "uses the OFFSET:FILE form instead.")
    p_write.add_argument("--verify", action="store_true", help="MD5-verify each file after writing.")
    p_write.add_argument("--no-reboot", action="store_true", help="Stay in bootloader after flashing.")
    p_write.add_argument("--no-stub", action="store_true",
                          help="Skip the RAM stub entirely and use the plain ROM loader "
                               "(1024 byte blocks, slower). Recommended on boards with no "
                               "auto-reset circuit (native-USB S2 boards with only a BOOT "
                               "button): a failed stub handshake jumps the chip out of the "
                               "ROM bootloader with no way to re-enter it except physically "
                               "re-holding BOOT, so it's safer to never attempt the jump.")
    p_write.add_argument("--erase", action="store_true",
                          help="Explicitly erase exactly [offset, offset+len(file)) via "
                               "ERASE_REGION before writing each file, instead of relying "
                               "only on flash_begin's own per-block erase. Falls back "
                               "silently if the ROM doesn't support ERASE_REGION.")
    p_write.add_argument("files", nargs="+",
                          help="One or more targets. Either a single FILE (paired with "
                               "--offset, default 0x0), or one-or-more OFFSET:FILE pairs "
                               "(e.g. 0x0:bootloader.bin 0x10000:firmware.bin).")

    p_verify = sub.add_parser("verify", help="Compare a local .bin's MD5 against what's on flash.")
    p_verify.add_argument("--chip", **common)
    p_verify.add_argument("--offset", default="0x0")
    p_verify.add_argument("file")

    p_erase = sub.add_parser("erase-info", help="Explain why full chip erase isn't offered.")
    p_erase.add_argument("--chip", **common_required)

    return p


def _parse_offset(s: str) -> int:
    return int(s, 16) if s.lower().startswith("0x") else int(s)


def _parse_write_targets(files: list, default_offset) -> list:
    """
    Turn the write subcommand's positional 'files' list into a sorted list
    of (offset:int, path:str) tuples.

    Each item is either:
      - "OFFSET:PATH"  (e.g. "0x8000:partitions.bin") - explicit offset
      - "PATH"         (e.g. "firmware.bin") - uses --offset / default_offset,
                        and is only valid when it's the ONLY item, since two
                        bare files with no offsets would silently collide.

    Windows-style drive letters (C:\\...) aren't a concern here since this
    tool only targets Termux/Linux paths, so a single colon is unambiguous.
    """
    targets = []
    bare_files = []
    for item in files:
        if ":" in item:
            off_str, path = item.split(":", 1)
            try:
                offset = _parse_offset(off_str)
            except ValueError:
                raise RuntimeError(
                    f"Can't parse offset in '{item}' - expected OFFSET:PATH, "
                    f"e.g. 0x10000:firmware.bin"
                )
            targets.append((offset, path))
        else:
            bare_files.append(item)

    if bare_files:
        if targets:
            raise RuntimeError(
                "Can't mix bare FILE arguments with OFFSET:FILE arguments - "
                "give every file an explicit OFFSET:FILE, e.g. "
                f"0x0:{bare_files[0]}"
            )
        if len(bare_files) > 1:
            raise RuntimeError(
                "Multiple files given with no offsets - use OFFSET:FILE for "
                "each one, e.g. 0x0:bootloader.bin 0x8000:partitions.bin"
            )
        offset = _parse_offset(default_offset) if default_offset is not None else 0
        targets.append((offset, bare_files[0]))

    seen_offsets = {}
    for offset, path in targets:
        if offset in seen_offsets:
            raise RuntimeError(
                f"Offset 0x{offset:06X} given twice "
                f"({seen_offsets[offset]!r} and {path!r})"
            )
        seen_offsets[offset] = path

    targets.sort(key=lambda t: t[0])
    return targets


def bootstrap(argv_tail: list) -> None:
    """
    No-root Termux bootstrap, mirroring nrcap32's bootstrap() exactly:

      1. auto_detect_device()   - find the device path under /dev/bus/usb/...
      2. request_permission()   - dedicated permission-dialog call, blocks
                                   until the user taps Allow (or denies/times
                                   out). This is NOT the same call that opens
                                   the device - keeping them separate is what
                                   makes the dialog reliable.
      3. launch_with_fd()       - fork termux-api's *open* call with output
                                   redirected to LOG_FILE (not a blocking
                                   pipe read), tail the log back to this
                                   terminal, and let the child re-exec this
                                   same script with TERMUX_USB_FD set.

    Earlier nrflash builds called usb_device.open_usb_device() directly,
    which folds the permission dialog and the open into a single blocking
    os.read() on a pipe with no log-file isolation. That is what caused the
    dialog to appear to "vanish" / random Permission denied flip-flopping -
    request_permission() + launch_with_fd() is the path that's actually
    been proven to work reliably (it's what nrcap32 itself uses).
    """
    try:
        device_path = usb_device.auto_detect_device()
    except RuntimeError as e:
        _log(f"\033[1;36m[-]\033[0m {e}")
        sys.exit(1)
    _log(f"\033[1;32m[+]\033[0m Found device: \033[1;32m{device_path}\033[0m")

    _log("\033[1;34m[*]\033[0m Requesting USB permission (tap Allow on your phone)...")
    granted = usb_device.request_permission(device_path)
    if not granted:
        _log("\033[1;36m[-]\033[0m Permission denied or timed out.")
        sys.exit(1)

    # Re-invoke via `python3 -m nrflash.cli ...` rather than a hardcoded
    # source path - works identically whether this is a flat checkout or
    # a pip-installed package (site-packages path still resolves fine for
    # -m, but this also survives future packaging changes like zipapps).
    cmd = f"{sys.executable} -m nrflash.cli " + " ".join(argv_tail)

    child_pid, child_done, tail_thread = usb_device.launch_with_fd(
        cmd=cmd,
        device_path=device_path,
        log_file=LOG_FILE,
        tail_fn=lambda line: print(line, end="", flush=True),
    )

    try:
        _, status = os.waitpid(child_pid, 0)
        exit_code = os.waitstatus_to_exitcode(status)
    except KeyboardInterrupt:
        import signal
        os.kill(child_pid, signal.SIGTERM)
        _, status = os.waitpid(child_pid, 0)
        exit_code = os.waitstatus_to_exitcode(status)

    child_done.set()
    tail_thread.join(timeout=2)

    if exit_code != 0:
        sys.exit(exit_code)


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Root backend (Nethunter / Termux:Root) needs no fd bootstrap -
    # go straight to the requested command.
    if usb_device.is_root() or not usb_device.has_termux_api():
        _dispatch(args)
        return

    # Termux no-root, fd already set: this IS the child launch_with_fd()
    # spawned - run the real command against the wrapped fd.
    if IS_CHILD:
        _dispatch(args)
        return

    # First invocation on no-root Termux: request permission, then launch
    # the fd-bound child. Does not return until the child exits.
    bootstrap(sys.argv[1:])


def _dispatch(args):
    logo = r"""
{E}[1;32m  _  _ ___ {E}[1;36m___ _         _    {E}[0m
{E}[1;32m | \| | _ \{E}[1;36m __| |__ _ __| |_  {E}[0m
{E}[1;32m | .` |   / {E}[1;36m_|| / _` (_-< ' \ {E}[0m
{E}[1;32m |_|\_|_|_\{E}[1;36m_| |_\__,_/__/_||_| {E}[1;32mv{version}{E}[0m
""".format(E="\x1b", version=__version__)

    # Route through _log(), not a bare print(): when running as the
    # fd-wrapped no-root Termux child, stdout is dup2'd into LOG_FILE and
    # every other line goes out via _log()'s explicit open()/write(), which
    # hits the file immediately. A raw print() here instead lands in
    # Python's stdout buffer (block-buffered, since the fd is a file, not
    # a tty) and only gets flushed later - which is exactly why the logo
    # was showing up mid-run instead of at the very top. _log() also
    # already does the right thing for the non-child/root case (prints
    # directly), so this fixes ordering for both paths.
    _log(logo)

    try:
        if args.action == "probe":
            cmd_probe(args.chip)
        elif args.action == "write":
            targets = _parse_write_targets(args.files, args.offset)
            cmd_write(args.chip, targets, args.verify, args.no_reboot, args.erase, args.no_stub)
        elif args.action == "verify":
            cmd_verify(args.chip, _parse_offset(args.offset), args.file)
        elif args.action == "erase-info":
            cmd_erase_info(args.chip)
    except RuntimeError as e:
        _log(f"\033[1;36m[-]\033[0m {e}")
        sys.exit(1)
    except FileNotFoundError as e:
        _log(f"\033[1;36m[-]\033[0m {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        _log("\n\033[1;36m[-]\033[0m Interrupted.")
        sys.exit(130)


if __name__ == "__main__":
    main()