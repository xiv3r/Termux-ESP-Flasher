#!/usr/bin/env python3
"""
nrflash - Termux-native .bin flasher for ESP32-S3 / C3 / S2 native-USB-CDC
boards. No esptool.py subprocess, no pyserial, no root.

Sibling tool to nrcap32 - reuses usb_device.py for all USB backend
detection / fd-wrapping / endpoint discovery, so it inherits the same
no-root Termux support that the capture tool already has.

Scope, on purpose:
  - Targets ONLY native USB CDC chips (303A:1001 / 303A:0002): S3, C3, S2.
    Boards wired through a UART bridge (CP2102/CH340/FTDI) already work
    fine with the *real* esptool.py over /dev/ttyUSB*, which Termux can
    open directly without any of this — there's no fd-wrapping problem to
    solve there, just a USB-permission one, and esptool.py over pyserial
    already does its own DTR/RTS reset correctly for those chips. This
    tool exists for the no-bridge-chip case esptool.py's serial-only
    transport can't reach on stock Termux.
  - ROM-loader only, no stub upload. Slower per-byte than real esptool,
    but it means zero embedded stub binaries to maintain per chip and a
    much smaller attack surface to review.

Usage:
    ./nrflash write --chip esp32c3 --offset 0x0 firmware.bin
    ./nrflash write --chip esp32s3 --offset 0x0 firmware.bin --verify
    ./nrflash verify --chip esp32c3 --offset 0x0 firmware.bin
    ./nrflash erase-info --chip esp32c3
    ./nrflash probe
"""

import argparse
import os
import sys
import time

import usb_device
import rom_loader
import cdc_reset

CHUNK = rom_loader.FLASH_WRITE_SIZE
CHIP_CHOICES = ("esp32s3", "esp32c3", "esp32s2")


def _log(msg: str):
    print(msg, flush=True)


def _progress(done: int, total: int, label: str = "Flashing"):
    pct = 0 if total == 0 else int(done * 100 / total)
    bar_len = 24
    filled = int(bar_len * pct / 100)
    bar = "#" * filled + "-" * (bar_len - filled)
    sys.stdout.write(f"\r{label} [{bar}] {pct:3d}%  ({done}/{total} bytes)")
    sys.stdout.flush()
    if done >= total:
        sys.stdout.write("\n")


# ---- device acquisition (mirrors nrcap32's bootstrap dance) ----------------

def acquire_device():
    """
    Detect backend (root vs termux), open the device, and return a ready
    usb.core.Device with the interface already claimed and toggles reset.

    On the 'termux' backend this relies on TERMUX_USB_FD already being set
    in the environment (i.e. this process was re-exec'd by
    usb_device.relaunch_with_fd / open_usb_device — same bootstrap nrcap32
    uses), so main() below does that re-exec before calling this.
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

    _log(f"[*] Device: {usb_device.describe_device(device)}")
    ep_in, ep_out, iface = usb_device.get_cdc_endpoints(device)
    usb_device.claim_device(device, iface, fd_wrapped=fd_wrapped)
    usb_device.reset_endpoint_toggles(device, ep_in, ep_out)
    return device, ep_in, ep_out


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
        return device.write(ep_out, data, timeout=timeout_ms)

    return read_fn, write_fn


# ---- high level flash operations -------------------------------------------

def cmd_probe(chip: str):
    device, ep_in, ep_out = acquire_device()
    _log("[*] Resetting into ROM bootloader...")
    cdc_reset.enter_bootloader(device)

    read_fn, write_fn = make_read_write(device, ep_in, ep_out)
    loader = rom_loader.RomLoader(read_fn, write_fn, chip)

    _log("[*] Syncing with ROM bootloader...")
    if not loader.sync():
        _log("[-] No response from bootloader. Checks:")
        _log("      - Is this actually a native-USB-CDC board (303A:xxxx)?")
        _log("      - Try unplugging/replugging the OTG cable once.")
        _log("      - Some clones need the BOOT button held manually.")
        cdc_reset.reset_to_app(device)
        sys.exit(1)
    _log("[+] Bootloader is alive and synced.")

    loader.spi_attach()
    _log("[+] SPI flash attached.")
    _log(f"[+] Chip target: {chip}  (declared, not autodetected)")

    cdc_reset.reset_to_app(device)
    _log("[*] Reset back to application.")


def _read_bin(path: str) -> bytes:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"No such file: {path}")
    with open(path, "rb") as f:
        return f.read()


def cmd_write(chip: str, offset: int, path: str, verify: bool, no_reboot: bool):
    data = _read_bin(path)
    total = len(data)
    _log(f"[*] {path}: {total} bytes -> offset 0x{offset:06X}")

    device, ep_in, ep_out = acquire_device()
    _log("[*] Resetting into ROM bootloader...")
    cdc_reset.enter_bootloader(device)

    read_fn, write_fn = make_read_write(device, ep_in, ep_out)
    loader = rom_loader.RomLoader(read_fn, write_fn, chip)

    _log("[*] Syncing with ROM bootloader...")
    if not loader.sync():
        _log("[-] Sync failed - device did not respond as a ROM bootloader.")
        cdc_reset.reset_to_app(device)
        sys.exit(1)
    _log("[+] Synced.")

    loader.spi_attach()
    _log("[+] SPI flash attached.")

    _log("[*] Erasing target region + starting flash_begin...")
    loader.flash_begin(total, offset)

    seq = 0
    sent = 0
    t0 = time.time()
    try:
        while sent < total:
            block = data[sent:sent + CHUNK]
            loader.flash_block(block, seq)
            sent += len(block)
            seq += 1
            _progress(sent, total)
    except rom_loader.RomLoaderError as e:
        _log(f"\n[-] Flash write failed at byte {sent}/{total}: {e}")
        cdc_reset.reset_to_app(device)
        sys.exit(1)

    elapsed = time.time() - t0
    rate = (total / 1024) / elapsed if elapsed > 0 else 0
    _log(f"[+] Wrote {total} bytes in {elapsed:.1f}s ({rate:.1f} KB/s)")

    if verify:
        _log("[*] Verifying via on-device MD5...")
        import hashlib
        local_md5 = hashlib.md5(data).hexdigest()
        try:
            remote_md5 = loader.flash_md5(offset, total)
        except rom_loader.RomLoaderError as e:
            _log(f"[-] Could not read back MD5: {e}")
            cdc_reset.reset_to_app(device)
            sys.exit(1)
        if remote_md5 == local_md5:
            _log(f"[+] Verify OK  (md5 {local_md5})")
        else:
            _log(f"[-] Verify MISMATCH  local={local_md5}  device={remote_md5}")
            cdc_reset.reset_to_app(device)
            sys.exit(1)

    loader.flash_finish(reboot=not no_reboot)
    if no_reboot:
        _log("[*] Staying in bootloader (--no-reboot set).")
    else:
        _log("[*] Rebooting into application...")
        time.sleep(0.3)
        cdc_reset.reset_to_app(device)
    _log("[+] Done.")


def cmd_verify(chip: str, offset: int, path: str):
    import hashlib
    data = _read_bin(path)
    total = len(data)
    local_md5 = hashlib.md5(data).hexdigest()

    device, ep_in, ep_out = acquire_device()
    cdc_reset.enter_bootloader(device)
    read_fn, write_fn = make_read_write(device, ep_in, ep_out)
    loader = rom_loader.RomLoader(read_fn, write_fn, chip)

    _log("[*] Syncing with ROM bootloader...")
    if not loader.sync():
        _log("[-] Sync failed.")
        cdc_reset.reset_to_app(device)
        sys.exit(1)
    loader.spi_attach()

    _log(f"[*] Reading back MD5 of {total} bytes @ 0x{offset:06X}...")
    remote_md5 = loader.flash_md5(offset, total)
    cdc_reset.reset_to_app(device)

    if remote_md5 == local_md5:
        _log(f"[+] MATCH  (md5 {local_md5})")
    else:
        _log(f"[-] MISMATCH  local={local_md5}  device={remote_md5}")
        sys.exit(1)


def cmd_erase_info(chip: str):
    _log(f"[*] {chip}: this build does not implement full chip erase.")
    _log("    Use 'write' with a blank/0xFF-filled image, or flash a")
    _log("    fresh bootloader+app image at the correct offsets instead.")
    _log("    (Kept out on purpose - full erase has no recovery path if")
    _log("     it's interrupted mid-operation over an unreliable OTG link.)")


# ---- argv parsing + termux-usb fd bootstrap --------------------------------

def build_parser():
    p = argparse.ArgumentParser(prog="nrflash",
                                 description="Termux-native .bin flasher for native-USB-CDC ESP32 boards (S3/C3/S2).")
    sub = p.add_subparsers(dest="action", required=True)

    common = dict(required=True, choices=CHIP_CHOICES)

    p_probe = sub.add_parser("probe", help="Check the device answers the ROM bootloader protocol.")
    p_probe.add_argument("--chip", **common)

    p_write = sub.add_parser("write", help="Flash a .bin file at the given offset.")
    p_write.add_argument("--chip", **common)
    p_write.add_argument("--offset", default="0x0", help="Flash offset, e.g. 0x0 or 0x10000")
    p_write.add_argument("--verify", action="store_true", help="MD5-verify after writing.")
    p_write.add_argument("--no-reboot", action="store_true", help="Stay in bootloader after flashing.")
    p_write.add_argument("file", help="Path to the .bin file to write.")

    p_verify = sub.add_parser("verify", help="Compare a local .bin's MD5 against what's on flash.")
    p_verify.add_argument("--chip", **common)
    p_verify.add_argument("--offset", default="0x0")
    p_verify.add_argument("file")

    p_erase = sub.add_parser("erase-info", help="Explain why full chip erase isn't offered.")
    p_erase.add_argument("--chip", **common)

    return p


def _parse_offset(s: str) -> int:
    return int(s, 16) if s.lower().startswith("0x") else int(s)


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Root backend (Nethunter / Termux:Root) needs no fd bootstrap -
    # go straight to the requested command.
    if usb_device.is_root() or not usb_device.has_termux_api():
        _dispatch(args)
        return

    # Termux no-root: if TERMUX_USB_FD is already set, we were re-exec'd
    # by the open_usb_device() call below - just run the command.
    if "TERMUX_USB_FD" in os.environ:
        _dispatch(args)
        return

    # First invocation on no-root Termux: find the device, request the
    # permission dialog, then re-exec ourselves with TERMUX_USB_FD set -
    # identical bootstrap shape to what nrcap32 does for the capture tool.
    _log("[*] Looking for USB device...")
    device_path = usb_device.auto_detect_device()
    _log(f"[*] Found device: {device_path}")
    _log("[*] Requesting USB permission (tap OK on your phone)...")

    self_path = os.path.abspath(__file__)
    callback = f"python3 {self_path} " + " ".join(sys.argv[1:])
    usb_device.open_usb_device(device_path, callback)
    # open_usb_device() execve's into the callback on success and does not
    # return here. If we get past it, something went wrong.
    _log("[-] Unexpected: open_usb_device() returned without exec'ing.")
    sys.exit(1)


def _dispatch(args):
    try:
        if args.action == "probe":
            cmd_probe(args.chip)
        elif args.action == "write":
            cmd_write(args.chip, _parse_offset(args.offset), args.file,
                       args.verify, args.no_reboot)
        elif args.action == "verify":
            cmd_verify(args.chip, _parse_offset(args.offset), args.file)
        elif args.action == "erase-info":
            cmd_erase_info(args.chip)
    except RuntimeError as e:
        _log(f"[-] {e}")
        sys.exit(1)
    except FileNotFoundError as e:
        _log(f"[-] {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        _log("\n[-] Interrupted.")
        sys.exit(130)


if __name__ == "__main__":
    main()
