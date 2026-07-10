# Termux-ESP-Flasher

A Termux-native `.bin` flasher for ESP32 boards — no root, no
`esptool.py` subprocess, no pyserial.

Covers **two** device families, auto-detected from the USB VID:PID:

- **Native USB CDC** — ESP32-S3 / C3 / S2 (`303A:1001` / `303A:0002`).
  Talks directly to the chip's USB-Serial-JTAG peripheral; bootloader
  entry/exit is driven by `cdc_reset.py`.
- **UART bridge** — classic ESP32 and ESP8266 devkits behind a
  CP2102, CH340/CH340G, CH9102, or FTDI FT232 bridge chip. The bridge
  is opened over raw USB (same fd-wrapping as the native path — Termux
  never gives you a `/dev/ttyUSB*` node without root, so this isn't
  optional) and its DTR/RTS lines are pulsed in the classic GPIO0+EN
  pattern by `uart_reset.py`.

Both paths upload the real ESP-IDF RAM stub for faster block writes, and
both fall back automatically to the plain ROM bootloader if the stub
fails to load for any reason.

## Why this exists instead of just running esptool.py

`esptool.py` talks over pyserial, which expects a `/dev/ttyUSB*` or
`/dev/ttyACM*` node. On stock no-root Termux there often isn't one —
Android hands USB access to apps as a raw file descriptor via
`termux-usb`, not a serial device node. This tool talks straight to the
USB endpoints (and, for UART-bridge boards, straight to the bridge
chip's own vendor registers) instead of assuming a tty exists.

**If you're on a desktop/laptop Linux box with a real `/dev/ttyUSB*`
node, just use real `esptool.py` — it's more battle-tested.** This tool
exists for the no-root-Termux gap esptool's pyserial transport can't
reach.

## What's supported

- **Chip auto-detection.** `--chip` is optional on `probe`, `write`, and
  `verify` — omit it and the tool syncs with the ROM bootloader, reads
  the chip's magic-value register, and picks the right chip for you.
  Pass `--chip` explicitly to override (still required for
  `erase-info`, which never touches hardware).
- **Stub loader.** Both native-USB and UART-bridge sessions upload the
  real ESP-IDF RAM stub after syncing, switching from 1 KiB ROM-only
  blocks to 16 KiB stub blocks. Falls back to ROM-only if the stub
  upload doesn't succeed for some reason — flashing still works, just
  slower.
- **Automatic baud renegotiation on UART-bridge boards.** Once the stub
  is running, the tool steps up through 921600 → 460800 → 230400 baud,
  verifying each one actually holds (a cheap `sync()`) before trusting
  it. If a candidate rate doesn't hold, it physically re-enters the ROM
  bootloader and rebuilds the whole session at 115200 before trying the
  next slower candidate — a bad guess costs a couple seconds, not the
  whole flash. Settles on whatever your specific board/cable can
  actually sustain, or stays at 115200 if nothing higher works. Native
  USB CDC boards skip this entirely — there's no real UART clock
  underneath USB CDC, so baud rate doesn't mean anything there.
- **Live progress.** The flash progress bar streams in real time on the
  no-root Termux fd-bootstrap path — earlier builds buffered progress
  updates invisibly until the transfer finished, then dumped the whole
  bar at once, which looked like a stall.
- **Multi-file writes in one session.** `write` accepts one or more
  `OFFSET:FILE` pairs and flashes all of them under a single USB
  permission prompt / bootloader handshake.

## Scope / limitations (read before filing an issue)

- **No full-chip erase command.** `erase-info` explains why: an erase
  that gets interrupted by a flaky OTG connection mid-operation has no
  recovery path. Flash a full image (bootloader + partition table +
  app) at the correct offsets instead of erasing first. `write --erase`
  will erase exactly the bytes about to be overwritten via
  `ERASE_REGION` if the ROM supports it, falling back silently to
  `flash_begin`'s own per-block erase otherwise.
- **Auto-detection is a best-effort heuristic**, not a guarantee for
  every silicon revision — it reads the chip magic value at a register
  address that's held true across ESP32/S2/S3/C3 for years, but an
  unrecognized value means "pass `--chip` explicitly," not a guess.
- **Stub data is vendored for more chips than are currently wired up**
  (`esp32c2`, `c5`, `c6`, `c61`, `h2`, `h4`, `p4`, `s31` all have stub
  binaries already sitting in `stub_flasher_data.py`), but
  `CHIP_PARAMS`/`KNOWN_MAGIC`/`CHIP_CHOICES` only cover `esp32`,
  `esp8266`, `esp32s2`, `esp32s3`, `esp32c3` today. Wiring up one of the
  others means adding its real chip-magic value and SPI-attach params —
  open an issue (or ask) with the specific chip if you need one.

## Install

```bash
pkg update && pkg install python termux-api libusb
pip install pyusb
```

Termux:API app from F-Droid (not Play Store) must be installed.

## Usage

```bash
chmod +x nrflash

# --chip is optional everywhere except erase-info - omit it to auto-detect
./nrflash probe
./nrflash write --offset 0x0 firmware.bin

# Still works if you want to force a specific chip
./nrflash write --chip esp32c3 --offset 0x0 firmware.bin

# Flash + MD5-verify against the device afterward
./nrflash write --chip esp32s3 --offset 0x0 firmware.bin --verify

# Flash a sub-image at a partition offset (e.g. app partition only)
./nrflash write --offset 0x10000 app.bin

# Flash bootloader + partition table + app in one session
./nrflash write 0x0:bootloader.bin 0x8000:partitions.bin 0x10000:app.bin

# Stand-alone MD5 check against a file already on disk
./nrflash verify --chip esp32c3 --offset 0x0 firmware.bin

# Explicitly erase the write range first via ERASE_REGION
./nrflash write --offset 0x0 firmware.bin --erase

# Stay in the bootloader after flashing instead of rebooting
./nrflash write --offset 0x0 firmware.bin --no-reboot
```

First run on no-root Termux pops the same USB permission dialog every
time — tap **OK**. The permission persists until you unplug.

## What actually flashes — single binary vs. esptool's three images

Real `esptool.py write_flash` usually takes **three** files at three
offsets (bootloader, partition table, app), e.g.:

```
0x0      bootloader.bin
0x8000   partition-table.bin
0x10000  app.bin
```

`nrflash write` can take all three in one call (see the multi-file
example above), or one file per invocation if you prefer. If your build
only produces a single merged/combined `.bin` (PlatformIO can do this —
check for a `firmware.factory.bin` or similar after `pio run`), one call
at `0x0` is all you need.

## How bootloader entry works

- **UART-bridge boards** (CP2102/CH340/CH9102/FTDI) have real DTR/RTS
  lines wired into EN/BOOT. `usb_device.py`'s `init_uart_bridge()`
  configures the bridge's line coding and baud divisor via its own
  vendor control transfers, then `uart_reset.py` pulses DTR/RTS in the
  classic auto-reset pattern.
- **Native USB CDC boards** have no bridge chip — the chip's internal
  USB-Serial-JTAG peripheral watches the CDC class's
  `SET_CONTROL_LINE_STATE` DTR/RTS bits in hardware and maps specific
  transitions to an internal EN/BOOT reset, the software-only
  equivalent of the same trick. That sequence lives in `cdc_reset.py`,
  separate from `usb_device.py`'s bridge-chip logic since the two have
  nothing in common at the wire level.

## Files

```
nrflash               # exec shim, chmod +x and run this
nrflash.py            # CLI: argv parsing, fd bootstrap, flash/verify/probe commands
rom_loader.py         # SLIP framing + ROM bootloader command/response protocol,
                       # chip auto-detection, baud-change command
usb_device.py         # USB backend detection, fd wrapping, UART-bridge register
                       # init/baud reprogramming, endpoint discovery
cdc_reset.py          # native-USB-CDC bootloader entry/exit (DTR/RTS bit tricks)
uart_reset.py         # UART-bridge bootloader entry/exit (DTR/RTS pulse pattern)
stub_flasher_data.py  # vendored ESP-IDF RAM stub binaries (Apache-2.0/MIT, Espressif)
```

All files must live in the same directory — there's no packaging/install
step, it's a flat script.

## Troubleshooting

**`probe` reports no response from bootloader**
Some clone boards need the physical BOOT button held while plugging in,
or the CH340/CP2102/FTDI wired to EN+GPIO0 for auto-reset to work at
all. Try unplugging/replugging the OTG cable once before assuming it's a
protocol problem.

**Flashing is slow / progress bar looks stuck**
On UART-bridge boards, check the log for a `Baud rate raised to ... and
verified` line — if it's missing or stuck at 115200, your specific
board/cable couldn't hold anything higher and the tool already fell back
automatically. If the progress bar itself looks frozen and then jumps to
100% all at once, make sure you're on a build with the live-progress fix
(see "What's supported" above) rather than an older buffered-tail-thread
build.

**`Verify MISMATCH` after a successful-looking write**
Don't trust a partial flash. Re-run `write --verify` rather than
assuming the device is fine — a flaky OTG link or an unstable baud rate
can drop bytes mid-transfer in a way that still returns "success" status
on a given block.

## Legal

For use on hardware you own. Flashing arbitrary firmware to a device you
don't own or have written permission to modify can violate warranty
terms or, depending on the device and jurisdiction, the law.