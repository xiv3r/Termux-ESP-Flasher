# nrflash

A Termux-native `.bin` flasher for ESP32-S3 / C3 / S2 boards that use
**native USB CDC** (no CP2102/CH340/FTDI bridge chip). No root, no
`esptool.py` subprocess, no pyserial.

This is a sibling tool to **[NRcap32](https://github.com/0xhim4ri-81x/NRcap32)** — it reuses `usb_device.py` as-is for
USB backend detection, fd-wrapping, and endpoint discovery, so it inherits
the same no-root Termux support the capture tool already has.

## Why this exists instead of just running esptool.py 

`esptool.py` talks over pyserial, which expects a `/dev/ttyUSB*` or
`/dev/ttyACM*` node. On stock no-root Termux there often isn't one — Android
hands USB access to apps as a raw file descriptor via `termux-usb`, not a
serial device node. Boards with a real UART bridge chip (CP2102/CH340/FTDI)
mostly still get a kernel-created tty node and work with real esptool fine.

Bare native-USB-CDC modules (the `303A:1001` / `303A:0002` boards this repo
targets) are the case that's actually annoying on no-root Termux: there's no
bridge chip, so the entire reset-into-bootloader sequence has to happen over
USB CDC control transfers on a libusb-wrapped fd rather than DTR/RTS lines a
bridge chip exposes physically. `nrflash` implements exactly that path and
nothing else.

**If your board has a CP2102/CH340/FTDI chip and Termux gives you a tty
node, just use real `esptool.py` — it's faster (stub loader, baud
renegotiation) and far more battle-tested.** This tool is for the gap real
esptool's serial transport can't reach on stock Termux.

## Scope / limitations (read before filing an issue)

- **Targets only S3 / C3 / S2.** These are the only chips NRcap32 itself
  targets, and the only ones with native USB CDC.
- **ROM bootloader only — no stub loader.** Real esptool uploads a small
  stub into RAM first, which speeds up flashing a lot and unlocks features
  like flash-compression and reliable baud-rate changes. None of that is
  implemented here on purpose: it would mean embedding and maintaining
  three separate stub binaries. This tool is slower per byte but the
  protocol surface is small enough to read top-to-bottom in one sitting.
- **No full-chip erase command.** `erase-info` explains why: an erase
  that gets interrupted by a flaky OTG connection mid-operation has no
  recovery path. Flash a full image (bootloader + partition table + app)
  at the correct offsets instead of erasing first.
- **No `--baud` flag.** Native USB CDC has no real UART clock underneath
  it, so "baud rate" doesn't mean anything here — it's just bulk transfer
  speed, which is fixed by USB itself.

## Install

Same dependencies as NRcap32 — nothing extra to install:

```bash
pkg update && pkg install python termux-api libusb
pip install pyusb
```

Termux:API app from F-Droid (not Play Store) must be installed, same as
the main project.

## Usage

```bash
chmod +x nrflash

# Sanity-check the bootloader handshake without writing anything
./nrflash probe --chip esp32c3

# Flash a full image at offset 0x0
./nrflash write --chip esp32c3 --offset 0x0 firmware.bin

# Flash + MD5-verify against the device afterward
./nrflash write --chip esp32s3 --offset 0x0 firmware.bin --verify

# Flash a sub-image at a partition offset (e.g. app partition only)
./nrflash write --chip esp32c3 --offset 0x10000 app.bin

# Stand-alone MD5 check against a file already on disk
./nrflash verify --chip esp32c3 --offset 0x0 firmware.bin

# Stay in the bootloader after flashing instead of rebooting
./nrflash write --chip esp32c3 firmware.bin --no-reboot
```

First run on no-root Termux pops the same USB permission dialog as
`nrcap32` — tap **OK**. The permission persists until you unplug.

## What actually flashes — single binary vs. esptool's three images

Real `esptool.py write_flash` for these chips usually takes **three**
files at three offsets (bootloader, partition table, app), e.g.:

```
0x0      bootloader.bin
0x8000   partition-table.bin
0x10000  app.bin
```

`nrflash write` takes one file at one offset per invocation — call it
three times, once per offset, exactly like calling `esptool.py write_flash`
three separate times would work too. If your build only produces a single
merged/combined `.bin` (PlatformIO can do this — check for a
`firmware.factory.bin` or similar after `pio run`), one call at `0x0` is
all you need:

```bash
./nrflash write --chip esp32c3 --offset 0x0 nrcap32-c3.bin
```

## How bootloader entry works on native USB CDC

UART-bridge boards (CP2102 etc.) have real DTR/RTS-wired GPIO lines into
EN/BOOT, which is what `usb_device.py`'s `init_uart_bridge()` already
drives for those chips. Native USB CDC boards have no such bridge chip —
instead, the chip's internal USB-Serial-JTAG peripheral watches the CDC
class's `SET_CONTROL_LINE_STATE` DTR/RTS bits in *hardware* and maps
specific transitions to an internal EN/BOOT reset, the software-only
equivalent of the same trick. That sequence lives in `cdc_reset.py`,
separate from `usb_device.py`'s bridge-chip logic since the two have
nothing in common at the wire level.

## Files

```
nrflash          # exec shim, chmod +x and run this
nrflash.py       # CLI: argv parsing, fd bootstrap, flash/verify/probe commands
rom_loader.py    # SLIP framing + ROM bootloader command/response protocol
cdc_reset.py     # native-USB-CDC bootloader entry/exit (DTR/RTS bit tricks)
usb_device.py    # shared with **[NRcap32](https://github.com/0xhim4ri-81x/NRcap32)** — USB backend detection, fd wrapping
```

`nrflash.py` and `usb_device.py` must live in the same directory — there's
no packaging/install step, it's a flat script like `nrcap32` itself.

## Troubleshooting

**`probe` reports no response from bootloader**
Some clone boards need the physical BOOT button held while plugging in —
the soft DTR/RTS reset trick depends on the USB-Serial-JTAG peripheral
being wired the way Espressif's reference design wires it, and very cheap
clones occasionally don't.

**Works for `probe` but `write` stalls partway through**
Likely the same USB DATA toggle issue documented in the main **[NRcap32](https://github.com/0xhim4ri-81x/NRcap32)**
README — try unplugging and replugging the OTG cable once before retrying.

**`Verify MISMATCH` after a successful-looking write**
Don't trust a partial flash. Re-run `write --verify` rather than assuming
the device is fine — a flaky OTG link can drop bytes mid-transfer in a way
that still returns "success" status on a given block.

## Legal

Use on hardware you own. Flashing arbitrary firmware
to a device you don't own or have written permission to modify can violate
warranty terms or, depending on the device and jurisdiction, the law.
