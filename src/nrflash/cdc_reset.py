"""
cdc_reset.py - Put a native-USB-CDC ESP32 (S2 / S3 / C3) into the ROM
download bootloader, and reset it back to running firmware afterward.

usb_device.py's init_uart_bridge() already solves this for *external*
UART bridge chips (CP2102/CH340/FTDI) by toggling DTR/RTS, which are real
GPIO-wired reset/boot lines on those boards. Native USB CDC boards
(303A:1001 / 303A:0002) have no such bridge chip and no real DTR/RTS
hardware — Espressif's USB-CDC-ACM stack on these chips instead exposes a
*software* convention over CDC "SET_LINE_CODING" / control transfers:

  - Standard CDC has no boot-strap pins, so Espressif's `esptool.py`
    triggers entry into the ROM bootloader two different ways depending
    on board age/firmware:

      1. "USB-JTAG/serial" reset hack: send a CDC `SET_CONTROL_LINE_STATE`
         class request with specific DTR/RTS *bit patterns* — the chip's
         internal USB-Serial-JTAG peripheral watches these bits in
         hardware and maps them to EN/BOOT internally, the same logic
         CP2102/CH340 boards do externally with real wires.

      2. If that doesn't trigger it (older bootloader ROMs, or the
         sketch never enabled the watch), we fall back to asking the
         *currently running sketch* to reboot into bootloader via the
         well-known "0xAD 0xDE" magic isn't relevant here -- that's
         NRcap32's own app protocol, not a chip-level reset path, so we
         do NOT depend on the firmware cooperating. Native USB CDC entry
         must work even when the flash is blank/corrupt.

This module only implements path (1), which is what every ESP32-S3/C3/S2
dev board with native USB (including bare modules wired per Espressif's
reference) supports at the silicon level — it's part of the USB-Serial-JTAG
peripheral, not the user sketch.
"""

import time

# bmRequestType for CDC class-specific request, host->device
_CDC_REQTYPE = 0x21
_SET_CONTROL_LINE_STATE = 0x22

# DTR/RTS bit positions inside wValue of SET_CONTROL_LINE_STATE
_DTR = 0x01
_RTS = 0x02


def _set_line_state(device, dtr: bool, rts: bool):
    value = (_DTR if dtr else 0) | (_RTS if rts else 0)
    # wIndex is the CDC control interface number; native ESP32 USB CDC
    # always exposes it as interface 0 (data class lives on interface 1,
    # matched by get_cdc_endpoints()'s native-CDC branch).
    device.ctrl_transfer(_CDC_REQTYPE, _SET_CONTROL_LINE_STATE, value, 0, None)


def enter_bootloader(device, settle: float = 0.3):
    """
    Drive the native USB-CDC reset-into-bootloader sequence.

    Sequence (matches the espressif esptool 'usb_jtag_serial' reset
    strategy, re-derived from the public USB-Serial-JTAG peripheral
    behavior, no esptool code reused):

      1. DTR=1, RTS=0   -> internal EN  released  (chip held in reset briefly)
      2. DTR=0, RTS=1   -> internal BOOT pulled low, EN toggled    -> reset
                           into bootloader because BOOT is sampled low
      3. DTR=0, RTS=0   -> release lines, let ROM bootloader start cleanly

    Some boards need this whole sequence sent twice back-to-back because
    the very first control transfer after USB enumeration can be eaten by
    the host controller settling — harmless to repeat, the chip is already
    held in reset for the duration.
    """
    for _ in range(2):
        _set_line_state(device, dtr=True, rts=False)
        time.sleep(0.05)
        _set_line_state(device, dtr=False, rts=True)
        time.sleep(0.05)
        _set_line_state(device, dtr=False, rts=False)
        time.sleep(0.05)
    time.sleep(settle)


def reset_to_app(device, settle: float = 0.5):
    """
    Reset the chip back out of the bootloader into the normal app
    (whatever was just flashed, or the existing firmware if the user
    only wanted to probe/verify).

      1. DTR=1, RTS=1 -> EN pulled low -> reset asserted
      2. DTR=0, RTS=0 -> EN released, BOOT high (not sampled)
                         -> boots the flashed application, not the ROM loader
    """
    _set_line_state(device, dtr=True, rts=True)
    time.sleep(0.1)
    _set_line_state(device, dtr=False, rts=False)
    time.sleep(settle)
