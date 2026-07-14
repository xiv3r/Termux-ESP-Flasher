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

import espbridge as usb_device

# bmRequestType for CDC class-specific request, host->device
_CDC_REQTYPE = 0x21
_SET_CONTROL_LINE_STATE = 0x22

# DTR/RTS bit positions inside wValue of SET_CONTROL_LINE_STATE
_DTR = 0x01
_RTS = 0x02


def _set_line_state(device, dtr: bool, rts: bool, control_interface: int = 0):
    value = (_DTR if dtr else 0) | (_RTS if rts else 0)
    # wIndex is the CDC control interface number. This is NOT always 0:
    # boards that expose more than plain CDC (e.g. ESP32-S3 with the
    # USB-JTAG interface active) can push JTAG to interface 0 and shift
    # CDC control/data to 1/2 instead. Hardcoding 0 here silently targets
    # the wrong interface on those boards. usb_device.py's
    # find_cdc_control_interface() (called from enter_bootloader()/
    # reset_to_app() below) figures out the right number from the
    # device's actual descriptors, same fix already applied to
    # open_native_cdc_port().
    device.ctrl_transfer(_CDC_REQTYPE, _SET_CONTROL_LINE_STATE, value, control_interface, None)


def _control_interface(device) -> int:
    """
    Resolve the CDC control interface number for `device`.

    get_cdc_endpoints() returns the *data* interface (hardcoded to 1 for
    native ESP32 USB CDC), which is only the control interface's neighbor
    by convention - not guaranteed on boards that also expose USB-JTAG,
    where JTAG can occupy interface 0 and push CDC control/data to 1/2.
    find_cdc_control_interface() reads the actual descriptors instead of
    assuming, same fix already applied to open_native_cdc_port().
    Falls back to 0 (the common case) if endpoint discovery fails for
    any reason - better to try the old hardcoded behavior than crash a
    reset that was working before this change.
    """
    try:
        _, _, data_iface = usb_device.get_cdc_endpoints(device)
        return usb_device.find_cdc_control_interface(device, data_iface)
    except Exception:
        return 0


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
    ctrl_iface = _control_interface(device)
    for _ in range(2):
        _set_line_state(device, dtr=True, rts=False, control_interface=ctrl_iface)
        time.sleep(0.05)
        _set_line_state(device, dtr=False, rts=True, control_interface=ctrl_iface)
        time.sleep(0.05)
        _set_line_state(device, dtr=False, rts=False, control_interface=ctrl_iface)
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
    ctrl_iface = _control_interface(device)
    _set_line_state(device, dtr=True, rts=True, control_interface=ctrl_iface)
    time.sleep(0.1)
    _set_line_state(device, dtr=False, rts=False, control_interface=ctrl_iface)
    time.sleep(settle)
