"""
uart_reset.py - Put an ESP32/ESP8266 wired through an external UART bridge
(CH340/CH340G, CP2102, CH9102, FTDI FT232) into the ROM download bootloader,
and reset it back to running firmware afterward.

Same job as cdc_reset.py, different hardware: native USB-CDC boards (S3/C3/
S2) have no real DTR/RTS lines and need a CDC-class control transfer trick
instead (see cdc_reset.py's module docstring). Boards behind a UART bridge
chip - classic ESP32 and ESP8266 devkits (NodeMCU, Wemos D1 mini, DOIT
ESP32 DevKit, etc.) - have *real* DTR/RTS handshake lines wired to GPIO0
and EN through inverting transistors, same as esptool.py drives over
pyserial on a real /dev/ttyUSB*. This module reimplements that same
DTR/RTS dance directly over the bridge chip's vendor USB control requests
(via usb_device.set_dtr_rts), since Termux no-root never gets a ttyUSB
device node - the whole reason this project talks to bridge chips over raw
USB in the first place.

Wiring assumed (the standard "auto-reset" circuit on every ESP32/ESP8266
devkit that has one - NodeMCU, Wemos, DOIT, etc.):
    RTS --[inverting transistor]--> EN     (RTS asserted -> EN pulled low -> reset)
    DTR --[inverting transistor]--> GPIO0  (DTR asserted -> GPIO0 pulled low -> boot mode)
"""

import time

import usb_device


def enter_bootloader(device, settle: float = 0.3, attempts: int = 3):
    """
    Classic esptool "classic_reset" sequence: pulse EN low with GPIO0 held
    low, so the chip's boot ROM samples GPIO0=0 on reset and drops into the
    UART download bootloader instead of booting the flashed app.

      1. RTS asserted, DTR released -> GPIO0 high, EN pulled low (reset)
      2. RTS released, DTR asserted -> GPIO0 pulled low (boot strap),
                                        EN released -> chip resets and
                                        starts sampling GPIO0
      3. DTR released                -> GPIO0 line freed; ROM bootloader
                                        is already running and only reads
                                        the strap pin at reset time, so
                                        this is safe to release now.

    Repeated `attempts` times back-to-back. Android USB-OTG adds much more
    latency between control transfers than a real serial port does, so the
    very first pulse after opening the device is often eaten by the host
    controller settling before the chip ever sees the EN edge - the same
    problem cdc_reset.py already had to work around for native-USB boards.
    Harmless to repeat: each pass just re-asserts a chip that's already in
    reset or already in the bootloader.
    """
    for _ in range(attempts):
        usb_device.set_dtr_rts(device, dtr=False, rts=True)
        time.sleep(0.1)
        usb_device.set_dtr_rts(device, dtr=True, rts=False)
        time.sleep(0.1)
        usb_device.set_dtr_rts(device, dtr=False, rts=False)
        time.sleep(0.05)
    time.sleep(settle)


def reset_to_app(device, settle: float = 0.5):
    """
    Reset the chip back out of the bootloader into the normal app, i.e.
    the same pulse as enter_bootloader() but with GPIO0 released (high)
    for the whole reset, so the ROM boots the flashed application instead
    of the download bootloader.

      1. RTS asserted, DTR released -> EN pulled low (reset), GPIO0 high
      2. RTS released, DTR released -> EN released, GPIO0 still high
                                        -> boots the flashed application
    """
    usb_device.set_dtr_rts(device, dtr=False, rts=True)
    time.sleep(0.1)
    usb_device.set_dtr_rts(device, dtr=False, rts=False)
    time.sleep(settle)
