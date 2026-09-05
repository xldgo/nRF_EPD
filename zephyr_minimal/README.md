# nRF EPD minimal draw (Zephyr)

First Zephyr migration slice for the nRF52811 7.5-inch 800x480 B/W/Red UC8179 tag.

## SDK baseline

- nRF Connect SDK **v3.4.0** (Zephyr 4.4)
- Build target: `nrf52840dk/nrf52811`
- The application overlay removes the DK code partition and remaps the real EPD board pins.

NCS v3.4.0 is the LTS release and the final NCS release line that supports nRF52 devices, so this branch pins that release rather than development/main.

## Scope of this first slice

Kept:

- BLE Peripheral / GATT Server
- Existing EPD UUIDs (`...0001`, `...0002`, `...0003`)
- APP version `0x21`
- Existing 14-byte hardware configuration notification
- Central-initiated ATT MTU 247 (244-byte GATT payload)
- CRC block transfer (`0x31`), status (`0x32`), reset (`0x33`)
- Block ACK/NACK (`0xA0`) and status bitmap (`0xA1`)
- UC8179 full-refresh image path for the B/W and Red planes
- `0x01` init, `0x05` refresh, `0x06` panel sleep

Intentionally not in this slice:

- Calendar / lunar / clock UI
- Local font/text renderer (next migration slice)
- FDS/settings persistence
- ADC/battery UI
- Partial refresh LUTs
- MCUboot/OTA

The goal is to prove that the existing browser and ESP transfer clients can send the same two 48,000-byte planes to Zephyr without changing the wire protocol.

## Hardware mapping

The overlay preserves the working 14-byte configuration:

`14 13 06 05 04 03 02 07 ff 12 07 01 01 00`

Relevant drawing pins:

- MOSI: P0.20
- SCLK: P0.19
- CS: P0.06
- DC: P0.05
- RESET: P0.04
- BUSY: P0.03 (active-low behavior preserved from the existing driver)
- BS: P0.02, driven low
- EPD enable: P0.07, driven high

P0.22 is deliberately not configured as SPI MISO; it remains available for the board's D/L test pad.

## Build

From an nRF Connect SDK v3.4.0 west workspace:

```sh
west build -p always -b nrf52840dk/nrf52811 /path/to/nRF_EPD/zephyr_minimal
```

Useful size reports:

```sh
west build -d build -t rom_report
west build -d build -t ram_report
```

Target application budget is at most about 55 KiB. The branch keeps logging, console, SMP, settings, GATT client support and the calendar stack disabled.

## First hardware flash

This application uses the Zephyr controller/host and does **not** use the old S112 SoftDevice. The overlay therefore links the standalone image at flash offset 0.

First bring-up must be done through SWD and replaces the existing SoftDevice/application layout. Do not use the old Nordic Secure DFU application ZIP for this image.

OTA/MCUboot partitioning will be designed only after the minimal drawing path and image size are verified on hardware.
