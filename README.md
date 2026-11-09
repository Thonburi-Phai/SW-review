# BMS TAAP LFP 4S -- NXP firmware

Battery management firmware for a 4S LiFePO4 pack: NXP S32K116 host MCU driving
a TI BQ76907 analog front end.

The MCU owns configuration, cell balancing, state of charge, the open-wire
response and the host protocol. The BQ76907 keeps its own protections
autonomous, so cell over/under-voltage, overcurrent, short-circuit and
temperature act on the FETs whether or not the MCU is running.

## Build

```bash
make            # production image -> build/nxp_bms.elf
make test       # host checks: open-wire classifier, NTC band ladders
make clean
```

Requires the ARM toolchain at `~/gcc-arm-gnu-tool-stm32/bin/arm-none-eabi-gcc`.
`make test` builds for the host instead and uses `cc`.

## Flash

```bash
make flash_openocd   # local, via MCU-Link (CMSIS-DAP)
make flash_017       # remote, via the MQTT proxy (see the notes in the Makefile)
```

Both need OpenOCD built with S32K support, at `~/openocd-code/src/openocd`.
`make unsecure` mass-erases a chip that has ended up in secured state.

## Layout

```
src/hal/          MCU peripherals: I2C, UART, ADC, GPIO, timebase
src/bq7690x/      BQ76907 register driver
src/controller/   application: configuration, balancing, SOC/SOH, protection
                  policy, open-wire detection and response, parameters
src/protocol/     binary packet definitions for the host link
src/tools/        hw_probe, a standalone pin and peripheral probe image
host/             Python host tools for the binary protocol
tests/            host-side unit tests, run by `make test`
tools/            offline analysis: SOC simulator, capture extraction
```

## Power

The application runs in VLPR (SIRC 8 MHz, core 4 MHz, FIRC off) and idles in
VLPS between periodic wakes. Startup holds for 5 seconds before entering VLPR,
which keeps the core catchable over SWD for a reflash.
