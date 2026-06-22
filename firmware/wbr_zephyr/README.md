# WBR Zephyr application

This directory contains the Zephyr application shell and hardware contracts.
Concrete board, ICM42688P and motor-CAN code belongs in `port/` and is supplied
by the hardware integration project. The portable controller must remain
independent of those implementations.

See `docs/ZEPHYR_PORTING.md` for timing, units and bring-up requirements.

The checked-in `hardware_stub.cc` is deliberately fail-safe: initialization
returns false and no motor command is accepted. Replace that file with the
board port when integrating hardware.

Build from a configured Zephyr workspace with:

```sh
west build -b <board> firmware/wbr_zephyr
```

The application compiles the complete portable Ground Balance cycle, observer
and 1 kHz control thread. The checked-in hardware stub fails initialization, so
motors remain disabled until a real port replaces it.
