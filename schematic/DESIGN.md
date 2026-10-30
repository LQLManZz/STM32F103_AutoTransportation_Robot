# srf05_scanner – schematic (connections view)

`srf05_scanner.kicad_sch` (KiCad 10) – simplified: only the MCU and the connectors it talks to.
The detailed version (LDO, crystal, decoupling, reset, divider, LED) is in `full_version/`.
Edit only through Konnect MCP tools, never by hand.

| MCU pin | Net | Connector |
|---|---|---|
| PB15 (28) | TRIG (label) | J1 SRF05 pin 2 |
| PA8 (29) | ECHO (wire) | J1 SRF05 pin 3 |
| PA2 / PA3 | USART2 TX / RX | J3 pins 1 / 2 |
| PA9 / PA10 | USART1 TX / RX | J2 pins 1 / 2 |
| PA13 / PA14 | SWDIO / SWCLK | J4 pins 1 / 2 |
| NRST (7) | NRST (label) | J4 pin 5 |
| BOOT0 (44) | GND | – |
| VDD/VDDA/VBAT | +3V3 | J4 pin 4 |
| VSS/VSSA | GND | J1 pin 4, J2/J3/J4 GND pins |

J1 pin 1 = +5V. Not in this view: PD0/PD1 (HSE crystal) are marked no-connect, so the
firmware's HSE clock needs the crystal from `full_version/`. SRF05 Echo is 5 V; add the
1k/2k divider (see `full_version/`) before PA8 on real hardware.
