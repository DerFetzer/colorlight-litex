# `Rust SoC Playground using LiteX, VexRiscV and the Colorlight 5A-75B board`

This repository contains a lot of fun with rust and RiscV.

The main branch is a small demo of a TCP connection with different options like turning LEDs on/off or registering an external button interrupt.
It is somewhat ceaned up and meant for people who want to get started. 

Most of the instructions are adapted from https://github.com/DerFetzer/colorlight-litex and https://github.com/icebreaker-fpga/icebreaker-litex-examples:

1. Install Litex as described [here][litex].
2. Install [ecpprog][ecpprog].
4. Connect a USB to JTAG adaptor to your board.
5. Build and flash the SOC with something like the following:
```
python3 colorlight_5a_75x.py  --flash --board=5a-75b --revision=7.0 --with-ethernet --sys-clk-freq 60e6

```
4. Follow the instructions in [eth_demo][eth_demo] to build the software.


There are a few things added to the standard Colorlight 5A-75B. If you just want to try the SoC with one onboard LED, the onboard button and just the TX of the UART you do not need to modify anything. 


Working in some branch:
- Ethernet with smoltcp thanks to https://github.com/DerFetzer/colorlight-litex 
- Interrupts by staring a lot at https://github.com/betrusted-io/xous-core
- A simple second order sigma-delta ADC using a CIC decimator
- An IIR filter using on-chip sigma-delta ADC/DAC
- A peltier temperature controller using a custom on-SoC ratiometric ADC, an IIR filter triggered by timer interrupts, TCP telemetry, UDP temperature set and the onboard line drivers in parallel as peltier drivers ;)

## Credits

This project is based on the [icebreaker-litex-examples][litex-example] repo using a great deal of its files
and the Litex [target file][target] for the Colorlight board.

[team]: https://github.com/DerFetzer
[litex]: https://github.com/enjoy-digital/litex#quick-start-guide
[ecpprog]: https://github.com/gregdavill/ecpprog
[litex-example]: https://github.com/icebreaker-fpga/icebreaker-litex-examples
[colorlight]: http://www.colorlight-led.com/product/colorlight-5a-75e-led-display-receiving-card.html
[target]: https://github.com/litex-hub/litex-boards/blob/master/litex_boards/targets/colorlight_5a_75x.py
[SoC_demo]: rust/SoC_demo/README.md
