# `Rust SoC Playground using LiteX, VexRiscV and the Colorlight 5A-75B board`

Trying lots of different things (in different branches).

Working so far:
- Ethernet with smoltcp thanks to https://github.com/DerFetzer/colorlight-litex 
- Interrupts by staring a lot at https://github.com/betrusted-io/xous-core
- A simple second order sigma-delta ADC using a CIC decimator
- An IIR filter using on-chip sigma-delta ADC/DAC

## Credits

This project is based on the [icebreaker-litex-examples][litex-example] repo using a great deal of its files
and the Litex [target file][target] for the Colorlight board.

[team]: https://github.com/DerFetzer
[litex]: https://github.com/enjoy-digital/litex#quick-start-guide
[ecpprog]: https://github.com/gregdavill/ecpprog
[litex-example]: https://github.com/icebreaker-fpga/icebreaker-litex-examples
[colorlight]: http://www.colorlight-led.com/product/colorlight-5a-75e-led-display-receiving-card.html
[target]: https://github.com/litex-hub/litex-boards/blob/master/litex_boards/targets/colorlight_5a_75x.py
[eth_demo]: rust/eth_demo/README.md
