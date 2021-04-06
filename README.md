# `Colorlight 5A-75X Litex Ethernet Demo`

> Litex SoC and Rust software for [Colorlight 5A-75X][colorlight] board as a simple Ethernet demo.

This project is developed and maintained by [DerFetzer][team].

## Usage

1. Install Litex as described [here][litex].
2. Install [ecpprog][ecpprog].
4. Connect a USB to JTAG adaptor to your board.
5. Build and flash the SOC with the following (in the soc directory):
```
python colorlight_5a_75x.py --build --flash --board=5a-75e --revision=6.0 --with-ethernet --sys-clk-freq 50e6 --use-internal-osc
```
4. Follow the instructions in [eth_demo][eth_demo] to build the software.

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
