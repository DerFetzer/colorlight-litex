Welcome to the Rust example code for LiteX RISC-V SOC on Colorlight 5A-75X.

## Prerequisites

Make sure you have the soc installed as described in the `soc` directory of this repository.

You will also need to install the Rust compiler. Install rustup by following the instructions at https://rustup.rs.
Because of a cargo [bug][bug] you will have to use the nightly toolchain.

NOTE Make sure you have a compiler version equal to or newer than 1.48. rustc -V should return a date newer than the one shown below.

```
$ rustc -V
rustc 1.48.0-nightly (e2be5f568 2020-09-09)
```

For bandwidth and disk usage concerns the default installation only supports native compilation. To add cross compilation support for the RISC-V architecture install the `riscv32imac-unknown-none-elf` target.

```
rustup target add riscv32imac-unknown-none-elf
```

To learn more about Rust embedded, take a look at the Rust [embedded book](https://rust-embedded.github.io/book/).

## Build and Flash

Make sure you have an FTDI JTAG Adapter connected to your board. Build and flash the firmware for your soc by running:
```
cargo build --release
cargo run --release
```

It might be necessary to reset the board with a power cycle or by loading the SOC bitstream again. 

You now should have a TCP endpoint at `192.168.1.50:1234` and a UDP endpoint at `192.168.1.50:5678` listening for connections.

Connect your computer to the board's Ethernet port an assign a static IP from the `192.168.1.0/24` subnet.

You can now connect to the sockets and get a `Hello World!` back.

```
echo "" | netcat 192.168.1.50 1234
echo "" | netcat -u 192.168.1.50 5678
```

Connect a USB to UART adapter as described in [litex-boards][uart].

You should be able to access the console output by running the wishbone-tool.

```
wishbone-tool --uart /dev/ttyUSB1 -s terminal
```

You can find the wishbone-tool here: https://github.com/litex-hub/wishbone-utils

NOTE: If you decided to build your SOC without the `--debug` parameter you can access the console output directly. For example using screen:

```
screen /dev/ttyUSB1 115200
```

To exit screen you can type `Ctrl-a k` or `Ctrl-a Ctrl-k`

# License

This crate is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.


[bug]: https://github.com/rust-lang/cargo/issues/7915#issuecomment-683294870
[uart]: https://github.com/litex-hub/litex-boards/blob/e4cdbe0f7ad0653e825556d992d233a1723273e3/litex_boards/targets/colorlight_5a_75x.py#L11
