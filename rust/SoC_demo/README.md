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

You now should have a TCP endpoint at `192.168.1.50:1234`.

Connect your computer to the board's Ethernet port an assign a static IP from the `192.168.1.0/24` subnet.

You can now connect to the SoC using netcat:

```
rlwrap nc -vv 192.168.1.50 1234

```

Now you should have a TCP connection with the SoC and you can talk to it. For example just type "hi" and press enter. Or "led on". If you press the button on the board, you should get a "button pressed" message via TCP. Have a look at main.rs to see which commands the SoC can parse.



NOTE: You can also look at the UART output. Just connect a UART probe to the correct pins (look at the custom pinout at the top of the LiteX SoC description). Example using screen:

```
screen /dev/ttyUSB1 115200
```

To exit screen you can type `Ctrl-a k` or `Ctrl-a Ctrl-k`


[bug]: https://github.com/rust-lang/cargo/issues/7915#issuecomment-683294870
