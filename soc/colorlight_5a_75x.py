#!/usr/bin/env python3

# This file is Copyright (c) 2019 Sean Cross <sean@xobs.io>
# This file is Copyright (c) 2018 David Shah <dave@ds0.me>
# This file is Copyright (c) 2020 Piotr Esden-Tempski <piotr@esden.net>
# This file is Copyright (c) 2020 DerFetzer <kontakt@der-fetzer.de>
# License: BSD

# This target was originally based on the Fomu target.

import argparse
import subprocess
import textwrap

from litedram.modules import M12L64322A, M12L16161A
from litedram.phy import HalfRateGENSDRPHY, GENSDRPHY
from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from litex.build.generic_programmer import GenericProgrammer
from litex.build.io import DDROutput
from litex.soc.cores.clock import ECP5PLL
from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.cores.spi_flash import SpiFlash
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder, builder_argdict, builder_args
from litex.soc.integration.soc_core import soc_core_argdict, soc_core_args
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField
from litex.soc.integration.doc import AutoDoc, ModuleDoc

from litex.build.generic_platform import *

from litex_boards.platforms import colorlight_5a_75b, colorlight_5a_75e

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.uart import UARTWishboneBridge

import litex.soc.doc as lxsocdoc

from adc import ADC

from litex.soc.cores.gpio import GPIOIn


class ECP5Programmer(GenericProgrammer):
    needs_bitreverse = False

    def flash(self, address, bitstream_file):
        subprocess.call(["ecpprog", "-o", str(address), bitstream_file])

    def load_bitstream(self, bitstream_file):
        subprocess.call(["ecpprog", "-S", bitstream_file])


# My IOs -------------------------------------------------------------------------------------------

        Subsignal("tx", Pins("P11")), # led (J19 DATA_LED-)
        Subsignal("rx", Pins("M13")), # btn (J19 KEY+)
        IOStandard("LVCMOS33")

_myserial = [
    ("myserial", 0,
         Subsignal("tx", Pins("j1:14")),
         Subsignal("rx", Pins("j1:13")),
         IOStandard("LVCMOS33")
     )
]

_dac = [
    ("dac", 0, Pins("j4:0"), IOStandard("LVCMOS33")),
    ("dac", 1, Pins("j4:1"), IOStandard("LVCMOS33")),       # sigma-delta dac output
]

_leds = [
    ("g", 0, Pins("j6:5"), IOStandard("LVCMOS33")),
    ("r", 0, Pins("j6:10"), IOStandard("LVCMOS33")),        # Not really here but there is congestion with the pins otherwise..
    ("y", 0, Pins("j6:9"), IOStandard("LVCMOS33")),
]

_adc_first_order = [
    ("in", 0, Pins("j1:1"), IOStandard("LVDS")),
    ("sd", 0, Pins("j1:2"), IOStandard("LVCMOS33")),        # sigma delta out
    ("p3v", 0, Pins("j1:0"), IOStandard("LVCMOS33")),      # this will make 3V on the connector
    ("p5v", 0, Pins("j1:14"), IOStandard("LVCMOS33")),      # this will make 5V on the connector bc buffer IC
]

_adc_second_order = [
    ("in", 0, Pins("j1:1"), IOStandard("LVDS")),
    ("sd", 0, Pins("j1:5"), IOStandard("LVCMOS33")),        # sigma delta out
    ("sd", 1, Pins("j1:7"), IOStandard("LVCMOS33")),        # sigma delta out (copy of first)
    ("p3v", 0, Pins("j1:0"), IOStandard("LVCMOS33")),      # this will make 3V on the connector
    ("p5v", 0, Pins("j1:14"), IOStandard("LVCMOS33")),      # this will make 5V on the connector bc buffer IC
]


# LEDs ---------------------------------------------------------------------------------------------

class Leds(Module, AutoCSR, AutoDoc):
    """LED control.
    3 LEDs connected to random IOs
    Attributes:
        led_pin: Signals of the LED pin outputs.
        led_polarity: Bit pattern to adjust polarity. 0 stays the same 1 inverts the signal.
        led_name: Array of the LED names and descriptions. [["name1", "description1"], ["name2", "description2"]]
    """
    def __init__(self, led_pin, led_polarity=0x00, led_name=[]):
        # Documentation
        self.intro = ModuleDoc("""LED control.
        The LEDs are normal LEDs. Good information. :) 
        """)

        # HDL Implementationj
        self._out = CSRStorage(len(led_pin), fields=[
            CSRField(fld[0], description=fld[1]) for fld in led_name
        ])
        self.comb += led_pin.eq(self._out.storage ^ led_polarity)


# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, use_internal_osc=False, with_usb_pll=False, with_rst=True, sdram_rate="1:1"):
        self.clock_domains.cd_sys    = ClockDomain()
        if sdram_rate == "1:2":
            self.clock_domains.cd_sys2x    = ClockDomain()
            self.clock_domains.cd_sys2x_ps = ClockDomain(reset_less=True)
        else:
            self.clock_domains.cd_sys_ps = ClockDomain(reset_less=True)

        # # #

        # Clk / Rst
        if not use_internal_osc:
            clk = platform.request("clk25")
            clk_freq = 25e6
        else:
            clk = Signal()
            div = 5
            self.specials += Instance("OSCG",
                                p_DIV = div,
                                o_OSC = clk)
            clk_freq = 310e6/div

        rst_n = 1 if not with_rst else platform.request("user_btn_n", 0)

        # PLL
        self.submodules.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~rst_n)
        pll.register_clkin(clk, clk_freq)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        if sdram_rate == "1:2":
            pll.create_clkout(self.cd_sys2x,    2*sys_clk_freq)
            pll.create_clkout(self.cd_sys2x_ps, 2*sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.
        else:
           pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.

        # USB PLL
        if with_usb_pll:
            self.submodules.usb_pll = usb_pll = ECP5PLL()
            self.comb += usb_pll.reset.eq(~rst_n)
            usb_pll.register_clkin(clk, clk_freq)
            self.clock_domains.cd_usb_12 = ClockDomain()
            self.clock_domains.cd_usb_48 = ClockDomain()
            usb_pll.create_clkout(self.cd_usb_12, 12e6, margin=0)
            usb_pll.create_clkout(self.cd_usb_48, 48e6, margin=0)

        # SDRAM clock
        sdram_clk = ClockSignal("sys2x_ps" if sdram_rate == "1:2" else "sys_ps")
        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), sdram_clk)


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    """A SoC on 5A-75X, optionally with a softcore CPU"""

    # Statically-define the memory map, to prevent it from shifting across various litex versions.
    SoCCore.mem_map = {
        "sram":             0x10000000,  # (default shadow @0xa0000000)
        "spiflash":         0x20000000,  # (default shadow @0xa0000000)
        "main_ram":         0x40000000,
        "ethphy":           0x80000000,
        "csr":              0xe0000000,  # (default shadow @0x60000000)
        "vexriscv_debug":   0xf00f0000,
    }

    def __init__(self, debug, flash_offset, board, revision, with_ethernet=False, with_etherbone=False, eth_phy=0, sys_clk_freq=60e6, use_internal_osc=False, sdram_rate="1:1", **kwargs):
        """Create a basic SoC for Colorlight 5A-75X.

        Returns:
            Newly-constructed SoC
        """
        board = board.lower()
        assert board in ["5a-75b", "5a-75e"]
        if board == "5a-75b":
            platform = colorlight_5a_75b.Platform(revision=revision)
        elif board == "5a-75e":
            platform = colorlight_5a_75e.Platform(revision=revision)

        if board == "5a-75e" and revision == "6.0" and (with_etherbone or with_ethernet):
            assert use_internal_osc, "You cannot use the 25MHz clock as system clock since it is provided by the Ethernet PHY and will stop during PHY reset."

        # add custom serial port ios
        platform.add_extension(_myserial)

        # Set cpu name and variant defaults when none are provided
        if "cpu_variant" not in kwargs:
            if debug:
                kwargs["cpu_variant"] = "imac+debug"
            else:
                kwargs["cpu_variant"] = "imac"

        kwargs["integrated_main_ram_size"] = 0
        kwargs["integrated_rom_size"]  = 0

        kwargs["csr_data_width"] = 32

        # Set CPU reset address
        kwargs["cpu_reset_address"] = self.mem_map["spiflash"] + flash_offset

        # defaul uart myserial
        kwargs["uart_name"] = "myserial"

        # Select "crossover" as soc uart instead of "serial"
        # We have to make that selection before calling the parent initializer
        if debug:
            kwargs["uart_name"]   = "crossover"

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, int(sys_clk_freq),
            ident          = "LiteX SoC on Colorlight " + board.upper(),
            ident_version  = True,
            **kwargs)

        with_rst = False # kwargs["uart_name"] not in ["serial", "bridge", "crossover"] # serial_rx shared with user_btn_n.
        with_usb_pll = kwargs.get("uart_name", None) == "usb_acm"
        self.submodules.crg = _CRG(platform, sys_clk_freq, use_internal_osc=use_internal_osc, with_usb_pll=with_usb_pll, with_rst=with_rst, sdram_rate=sdram_rate)

        # SDR SDRAM --------------------------------------------------------------------------------
        sdrphy_cls = HalfRateGENSDRPHY if sdram_rate == "1:2" else GENSDRPHY
        self.submodules.sdrphy = sdrphy_cls(platform.request("sdram"))
        if board == "5a-75e" and revision == "6.0":
            sdram_cls  = M12L64322A
            sdram_size = 0x80000000
        else:
            sdram_cls  = M12L16161A
            sdram_size = 0x40000000
        self.add_sdram("sdram",
            phy                     = self.sdrphy,
            module                  = sdram_cls(sys_clk_freq, sdram_rate),
            origin                  = self.mem_map["main_ram"],
            size                    = kwargs.get("max_sdram_size", sdram_size),
            l2_cache_size           = kwargs.get("l2_size", 8192),
            l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
            l2_cache_reverse        = True
        )

        # The litex SPI module supports memory-mapped reads, as well as a bit-banged mode
        # for doing writes.
        spiflash_size = 32 * 1024 * 1024
        self.submodules.spiflash = spiflash = SpiFlash(platform.request("spiflash"), dummy=8, endianness="little")
        spiflash.add_clk_primitive(platform.device)
        self.register_mem("spiflash", self.mem_map["spiflash"], self.spiflash.bus, size=spiflash_size)
        self.add_csr("spiflash")

        # Add ROM linker region
        self.add_memory_region("rom", self.mem_map["spiflash"] + flash_offset, spiflash_size - flash_offset, type="cached+linker")

        # In debug mode, add a UART bridge.  This takes over from the normal UART bridge,
        # however you can use the "crossover" UART to communicate with this over the bridge.
        if debug:
            self.submodules.uart_bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq, baudrate=115200)
            self.add_wb_master(self.uart_bridge.wishbone)
            if hasattr(self, "cpu") and self.cpu.name == "vexriscv":
                self.register_mem("vexriscv_debug", 0xf00f0000, self.cpu.debug_bus, 0x100)

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            if board == "5a-75b" and revision == "7.0":
                self.submodules.ethphy = LiteEthPHYRGMII(
                    clock_pads = self.platform.request("eth_clocks", eth_phy),
                    pads       = self.platform.request("eth", eth_phy),
                    tx_delay   = 0e-9)
            else:
                self.submodules.ethphy = LiteEthPHYRGMII(
                    clock_pads = self.platform.request("eth_clocks", eth_phy),
                    pads       = self.platform.request("eth", eth_phy))
            self.add_csr("ethphy")
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy)


        # add IO extentions
        platform.add_extension(_leds)
        platform.add_extension(_adc_second_order)


        # LEDs blinkyblinky :)

        self.submodules.leds = Leds(Cat(
            platform.request("r"),
            platform.request("y"),
            platform.request("g")),
            led_polarity=0x00,
            led_name=[
                ["r", "The Red LED."],
                ["y", "The Yellow LED."],
                ["g", "The Green Red LED."]])

        self.add_csr("leds")


        # sigma delta ADC using lame CSR for now

        self.submodules.adc = adc = ADC()

        adc_in = platform.request("in")
        adc_sd = platform.request("sd", 0)
        adc_sd2 = platform.request("sd", 1)
        #p5v = platform.request("p5v")
        p3v = platform.request("p3v")

        self.comb += [
            adc.inp.eq(adc_in),
            adc_sd.eq(adc.sd),
            adc_sd2.eq(adc.sd),
            #p5v.eq(1),
            p3v.eq(1),
        ]

        self.add_csr("adc")

        self.submodules.gpio = gpio = GPIOIn(platform.request("user_btn_n", 0), with_irq=True)

        self.add_csr("gpio")
        self.add_interrupt("gpio")


# Helper functions ---------------------------------------------------------------------------------

def modify_svd(builder_kwargs):
    # Add Ethernet buffer peripheral to svd
    with open(builder_kwargs["csr_svd"], "r") as f:
        line_number = 0
        for l in f:         # search for the right place to insert
            line_number += 1
            if """</peripherals>""" in l:
                line = line_number
        print("inserting before line:")
        print(line)
        f.seek(0)
        s = f.readlines()
    registers = """        <peripheral>
            <name>ETHMEM</name>
            <baseAddress>0x80000000</baseAddress>
            <groupName>ETHMEM</groupName>
            <registers>
                <register>
                    <name>RX_BUFFER_0[%s]</name>
                    <dim>2048</dim>
                    <dimIncrement>1</dimIncrement>
                    <description><![CDATA[rx buffers]]></description>
                    <addressOffset>0x0000</addressOffset>
                    <resetValue>0x00</resetValue>
                    <size>8</size>
                    <access>read-only</access>
                    <fields>
                        <field>
                            <name>rx_buffer_0</name>
                            <msb>7</msb>
                            <bitRange>[7:0]</bitRange>
                            <lsb>0</lsb>
                        </field>
                    </fields>
                </register>
                <register>
                    <name>RX_BUFFER_1[%s]</name>
                    <dim>2048</dim>
                    <dimIncrement>1</dimIncrement>
                    <description><![CDATA[rx buffers]]></description>
                    <addressOffset>0x0800</addressOffset>
                    <resetValue>0x00</resetValue>
                    <size>8</size>
                    <access>read-only</access>
                    <fields>
                        <field>
                            <name>rx_buffer_1</name>
                            <msb>7</msb>
                            <bitRange>[7:0]</bitRange>
                            <lsb>0</lsb>
                        </field>
                    </fields>
                </register>
                <register>
                    <name>TX_BUFFER_0[%s]</name>
                    <dim>2048</dim>
                    <dimIncrement>1</dimIncrement>
                    <description><![CDATA[tx buffers]]></description>
                    <addressOffset>0x1000</addressOffset>
                    <resetValue>0x00</resetValue>
                    <size>8</size>
                    <access>read-write</access>
                    <fields>
                        <field>
                            <name>tx_buffer_0</name>
                            <msb>7</msb>
                            <bitRange>[7:0]</bitRange>
                            <lsb>0</lsb>
                        </field>
                    </fields>
                </register>
                <register>
                    <name>TX_BUFFER_1[%s]</name>
                    <dim>2048</dim>
                    <dimIncrement>1</dimIncrement>
                    <description><![CDATA[tx buffers]]></description>
                    <addressOffset>0x1800</addressOffset>
                    <resetValue>0x00</resetValue>
                    <size>8</size>
                    <access>read-write</access>
                    <fields>
                        <field>
                            <name>tx_buffer_1</name>
                            <msb>7</msb>
                            <bitRange>[7:0]</bitRange>
                            <lsb>0</lsb>
                        </field>
                    </fields>
                </register>
            </registers>
            <addressBlock>
                <offset>0</offset>
                <size>0x4000</size>
                <usage>buffer</usage>
            </addressBlock>
        </peripheral>
    """
    s.insert(line-1, registers)
    with open(builder_kwargs["csr_svd"], "w") as f:
        f.writelines(s)


def modify_memory_x(builder_kwargs):
    # Add main_ram section to memory.x
    with open(builder_kwargs["memory_x"], "a") as f:
        f.write(textwrap.dedent(
            """
            SECTIONS
            {
                .main_ram (NOLOAD) : ALIGN(4)
                {
                    *(.main_ram .main_ram.*);
                    . = ALIGN(4);
                } > main_ram
            } INSERT AFTER .bss;
            """)
        )


# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Colorlight 5A-75X")
    parser.add_argument("--build",            action="store_true",      help="Build bitstream")
    parser.add_argument("--flash-offset",     default=0x100000,         help="Boot offset in SPI Flash")
    parser.add_argument("--sys-clk-freq",     type=float, default=50e6, help="Select system clock frequency")
    parser.add_argument("--debug",            action="store_true",      help="Enable debug features. (UART has to be used with the wishbone-tool.)")
    parser.add_argument("--board",            default="5a-75b",         help="Board type: 5a-75b (default) or 5a-75e")
    parser.add_argument("--revision",         default="7.0", type=str,  help="Board revision 7.0 (default), 6.0 or 6.1")
    parser.add_argument("--with-ethernet",    action="store_true",      help="Enable Ethernet support")
    parser.add_argument("--with-etherbone",   action="store_true",      help="Enable Etherbone support")
    parser.add_argument("--eth-phy",          default=0, type=int,      help="Ethernet PHY 0 or 1 (default=0)")
    parser.add_argument("--use-internal-osc", action="store_true",      help="Use internal oscillator")
    parser.add_argument("--sdram-rate",       default="1:1",            help="SDRAM Rate 1:1 Full Rate (default), 1:2 Half Rate")
    parser.add_argument("--document-only",    action="store_true",      help="Do not build a soc. Only generate documentation.")
    parser.add_argument("--flash",            action="store_true",      help="Load bitstream to flash")
    parser.add_argument("--load",             action="store_true",      help="Load bitstream to SRAM")
    builder_args(parser)
    soc_core_args(parser)
    trellis_args(parser)
    args = parser.parse_args()

    assert not (args.with_ethernet and args.with_etherbone)

    # Create the SOC
    soc = BaseSoC(board            = args.board,
                  revision         = args.revision,
                  debug            = args.debug,
                  flash_offset     = args.flash_offset,
                  sys_clk_freq     = int(args.sys_clk_freq),
                  with_ethernet    = args.with_ethernet,
                  with_etherbone   = args.with_etherbone,
                  eth_phy          = args.eth_phy,
                  use_internal_osc = args.use_internal_osc,
                  sdram_rate       = args.sdram_rate,
                  **soc_core_argdict(args))

    # Configure command line parameter defaults
    # Don't build software -- we don't include it since we just jump to SPI flash.
    builder_kwargs = builder_argdict(args)
    builder_kwargs["compile_software"] = False

    if args.document_only:
        builder_kwargs["compile_gateware"] = False
    if builder_kwargs["csr_svd"] is None:
        builder_kwargs["csr_svd"] = "../rust/litex-pac/clSOC.svd"
    if builder_kwargs["memory_x"] is None:
        builder_kwargs["memory_x"] = "../rust/litex-pac/memory.x"

    # Create and run the builder
    builder = Builder(soc, **builder_kwargs)
    builder.build(**trellis_argdict(args), run=args.build)
    lxsocdoc.generate_docs(soc, "build/documentation/", project_name="Colorlight 5A-75x LiteX Riscv SOC", author="DerFetzer")

    modify_memory_x(builder_kwargs)
    modify_svd(builder_kwargs)

    # If requested load the resulting bitstream onto the 5A-75X
    if args.flash or args.load:
        prog = ECP5Programmer()
        if args.load:
            prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))
        if args.flash:
            prog.flash(0x00000000, os.path.join(builder.gateware_dir, soc.build_name + ".bit"))


if __name__ == "__main__":
    main()
