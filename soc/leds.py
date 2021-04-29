
import numpy as np
from migen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField
from litex.soc.integration.doc import AutoDoc, ModuleDoc

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
