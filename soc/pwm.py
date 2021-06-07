# SingularitySurfer 2020


import numpy as np
from migen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField
from litex.soc.integration.doc import AutoDoc, ModuleDoc


class PWM(Module, AutoCSR):
    """A simple PWM peripheral that uses a timer at sysclock.

     Parameters
    ----------
    pin : output pin
    width : width of the physical counter
    """
    def __init__(self, pin, width=32):

        # Documentation
        self.intro = ModuleDoc("""A simple PWM peripheral that uses a timer at sysclock.
        PWM period software adjustable. If pwm output value larger that period -> kaputt.
        """)

        self.period = CSRStorage(width, description='pwm period in sys cycles')
        self.value = CSRStorage(width, description='pwm output value in sys cycles')


        cnt = Signal(width)

        self.sync += {
            cnt.eq(cnt+1),
            If(cnt == self.period.storage, cnt.eq(0)),
            pin.eq(cnt < self.value.storage)
        }