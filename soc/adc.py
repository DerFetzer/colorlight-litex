# SingularitySurfer 2020


import numpy as np
from migen import *
from litex.soc.interconnect.csr import *
from functools import reduce
from operator import and_


class ADC(Module, AutoCSR):
    """Basic sigma-delta ADC with a CIC decimator running at sys clock.
    The decimator gain is such that the output bitwidth will always be maximized.
    """
    def __init__(self, cic_order=6, cic_ratechange=2**7, width_o=32, clk_div=4):
        self.inp = Signal()         # analog in
        self.sd = Signal()          # sigma-delta switching pin
        self.dout = Signal(width_o)
        self.stb = Signal()         # data out strobe
        ###

        width_csr = 32
        self.adc_val = CSRStatus(width_csr, name="adc_value", description="adc conversion value (continuously updated)")


        if not(cic_ratechange & (cic_ratechange-1) == 0) and not cic_ratechange != 0:
            raise ValueError()      # not a power of two


        if clk_div != 0:
            divcnt = Signal(clk_div)
            self.sync += divcnt.eq(divcnt+1)

        b_max = np.ceil(np.log2(cic_ratechange))  # max bit growth

        if clk_div != 0:
            self.sync += [
                If(reduce(and_, divcnt),      # do the sigma-delta
                    self.sd.eq(self.inp))
            ]
        else:
            self.sync += [              # do the sigma-delta
                self.sd.eq(self.inp)
            ]

        width = (int(b_max)*cic_order)+1
        sig = Signal((width, True), reset_less=True)
        self.comb += sig.eq(self.sd)
        for _ in range(cic_order):                      # integrator cascade
            sum = Signal((width, True), reset_less=True)
            self.sync += [
                sum.eq(sum + sig)
            ]
            sig = sum

        cnt = Signal(int(b_max))
        self.sync += cnt.eq(cnt+1)
        for _ in range(cic_order):                      # comb cascade
            dif = Signal((width, True), reset_less=True)
            reg = Signal((width, True), reset_less=True)
            self.sync += [
                self.stb.eq(0),
                If(reduce(and_, cnt),       # counter up and therefore new output sample
                    self.stb.eq(1),
                    reg.eq(sig),
                    dif.eq(sig - reg)
                   )
            ]
            sig = dif

        self.comb += self.dout.eq(sig[-width_o:])               # highest bits are the data output
        self.sync += self.adc_val.status.eq(sig[-width_csr:])               # continuously update CSR


    def sim(self):
        yield 
        yield self.inp.eq(1)
        for i in range(10000):
            yield
            if(i==3000):
                yield self.inp.eq(~self.inp)
            if (i == 5000):
                yield self.inp.eq(~self.inp)
            if i>7000:
                yield self.inp.eq(~self.inp)


if __name__ == "__main__":
    test = ADC(6,2**8,16)
    run_simulation(test, test.sim(), vcd_name="adc.vcd")