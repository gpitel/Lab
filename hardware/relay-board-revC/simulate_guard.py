"""
Driven guard under a terminal's lines: what it leaves of the ground
capacitance, 10 kHz - 50 MHz, in ngspice.

    python simulate_guard.py

Study for the question raised by N. Rosano's review (2026-10-13): a copper
island on the inner layers under one terminal's buses, arms and clips, held
at the terminal's own voltage by a unity buffer, instead of the plane opening
used today. The island takes the place of ground under the lines, so the
line-to-island capacitance is the solid-plane figure (22-50 pF per terminal,
microstrip estimate); the buffer makes it carry no current as far as its gain
is exactly 1.

Circuit: terminal node N; C_lg from N to the guard island; buffer input
through R_s (protection) with a bootstrapped bias network; follower; R_iso;
island with C_gg to ground. Optional attenuation k of the guard drive.
The admittance seen from N to ground is reported as C_eff = Im(Y)/w and
G_eff = Re(Y): what a floating winding would be loaded with.

OPA356 as a macro model: 200 MHz GBW, second pole at 400 MHz, 20 R output.
A macro model, not the device: the numbers size the effect, they are not a
guarantee of stability.
"""

import os
import sys

import numpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
from SpiceModel import NgSpice  # noqa: E402

OPAMP = """
.subckt opamp inp inn out
egain n1 0 inp inn 1e5
rp n1 n2 1k
cp n2 0 79.6n
e2 n3 0 n2 0 1
rp2 n3 n4 1k
cp2 n4 0 0.398p
ebuf n5 0 n4 0 1
ro n5 out 20
.ends
"""

C_IN = 1.5e-12          # op-amp common-mode input capacitance
C_DIRECT = 0.5e-12      # line capacitance the island does not screen (fringe past its edge)


def netlist(c_lg, c_gg, r_s, r_iso, k):
    r_att = "" if k >= 1 else f"ratt g 0 {r_iso * k / (1 - k)}"
    return f"""* driven guard
{OPAMP}
vn n 0 ac 1
clg n g {c_lg}
cdir n 0 {C_DIRECT}
rs n ni {r_s}
cin ni 0 {C_IN}
rb1 ni m 1meg
rb2 m 0 1meg
cb bo m 100n
xu ni bo bo opamp
riso bo g {r_iso}
{r_att}
cgg g 0 {c_gg}
.ac dec 20 1e3 100e6
.end
""".splitlines()


def admittance(c_lg, c_gg=20e-12, r_s=470.0, r_iso=22.0, k=1.0):
    spice = NgSpice.get()
    spice.run(netlist(c_lg, c_gg, r_s, r_iso, k))
    frequency = spice.vector("frequency").real
    return frequency, -spice.vector("vn#branch"), spice.vector("g")


def main():
    targets = (1e4, 1e5, 1e6, 3e6, 1e7, 2e7, 5e7)
    for label, c_lg, unguarded in (("terminal A/C/D", 25e-12, 7e-12), ("terminal B", 50e-12, 15e-12)):
        print(f"\n{label}: {c_lg * 1e12:.0f} pF line-to-island; today (plane opening) ~{unguarded * 1e12:.0f} pF to ground")
        for k in (1.0, 0.95, 0.9):
            frequency, y, guard = admittance(c_lg, k=k)
            print(f"  guard gain {k:.2f}:  " + "  ".join(f"{t:8.0e}" for t in targets) + "  Hz")
            pick = [int(numpy.argmin(abs(frequency - t))) for t in targets]
            print("    C_eff pF       " + "  ".join(f"{y[i].imag / (2 * numpy.pi * frequency[i]) * 1e12:8.2f}" for i in pick))
            print("    G_eff uS       " + "  ".join(f"{y[i].real * 1e6:8.2f}" for i in pick))
            print("    |guard/node|   " + "  ".join(f"{abs(guard[i]):8.3f}" for i in pick))
            negative = frequency[y.real < 0]
            worst = int(numpy.argmin(y.real))
            if len(negative):
                print(f"    negative conductance {negative.min():.2g} .. {negative.max():.2g} Hz, "
                      f"worst {y.real[worst] * 1e6:.1f} uS (= {1 / y.real[worst] / 1e3:.1f} kohm) at {frequency[worst]:.2g} Hz")
            else:
                print("    conductance positive everywhere")
    print("\nA negative G_eff across a winding at its self-resonance cancels part of its loss "
          "(reads as higher Q) and oscillates if it exceeds it.")


if __name__ == "__main__":
    main()
