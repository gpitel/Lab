"""
Generate the OpenMagnetics relay board rev C schematic.

    python generate_schematic.py

Produces relay_board.kicad_sch, runs ERC via kicad-cli, and exports the
netlist.  Connectivity is by net label on a wire stub at each pin; genuinely
unused pins get a no-connect flag, which is what keeps ERC clean.

ARCHITECTURE -- see ARCHITECTURE.md (rev C: Kelvin force/sense everywhere).

  Every signal relay is a DPDT G6K: pole 1 switches a terminal's FORCE
  path, pole 2 the same terminal's SENSE path. Each DUT clamp is a Kelvin
  pair: pad 1 = force (DUT_x), pad 2 = sense (DUTS_x).

      K1..K8    A/B/C/D -> HI / LO       force T_x -> RAIL_*, sense S_x -> SNS_*
      K9, K10   standards column E -> HI/LO
      K11..K14  isolation A..D           NC = clamp, NO = open (the OPEN standard)
      K15..K18  links A-B, C-D, B-C, B-D bus to bus, force and sense
      K19..K21  standard select          100R LOAD, 1R check, 10 pF check (Kelvin)
      K22       2-wire tie               de-energized: SNS_* = RAIL_* (2-wire)
      K23       CH1 select               de-energized: bridge divider; on: Kelvin front-end

  On-board standards (iso-relay plane; per configuration, automatic):
    OPEN   all iso relays energized
    SHORT  iso energized + the first HI terminal's LO relay: force rails
           bridged through that terminal's bus; in Kelvin mode both sense
           rails meet on its (current-free) sense bus -- an exact zero
    LOAD   R5 100R 0.1 % through the standards column, sensed at its pads
    CHECK  R18 1R and C30 10 pF, same column: post-calibration verification
  Clamp-plane fixture compensation (open + shorting bar per path, Keysight /
  Voltech practice) removes what the iso plane cannot see: the arms, the
  clamps and the SHORT path's residual. See ARCHITECTURE.md.

RELAY PINOUT (KiCad Relay:G6K-2, confirmed by footprint geometry):

      coil    1, 8
      pole 1  COM = 3    NC = 2    NO = 4
      pole 2  COM = 6    NC = 7    NO = 5
"""

import os
import subprocess
import sys

import glob as _glob
import os as _os

# kicad-sch-api searches KiCad 7-9 install paths only; point it at the newest
# installed KiCad's symbol libraries (KiCad 10 here) before it loads.
_SYMBOLS = sorted(_glob.glob(r"C:\Program Files\KiCad\*\share\kicad\symbols"),
                  key=lambda p: float(p.split("\\")[3]))
if _SYMBOLS:
    _os.environ.setdefault("KICAD_SYMBOL_DIR", _SYMBOLS[-1])
import kicad_sch_api as ksa
from kicad_sch_api.library.cache import get_symbol_cache as _get_symbol_cache

if _SYMBOLS:
    # Same-named libraries are first-come in kicad-sch-api, and an older
    # KiCad (7.0 here) would shadow symbols that only exist in the newest one.
    _cache = _get_symbol_cache()
    _cache._library_paths.clear()
    _cache._library_index.clear()
    _cache._lib_stats.clear()
    _cache.discover_libraries(search_paths=[_SYMBOLS[-1]])

KICAD_CLI = r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMATIC = os.path.join(HERE, "relay_board.kicad_sch")

# --------------------------------------------------------------------------
# G6K-2 pin names -- single source of truth
# --------------------------------------------------------------------------
COIL_P, COIL_N = "1", "8"
P1_COM, P1_NC, P1_NO = "3", "2", "4"
P2_COM, P2_NC, P2_NO = "6", "7", "5"

RELAY_LIB = "Relay:G6K-2"
RELAY_FP = "OpenMagnetics:Relay_Omron_G6K-2F-RF-S"
RELAY_PART = "G6K-2F-RF-S DC5"

OPAMP = "Amplifier_Operational:OPA356xxDBV"   # SOT-23-5: 1 OUT 2 V- 3 +IN 4 -IN 5 V+
SOT235 = "Package_TO_SOT_SMD:SOT-23-5"
R0402 = "Resistor_SMD:R_0402_1005Metric"
R0603 = "Resistor_SMD:R_0603_1608Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"

# STM32F072CBTx LQFP-48, parsed from the KiCad symbol.
# BOOT0 is a DEDICATED pin (44) on this part -- on the F042 it shares PB8,
# which is how rev A ended up strapping PF0 instead.
MCU = {
    "VBAT": "1", "PC13": "2", "PC14": "3", "PC15": "4", "PF0": "5", "PF1": "6",
    "NRST": "7", "VSSA": "8", "VDDA": "9",
    "PA0": "10", "PA1": "11", "PA2": "12", "PA3": "13", "PA4": "14", "PA5": "15",
    "PA6": "16", "PA7": "17", "PB0": "18", "PB1": "19", "PB2": "20",
    "PB10": "21", "PB11": "22", "PB12": "25", "PB13": "26", "PB14": "27", "PB15": "28",
    "PA8": "29", "PA9": "30", "PA10": "31", "PA11": "32", "PA12": "33", "PA13": "34",
    "VDDIO2": "36", "PA14": "37", "PA15": "38",
    "PB3": "39", "PB4": "40", "PB5": "41", "PB6": "42", "PB7": "43",
    "BOOT0": "44", "PB8": "45", "PB9": "46", "VSS": "47", "VDD": "48",
}

# ULN2003 / TBD62003APG: inputs 1..7, outputs 16..10, GND 8, COM 9.
ULN_IN = {n: str(n) for n in range(1, 8)}
ULN_OUT = {n: str(17 - n) for n in range(1, 8)}

#: Relay index (1..24) -> GPIO, in the same order as the relay bits in
#: scripts/RelayBoardController.py (relay K(n) drives bit n-1) and
#: RELAY_GPIO_TABLE in firmware/include/relay_map.h (the firmware encodes a
#: pin as port*16 + bit, ports A=0, B=1, C=2, F=3).
#: Every relay-drive pin is on a package side that faces the drivers (north
#: row, west and east columns): none wraps around the MCU through the USB/SWD
#: The order is lane order (non-crossing): every driver input sits on one
#: line (x = 181.5, pads facing east), U6 p1 southmost .. U2 p7 northmost.
#: North-row pins are the innermost lanes (west->east: U6 p1, p2, U4 p1..p5),
#: east-column pins the next (top->bottom: U4 p6, p7, U3 p1..p6), west-column
#: pins reach their inputs from inside the driver packages (top->bottom:
#: U3 p7, U2 p1..p7). Each group is monotonic in input height: no crossings.
#: corner. I2C2 is on the west pins PB10/PB11, facing the TMP117 (I2C on
#: the east pins crossed the whole relay corridor). PC13/PC14/PC15/PF0 are plain GPIO here (no crystals, 3 mA is plenty
#: for a TBD62003 input). The south row keeps the functions that go south or
#: east: coil-switch enable/fault, BUSY/KELVIN/PROG LEDs.
#: The north row splits at the VDDA/NRST parts over pins 7-9: PA0..PA2 (west
#: of them) can only leave north-west and reach the drivers from the WEST,
#: under U6's body, where the order is mirrored (eastmost pin = southmost
#: input: PA0 -> U6 p1, PA1 -> U6 p2, PA2 -> U4 p1). PC13..PF0 (east of them)
#: join the east channel (PF0 -> U4 p2 .. PC13 -> U4 p5). PF1 (pin 6) sits
#: under the NRST parts with no outward escape and stays unused.
GPIO_PINS = ["PA4", "PA5", "PA6", "PA7", "PB0", "PB1", "PB2", "PB7",
             "PB6", "PB5", "PB4", "PB3", "PA15", "PA3", "PA2", "PF0",
             "PC15", "PC14", "PC13", "PB9", "PB8", "PA0", "PA1"]
RELAY_COUNT = len(GPIO_PINS)

#: (terminal, rail) for the eight crossbar relays, K1..K8.
MATRIX = [(t, r) for t in ("A", "B", "C", "D") for r in ("HI", "LO")]
#: Direct link relays K15..K18 (every link the configuration table uses).
LINKS = [("A", "B"), ("C", "D"), ("B", "C"), ("B", "D")]
#: Standards column K19..K21: (select relay, net stem, part, value, footprint, why)
STANDARDS = [
    ("K19", "STD1", "R5", "100R 0.1%", "Resistor_SMD:R_0805_2012Metric",
     "LOAD standard: 100R 0.1% 25ppm thin film (e.g. Vishay TNPW0805100RBEEA)"),
    ("K20", "STD2", "R18", "1R 0.5%", "Resistor_SMD:R_1206_3216Metric",
     "low-Z check standard (Rac / leakage range); value characterized 4-wire once"),
    ("K21", "STD3", "C30", "10pF C0G", "Capacitor_SMD:C_0603_1608Metric",
     "capacitance check standard (Cps range), C0G +/-0.1 pF"),
]


class Builder:
    def __init__(self):
        self.sch = ksa.create_schematic("relay_board")
        self.sch.set_paper_size("A1")
        self.sch.set_title_block(
            title="OpenMagnetics Automated Relay Board rev C",
            date="2026-08-29",
            company="OpenMagnetics Lab",
            comments={
                1: "23x G6K-2F-RF-S DPDT | Kelvin 4x2 crossbar + links + Kelvin standards column",
                2: "Pin map: coil 1/8, pole1 COM=3 NC=2 NO=4, pole2 COM=6 NC=7 NO=5",
            },
        )
        self.centres = {}
        self.net_uses = {}

    # ---------------------------------------------------------------- parts

    def add(self, lib_id, reference, value, x, y, footprint=None, **properties):
        self.sch.components.add(lib_id, reference, value, position=(x, y),
                                footprint=footprint, **properties)
        self.centres[reference] = (x, y)
        return reference

    def _pin_positions(self, reference):
        if reference not in getattr(self, "_pin_cache", {}):
            self._pin_cache = getattr(self, "_pin_cache", {})
            self._pin_cache[reference] = [
                (number, position) for number, position
                in self.sch.list_component_pins(reference)]
        return self._pin_cache[reference]

    def _stub_end(self, reference, pin, length=3.81):
        point = self.sch.get_component_pin_position(reference, pin)
        if point is None:
            raise KeyError(f"{reference} has no pin {pin}")
        cx, cy = self.centres[reference]
        others = [q for number, q in self._pin_positions(reference)
                  if abs(q.x - point.x) > 0.01 or abs(q.y - point.y) > 0.01]
        shares_column = any(abs(q.x - point.x) < 0.01 for q in others)
        shares_row = any(abs(q.y - point.y) < 0.01 for q in others)

        # A stub along a line that other pins sit on can overlap their stubs
        # (collinear wires merge nets in KiCad), so escape perpendicular to
        # whichever line this pin shares with its neighbours.
        if shares_column and not shares_row:
            direction = "horizontal"
        elif shares_row and not shares_column:
            direction = "vertical"
        elif shares_column and shares_row:
            # grid: escape away from the body along the axis the pin sits
            # out on (top/bottom rows vertical, side columns horizontal), so
            # 2.54 mm-pitch power pins on an MCU's edge never overlap.
            ys = [q.y for _, q in self._pin_positions(reference)]
            on_edge_row = (abs(point.y - min(ys)) < 0.01
                           or abs(point.y - max(ys)) < 0.01)
            direction = "vertical" if on_edge_row else "horizontal"
        else:
            direction = "vertical" if abs(point.y - cy) >= abs(point.x - cx) else "horizontal"

        if direction == "vertical":
            end_y = point.y - length if point.y < cy else point.y + length
            return (point.x, point.y), (point.x, end_y), 90
        end_x = point.x - length if point.x < cx else point.x + length
        return (point.x, point.y), (end_x, point.y), 0

    def net(self, reference, pin, net_name, length=3.81):
        """Wire stub from a pin, with a net label at its end."""
        start, end, rotation = self._stub_end(reference, pin, length)
        self.sch.add_wire(start=start, end=end)
        self.sch.add_label(net_name, position=end, rotation=rotation)
        self.net_uses.setdefault(net_name, []).append(f"{reference}.{pin}")

    def unused(self, reference, *pins):
        """No-connect flag. A label on a one-pin net is what ERC calls dangling."""
        for pin in pins:
            point = self.sch.get_component_pin_position(reference, pin)
            self.sch.no_connects.add((point.x, point.y))

    def power_flag(self, net_name, x, y):
        self._flag_count = getattr(self, "_flag_count", 0) + 1
        reference = f"#FLG{self._flag_count:03d}"
        self.add("power:PWR_FLAG", reference, "PWR_FLAG", x, y)
        self.net(reference, "1", net_name)

    # ------------------------------------------------------------- sections

    def build_matrix(self):
        """K1..K8: terminal -> rail. Pole 1 force, pole 2 sense.

        Using the second pole for the same terminal's sense line is what makes
        the board Kelvin without a separate sense matrix: the force and sense
        of one terminal share a relay (the ~0.1 pF pole-to-pole coupling then
        sits between two ends of the same node, where it does nothing).
        """
        for index, (terminal, rail) in enumerate(MATRIX, start=1):
            reference = f"K{index}"
            column, row = divmod(index - 1, 2)
            self.add(RELAY_LIB, reference, RELAY_PART,
                     50 + column * 52, 45 + row * 46, RELAY_FP,
                     Description=f"{terminal} -> {rail} (force + sense)")
            self.net(reference, COIL_P, "+5V")
            self.net(reference, COIL_N, f"{reference}_COIL")
            self.net(reference, P1_COM, f"T{terminal}")
            self.net(reference, P1_NO, f"RAIL_{rail}")
            self.net(reference, P2_COM, f"S{terminal}")
            self.net(reference, P2_NO, f"SNS_{rail}")
            self.unused(reference, P1_NC, P2_NC)

    def build_load_column(self):
        """K9/K10: the standards column, a fifth Kelvin matrix column.

        Keysight: "use a load of same size and measure it in the same way as
        the DUT will be measured." Force reaches the selected standard through
        one relay contact and a column bus per side; pole 2 carries sense to
        the standard's own pads through net ties. K19..K21 pick the standard:
        100R (the OSL LOAD), 1R and 10 pF (verification after calibration,
        in the low-Z and the electrostatic range).
        """
        for reference, cal_net, rail, y in (("K9", "CAL_E", "HI", 45),
                                            ("K10", "CAL_F", "LO", 91)):
            self.add(RELAY_LIB, reference, RELAY_PART, 258, y, RELAY_FP,
                     Description=f"standards column: {cal_net} -> {rail}")
            self.net(reference, COIL_P, "+5V")
            self.net(reference, COIL_N, f"{reference}_COIL")
            self.net(reference, P1_COM, cal_net)
            self.net(reference, P1_NO, f"RAIL_{rail}")
            self.net(reference, P2_COM, f"{cal_net}S")
            self.net(reference, P2_NO, f"SNS_{rail}")
            self.unused(reference, P1_NC, P2_NC)

        for index, (relay, stem, part, value, footprint, description) in enumerate(STANDARDS):
            x = 290 + index * 40
            self.add(RELAY_LIB, relay, RELAY_PART, x, 140, RELAY_FP,
                     Description=f"select {part} ({value})")
            self.net(relay, COIL_P, "+5V")
            self.net(relay, COIL_N, f"{relay}_COIL")
            self.net(relay, P1_COM, "CAL_E")
            self.net(relay, P1_NO, f"{stem}_F")
            self.net(relay, P2_COM, "CAL_ES")
            self.net(relay, P2_NO, f"{stem}_S")
            self.unused(relay, P1_NC, P2_NC)
            symbol = "Device:C" if part.startswith("C") else "Device:R"
            self.add(symbol, part, value, x, 175, footprint, Description=description)
            self.net(part, "1", f"{stem}_F")
            self.net(part, "2", "CAL_F")
            tie = f"NT{[1, 4, 5][index]}"
            self.add("Device:NetTie_2", tie, "NetTie", x + 15, 175,
                     "NetTie:NetTie-2_SMD_Pad0.5mm",
                     Description=f"Kelvin sense tie at {part} pad 1")
            self.net(tie, "1", f"{stem}_F")
            self.net(tie, "2", f"{stem}_S")
        # one return-side sense tie, at the 1R's pad 2 (where it matters)
        self.add("Device:NetTie_2", "NT2", "NetTie", 305, 81,
                 "NetTie:NetTie-2_SMD_Pad0.5mm",
                 Description="Kelvin sense tie CAL_FS at R18 pad 2")
        self.net("NT2", "1", "CAL_F")
        self.net("NT2", "2", "CAL_FS")

    def build_isolation(self):
        """K11..K14: one isolation relay per terminal, force and sense.

        Per-terminal so no two DUT terminals ever meet inside one relay
        package (G6K pole-to-pole ~0.1 pF would sit uncalibratably across the
        quantity under measurement). De-energized, NC joins the clamp to the
        terminal buses. Energized, the DUT is out of circuit and the buses
        end on the NO throws: open there, that IS the OPEN standard; closed
        through the SHORT relays, it is the SHORT.
        """
        for offset, terminal in enumerate("ABCD"):
            reference = f"K{11 + offset}"
            self.add(RELAY_LIB, reference, RELAY_PART, 50 + offset * 52, 200,
                     RELAY_FP,
                     Description=f"isolate {terminal}: COM=bus NC=clamp NO=open")
            self.net(reference, COIL_P, "+5V")
            self.net(reference, COIL_N, f"{reference}_COIL")
            self.net(reference, P1_COM, f"T{terminal}")
            self.net(reference, P1_NC, f"DUT_{terminal}")
            self.net(reference, P2_COM, f"S{terminal}")
            self.net(reference, P2_NC, f"DUTS_{terminal}")
            self.unused(reference, P1_NO, P2_NO)

    def build_links_and_shorts(self):
        """K15..K18 direct links, K22 2-wire tie.

        Links join two terminal buses directly (force and sense), replacing
        rev B's LINK rail (14.5 pF to ground, 39-94 nH loops through two
        column buses).
        """
        for offset, (first, second) in enumerate(LINKS):
            reference = f"K{15 + offset}"
            self.add(RELAY_LIB, reference, RELAY_PART, 50 + offset * 52, 250,
                     RELAY_FP, Description=f"link {first}-{second} (force + sense)")
            self.net(reference, COIL_P, "+5V")
            self.net(reference, COIL_N, f"{reference}_COIL")
            self.net(reference, P1_COM, f"T{first}")
            self.net(reference, P1_NO, f"T{second}")
            self.net(reference, P2_COM, f"S{first}")
            self.net(reference, P2_NO, f"S{second}")
            self.unused(reference, P1_NC, P2_NC)

        # K22: 2-wire / Kelvin. De-energized (NC) the sense rails are tied to
        # the force rails, so sense is a second force path and the bridge is
        # the rev B B-WIC. Energized, sense goes only to the front-end.
        self.add(RELAY_LIB, "K22", RELAY_PART, 640, 45, RELAY_FP,
                 Description="2-wire tie: NC joins SNS_HI/SNS_LO to RAIL_HI/RAIL_LO")
        self.net("K22", COIL_P, "+5V")
        self.net("K22", COIL_N, "K22_COIL")
        self.net("K22", P1_COM, "SNS_HI")
        self.net("K22", P1_NC, "RAIL_HI")
        self.net("K22", P2_COM, "SNS_LO")
        self.net("K22", P2_NC, "RAIL_LO")
        self.unused("K22", P1_NO, P2_NO)

    def build_connectors(self):
        # DUT interface: a Kelvin PAIR of Grayhill 02-21 miniature test clips
        # per terminal (force + sense, 8 mm apart: the same point of the DUT
        # lead, which goes through the F clip into the S clip). Each clip's
        # 4 mm banana plug sits straight in a plated 4.0 mm hole -- no jack
        # (the other Series 02 styles are obsolete).
        for index, terminal in enumerate("ABCD"):
            for offset, (kind, net) in enumerate((("F", "DUT"), ("S", "DUTS"))):
                reference = f"J{10 + 2 * index + offset}"
                self.add("Connector:Conn_01x01_Pin", reference, f"DUT {terminal} {kind}",
                         330 + 15 * (2 * index + offset), 45,
                         "OpenMagnetics:Grayhill_02-21_BananaPlug_4mm",
                         Description=f"DUT terminal {terminal} "
                                     f"{'force' if kind == 'F' else 'sense'} clip (Grayhill 02-21)")
                self.net(reference, "1", f"{net}_{terminal}")

        # Bode 100 interface: the B-WIC's impedance bridge, integrated.
        # (Omicron application note "Impedance Measurement Bridge", 2015-09-16:
        # CH1 senses DUT voltage through RV1/RV2, CH2 senses DUT current
        # through the RC1 shunt via RC2. Cloning the B-WIC bridge keeps the
        # instrument's IAD impedance method valid with no adapter attached.)
        for reference, name, x, net_name in (("J6", "SOURCE", 330, "RAIL_HI"),
                                             ("J7", "CH1", 360, "CH1_BNC"),
                                             ("J9", "CH2", 390, "CH2_NODE")):
            self.add("Connector:Conn_Coaxial", reference, f"BNC {name}", x, 90,
                     "Connector_Coaxial:BNC_Amphenol_B6252HB-NPP3G-50_Horizontal",
                     Description=f"BNC to Bode 100 {name}")
            self.net(reference, "1", net_name)
            self.net(reference, "2", "GND")

        bridge = [
            ("R12", "1k 0.1%", "RAIL_HI", "CH1_NODE",
             "bridge RV1: DUT voltage divider, top"),
            ("R13", "47R 0.1%", "CH1_NODE", "GND",
             "bridge RV2: DUT voltage divider, bottom"),
            ("R14", "4R7", "RAIL_LO", "GND",
             "bridge RC1 a: current shunt (2x4R7 parallel = 2.35R)"),
            ("R15", "4R7", "RAIL_LO", "GND",
             "bridge RC1 b: current shunt pair"),
            ("R16", "47R 0.1%", "RAIL_LO", "CH2_NODE",
             "bridge RC2: current sense series"),
        ]
        for index, (reference, value, net_a, net_b, description) in enumerate(bridge):
            self.add("Device:R", reference, value, 330 + index * 18, 120,
                     "Resistor_SMD:R_0805_2012Metric", Description=description)
            self.net(reference, "1", net_a)
            self.net(reference, "2", net_b)

        # K23: CH1 source. De-energized (NC) CH1 sees the bridge divider as
        # on rev B; energized it sees the Kelvin front-end.
        self.add(RELAY_LIB, "K23", RELAY_PART, 640, 91, RELAY_FP,
                 Description="CH1 select: NC bridge divider, NO Kelvin front-end")
        self.net("K23", COIL_P, "+5V")
        self.net("K23", COIL_N, "K23_COIL")
        self.net("K23", P1_COM, "CH1_BNC")
        self.net("K23", P1_NC, "CH1_NODE")
        self.net("K23", P1_NO, "FE_OUT")
        self.unused("K23", P2_COM, P2_NC, P2_NO)

        # SWD. Rev A had SWDIO/SWCLK dangling and no USB, so a blank MCU could
        # never be programmed at all.
        self.add("Connector:Conn_ARM_JTAG_SWD_10", "J8", "SWD", 460, 90,
                 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
                 Description="Cortex Debug 10-pin (ST-LINK V3 / J-Link), next to U1")
        for pin, net_name in (("1", "+3V3"), ("2", "SWDIO"), ("3", "GND"),
                              ("4", "SWCLK"), ("5", "GND"), ("9", "GND"),
                              ("10", "NRST")):
            self.net("J8", pin, net_name)
        self.unused("J8", "6", "7", "8")

    def build_usb(self):
        self.add("Connector:USB_C_Receptacle_USB2.0_16P", "J1", "USB-C", 330, 150,
                 "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
                 Description="USB-C receptacle, USB 2.0 device")
        usb = {
            "A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND",
            "A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS",
            "A5": "CC1", "B5": "CC2",
            "A6": "USB_DP_CON", "B6": "USB_DP_CON",
            "A7": "USB_DM_CON", "B7": "USB_DM_CON",
            "S1": "USB_SHIELD",     # shield: KiCad 9 symbol pin name
            "SH": "USB_SHIELD",     # shield: KiCad 10 symbol pin name
        }
        for pin, net_name in usb.items():
            try:
                self.net("J1", pin, net_name)
            except KeyError:
                pass
        for pin in ("A8", "B8"):        # SBU1/SBU2, unused in USB 2.0
            try:
                self.unused("J1", pin)
            except Exception:
                pass

        for reference, value, net_name, x in (("R1", "5.1k", "CC1", 300), ("R2", "5.1k", "CC2", 315)):
            self.add("Device:R", reference, value, x, 185, "Resistor_SMD:R_0402_1005Metric",
                     Description="USB-C CC pull-down, sink advertisement")
            self.net(reference, "1", net_name)
            self.net(reference, "2", "GND")

        # Shield bonding chosen at the pre-compliance bench, not in a respin:
        # R37 0R (fitted) bonds the shell straight to the plane (consistent
        # with the unified ground); R36 1M || C31 4.7nF is the common
        # alternative (DC-isolated shield, RF-bonded).
        for reference, value, footprint, description in (
                ("R37", "0R", "Resistor_SMD:R_0603_1608Metric", "USB shield direct bond (fitted)"),
                ("R36", "1M", "Resistor_SMD:R_0603_1608Metric", "USB shield bleed (option, DNP)"),
                ("C31", "4.7nF 100V", "Capacitor_SMD:C_0603_1608Metric", "USB shield RF bond (option, DNP)")):
            self.add("Device:C" if reference.startswith("C") else "Device:R", reference, value,
                     420 + 15 * ("R37", "R36", "C31").index(reference), 210, footprint,
                     Description=description)
            self.net(reference, "1", "USB_SHIELD")
            self.net(reference, "2", "GND")
        # VBUS TVS at the connector (USBLC6's VBUS pin only partly covers it).
        self.add("Diode:SMAJ5.0A", "D9", "SMF5.0A", 470, 210, "Diode_SMD:D_SOD-123F",
                 Description="VBUS TVS, 200 W, SOD-123F; pad 1 = cathode")
        self.net("D9", "1", "VBUS")
        self.net("D9", "2", "GND")

        # ESD on the data pair. Rev A had none anywhere, on a bench instrument
        # that gets re-plugged dozens of times a day.
        self.add("Power_Protection:USBLC6-2P6", "D2", "USBLC6-2SC6", 390, 185,
                 "Package_TO_SOT_SMD:SOT-23-6",
                 Description="USB ESD protection, IEC 61000-4-2 level 4")
        self.net("D2", "1", "USB_DM_CON")
        self.net("D2", "2", "GND")
        self.net("D2", "3", "USB_DP_CON")
        self.net("D2", "4", "USB_DP_ESD")
        self.net("D2", "5", "VBUS")
        self.net("D2", "6", "USB_DM_ESD")
        # Common-mode choke: keeps board noise off the cable as CM current
        # (the main radiated-emission path of a bus-powered board).
        self.add("Device:Filter_EMI_CommonMode", "L1", "90R CM", 420, 185,
                 "Inductor_SMD:L_CommonModeChoke_Coilank_ACM2012",
                 Description="USB CM choke, 90R @100MHz (e.g. TDK ACM2012-900-2P)")
        # TDK ACM2012 windings are 1-4 and 2-3 (pads facing each other across
        # the body; TDK equivalent-circuit library), not 1-2 / 3-4: the first
        # netlist sent D+ out on the connector's D- line. 1-4 carries D+,
        # 2-3 carries D-; the footprint is turned 180 deg so D- (PA11) is on
        # the left at the MCU side and the pair runs to D2 without crossing.
        self.net("L1", "1", "USB_DP")
        self.net("L1", "4", "USB_DP_ESD")
        self.net("L1", "2", "USB_DM")
        self.net("L1", "3", "USB_DM_ESD")

    def build_power(self):
        # AP2112K: 600 mA LDO with 250 mV dropout, versus the AMS1117's ~1.1 V.
        # Worst-case coil load is 8 relays at 23 mA plus the MCU, and the LDO
        # must hold 3.3 V from a 4.40 V bus-powered-hub VBUS.
        # The MCU is powered from the cable side (+5V_IN); the relay coils,
        # drivers and front-end sit behind a current-limited, soft-starting
        # USB power switch (+5V) that the firmware enables only after USB
        # enumeration: inrush stays inside the 100 mA / 10 uF rules, a short
        # cannot take the host port down, and FLT reports it.
        self.add("Regulator_Linear:AP2112K-3.3", "U5", "AP2112K-3.3", 300, 240,
                 "Package_TO_SOT_SMD:SOT-23-5",
                 Description="3.3V LDO, 600mA, 250mV dropout")
        self.net("U5", "1", "+5V_IN")
        self.net("U5", "2", "GND")
        self.net("U5", "3", "+5V_IN")  # enable tied high
        self.add("Power_Management:TPS2051CDBV", "U11", "TPS2051CDBV", 340, 270,
                 "Package_TO_SOT_SMD:SOT-23-5",
                 Description="coil supply switch: soft start, current limit, fault flag")
        self.net("U11", "5", "+5V_IN")
        # 100 nF at the IN pin (TI: 0.1 uF close to IN); the +5V_IN bulk C1 is 12 mm away
        self.add("Device:C", "C37", "100nF", 340, 295, C0402,
                 Description="coil switch input decoupling at U11 IN")
        self.net("C37", "1", "+5V_IN")
        self.net("C37", "2", "GND")
        self.net("U11", "1", "+5V")
        self.net("U11", "2", "GND")
        self.net("U11", "4", "COIL_EN")
        self.net("U11", "3", "COIL_FLT")
        for reference, value, a, b, description in (
                ("R40", "100k", "COIL_EN", "GND", "coil supply off through reset/enumeration"),
                ("R41", "10k", "COIL_FLT", "+3V3", "fault flag pull-up")):
            self.add("Device:R", reference, value, 360 + 15 * ("R40", "R41").index(reference), 270,
                     "Resistor_SMD:R_0402_1005Metric", Description=description)
            self.net(reference, "1", a)
            self.net(reference, "2", b)
        # 10 uF per driver behind the switch: coil edges stay local
        for index, reference in enumerate(("C32", "C33", "C34", "C35")):
            self.add("Device:C", reference, "10uF", 250 + index * 14, 350,
                     "Capacitor_SMD:C_0805_2012Metric",
                     Description=f"coil supply bulk at driver {('U2', 'U3', 'U4', 'U6')[index]}")
            self.net(reference, "1", "+5V")
            self.net(reference, "2", "GND")
        self.net("U5", "5", "+3V3")
        self.unused("U5", "4")

        # VBUS is the +5V rail (a 5 V regulator would cost the coil headroom),
        # but through a power ferrite at the connector: relay-coil switching
        # edges stay on the board instead of riding out on the USB cable as
        # common-mode current, the dominant radiated-emission path.
        self.add("Device:FerriteBead", "FB1", "220R@100MHz 2A", 300, 210,
                 "Inductor_SMD:L_0805_2012Metric",
                 Description="VBUS EMI filter (e.g. Murata BLM21PG221SN1D)")
        self.net("FB1", "1", "VBUS")
        self.net("FB1", "2", "+5V_IN")

        # Bulk kept near the USB inrush limit (10 uF equivalent on VBUS).
        bulk = [("C1", "10uF", "+5V_IN", "Capacitor_SMD:C_0805_2012Metric"),
                ("C2", "4.7uF", "+3V3", "Capacitor_SMD:C_0805_2012Metric"),
                ("C3", "1uF", "+5V_IN", "Capacitor_SMD:C_0603_1608Metric"),
                ("C4", "1uF", "+3V3", "Capacitor_SMD:C_0603_1608Metric")]
        for index, (reference, value, rail, footprint) in enumerate(bulk):
            self.add("Device:C", reference, value, 250 + index * 14, 300, footprint,
                     Description=f"bulk decoupling, {rail}")
            self.net(reference, "1", rail)
            self.net(reference, "2", "GND")

        for index in range(5, 13):
            reference = f"C{index}"
            # C5-C7 + C12 decouple the +5V coil rail at the four drivers;
            # C8/C9 sit on the MCU's VDD pins 48/24; C10/C11 +5V distribution.
            rail = "+3V3" if index in (8, 9) else "+5V"
            self.add("Device:C", reference, "100nF", 250 + (index - 5) * 14, 325,
                     "Capacitor_SMD:C_0402_1005Metric",
                     Description=f"decoupling, {rail}")
            self.net(reference, "1", rail)
            self.net(reference, "2", "GND")

        # Status LEDs, each GPIO -> resistor -> LED -> GND (PWR is hardwired).
        # ~0.3 mA: readable, not distracting on a bench (rev B's 1k was).
        #   PWR    green  +3V3 present
        #   RUN    green  PC13: slow blink = running, no USB host; steady = host
        #   BUSY   amber  PB6: relays switching / settling
        #   KELVIN blue   PB7: Kelvin (4-wire) mode
        #   PROG   red    PC14: set just before the jump into the ROM DFU
        #                 bootloader; stays lit through a firmware update
        leds = [("D5", "R32", "PWR", "4.7k", "+3V3", "green"),
                ("D1", "R10", "RUN", "4.7k", "LED_RUN", "green"),
                ("D6", "R33", "BUSY", "4.7k", "LED_BUSY", "amber"),
                ("D7", "R34", "KELVIN", "1k", "LED_KELVIN", "blue (Vf ~2.9 V)"),
                ("D8", "R35", "PROG", "4.7k", "LED_PROG", "red")]
        for index, (led, resistor, label, value, drive, colour) in enumerate(leds):
            x = 460 + index * 20
            self.add("Device:LED", led, label, x, 240, "LED_SMD:LED_0603_1608Metric",
                     Description=f"status LED {label}, {colour}")
            self.net(led, "1", "GND")
            self.net(led, "2", f"{label}_A")
            self.add("Device:R", resistor, value, x, 215, "Resistor_SMD:R_0402_1005Metric",
                     Description=f"{label} LED current limit")
            self.net(resistor, "1", drive)
            self.net(resistor, "2", f"{label}_A")

        # +3V3 needs no flag: U5's VOUT is a power output and drives it.
        # +5V needs no flag: U11's OUT is a power output and drives it.
        for net_name, x in (("VBUS", 240), ("GND", 300),
                            ("+5VA", 320), ("+3V3A", 340), ("+5V_IN", 360)):
            self.power_flag(net_name, x, 355)

    def build_mcu(self):
        self.add("MCU_ST_STM32F0:STM32F072CBTx", "U1", "STM32F072CBT6", 420, 150,
                 "Package_QFP:LQFP-48_7x7mm_P0.5mm",
                 Description="Cortex-M0, 128K flash, native USB with crystal-driven 48 MHz")

        for name in ("VDD", "VDDIO2", "VBAT"):
            self.net("U1", MCU[name], "+3V3")
        # AN4325: VDDA filtered from VDD (ferrite + 1 uF + 10 nF at the pin).
        self.net("U1", MCU["VDDA"], "+3V3A")
        self.net("U1", "24", "+3V3")   # second VDD pin; 48 is the other
        for name in ("VSS", "VSSA"):
            self.net("U1", MCU[name], "GND")
        # VSS pins 23 and 35 are drawn stacked on pin 47 in this symbol, so the
        # single GND stub above already connects all three.

        self.net("U1", MCU["NRST"], "NRST")
        self.net("U1", MCU["BOOT0"], "BOOT0")
        self.net("U1", MCU["PA13"], "SWDIO")
        self.net("U1", MCU["PA14"], "SWCLK")
        self.net("U1", MCU["PA11"], "USB_DM")
        self.net("U1", MCU["PA12"], "USB_DP")
        # RUN on the south row too (PA9): every LED line leaves the MCU
        # westward with the coil-switch lines to the LED column south-west of
        # it; on PC13 (north row) it would cut across the relay lanes
        self.net("U1", MCU["PA9"], "LED_RUN")
        # south row, west to east: coil switch (U11 is south-west), then LEDs
        # (PROG off PA9/PA10/PA14/PA15: the ROM bootloader drives those)
        self.net("U1", MCU["PB12"], "COIL_EN")
        self.net("U1", MCU["PB13"], "COIL_FLT")
        self.net("U1", MCU["PB14"], "LED_PROG")
        self.net("U1", MCU["PB15"], "LED_KELVIN")
        self.net("U1", MCU["PA8"], "LED_BUSY")
        self.net("U1", MCU["PB10"], "I2C_SCL")         # I2C2 (AF1), west pins
        self.net("U1", MCU["PB11"], "I2C_SDA")

        # TMP117 (+/-0.1 C) beside the standards column: every calibration
        # and sweep records the fixture temperature (R5 25 ppm/K, contacts).
        self.add("Sensor_Temperature:TMP117xxDRV", "U12", "TMP117AIDRV", 560, 200,
                 "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm",
                 Description="fixture temperature, I2C 0x48")
        for pin, net_name in (("1", "I2C_SCL"), ("6", "I2C_SDA"), ("5", "+3V3"),
                              ("2", "GND"), ("4", "GND")):
            self.net("U12", pin, net_name)
        self.unused("U12", "3")
        for reference, a, b, value, description in (
                ("C36", "+3V3", "GND", "100nF", "TMP117 decoupling"),
                ("R38", "I2C_SCL", "+3V3", "4.7k", "I2C pull-up"),
                ("R39", "I2C_SDA", "+3V3", "4.7k", "I2C pull-up")):
            self.add("Device:C" if reference.startswith("C") else "Device:R", reference, value,
                     580 + 15 * ("C36", "R38", "R39").index(reference), 200,
                     "Capacitor_SMD:C_0402_1005Metric" if reference.startswith("C")
                     else "Resistor_SMD:R_0402_1005Metric", Description=description)
            self.net(reference, "1", a)
            self.net(reference, "2", b)

        # Bring-up / debug pads (no BOM cost)
        for index, net_name in enumerate(("+5V", "+5V_IN", "+3V3", "+5VA", "VMID", "FE_OUT", "GND")):
            reference = f"TP{3 + index}"
            self.add("Connector:TestPoint", reference, net_name, 560 + index * 12, 240,
                     "TestPoint:TestPoint_Pad_D1.5mm", Description=f"test pad {net_name}")
            self.net(reference, "1", net_name)

        for index, name in enumerate(GPIO_PINS, start=1):
            self.net("U1", MCU[name], f"GPIO{index}")

        used = {"VDD", "VDDA", "VDDIO2", "VBAT", "VSS", "VSSA", "NRST", "BOOT0",
                "PA13", "PA14", "PA11", "PA12", "PA9", "PB10", "PB11",
                "PB12", "PB13", "PB14", "PB15", "PA8", *GPIO_PINS}
        self.unused("U1", *[MCU[name] for name in MCU if name not in used])

        # No crystal: the F072 runs USB from HSI48 with CRS auto-trim off
        # the USB SOF -- proven silicon, and it removes a congested routing
        # cluster next to the MCU (PF1 stays free for future use).

        self.add("Device:R", "R3", "10k", 460, 120, "Resistor_SMD:R_0402_1005Metric",
                 Description="NRST pull-up")
        self.net("R3", "1", "+3V3")
        self.net("R3", "2", "NRST")
        self.add("Device:C", "C15", "100nF", 480, 120, "Capacitor_SMD:C_0402_1005Metric",
                 Description="NRST filter")
        self.net("C15", "1", "NRST")
        self.net("C15", "2", "GND")
        # AN4325 / datasheet: 100 nF on every VDD-type pin, at the pin, plus
        # bulk (C2, 4.7 uF). C8 sits in the NE corner notch and serves VDD 48
        # and VBAT 1 (adjacent +3V3 pins, VBAT tied to VDD); C9 VDD 24; C16
        # VDDIO2 36; C17 is extra +3V3 HF decoupling beside the bulk C2.
        for reference, pin_name, x in (("C16", "VDDIO2 (36)", 600), ("C17", "+3V3 bulk (HF)", 615)):
            self.add("Device:C", reference, "100nF", x, 120, C0402,
                     Description=f"MCU decoupling at {pin_name}")
            self.net(reference, "1", "+3V3")
            self.net(reference, "2", "GND")
        self.add("Device:FerriteBead", "FB3", "600R@100MHz", 630, 150, "Inductor_SMD:L_0603_1608Metric",
                 Description="VDDA filter")
        self.net("FB3", "1", "+3V3")
        self.net("FB3", "2", "+3V3A")
        for reference, value, x in (("C18", "1uF", 645), ("C19", "10nF", 660)):
            self.add("Device:C", reference, value, x, 150, C0402,
                     Description="VDDA decoupling at pin 9")
            self.net(reference, "1", "+3V3A")
            self.net(reference, "2", "GND")

        self.add("Device:R", "R4", "10k", 500, 120, "Resistor_SMD:R_0402_1005Metric",
                 Description="BOOT0 pull-down: boot from flash")
        self.net("R4", "1", "BOOT0")
        self.net("R4", "2", "GND")

        # A blank F072 does not fall back to the ROM bootloader on empty
        # flash, so a fresh board needs BOOT0 high at reset: hold SW1 while
        # plugging in (or hold SW1, tap SW2). R17 keeps a stuck switch from
        # shorting +3V3 straight onto BOOT0 and leaves R4 in charge when open.
        self.add("Switch:SW_Push", "SW1", "BOOT", 520, 120, "Button_Switch_SMD:SW_SPST_TL3342",
                 Description="hold at power-up/reset: ROM DFU bootloader")
        self.net("SW1", "1", "+3V3")
        self.net("SW1", "2", "BOOT_SW")
        self.add("Device:R", "R17", "1k", 540, 120, "Resistor_SMD:R_0402_1005Metric",
                 Description="BOOT0 button series resistor")
        self.net("R17", "1", "BOOT_SW")
        self.net("R17", "2", "BOOT0")
        self.add("Switch:SW_Push", "SW2", "RESET", 560, 120, "Button_Switch_SMD:SW_SPST_TL3342",
                 Description="MCU reset (NRST to GND)")
        self.net("SW2", "1", "NRST")
        self.net("SW2", "2", "GND")
        # No-BOM fallback for the strap: short TP1 to TP2 while plugging in.
        self.add("Connector:TestPoint", "TP1", "BOOT0", 520, 145, "TestPoint:TestPoint_Pad_1.5x1.5mm",
                 Description="BOOT0 strap pad")
        self.net("TP1", "1", "BOOT0")
        self.add("Connector:TestPoint", "TP2", "+3V3", 540, 145, "TestPoint:TestPoint_Pad_1.5x1.5mm",
                 Description="+3V3 strap pad")
        self.net("TP2", "1", "+3V3")

    def build_drivers(self):
        """Four TBD62003AFG (ULN2003 pinout) for 23 relays.

        DMOS, so about 0.1-0.2 V drop at 23 mA against the ULN2003's 0.8-1.0 V.
        With a 3.6 V must-operate coil and a 4.40 V worst-case bus-powered VBUS,
        the bipolar part leaves 3.40 V -- below spec. This one leaves ~4.2 V.
        COM (pin 9) is +5V: the internal clamp diodes return each coil's
        turn-off current locally instead of letting it ring on the supply.
        """
        assignments = [("U2", range(1, 8)), ("U3", range(8, 15)),
                       ("U4", range(15, 22)), ("U6", range(22, 24))]
        for index, (reference, relays) in enumerate(assignments):
            self.add("Transistor_Array:ULN2003", reference, "TBD62003APG",
                     330, 380 + index * 60, "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm",
                     Description="7ch DMOS relay driver, pin-compatible with ULN2003")
            self.net(reference, "8", "GND")
            self.net(reference, "9", "+5V")
            channels = list(relays)
            for channel, relay in enumerate(channels, start=1):
                self.net(reference, ULN_IN[channel], f"GPIO{relay}")
                self.net(reference, ULN_OUT[channel], f"K{relay}_COIL")
            for channel in range(len(channels) + 1, 8):
                self.unused(reference, ULN_IN[channel], ULN_OUT[channel])

    def build_frontend(self):
        """Kelvin front-end: (SNS_HI - SNS_LO) -> CH1, single +5VA supply.

        Two unity-gain input buffers, a 0.1 % four-resistor difference stage
        (CMRR ~54 dB from resistor match; the residual common-mode term is a
        fixed, calibrated offset impedance) and a buffered mid-rail. Inputs
        are AC-coupled (1 uF / 100 k: 1.6 Hz) and clamped after a 100 R
        series resistor; the output is AC-coupled into the Bode's 50 R input
        through 49.9 R. VMID = 1.8 V centres the signal in the OPA356's
        0..3.5 V input range: +/-1.6 V of sense signal before clipping.
        Gain and phase sit inside the Kelvin OSL, so they only need to be
        stable.
        """
        x0, y0 = 700, 160
        self.add("Device:FerriteBead", "FB2", "600R@100MHz", x0, y0 - 60,
                 "Inductor_SMD:L_0603_1608Metric", Description="+5VA filter for the front-end")
        self.net("FB2", "1", "+5V")
        self.net("FB2", "2", "+5VA")
        self.add("Device:C", "C29", "4.7uF", x0 + 20, y0 - 60, C0805,
                 Description="+5VA bulk")
        self.net("C29", "1", "+5VA")
        self.net("C29", "2", "GND")

        def opamp(reference, plus, minus, out, x, y, description):
            self.add(OPAMP, reference, "OPA356", x, y, SOT235, Description=description)
            self.net(reference, "3", plus)
            self.net(reference, "4", minus)
            self.net(reference, "1", out)
            self.net(reference, "5", "+5VA")
            self.net(reference, "2", "GND")

        # input channels
        for side, (cap, bias, series, clamp, amp, dec), y in (
                ("HI", ("C20", "R20", "R21", "D3", "U7", "C25"), y0),
                ("LO", ("C21", "R22", "R23", "D4", "U8", "C26"), y0 + 60)):
            tag = "P" if side == "HI" else "N"
            self.add("Device:C", cap, "1uF", x0, y, C0603,
                     Description=f"SNS_{side} AC coupling")
            self.net(cap, "1", f"SNS_{side}")
            self.net(cap, "2", f"FE_IN{tag}")
            self.add("Device:R", bias, "100k", x0 + 15, y + 15, R0402,
                     Description="input bias to VMID")
            self.net(bias, "1", f"FE_IN{tag}")
            self.net(bias, "2", "VMID")
            self.add("Device:R", series, "100R", x0 + 30, y, R0402,
                     Description="input protection series resistor")
            self.net(series, "1", f"FE_IN{tag}")
            self.net(series, "2", f"FE_{tag}")
            # BAV99 (Nexperia): 1 anode, 2 cathode, 3 common
            self.add("Diode:BAV99", clamp, "BAV99", x0 + 45, y + 20,
                     "Package_TO_SOT_SMD:SOT-23", Description="input clamp to the rails")
            self.net(clamp, "1", "GND")
            self.net(clamp, "2", "+5VA")
            self.net(clamp, "3", f"FE_{tag}")
            opamp(amp, f"FE_{tag}", f"FE_B{tag}", f"FE_B{tag}", x0 + 70, y,
                  f"SNS_{side} unity buffer")
            self.add("Device:C", dec, "100nF", x0 + 90, y + 15, C0402,
                     Description=f"{amp} supply decoupling")
            self.net(dec, "1", "+5VA")
            self.net(dec, "2", "GND")

        # difference stage: out = VMID + (SNS_HI - SNS_LO)
        xd, yd = x0 + 130, y0 + 30
        for reference, a, b, description in (
                ("R24", "FE_BN", "FE_DM", "diff stage input, 0.1%"),
                ("R25", "FE_DM", "FE_AMP", "diff stage feedback, 0.1%"),
                ("R26", "FE_BP", "FE_DP", "diff stage input, 0.1%"),
                ("R27", "FE_DP", "VMID", "diff stage reference, 0.1%")):
            index = int(reference[1:]) - 24
            self.add("Device:R", reference, "1k 0.1%", xd - 30, yd - 30 + index * 15, R0603,
                     Description=description)
            self.net(reference, "1", a)
            self.net(reference, "2", b)
        opamp("U9", "FE_DP", "FE_DM", "FE_AMP", xd, yd, "difference stage")
        self.add("Device:C", "C27", "100nF", xd + 20, yd + 15, C0402,
                 Description="U9 supply decoupling")
        self.net("C27", "1", "+5VA")
        self.net("C27", "2", "GND")
        self.add("Device:R", "R28", "49.9R", xd + 40, yd, R0402,
                 Description="back-termination into the Bode 50 R input")
        self.net("R28", "1", "FE_AMP")
        self.net("R28", "2", "FE_R")
        # 47 uF: 34 Hz corner into the Bode's 50 R; Kelvin mode sets CH1 to
        # 1 Mohm, which moves it to millihertz (simulate_frontend.py).
        self.add("Device:C", "C22", "47uF", xd + 55, yd, "Capacitor_SMD:C_1206_3216Metric",
                 Description="output DC block, X5R 10 V")
        self.net("C22", "1", "FE_R")
        self.net("C22", "2", "FE_OUT")
        self.add("Device:R", "R29", "100k", xd + 70, yd + 15, R0402,
                 Description="FE_OUT bleed")
        self.net("R29", "1", "FE_OUT")
        self.net("R29", "2", "GND")

        # mid-rail
        xm, ym = x0 + 130, y0 + 110
        self.add("Device:R", "R30", "18k", xm - 30, ym - 10, R0402, Description="VMID divider top")
        self.net("R30", "1", "+5VA")
        self.net("R30", "2", "VMID_DIV")
        self.add("Device:R", "R31", "10k", xm - 30, ym + 10, R0402, Description="VMID divider bottom")
        self.net("R31", "1", "VMID_DIV")
        self.net("R31", "2", "GND")
        self.add("Device:C", "C23", "10uF", xm - 15, ym + 20, C0805, Description="VMID divider filter")
        self.net("C23", "1", "VMID_DIV")
        self.net("C23", "2", "GND")
        opamp("U10", "VMID_DIV", "VMID", "VMID", xm, ym, "VMID buffer")
        self.add("Device:C", "C28", "100nF", xm + 20, ym + 15, C0402,
                 Description="U10 supply decoupling")
        self.net("C28", "1", "+5VA")
        self.net("C28", "2", "GND")

    def build(self):
        self.build_matrix()
        self.build_load_column()
        self.build_isolation()
        self.build_links_and_shorts()
        self.build_connectors()
        self.build_usb()
        self.build_power()
        self.build_mcu()
        self.build_drivers()
        self.build_frontend()
        return self

    def save(self):
        self.sch.save(SCHEMATIC)
        return SCHEMATIC

    def audit(self):
        """Report nets with a single connection -- always a wiring mistake."""
        singles = {name: uses for name, uses in self.net_uses.items() if len(uses) < 2}
        return singles


def run_erc(path):
    report = path.replace(".kicad_sch", "_erc.txt")
    result = subprocess.run([KICAD_CLI, "sch", "erc", "--format", "report",
                             "--severity-all", "-o", report, path],
                            capture_output=True, text=True)
    print(result.stdout.strip() or result.stderr.strip())
    if os.path.exists(report):
        counts = {}
        for line in open(report, encoding="utf-8", errors="replace"):
            if line.startswith("["):
                key = line.split("]")[0][1:]
                counts[key] = counts.get(key, 0) + 1
        for key, count in sorted(counts.items(), key=lambda kv: -kv[1]):
            print(f"    {count:4d}  {key}")
        return counts
    return {}


def export_netlist(path):
    destination = path.replace(".kicad_sch", ".xml")
    result = subprocess.run([KICAD_CLI, "sch", "export", "netlist", "--format",
                             "kicadxml", "-o", destination, path],
                            capture_output=True, text=True)
    if result.returncode:
        print(result.stderr.strip())
    return destination


if __name__ == "__main__":
    builder = Builder().build()
    path = builder.save()
    print(f"Schematic: {path}")
    statistics = builder.sch.get_statistics()
    print(f"  components {statistics.get('components')}  labels {statistics.get('labels')}  "
          f"wires {statistics.get('wires')}")

    singles = builder.audit()
    if singles:
        print(f"\n  {len(singles)} single-connection nets (each is a wiring bug):")
        for name, uses in sorted(singles.items()):
            print(f"    {name}: {uses}")
    else:
        print("  every net has at least two connections")

    print("\nERC:")
    counts = run_erc(path)
    print(f"\nNetlist: {export_netlist(path)}")
    sys.exit(1 if counts.get("error") else 0)
