"""
Generate the rev C PCB: 4 layers, Kelvin crossbar, placement + locked
pre-routes for the measurement-critical copper.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" generate_pcb.py

See ARCHITECTURE.md for the circuit and the review it answers. Layout rules
(sources in TODO.md / ARCHITECTURE.md):

STACKUP (JLC04161H-7628, 1.6 mm, written into the board file)
    L1 F.Cu   35 um   parts, arms, short local measurement nets
              7628 prepreg 0.2104 mm, er 4.4
    L2 In1.Cu 15 um   GND plane; inside the measurement void: locked routing
                      only (+5V coil tree, hand-routed measurement nets)
              core 1.065 mm, er 4.6
    L3 In2.Cu 15 um   GND plane; inside the void: GND guard strips under the
                      rails, locked routing elsewhere
              7628 prepreg 0.2104 mm, er 4.4
    L4 B.Cu   35 um   routing; the FORCE and SENSE rails side by side, 0.21 mm
                      over their In2 guard strip

    SIG / GND / GND / SIG: every signal layer is 0.21 mm from a solid GND
    plane outside the void (the standard EMC 4-layer stack for a USB-FS +
    relay board; 6 layers bought no routing room inside the void, where the
    planes are voided anyway). +5V is routed, not a plane.

GROUND
    One unified ground (Ott: no RF potential between the USB shield and the
    BNC shells). Planes are continuous under the bridge band, the BNC
    launches, the USB entry and the control strip; all four cable connectors
    sit on the bottom edge. The measurement area (bay, iso/short/link relays,
    buses, matrix, rails) has no plane on any layer: column buses and DUT
    nodes sit at intermediate potentials, where ground capacitance is a
    DUT-dependent error, not a guard.

FLOORPLAN (top view, bottom edge = cable edge)

    K15 lnkAB          K11 isoA [A>   DUT BAY   <C] isoC K13           K16 lnkCD | drivers
                       K12 isoB [B>             <D] isoD K14                     | U2 U3
       K1 A-HI      K3 B-HI      K9 E-HI      K5 C-HI      K7 D-HI              | U4 U6
  K22  K2 A-LO      K4 B-LO      K10 E-LO     K6 C-LO      K8 D-LO   |LO feed   |
  tie        front-end     K19 sel  R5 R18 C30  K17 lnkBC  K18 lnkBD  |         | MCU  SWD
      K23 R12/13           K20 sel  K21 sel                 shunt               BOOT
  [SOURCE]  [CH1]                                       [CH2]      [USB-C]   RESET

    HI is fed from the west end of its rail (SOURCE), LO from the east end
    (shunt + CH2): every column sees the same HI + LO rail length.

PRE-ROUTED (locked): the Kelvin arms (force F.Cu, sense B.Cu), the four rails
and their taps, both rail feeds, the +5V coil tree. Everything else is
freerouted by route_pcb.py with the JLC rules below.
"""

import json
import math
import os
import xml.etree.ElementTree as ET

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
NETLIST = os.path.join(HERE, "relay_board.xml")
OUTPUT = os.path.join(HERE, "relay_board.kicad_pcb")
#: UUIDs of the generator's locked pre-routes (arms, rails, coil tree, USB
#: entry, GND fanout, stitching): protected from rip-up in maze_route.py.
PREROUTED = os.path.join(HERE, "relay_board_prerouted.json")
#: mm outward from a fine-pitch package's pin toes kept free of other parts'
#: GND vias: the pins' escape band (maze_route.escape_fine_pitch)
ESCAPE_BAND = 2.2
FP_SYSTEM = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"
FP_PROJECT = HERE                       # OpenMagnetics.pretty lives here


def mm(value):
    return pcbnew.FromMM(value)


def vec(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


# ------------------------------------------------------------ JLC rules
#: JLCPCB 6-layer capability with margin (see TODO.md "JLC rules").
JLC = {
    "track_min": 0.15, "clearance": 0.18, "via_drill": 0.3, "via_diameter": 0.6,
    "via_min_drill": 0.2, "via_min_diameter": 0.45, "annular_min": 0.1,
    "hole_to_hole": 0.25, "edge_clearance": 0.5, "silk_line": 0.15,
    "silk_text": 1.0, "part_to_edge": 2.5,
}
SIGNAL_TRACK = 0.5                      # measurement nets
RAIL_TRACK = 0.6                        # force rails and feeds (over a guard strip)
SENSE_TRACK = 0.6                       # sense rails
PLAIN_TRACK = 0.25

# ------------------------------------------------------------- geometry
BOARD_X0, BOARD_Y0, BOARD_X1, BOARD_Y1 = 40.0, 40.0, 214.0, 150.0
#: Everything below the DUT bay sits this much lower than in the first rev C
#: floorplan: the Kelvin banana-jack bay is taller than the spring clamps.
DY = 10.0

#: Terminal columns, 22 mm apart; E is the load-standard column. Pole-1
#: (force) pads land on the column line, pole-2 (sense) pads 7 mm east.
#: East mirrors west: the outer iso relay (K11 / K13) sits over the outer column
#: (A / C), the inner one (K12 / K14) beside the inner column (B / D), so no
#: terminal bus has to cross another on its way down (C and D used to).
COLUMNS = {"A": 61.0, "B": 83.0, "E": 105.0, "D": 127.0, "C": 149.0}
RELAY_DX = 3.5
ROW_HI, ROW_LO = 84.0, 96.0
PAD_NC, PAD_COM, PAD_NO = -0.6, 1.6, 3.8
POLE2_DX = 7.0

#: Force rails on B.Cu under the NO-pad rows; sense rails on B.Cu too,
#: 1.5 mm south (0.9 mm gap), each tapped by its own vias; both run over
#: the same In2 guard strip.
RAIL_Y = {"HI": ROW_HI + PAD_NO, "LO": ROW_LO + PAD_NO}
SENSE_Y = {"HI": ROW_HI + PAD_NO + 1.5, "LO": ROW_LO + PAD_NO + 1.5}
TAP_DX = 1.8
HI_FEED_X = 56.0                       # west of column A: SOURCE BNC below
LO_FEED_X = 160.0                      # east of column D: shunt + CH2 below

#: DUT bay: no copper on any layer.
#: 41 mm wide: the clip pairs and their iso relays sit as far out as the link
#: relays K15/K16 allow (the bay height is set by the board edge and the matrix).
BAY = (84.5, 42.5, 125.5, 77.0)
#: DUT interface: a Kelvin PAIR of Grayhill 02-21 miniature test clips per
#: terminal, their banana plugs in plated 4.0 mm holes: F (inner, toward the
#: DUT) and S (outer), JACK_PITCH apart so both grip the same point of a short
#: lead (it passes through the F clip into the S clip). The clip is 5.54 mm
#: thick along the lead and 10.31 mm wide across it: 8 mm leaves 2.5 mm
#: between the two clips of a pair, the 24 mm rows 13.7 mm between terminals.
JACK_PITCH = 8.0
JACK_ROW_A, JACK_ROW_B = 47.5, 71.5
JACK_F_WEST, JACK_S_WEST = 78.0, 78.0 - JACK_PITCH
JACK_F_EAST, JACK_S_EAST = 132.0, 132.0 + JACK_PITCH
#: a Kelvin arm that would run over its pair's other hole first steps this far
#: toward the relay row (clear of the 6.5 mm pads)
ARM_JOG = 5.5
ISO_ROW = 59.5                       # iso relays sit between the jack rows

#: Bottom (cable) edge.
BNC_Y = BOARD_Y1 - 12.7
BNC_X = {"SOURCE": 56.0, "CH1": 76.0, "CH2": 150.0}
USB_X = 179.0

#: Measurement void: no plane on any layer (polygon, clockwise).
VOID = [(43.0, 43.0), (165.0, 43.0), (165.0, 119.0), (109.0, 119.0),
        (109.0, 107.5), (43.0, 107.5)]
#: Control strip (east of the measurement area).
STRIP_X0 = 166.0
#: GND guard strips inside the void, only under copper that always sits at
#: the HI or LO potential (the rails and their feeds): their capacitance to
#: ground is then inside every OSL, and the bridge current gets a return
#: path right under it (Keysight guard principle; Faraday return-path rule).
#: Buses, links and arms keep no plane: a linked terminal floats at an
#: intermediate potential, where ground capacitance is an uncalibrated error.
GUARD_HALF_WIDTH = 1.5
#: +5V coil-feed trunks on In1 inside the void.
TRUNK_TOP_Y, TRUNK_BOTTOM_Y = 78.2, 106.0

PLACEMENT = {
    # --- links: same-side pairs next to their iso relays, cross-bay pairs
    #     in the row under the matrix ---
    "K15": (48.5, 55.0, 0), "K16": (162.5, 55.0, 0),
    "K18": (139.0, 102.0, 0), "K17": (151.0, 102.0, 0),   # B-D under column D, B-C under column C
    # --- standards column: K9/K10 to the rails, K19..K21 pick the standard;
    #     each standard Kelvin-sensed at its pads through a net tie ---
    "K9": (COLUMNS["E"] + RELAY_DX, ROW_HI, 0), "K10": (COLUMNS["E"] + RELAY_DX, ROW_LO, 0),
    "K19": (117.0, 102.0, 0), "K20": (117.0, 114.0, 0), "K21": (129.0, 114.0, 0),
    # standards and their Kelvin net ties: spread so each standard's force and
    # sense pair has its own channel (the first, tighter cluster could not be
    # routed without boxing a net-tie pad in)
    "R5": (124.1, 99.4, 90), "R18": (124.1, 105.2, 90), "C30": (129.0, 99.4, 90),
    "NT1": (126.7, 97.4, 90), "NT4": (126.7, 103.2, 90), "NT5": (131.4, 97.4, 90),
    "NT2": (126.7, 107.2, 90),
    # --- 2-wire tie and CH1 select ---
    "K22": (48.5, 90.0, 0),
    "K23": (BNC_X["CH1"] + RELAY_DX, 113.0, 0),
    # --- bridge (B-WIC): divider at the HI feed, shunt at the LO feed ---
    "J6": (BNC_X["SOURCE"], BNC_Y, 180), "J7": (BNC_X["CH1"], BNC_Y, 180),
    "J9": (BNC_X["CH2"], BNC_Y, 180),
    "R12": (62.0, 112.0, 90), "R13": (65.0, 112.0, 90),
    "R14": (155.0, 114.0, 0), "R15": (155.0, 117.0, 0), "R16": (151.5, 120.5, 90),
    # --- Kelvin front-end (bottom band, between CH1 and the load column) ---
    "C20": (88.0, 100.5, 90), "R21": (90.6, 100.0, 0), "R20": (88.0, 103.6, 90),
    "D3": (91.6, 103.4, 0), "U7": (95.2, 100.6, 0), "C25": (98.6, 99.0, 90),
    "C21": (88.0, 108.5, 90), "R23": (90.6, 108.0, 0), "R22": (88.0, 111.6, 90),
    "D4": (91.6, 111.4, 0), "U8": (95.2, 108.6, 0), "C26": (98.6, 107.0, 90),
    "R26": (99.5, 103.0, 0), "R27": (99.5, 104.6, 0),
    "R24": (99.5, 111.0, 0), "R25": (99.5, 112.6, 0),
    "U9": (103.0, 108.0, 0), "C27": (106.3, 106.4, 90),
    "R28": (103.0, 112.0, 0), "C22": (103.6, 115.2, 0), "R29": (106.6, 114.4, 90),
    "R30": (91.0, 116.2, 0), "R31": (91.0, 117.8, 0), "C23": (94.2, 119.4, 0),
    "U10": (95.2, 116.6, 0), "C28": (98.6, 115.2, 90),
    "FB2": (102.0, 118.8, 0), "C29": (105.6, 118.8, 0),
    # ================= control strip =================
    # USB entry on the cable edge: connector, ESD, CM choke in a straight
    # line up to the MCU's USB pins (rotated MCU: PA11/PA12 face south).
    "J1": (USB_X, BOARD_Y1 - 3.6, 0),
    "D2": (USB_X, 129.3, 0), "L1": (USB_X, 125.4, 270),
    "R1": (USB_X - 4.6, 129.0, 90), "R2": (USB_X + 4.6, 129.0, 90),
    "FB1": (USB_X - 7.6, 130.6, 90), "C1": (USB_X - 10.4, 130.6, 90),
    "U5": (USB_X + 9.0, 127.4, 0), "C3": (USB_X + 9.0, 131.4, 0),
    "C4": (USB_X + 12.6, 127.4, 90),
    "U1": (USB_X, 110.0, -90),
    # AN4325: every supply pin's 100 nF on top, at the pin, own GND via.
    # VDD 48 and VBAT 1 are adjacent +3V3 pins around the NE corner: one
    # 100 nF in the corner notch, +3V3 pad at both pins, GND pad outward,
    # decouples both (AN4325, VBAT tied to VDD) and leaves the corner where
    # the relay lanes turn free
    "C8": (USB_X + 3.75, 105.55, 45),
    # VDD 24; 1.85 mm south of the pin row so the I2C pins above it (21, 22)
    # keep their outward escapes (beside the row it forced SDA under the MCU)
    "C9": (USB_X - 6.6, 114.6, 0),
    "C16": (USB_X + 2.6, 116.4, 0),         # VDDIO2 36
    # North side (the row facing the drivers) keeps its relay pins' lanes
    # free: the VDDA filter stacks vertically over VSSA/VDDA (pins 8/9) only,
    # VBAT's cap sits in the NE corner beside pin 1, NRST's R/C go north by
    # the driver bank (NRST is slow: a via to cross the relay lanes is fine),
    # the +3V3 bulk sits east of the lane fan.
    "C19": (USB_X - 1.25, 103.4, 90),       # VDDA: 10 nF over pin 9
    "C18": (USB_X - 1.25, 100.8, 90),       # VDDA 1 uF
    "FB3": (USB_X - 1.25, 98.2, 90),
    "C17": (USB_X + 14.6, 101.0, 90),       # extra +3V3 HF, beside the bulk C2
    "C15": (USB_X - 0.25, 103.4, 90),       # NRST, over pin 7
    "R3": (USB_X - 0.25, 100.8, 90),
    "C2": (USB_X + 13.0, 101.0, 90),        # +3V3 bulk, east of the lanes
    # BOOT0 strap parts east of the relay corridor (the east pins' relay
    # lines run north at x < USB_X + 11), by SW1/J8 they connect to: a static
    # strap, a few mm of track cost nothing; the pins and the corridor need room
    # BOOT0 strap resistors beside the BOOT button, out of the east lane fan
    # (between the MCU and the SWD header the relay lanes boxed them in)
    "R4": (USB_X + 19.0, 122.0, 90), "R17": (USB_X + 20.6, 122.0, 90),
    # Cortex debug 2x5, at the MCU; turned 180 deg so the SWDIO/SWCLK column
    # faces the MCU: on the far column they looped north around the header,
    # through the east relay-lane fan
    "J8": (USB_X + 14.0, 115.4, 180),
    "SW1": (205.0, 128.0, 90), "SW2": (205.0, 115.0, 90),
    "TP1": (198.0, 133.6, 0), "TP2": (200.6, 133.6, 0),
    # status LEDs in a column at the east edge, resistors beside them
    # Status LEDs in a column south-west of the MCU: their lines leave the
    # south row westward with the coil-switch lines (U11), in lane order --
    # PROG, KELVIN, BUSY, RUN top to bottom as pins PB14, PB15, PA8, PA9 run
    # west to east -- resistors on the east side facing the incoming lines.
    # series resistors turned 180: LED-side pad toward the LED, MCU-side pad east
    "D8": (160.2, 120.4, 0), "R35": (163.0, 120.4, 180),  # PROG
    "D7": (160.2, 122.4, 0), "R34": (163.0, 122.4, 180),  # KELVIN
    "D6": (160.2, 124.4, 0), "R33": (163.0, 124.4, 180),  # BUSY
    "D1": (160.2, 126.4, 0), "R10": (163.0, 126.4, 180),  # RUN
    "D5": (160.2, 128.4, 0), "R32": (163.0, 128.4, 180),  # PWR
    # drivers above the MCU, outputs west toward the relays
    "U2": (USB_X, 50.0, 180), "C5": (USB_X - 2.5, 43.5, 0),
    "U3": (USB_X, 63.0, 180), "C6": (USB_X - 2.5, 56.5, 0),
    "U4": (USB_X, 76.0, 180), "C7": (USB_X - 2.5, 69.5, 0),
    "U6": (USB_X, 89.0, 180), "C12": (USB_X - 2.5, 82.5, 0),
    "C10": (USB_X + 6.0, 56.5, 90), "C11": (USB_X + 6.0, 82.5, 90),
}

#: Entries placed relative to the board edge or the rows stay put.
_ANCHORED = {"J1", "J6", "J7", "J9", "K9", "K10"}
for _reference, (_x, _y, _r) in list(PLACEMENT.items()):
    if _y >= 64.0 and _reference not in _ANCHORED:
        PLACEMENT[_reference] = (_x, _y + DY, _r)

PLACEMENT.update({
    # --- DUT bay: Kelvin jack pairs (F inner, S outer) ---
    "J10": (JACK_F_WEST, JACK_ROW_A, 0), "J11": (JACK_S_WEST, JACK_ROW_A, 0),   # A F/S
    "J12": (JACK_F_WEST, JACK_ROW_B, 0), "J13": (JACK_S_WEST, JACK_ROW_B, 0),   # B F/S
    "J14": (JACK_F_EAST, JACK_ROW_A, 0), "J15": (JACK_S_EAST, JACK_ROW_A, 0),   # C F/S
    "J16": (JACK_F_EAST, JACK_ROW_B, 0), "J17": (JACK_S_EAST, JACK_ROW_B, 0),   # D F/S
    # --- isolation relays between the jack rows, poles toward their jacks ---
    "K11": (61.5, ISO_ROW, 180), "K12": (75.5, ISO_ROW, 180),
    "K13": (148.5, ISO_ROW, 0), "K14": (134.5, ISO_ROW, 0),
    # --- coil supply switch, VBUS TVS, USB shield option (cable edge) ---
    "U11": (169.0, 129.5, 90), "R40": (166.5, 127.6, 90), "R41": (166.6, 131.2, 90),
    "C37": (168.05, 126.5, 90),             # 100 nF at U11's IN pin (pin 5, just south of it)
    # VBUS TVS on the VBUS run from the receptacle to FB1 (pad 1 sits on that track)
    # (beside FB1 it hung 10 mm of track away: ~240 V of overshoot before it acts)
    "D9": (171.2, 143.7, 180),
    "R36": (187.0, 144.6, 90), "R37": (188.6, 144.6, 90), "C31": (190.2, 144.6, 90),
    # --- 10 uF per driver, at its COM pin side ---
    "C32": (172.6, 46.5, 90), "C33": (172.6, 59.5, 90),
    "C34": (172.6, 82.5, 90), "C35": (172.6, 95.5, 90),
    # --- fixture temperature beside the standards column ---
    "U12": (140.5, 124.0, 0), "C36": (143.4, 124.0, 90),
    "R38": (168.6, 116.5, 90), "R39": (167.2, 116.5, 90),
})
# bring-up / debug pads, each beside the circuit it probes: a probe point must
# not drag its net across the board (a 100 mm FE_OUT stub to an edge row was a
# pickup antenna on the measurement output and walled the MCU off from the
# drivers). TP3 +5V, TP4 +5V_IN, TP5 +3V3, TP6 +5VA, TP7 VMID, TP8 FE_OUT, TP9 GND.
#: test pad -> (anchor part, offset from its final position mm, silk label)
_TEST_PAD_ANCHORS = {"TP3": ("U11", -1.0, 5.0, "5V"), "TP4": ("FB1", -3.0, -3.0, "5VIN"),
                     "TP5": ("U5", 4.0, -4.0, "3V3"), "TP6": ("C29", 3.0, 0.0, "5VA"),
                     "TP7": ("R31", -3.0, 0.0, "VMID"), "TP8": ("R29", 3.0, 0.0, "FE"),
                     "TP9": (None, 191.5, 98.0, "GND")}
TEST_PADS = {}
for _reference, (_anchor, _dx, _dy, _label) in _TEST_PAD_ANCHORS.items():
    _x0, _y0 = (0.0, 0.0) if _anchor is None else PLACEMENT[_anchor][:2]
    TEST_PADS[_reference] = ((_x0 + _dx, _y0 + _dy), _label)
    PLACEMENT[_reference] = (_x0 + _dx, _y0 + _dy, 0)

# Matrix relays K1..K8: (terminal, rail) in RelayMapRevC order.
for _index, (_terminal, _rail) in enumerate(
        [(t, r) for t in "ABCD" for r in ("HI", "LO")], start=1):
    PLACEMENT[f"K{_index}"] = (COLUMNS[_terminal] + RELAY_DX,
                               ROW_HI if _rail == "HI" else ROW_LO, 0)

#: Plated mounting holes bonded to GND (with via rings): an enclosure lid
#: bonds to the ground plane at every screw, the bottom row at the cable edge.
MECHANICAL = [
    ("H1", "MountingHole", "MountingHole_3.2mm_M3_Pad_Via", 44.5, 44.5),
    ("H2", "MountingHole", "MountingHole_3.2mm_M3_Pad_Via", 209.5, 44.5),
    ("H3", "MountingHole", "MountingHole_3.2mm_M3_Pad_Via", 44.5, 126.0),
    ("H4", "MountingHole", "MountingHole_3.2mm_M3_Pad_Via", 209.5, BOARD_Y1 - 4.5),
    ("H5", "MountingHole", "MountingHole_3.2mm_M3_Pad_Via", 118.0, BOARD_Y1 - 4.5),
    ("FID1", "Fiducial", "Fiducial_1mm_Mask2mm", 44.0, 76.0),
    ("FID2", "Fiducial", "Fiducial_1mm_Mask2mm", 196.0, 44.0),
    ("FID3", "Fiducial", "Fiducial_1mm_Mask2mm", 130.0, BOARD_Y1 - 4.0),
]


def zone_layer(zone, layer):
    """ZONE.SetLayer silently no-ops in KiCad's Python API; use a layer set."""
    layer_set = pcbnew.LSET()
    layer_set.AddLayer(layer)
    zone.SetLayerSet(layer_set)


def point_in_polygon(x, y, polygon):
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def load_netlist():
    root = ET.parse(NETLIST).getroot()
    footprints = {}
    for comp in root.find(".//components"):
        node = comp.find("footprint")
        footprints[comp.get("ref")] = node.text if node is not None else None
    pin_nets, net_names = {}, []
    for net in root.find(".//nets"):
        net_names.append(net.get("name"))
        for node in net.findall("node"):
            pin_nets[(node.get("ref"), node.get("pin"))] = net.get("name")
    return footprints, pin_nets, net_names


def load_footprint(footprint_id):
    library, name = footprint_id.split(":")
    for base in (FP_PROJECT, FP_SYSTEM):
        path = os.path.join(base, library + ".pretty")
        if os.path.isdir(path):
            footprint = pcbnew.FootprintLoad(path, name)
            if footprint:
                return footprint
    raise RuntimeError(f"footprint {footprint_id} not found")


class Board:
    def __init__(self):
        footprints, self.pin_nets, net_names = load_netlist()
        self.footprint_ids = footprints
        self.board = pcbnew.BOARD()
        self.board.SetCopperLayerCount(4)
        settings = self.board.GetDesignSettings()
        settings.SetBoardThickness(mm(1.6))
        settings.m_CopperEdgeClearance = mm(JLC["edge_clearance"])
        settings.m_TrackMinWidth = mm(JLC["track_min"])
        settings.m_MinClearance = mm(JLC["clearance"])
        settings.m_ViasMinSize = mm(JLC["via_min_diameter"])
        settings.m_MinThroughDrill = mm(JLC["via_min_drill"])
        settings.m_ViasMinAnnularWidth = mm(JLC["annular_min"])
        settings.m_HoleToHoleMin = mm(JLC["hole_to_hole"])
        settings.m_MinSilkTextHeight = mm(JLC["silk_text"])
        try:
            netclass = settings.m_NetSettings.GetDefaultNetclass()
            netclass.SetClearance(mm(0.2))
            netclass.SetTrackWidth(mm(0.25))
            netclass.SetViaDiameter(mm(JLC["via_diameter"]))
            netclass.SetViaDrill(mm(JLC["via_drill"]))
        except Exception as error:          # API differences across versions
            print(f"  (default netclass not set: {error})")
        self.nets = {}
        for index, name in enumerate(sorted(net_names), start=1):
            net = pcbnew.NETINFO_ITEM(self.board, name, index)
            self.board.Add(net)
            self.nets[name] = net

    def net(self, name):
        if name not in self.nets:
            raise KeyError(f"net {name} not in netlist")
        return self.nets[name]

    def footprint(self, reference):
        for footprint in self.board.GetFootprints():
            if footprint.GetReference() == reference:
                return footprint
        raise KeyError(reference)

    def pad_position(self, reference, pad_number):
        for pad in self.footprint(reference).Pads():
            if pad.GetNumber() == str(pad_number):
                position = pad.GetPosition()
                return pcbnew.ToMM(position.x), pcbnew.ToMM(position.y)
        raise KeyError(f"{reference}.{pad_number} not found")

    @staticmethod
    def chamfered(points, size=0.8):
        """Replace each right-angle corner by a 45-degree chamfer (tidiness and
        manufacturability; electrically irrelevant below several GHz)."""
        out = [points[0]]
        for (x0, y0), (x1, y1), (x2, y2) in zip(points, points[1:], points[2:]):
            a = (x1 - x0, y1 - y0)
            b = (x2 - x1, y2 - y1)
            la, lb = math.hypot(*a), math.hypot(*b)
            right_angle = la > 1e-6 and lb > 1e-6 and abs(a[0] * b[0] + a[1] * b[1]) < 1e-6
            c = min(size, la / 3.0, lb / 3.0) if right_angle else 0.0
            if c > 0.05:
                out.append((x1 - a[0] / la * c, y1 - a[1] / la * c))
                out.append((x1 + b[0] / lb * c, y1 + b[1] / lb * c))
            else:
                out.append((x1, y1))
        out.append(points[-1])
        return out

    def track(self, points, net_name, layer=pcbnew.F_Cu, width=SIGNAL_TRACK,
              locked=True, chamfer=True):
        points = self.chamfered(points) if chamfer and len(points) > 2 else points
        for (x1, y1), (x2, y2) in zip(points, points[1:]):
            if abs(x1 - x2) < 1e-6 and abs(y1 - y2) < 1e-6:
                continue
            track = pcbnew.PCB_TRACK(self.board)
            track.SetStart(vec(x1, y1))
            track.SetEnd(vec(x2, y2))
            track.SetNet(self.net(net_name))
            track.SetLayer(layer)
            track.SetWidth(mm(width))
            track.SetLocked(locked)
            self.board.Add(track)

    def spot_free(self, x, y, radius, net_name, hole_gap=0.85):
        """A via of `radius` at (x, y) clears other-net copper by 0.25 mm and
        every existing via (any net) by `hole_gap` centre to centre."""
        code = self.net(net_name).GetNetCode()
        position = vec(x, y)
        for item in self.board.GetTracks():
            if item.GetClass() == "PCB_VIA":
                q = item.GetPosition()
                if math.hypot(pcbnew.ToMM(q.x) - x, pcbnew.ToMM(q.y) - y) < hole_gap:
                    return False
            if item.GetNetCode() != code and item.HitTest(position, mm(radius + 0.25)):
                return False
        for footprint in self.board.GetFootprints():
            for pad in footprint.Pads():
                if pad.GetNetCode() != code and pad.HitTest(position, mm(radius + 0.25)):
                    return False
        return True

    def via(self, x, y, net_name, locked=True, size=None, drill=None):
        via = pcbnew.PCB_VIA(self.board)
        via.SetPosition(vec(x, y))
        via.SetDrill(mm(drill or JLC["via_drill"]))
        via.SetWidth(mm(size or JLC["via_diameter"]))
        via.SetNet(self.net(net_name))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetLocked(locked)
        self.board.Add(via)

    def polygon_zone(self, net_name, layer, polygon, priority=0, rule_area=False,
                     name=None, thermal=False):
        zone = pcbnew.ZONE(self.board)
        if rule_area:
            zone.SetIsRuleArea(True)
            zone.SetDoNotAllowZoneFills(True)
            zone.SetDoNotAllowTracks(name == "dut_bay")
            zone.SetDoNotAllowVias(name == "dut_bay")
            zone.SetDoNotAllowPads(False)
            zone.SetDoNotAllowFootprints(False)
            if name == "usb_field":
                zone.SetDoNotAllowZoneFills(False)
        else:
            zone.SetNet(self.net(net_name))
            zone.SetAssignedPriority(priority)
            zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL if thermal
                                  else pcbnew.ZONE_CONNECTION_FULL)
            zone.SetMinThickness(mm(0.25))
            zone.SetLocalClearance(mm(0.25))
            zone.SetThermalReliefGap(mm(0.3))
            zone.SetThermalReliefSpokeWidth(mm(0.4))
        if name:
            zone.SetZoneName(name)
        if isinstance(layer, (list, tuple)):
            layer_set = pcbnew.LSET()
            for item in layer:
                layer_set.AddLayer(item)
            zone.SetLayerSet(layer_set)
        else:
            zone_layer(zone, layer)
        shape = zone.Outline()
        shape.NewOutline()
        for x, y in polygon:
            shape.Append(mm(x), mm(y))
        self.board.Add(zone)
        return zone

    # ------------------------------------------------------------ placement

    def outline(self):
        points = [(BOARD_X0, BOARD_Y0), (BOARD_X1, BOARD_Y0),
                  (BOARD_X1, BOARD_Y1), (BOARD_X0, BOARD_Y1)]
        for start, end in zip(points, points[1:] + points[:1]):
            shape = pcbnew.PCB_SHAPE(self.board)
            shape.SetShape(pcbnew.SHAPE_T_SEGMENT)
            shape.SetStart(vec(*start))
            shape.SetEnd(vec(*end))
            shape.SetLayer(pcbnew.Edge_Cuts)
            shape.SetWidth(mm(0.15))
            self.board.Add(shape)

    def place_components(self):
        missing = []
        for reference, footprint_id in sorted(self.footprint_ids.items()):
            if reference.startswith("#") or footprint_id is None:
                continue
            if reference not in PLACEMENT:
                missing.append(reference)
                continue
            footprint = load_footprint(footprint_id)
            footprint.SetReference(reference)
            self.board.Add(footprint)
            x, y, rotation = PLACEMENT[reference]
            footprint.SetPosition(vec(x, y))
            if rotation:
                footprint.SetOrientationDegrees(rotation)
            for pad in footprint.Pads():
                name = self.pin_nets.get((reference, pad.GetNumber()))
                if name:
                    pad.SetNet(self.net(name))
        if missing:
            raise RuntimeError(f"no placement for {missing}")
        for reference, library, name, x, y in MECHANICAL:
            footprint = load_footprint(f"{library}:{name}")
            footprint.SetReference(reference)
            self.board.Add(footprint)
            footprint.SetPosition(vec(x, y))
            if reference.startswith("H"):
                for pad in footprint.Pads():
                    pad.SetNet(self.net("/GND"))
        for footprint in self.board.GetFootprints():
            reference = footprint.GetReference()
            # clamp silk lands on pads once rotated; the bay silk says it
            if reference in ("J2", "J3", "J4", "J5"):
                for item in list(footprint.GraphicalItems()):
                    if item.GetLayer() == pcbnew.F_SilkS:
                        footprint.Remove(item)
            # connector bodies overhang the cable edge: drop silk beyond it
            if reference in ("J1", "J6", "J7", "J9"):
                for item in list(footprint.GraphicalItems()):
                    if item.GetLayer() != pcbnew.F_SilkS:
                        continue
                    box = item.GetBoundingBox()
                    if pcbnew.ToMM(box.GetBottom()) > BOARD_Y1 - 0.4:
                        footprint.Remove(item)
            # references to the fab layer: assembly works from it
            text = footprint.Reference()
            text.SetLayer(pcbnew.F_Fab)
            text.SetTextSize(pcbnew.VECTOR2I(mm(0.7), mm(0.7)))
            text.SetTextThickness(mm(0.11))

    # ------------------------------------------------------------ pre-routes

    def route_arms(self):
        """Kelvin arms, jack -> iso NC. Uncalibrated in 2-wire mode (the clamp
        -plane fixture compensation removes them), so short, F.Cu only, no
        vias: out of the jack along its row to a lane 2 mm beside the relay
        pole, straight down/up the lane, onto the NC pad. With the pair only
        JACK_PITCH apart, an arm whose row run would cross the pair's other
        hole steps ARM_JOG toward the relay row first."""
        pairs = {"A": ("J10", "J11", "K11"), "B": ("J12", "J13", "K12"),
                 "C": ("J14", "J15", "K13"), "D": ("J16", "J17", "K14")}
        for terminal, (force_jack, sense_jack, iso) in pairs.items():
            for jack, nc_pad, net in ((force_jack, 2, f"/DUT_{terminal}"),
                                      (sense_jack, 7, f"/DUTS_{terminal}")):
                jx, jy = self.pad_position(jack, 1)
                nx, ny = self.pad_position(iso, nc_pad)
                lane = nx + (2.0 if jx > nx else -2.0)
                other = self.pad_position(sense_jack if jack == force_jack else force_jack, 1)[0]
                if min(jx, lane) < other < max(jx, lane):
                    row = jy + (ARM_JOG if ny > jy else -ARM_JOG)
                    self.track([(jx, jy), (jx, row), (lane, row), (lane, ny), (nx, ny)], net)
                else:
                    self.track([(jx, jy), (lane, jy), (lane, ny), (nx, ny)], net)

    def route_rails(self):
        """Force rails (B.Cu) and sense rails (In2.Cu) with a via tap beside
        every NO pad; HI fed at the west end, LO at the east end."""
        for rail in ("HI", "LO"):
            y_force, y_sense = RAIL_Y[rail], SENSE_Y[rail]
            force_taps, sense_taps = [], []
            for terminal in "ABECD":
                column = COLUMNS[terminal]
                force_pad = (column, y_force)
                sense_pad = (column + POLE2_DX, y_force)
                fx, sx = column + TAP_DX, column + POLE2_DX + TAP_DX
                self.track([force_pad, (fx, y_force)], f"/RAIL_{rail}")
                self.via(fx, y_force, f"/RAIL_{rail}")
                self.track([sense_pad, (sx - 0.6, y_force), (sx, y_sense)], f"/SNS_{rail}",
                           width=0.4)
                self.via(sx, y_sense, f"/SNS_{rail}")
                force_taps.append(fx)
                sense_taps.append(sx)
            # force rail spans the taps plus its feed end
            west = HI_FEED_X if rail == "HI" else min(force_taps)
            east = max(force_taps) if rail == "HI" else LO_FEED_X
            self.track([(west, y_force), (east, y_force)], f"/RAIL_{rail}",
                       layer=pcbnew.B_Cu, width=RAIL_TRACK)
            # sense rail spans exactly its taps (Faraday: no open stubs)
            self.track([(min(sense_taps), y_sense), (max(sense_taps), y_sense)],
                       f"/SNS_{rail}", layer=pcbnew.B_Cu, width=SENSE_TRACK)

        # HI feed: via at the west end, F.Cu straight down to the SOURCE pin.
        source = self.pad_position("J6", 1)
        self.via(HI_FEED_X, RAIL_Y["HI"], "/RAIL_HI")
        self.track([(HI_FEED_X, RAIL_Y["HI"]), (HI_FEED_X, source[1] - 3.0),
                    (source[0], source[1] - 3.0), source], "/RAIL_HI", width=RAIL_TRACK)
        # LO feed: via at the east end, F.Cu down to the RC1 shunt pad.
        shunt = self.pad_position("R14", 1)
        self.via(LO_FEED_X, RAIL_Y["LO"], "/RAIL_LO")
        self.track([(LO_FEED_X, RAIL_Y["LO"]), (LO_FEED_X, shunt[1] - 2.0),
                    (shunt[0], shunt[1] - 2.0), shunt], "/RAIL_LO", width=RAIL_TRACK)

    def route_coil_tree(self):
        """+5V to every relay coil+ on In1.Cu inside the void: a via beside
        each coil+ pad (outboard; via-in-pad is a paid option on 4 layers),
        a vertical to one of two
        trunks (jogging sideways first where a through-hole pad -- a DUT
        jack -- is in the way), the trunks east into the control strip."""
        obstacles = []
        for footprint in self.board.GetFootprints():
            for pad in footprint.Pads():
                if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pad.GetNetname() != "/+5V":
                    box = pad.GetBoundingBox()
                    obstacles.append((pcbnew.ToMM(box.Centre().x), pcbnew.ToMM(box.Centre().y),
                                      pcbnew.ToMM(max(box.GetWidth(), box.GetHeight())) / 2 + 0.6))
        for item in self.board.GetTracks():
            if item.GetClass() == "PCB_VIA" and item.GetNetname() != "/+5V":
                position = item.GetPosition()
                obstacles.append((pcbnew.ToMM(position.x), pcbnew.ToMM(position.y), 0.85))

        def clear(x0, y0, x1, y1):
            for cx, cy, r in obstacles:
                if x0 == x1:
                    lo, hi = sorted((y0, y1))
                    d = abs(cx - x0) if lo <= cy <= hi else min(math.hypot(cx - x0, cy - lo),
                                                                 math.hypot(cx - x0, cy - hi))
                else:
                    lo, hi = sorted((x0, x1))
                    d = abs(cy - y0) if lo <= cx <= hi else min(math.hypot(cx - lo, cy - y0),
                                                                 math.hypot(cx - hi, cy - y0))
                if d < r:
                    return False
            return True

        trunk_ends = {TRUNK_TOP_Y: [], TRUNK_BOTTOM_Y: []}
        for footprint in self.board.GetFootprints():
            reference = footprint.GetReference()
            if not reference.startswith("K"):
                continue
            px, py = self.pad_position(reference, 1)
            cx = pcbnew.ToMM(footprint.GetPosition().x)
            cy = pcbnew.ToMM(footprint.GetPosition().y)
            out_x = 1.0 if px > cx else -1.0
            out_y = 1.0 if py > cy else -1.0
            for dx, dy in ((1.65 * out_x, 0.0), (0.0, 1.4 * out_y), (1.2 * out_x, 1.2 * out_y),
                           (-1.65 * out_x, 0.0)):
                x, y = px + dx, py + dy
                if self.spot_free(x, y, 0.3, "/+5V"):
                    break
            else:
                raise RuntimeError(f"no room for the coil+ via of {reference}")
            trunk = TRUNK_TOP_Y if y < RAIL_Y["LO"] else TRUNK_BOTTOM_Y
            self.track([(px, py), (x, y)], "/+5V", width=0.5)
            self.via(x, y, "/+5V", size=0.6, drill=0.3)
            lane = None
            for step in range(0, 41):
                for candidate in ((x + 0.5 * step, x - 0.5 * step) if step else (x,)):
                    if clear(x, y, candidate, y) and clear(candidate, y, candidate, trunk):
                        lane = candidate
                        break
                if lane is not None:
                    break
            if lane is None:
                raise RuntimeError(f"no In1 lane for {reference} coil feed")
            self.track([(x, y), (lane, y), (lane, trunk)], "/+5V", layer=pcbnew.In1_Cu, width=0.5)
            trunk_ends[trunk].append(lane)
        for trunk, xs in trunk_ends.items():
            # the trunks stop inside the void (In1 is a GND plane outside);
            # maze_route.py joins them to the +5V supply on an outer layer
            self.track([(min(xs), trunk), (VOID[1][0] - 1.0, trunk)], "/+5V",
                       layer=pcbnew.In1_Cu, width=0.8)

    def route_usb_entry(self):
        """USB-C fine-pitch entry (0.5 mm pads), pre-routed: the maze router's
        0.25 mm grid cannot resolve it. D- pads (B7, A7) join just north of the
        pad row and run up the gap between the ESD array's pin columns; D+
        pads (A6, B6) straddle A7, so they join just south of the row (under
        the receptacle) and cross over on B.Cu to the ESD array's pin 3. CC
        pins go to their 5.1k pull-downs along the top of the VBUS/GND pads.
        The MCU's USB pins get escape stubs that fan apart."""
        w = 0.2
        track = lambda *a, **k: self.track(*a, chamfer=False, **k)   # noqa: E731
        p = {name: self.pad_position("J1", name) for name in ("A5", "A6", "A7", "B5", "B6", "B7")}
        row_top = p["A6"][1] - 0.725 - 0.38        # north of the pad row
        row_bottom = p["A6"][1] + 0.725 + 0.52     # south of the pad row
        d2_dm, d2_dp = self.pad_position("D2", 1), self.pad_position("D2", 3)
        gap_x = (d2_dm[0] + self.pad_position("D2", 6)[0]) / 2
        # D-: B7 + A7 -> junction north -> corridor between D2's columns -> pin 1
        track([p["B7"], (p["B7"][0], row_top), (p["A7"][0], row_top), p["A7"]],
                   "/USB_DM_CON", width=w)
        track([(gap_x, row_top), (p["A7"][0], row_top)], "/USB_DM_CON", width=w)
        track([(gap_x, row_top), (gap_x, d2_dm[1]), d2_dm], "/USB_DM_CON", width=w)
        # D+: A6 + B6 -> junction south -> via -> B.Cu -> via beside D2 pin 3
        mid = (p["A6"][0] + p["B6"][0]) / 2
        track([p["A6"], (p["A6"][0], row_bottom), (p["B6"][0], row_bottom), p["B6"]],
                   "/USB_DP_CON", width=w)
        track([(mid, row_bottom), (mid, row_bottom + 0.6)], "/USB_DP_CON", width=w)
        self.via(mid, row_bottom + 0.6, "/USB_DP_CON")
        up = (d2_dp[0] - 1.25, d2_dp[1])
        track([(mid, row_bottom + 0.6), (up[0], row_bottom - 1.6), up], "/USB_DP_CON",
                   layer=pcbnew.B_Cu, width=w)
        self.via(*up, "/USB_DP_CON")
        track([up, d2_dp], "/USB_DP_CON", width=w)
        # CC1 / CC2 to the pull-downs
        lane = row_top - 0.0
        r1, r2 = self.pad_position("R1", 1), self.pad_position("R2", 1)
        track([p["A5"], (p["A5"][0], lane - 0.1), (p["A5"][0] - 1.2, lane - 0.1), r1],
                   "/CC1", width=w)
        track([p["B5"], (p["B5"][0], lane - 0.1), (p["B5"][0] + 1.2, lane - 0.1), r2],
                   "/CC2", width=w)
        # VBUS: CC1/CC2 run over the VBUS pads and D2's VBUS pin is boxed in
        # by the data lines on F.Cu, so VBUS goes under the receptacle on
        # B.Cu: each VBUS pad drops through a via just south of the pad row
        # (clear of the NPTH pegs and the D+ via), a 0.4 mm trunk runs between
        # the shell's front and rear legs to a via beside FB1; D2 pin 5 joins
        # through its own via outside the pin.
        vbus_w = 0.4
        left, right = self.pad_position("J1", "A4"), self.pad_position("J1", "A9")
        pad_bottom = left[1] + 0.725
        # straight down each VBUS pad's inner edge (0.2 mm: 0.2 to the B8/B5
        # pad beside it, 0.69 mm centre-to-peg from the NPTH alignment peg)
        stub_l, stub_r = left[0] + 0.25, right[0] - 0.25
        via_l = (stub_l, pad_bottom + 1.68)
        via_r = (stub_r, pad_bottom + 1.68)
        trunk_y = left[1] + 3.15
        fb1 = self.pad_position("FB1", 1)
        west = (fb1[0] + 1.2, trunk_y)
        for pad, via_xy in ((left, via_l), (right, via_r)):
            track([(via_xy[0], pad_bottom - 0.3), via_xy], "/VBUS", width=0.2)
            self.via(*via_xy, "/VBUS")
            track([via_xy, (via_xy[0], trunk_y)], "/VBUS", layer=pcbnew.B_Cu, width=vbus_w)
        track([(via_r[0], trunk_y), west], "/VBUS", layer=pcbnew.B_Cu, width=vbus_w)
        self.via(*west, "/VBUS")
        track([west, (west[0], fb1[1] + 0.95), fb1], "/VBUS", width=vbus_w)
        d2_vbus = self.pad_position("D2", 5)
        d2_via = (d2_vbus[0] + 1.46, d2_vbus[1])
        track([d2_vbus, d2_via], "/VBUS", width=0.3)
        self.via(*d2_via, "/VBUS")
        track([d2_via, (via_r[0], d2_vbus[1] + 0.7), via_r], "/VBUS", layer=pcbnew.B_Cu, width=0.2)
        # MCU USB pins: escape stubs fanning apart (0.5 mm pitch)
        x, y = self.pad_position("U1", 32)
        track([(x, y), (x, y + 1.2), (x - 0.8, y + 2.0)], "/USB_DM", width=w)
        x, y = self.pad_position("U1", 33)
        track([(x, y), (x, y + 2.0)], "/USB_DP", width=w)

    def fanout_ground(self):
        """A GND via at every SMD GND pad outside the void, placed now, before
        freerouting can take the room: via-in-pad where the pad is big
        enough (free POFV on JLC 6-layer), else the nearest free spot beside
        the pad (AN4325: each decoupler its own via into the plane)."""
        gnd = self.net("/GND").GetNetCode()
        pads = [pad for fp in self.board.GetFootprints() for pad in fp.Pads()]
        items = list(self.board.GetTracks())

        def free(x, y, radius):
            if radius >= 0.25:                     # a via: full check incl. hole spacing
                return self.spot_free(x, y, radius, "/GND")
            position = vec(x, y)                   # a stub sample point
            for item in self.board.GetTracks():
                if item.GetNetCode() != gnd and item.HitTest(position, mm(radius + 0.2)):
                    return False
            for pad in pads:
                if pad.GetNetCode() != gnd and pad.HitTest(position, mm(radius + 0.2)):
                    return False
            return True

        # escape bands of fine-pitch packages: the strip from every pin's toe
        # to ESCAPE_BAND mm outward is where those pins fan out (maze_route
        # escape_fine_pitch); other parts' GND vias stay out of it
        bands = []
        for footprint in self.board.GetFootprints():
            smd = [q for q in footprint.Pads() if q.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
            if len(smd) < 20:
                continue
            for q in smd:
                qx, qy = pcbnew.ToMM(q.GetPosition().x), pcbnew.ToMM(q.GetPosition().y)
                box = q.GetBoundingBox()
                hw, hh = pcbnew.ToMM(box.GetWidth()) / 2, pcbnew.ToMM(box.GetHeight()) / 2
                fx, fy = pcbnew.ToMM(footprint.GetPosition().x), pcbnew.ToMM(footprint.GetPosition().y)
                if hw > hh:
                    sx = 1.0 if qx > fx else -1.0
                    x0, x1 = sorted((qx + sx * hw, qx + sx * (hw + ESCAPE_BAND)))
                    bands.append((footprint.GetReference(), x0, qy - hh - 0.3, x1, qy + hh + 0.3))
                else:
                    sy = 1.0 if qy > fy else -1.0
                    y0, y1 = sorted((qy + sy * hh, qy + sy * (hh + ESCAPE_BAND)))
                    bands.append((footprint.GetReference(), qx - hw - 0.3, y0, qx + hw + 0.3, y1))

        def in_band(x, y, owner):
            return any(ref != owner and x0 - 0.3 <= x <= x1 + 0.3 and y0 - 0.3 <= y <= y1 + 0.3
                       for ref, x0, y0, x1, y1 in bands)

        placed, in_pad, missed, done = 0, 0, [], set()
        for footprint in self.board.GetFootprints():
            centre = footprint.GetPosition()
            cx, cy = pcbnew.ToMM(centre.x), pcbnew.ToMM(centre.y)
            for pad in footprint.Pads():
                if pad.GetNetCode() != gnd or pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                    continue
                px, py = (pcbnew.ToMM(v) for v in (pad.GetPosition().x, pad.GetPosition().y))
                if point_in_polygon(px, py, VOID) or (round(px, 2), round(py, 2)) in done:
                    continue
                done.add((round(px, 2), round(py, 2)))
                size = pad.GetBoundingBox()
                w, h = pcbnew.ToMM(size.GetWidth()), pcbnew.ToMM(size.GetHeight())
                # outward from the footprint first, then sideways
                ux, uy = px - cx, py - cy
                norm = math.hypot(ux, uy) or 1.0
                ux, uy = ux / norm, uy / norm
                pads = list(footprint.Pads())
                if len(pads) == 2:
                    # two-terminal decoupler: via beside the GND pad at 90
                    # degrees to the pad axis, on the side facing the nearest
                    # IC -- the pad->via->plane loop is smallest there and its
                    # current path does not run back under the cap body
                    ax, ay = ux, uy
                    perps = [(-ay, ax), (ay, -ax)]
                    ic = min((f for f in self.board.GetFootprints()
                              if f.GetReference().startswith("U")),
                             key=lambda f: math.hypot(pcbnew.ToMM(f.GetPosition().x) - px,
                                                      pcbnew.ToMM(f.GetPosition().y) - py))
                    ix, iy = pcbnew.ToMM(ic.GetPosition().x), pcbnew.ToMM(ic.GetPosition().y)
                    perps.sort(key=lambda d: -(d[0] * (ix - px) + d[1] * (iy - py)))
                    side = min(w, h) / 2 if abs(ax) > abs(ay) else min(w, h) / 2
                    done_here = False
                    for dx, dy in perps:
                        extent = (h if abs(dy) > abs(dx) else w) / 2 + 0.5
                        x, y = px + dx * extent, py + dy * extent
                        if (point_in_polygon(x, y, VOID) or in_band(x, y, footprint.GetReference())
                                or not free(x, y, 0.3)):
                            continue
                        self.track([(px, py), (x, y)], "/GND", width=0.4)
                        self.via(x, y, "/GND")
                        items.extend(list(self.board.GetTracks())[-2:])
                        placed += 1
                        done_here = True
                        break
                    if done_here:
                        continue
                reach = max(w, h) / 2 + 0.5
                # along the pad's long axis, away from the part, comes first
                # (LQFP/SOIC pins: the only direction clear of the neighbours)
                if w >= h:
                    axis = (1.0 if px >= cx else -1.0, 0.0)
                else:
                    axis = (0.0, 1.0 if py >= cy else -1.0)
                options = [axis, (ux, uy), (-uy, ux), (uy, -ux), (ux - uy, uy + ux),
                           (ux + uy, uy - ux)]
                if len([q for q in footprint.Pads() if q.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]) >= 20:
                    # fine-pitch package: under the body (B.Cu there is free;
                    # outside, the pins' escape band is reserved)
                    options.append((-axis[0], -axis[1]))
                for dx, dy in options:
                    d = math.hypot(dx, dy) or 1.0
                    x, y = px + dx / d * reach, py + dy / d * reach
                    if (point_in_polygon(x, y, VOID) or in_band(x, y, footprint.GetReference())
                            or not free(x, y, 0.3)):
                        continue
                    toe = max(w, h) / 2
                    samples = [f for f in (0.5, 0.65, 0.8, 0.9) if f * reach > toe + 0.05]
                    if not all(free(px + (x - px) * f, py + (y - py) * f, 0.15) for f in samples):
                        continue                 # the stub, beyond its own pad, must clear
                    self.track([(px, py), (x, y)], "/GND", width=0.3)
                    self.via(x, y, "/GND")
                    items.extend(list(self.board.GetTracks())[-2:])
                    placed += 1
                    break
                else:
                    missed.append(f"{footprint.GetReference()}.{pad.GetNumber()}")
        print(f"  GND fanout: {placed} vias ({in_pad} in pad)"
              + (f", no room: {missed}" if missed else ""))

    # ------------------------------------------------------------ copper

    @staticmethod
    def guard_strips():
        """GND guard strips on In2, 0.21 mm above each row's B.Cu force and
        sense rails, plus under both feeds. In1 carries the +5V coil drops."""
        h = GUARD_HALF_WIDTH
        x0, x1 = VOID[0][0], VOID[1][0]
        return [(x0, RAIL_Y["HI"] - h, x1, SENSE_Y["HI"] + h),
                (x0, RAIL_Y["LO"] - h, x1, SENSE_Y["LO"] + h),
                (HI_FEED_X - h, RAIL_Y["HI"], HI_FEED_X + h, VOID[-1][1]),
                (LO_FEED_X - h, RAIL_Y["LO"], LO_FEED_X + h, VOID[2][1])]

    def void_area(self, layers, name, minus=()):
        """Rule area = VOID minus the given rectangles (pcbnew boolean)."""
        zone = self.polygon_zone(None, layers, VOID, rule_area=True, name=name)
        if minus:
            cut = pcbnew.SHAPE_POLY_SET()
            for x0, y0, x1, y1 in minus:
                cut.NewOutline()
                for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                    cut.Append(mm(x), mm(y))
            outline = zone.Outline()
            try:
                outline.BooleanSubtract(cut)
            except TypeError:
                outline.BooleanSubtract(cut, pcbnew.SHAPE_POLY_SET.PM_FAST)
        return zone

    def planes(self):
        """GND planes on In1/In4 and GND pours on F.Cu/B.Cu everywhere
        outside the void; guard strips inside it (In4 under the force rails,
        guard strips on In4 under both rails of each row). In2/In3 route."""
        board_poly = [(BOARD_X0 + 0.6, BOARD_Y0 + 0.6), (BOARD_X1 - 0.6, BOARD_Y0 + 0.6),
                      (BOARD_X1 - 0.6, BOARD_Y1 - 0.6), (BOARD_X0 + 0.6, BOARD_Y1 - 0.6)]
        for layer in (pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.F_Cu, pcbnew.B_Cu):
            self.polygon_zone("/GND", layer, board_poly, priority=0)
        # outer layers: no pour in the measurement area. Inner layers: solid GND
        # under everything -- they carry no tracks -- except under the nodes that
        # float at an intermediate potential (terminal buses, arms, clips), where
        # ground capacitance is a DUT-dependent error (Cg*v*(v-1)). Those
        # openings follow the routed copper: route_pcb.float_node_voids().
        self.void_area([pcbnew.F_Cu, pcbnew.B_Cu], "measurement_void")
        every = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
        x0, y0, x1, y1 = BAY
        self.polygon_zone(None, every, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
                          rule_area=True, name="dut_bay")
        # fine-pitch rule area over the USB-C pad row (see relay_board.kicad_dru)
        jx, jy = PLACEMENT["J1"][:2]
        self.polygon_zone(None, [pcbnew.F_Cu], [(jx - 3.8, jy - 5.0), (jx + 3.8, jy - 5.0),
                                                (jx + 3.8, jy - 3.0), (jx - 3.8, jy - 3.0)],
                          rule_area=True, name="usb_field")

    def stitching(self, pitch=4.0):
        """GND stitching vias along the board edge (outside the void) and
        around the void perimeter: lambda/20 at 1 GHz is ~7 mm, 4 mm is
        comfortably below. Skips any spot within reach of other copper."""
        items = list(self.board.GetTracks())
        pads = [pad for fp in self.board.GetFootprints() for pad in fp.Pads()]
        gnd = self.net("/GND").GetNetCode()
        keepouts = [fp.GetBoundingBox(False) for fp in self.board.GetFootprints()
                    if fp.GetReference().startswith(("H", "FID"))]

        def free(x, y):
            if point_in_polygon(x, y, VOID):
                return False
            bx0, by0, bx1, by1 = BAY
            if bx0 - 1 < x < bx1 + 1 and by0 - 1 < y < by1 + 1:
                return False
            position = vec(x, y)
            if any(box.Contains(position) for box in keepouts):
                return False
            return self.spot_free(x, y, 0.3, "/GND", hole_gap=1.0)

        points = []
        inset = 1.3
        x = BOARD_X0 + inset
        while x <= BOARD_X1 - inset:
            points += [(x, BOARD_Y0 + inset), (x, BOARD_Y1 - inset)]
            x += pitch
        y = BOARD_Y0 + inset
        while y <= BOARD_Y1 - inset:
            points += [(BOARD_X0 + inset, y), (BOARD_X1 - inset, y)]
            y += pitch
        # void perimeter, 1.2 mm outside it
        for (x1, y1), (x2, y2) in zip(VOID, VOID[1:] + VOID[:1]):
            length = math.hypot(x2 - x1, y2 - y1)
            steps = max(1, int(length / pitch))
            nx, ny = -(y2 - y1) / length, (x2 - x1) / length   # outward (cw in y-down)
            for step in range(steps + 1):
                t = step / steps
                points.append((x1 + (x2 - x1) * t + nx * 1.2, y1 + (y2 - y1) * t + ny * 1.2))
        placed = 0
        for x, y in points:
            if BOARD_X0 + 1.0 < x < BOARD_X1 - 1.0 and BOARD_Y0 + 1.0 < y < BOARD_Y1 - 1.0 \
                    and free(x, y):
                self.via(x, y, "/GND")
                items.append(list(self.board.GetTracks())[-1])
                placed += 1
        print(f"  stitching: {placed} GND vias")

    def silkscreen(self):
        def text(content, x, y, size=1.2, bold=False):
            item = pcbnew.PCB_TEXT(self.board)
            item.SetText(content)
            item.SetPosition(vec(x, y))
            item.SetLayer(pcbnew.F_SilkS)
            item.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
            item.SetTextThickness(mm(max(JLC["silk_line"], size * (0.2 if bold else 0.15))))
            self.board.Add(item)

        text("OpenMagnetics Relay Board rev C", 118.0, 138.0, 1.4, bold=True)
        bx0, by0, bx1, by1 = BAY
        text("DUT", (bx0 + bx1) / 2, (by0 + by1) / 2, 2.0, bold=True)
        for reference, label in (("J10", "A-F"), ("J11", "A-S"), ("J12", "B-F"), ("J13", "B-S"),
                                 ("J14", "C-F"), ("J15", "C-S"), ("J16", "D-F"), ("J17", "D-S")):
            x, y, _ = PLACEMENT[reference]
            text(label, x, y - 6.6 if y < ISO_ROW else y + 6.6, 1.2, bold=True)
        text("lead: through F, then S", (bx0 + bx1) / 2, by1 - 2.0, 1.0)
        for reference, ((x, y), label) in TEST_PADS.items():
            text(label, x, y - 2.0, 1.0)
        for x1, y1, x2, y2 in ((bx0, by0, bx1, by0), (bx0, by1, bx1, by1),
                               (bx0, by0, bx0, by1), (bx1, by0, bx1, by1)):
            shape = pcbnew.PCB_SHAPE(self.board)
            shape.SetShape(pcbnew.SHAPE_T_SEGMENT)
            shape.SetStart(vec(x1, y1))
            shape.SetEnd(vec(x2, y2))
            shape.SetLayer(pcbnew.F_SilkS)
            shape.SetWidth(mm(0.25))
            self.board.Add(shape)
        for name, x in BNC_X.items():
            text("OUT" if name == "SOURCE" else name, x, BNC_Y - 4.6, 1.3, bold=True)
        text("USB", USB_X, BOARD_Y1 - 9.8, 1.0)
        text("SWD", PLACEMENT["J8"][0], PLACEMENT["J8"][1] - 5.0, 1.0)
        text("BOOT", PLACEMENT["SW1"][0] - 4.8, PLACEMENT["SW1"][1], 1.0)
        text("RESET", PLACEMENT["SW2"][0] - 5.2, PLACEMENT["SW2"][1], 1.0)
        text("STD", PLACEMENT["C30"][0] + 2.6, PLACEMENT["C30"][1], 1.0)
        for reference, label in (("D5", "PWR"), ("D1", "RUN"), ("D6", "BUSY"),
                                 ("D7", "KELV"), ("D8", "PROG")):
            x, y, _ = PLACEMENT[reference]
            text(label, x - 3.4, y, 0.9)

    def bonding_strip(self):
        """Exposed GND copper (mask removed both sides) along the cable edge
        between the connectors: an enclosure or a gasketed lid bonds to the
        plane right where the cables enter (Ott: cable shields share one
        low-impedance reference). Built as GND pads, not bare mask openings:
        a netless mask aperture over GND vias is a solder_mask_bridge in DRC."""
        add_bonding_strip(self.board)

    def save(self):
        self.board.Save(OUTPUT)
        write_stackup(OUTPUT)
        return pcbnew.LoadBoard(OUTPUT)


def add_bonding_strip(board):
    """GND pad strips on both sides of the cable edge between J1/J6/J7/J9
    (also used to patch an already-routed board). Returns the span count."""
    keep_out = sorted(
        (pcbnew.ToMM(fp.GetBoundingBox(False).GetLeft()) - 0.5,
         pcbnew.ToMM(fp.GetBoundingBox(False).GetRight()) + 0.5)
        for fp in board.GetFootprints()
        if fp.GetReference() in ("J1", "J6", "J7", "J9"))
    spans, x = [], BOARD_X0 + 1.0
    for left, right in keep_out:
        if left > x + 2.0:
            spans.append((x, left))
        x = max(x, right)
    if BOARD_X1 - 1.0 > x + 2.0:
        spans.append((x, BOARD_X1 - 1.0))
    gnd = board.FindNet("/GND")
    y0, y1 = BOARD_Y1 - 2.2, BOARD_Y1 - 0.8
    for index, (x0, x1) in enumerate(spans):
        footprint = pcbnew.FOOTPRINT(board)
        footprint.SetReference(f"BOND{index + 1}")
        # a library ID is required: freerouting writes an empty image name
        # as "(component " and KiCad's SES import then fails
        footprint.SetFPID(pcbnew.LIB_ID("OpenMagnetics", "GND_BondingStrip"))
        footprint.SetValue("GND bonding strip")
        footprint.Reference().SetVisible(False)
        footprint.Value().SetVisible(False)
        footprint.SetAttributes(pcbnew.FP_BOARD_ONLY | pcbnew.FP_EXCLUDE_FROM_BOM
                                | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
        footprint.SetPosition(vec((x0 + x1) / 2, (y0 + y1) / 2))
        board.Add(footprint)
        for copper, mask in ((pcbnew.F_Cu, pcbnew.F_Mask), (pcbnew.B_Cu, pcbnew.B_Mask)):
            pad = pcbnew.PAD(footprint)
            pad.SetNumber("1")
            pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
            pad.SetShape(pcbnew.PAD_SHAPE_RECT)
            pad.SetSize(vec(x1 - x0, y1 - y0))
            layers = pcbnew.LSET()
            layers.AddLayer(copper)
            layers.AddLayer(mask)
            pad.SetLayerSet(layers)
            footprint.Add(pad)
            pad.SetPosition(vec((x0 + x1) / 2, (y0 + y1) / 2))
            pad.SetNet(gnd)
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    print(f"  bonding strip: {len(spans)} exposed GND spans on the cable edge")
    return len(spans)


#: JLC04161H-7628 (jlcpcb.com/impedance): name, type, thickness mm, er.
JLC_STACKUP = [("F.Cu", "copper", 0.035, None),
               ("dielectric 1", "prepreg", 0.2104, 4.4),
               ("In1.Cu", "copper", 0.0152, None),
               ("dielectric 2", "core", 1.065, 4.6),
               ("In2.Cu", "copper", 0.0152, None),
               ("dielectric 3", "prepreg", 0.2104, 4.4),
               ("B.Cu", "copper", 0.035, None)]


def write_stackup(path):
    """Put the JLC stackup into the board file's (setup), so DRC, impedance
    and every downstream tool (Faraday reads it) use the real dielectrics."""
    text = open(path, encoding="utf-8").read()
    if "(stackup" in text:
        return
    lines = ['\t\t(stackup',
             '\t\t\t(layer "F.SilkS" (type "Top Silk Screen"))',
             '\t\t\t(layer "F.Paste" (type "Top Solder Paste"))',
             '\t\t\t(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))']
    for name, kind, thickness, er in JLC_STACKUP:
        if kind == "copper":
            lines.append(f'\t\t\t(layer "{name}" (type "copper") (thickness {thickness}))')
        else:
            material = "7628" if kind == "prepreg" else "FR4"
            lines.append(f'\t\t\t(layer "{name}" (type "{kind}") (thickness {thickness}) '
                         f'(material "{material}") (epsilon_r {er}) (loss_tangent 0.02))')
    lines += ['\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))',
              '\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))',
              '\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen"))',
              '\t\t\t(copper_finish "HAL lead-free")',
              '\t\t\t(dielectric_constraints no)',
              '\t\t)']
    anchor = text.index("(setup") + len("(setup")
    text = text[:anchor] + "\n" + "\n".join(lines) + text[anchor:]
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


#: Inner layers freerouting must not route on (plane layers).
POWER_LAYERS = ("In1.Cu", "In2.Cu")


def patch_dsn_power_layers(path):
    with open(path, encoding="utf-8") as handle:
        lines = handle.readlines()
    count, pending = 0, False
    for index, line in enumerate(lines):
        stripped = line.strip()
        if any(stripped.startswith(f"(layer {name}") for name in POWER_LAYERS):
            pending = True
        elif pending and "(type signal)" in line:
            lines[index] = line.replace("(type signal)", "(type power)")
            count += 1
            pending = False
        elif stripped.startswith("(layer "):
            pending = False
    with open(path, "w", encoding="utf-8") as handle:
        handle.writelines(lines)
    return count


def main():
    builder = Board()
    builder.outline()
    builder.place_components()
    builder.route_arms()
    builder.route_rails()
    # the +5 V coil feed is no longer pre-routed on In1: tracks stay on the outer
    # layers (maze_route.py routes /+5V), the inner layers are planes and guards only
    builder.route_usb_entry()
    builder.fanout_ground()
    builder.planes()
    builder.stitching()
    builder.bonding_strip()
    builder.silkscreen()
    board = builder.save()
    tracks = list(board.GetTracks())
    # the routers may rip up and re-route their own copper, never these
    with open(PREROUTED, "w") as handle:
        json.dump(sorted(t.m_Uuid.AsString() for t in tracks), handle)
    print(f"PCB saved: {OUTPUT}")
    print(f"  {BOARD_X1 - BOARD_X0:.0f} x {BOARD_Y1 - BOARD_Y0:.0f} mm, "
          f"{board.GetCopperLayerCount()} layers (JLC04161H-7628 stackup in the file)")
    print(f"  footprints {len(board.GetFootprints())}, pre-routed tracks+vias "
          f"{len(tracks)} (locked), zones {len(list(board.Zones()))}")
    destination = OUTPUT.replace(".kicad_pcb", ".dsn")
    if pcbnew.ExportSpecctraDSN(board, destination):
        count = patch_dsn_power_layers(destination)
        if count != len(POWER_LAYERS):
            raise SystemExit(f"DSN power-layer patch matched {count}/{len(POWER_LAYERS)}")
        print(f"  DSN exported, {count} plane layers marked power: {destination}")
    return board


if __name__ == "__main__":
    main()
