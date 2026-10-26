"""
Generate JLCPCB SMT-assembly BOM and CPL spreadsheets.

    python export_jlcpcb.py

Follows JLC's sample templates exactly:
  BOM: Comment | Designator | Footprint | JLCPCB Part #(optional)
  CPL: Designator | Mid X | Mid Y | Layer | Rotation

BOM data derives from relay_board.xml (same source generate_bom.py
verifies against); CPL positions from fab/relay_board_pos.csv, so run
export_fab.py first. Through-hole connectors (BNC jacks, Phoenix DUT
clamps, SWD header) are excluded -- hand-solder those; the USB-C
receptacle is SMT and stays in.

The JLCPCB Part # column is left blank on purpose: pick the parts in
JLC's BOM-matching UI (it matches on Comment/MPN) rather than trusting
hardcoded LCSC numbers that may have gone end-of-life.
"""

import csv
import math
import os
import re
import xml.etree.ElementTree as ET
from collections import defaultdict

import openpyxl

from generate_bom import PART_NUMBERS

HERE = os.path.dirname(os.path.abspath(__file__))

#: Pin generic passives to JLC *Basic* library parts: every Extended line
#: item adds a ~$3 loading fee, Basic adds none. C45783/C15849/C25905/
#: C17477 were confirmed by JLC's own matching report for this BOM; the
#: rest are the canonical evergreen Basic picks -- the matching UI shows
#: what each number resolves to, confirm there before ordering.
JLC_BASIC = {
    ("22uF", "C_0805_2012Metric"): ("CL21A226MAQNNNE", "C45783"),
    ("1uF", "C_0603_1608Metric"): ("CL10A105KB8NNNC", "C15849"),
    ("100nF", "C_0402_1005Metric"): ("CL05B104KO5NNNC", "C1525"),
    ("5.1k", "R_0402_1005Metric"): ("0402WGF5101TCE", "C25905"),
    ("10k", "R_0402_1005Metric"): ("0402WGF1002TCE", "C25744"),
    ("1k", "R_0402_1005Metric"): ("0402WGF1001TCE", "C11702"),
    ("4.7k", "R_0402_1005Metric"): ("0402WGF4701TCE", "C25900"),
    # 5.1 mm SMD tact switch, 4 pads on the TL3342 pattern; confirm the
    # land pattern against JLC's matching report before ordering.
    ("BOOT", "SW_SPST_TL3342"): ("TS-1187A-B-A-B", "C318884"),
    ("RESET", "SW_SPST_TL3342"): ("TS-1187A-B-A-B", "C318884"),
    ("0R", "R_0805_2012Metric"): ("0805W8F0000T5E", "C17477"),
    # status LEDs: only the red one is a confirmed JLC part; green / amber /
    # blue are left to the matcher (confirm colour in its report)
    ("PROG", "LED_0603_1608Metric"): ("KT-0603R", "C2286"),
    # Confirmed by JLC's second matching report (2026-08-31) -- these are
    # the parts its own matcher resolved for this BOM. D2 and U5 are the
    # TECH PUBLIC clones JLC auto-picked (cheapest); swap to the ST/Diodes
    # originals in the UI if preferred.
    # RC1 shunt halves (R14, R15): UNI-ROYAL thin film 4R7 0.1 % 25 ppm, the
    # same grade as the Vishay TNPW08054R70BEEA that JLC does not stock
    # (jlcsearch 2026-10-12: 493 pcs; the 1 % thick film C17675 used before
    # showed 0). If it runs out: FOJAN FRP0805F4R70TS, C49196883 (1 %,
    # 200 ppm, 55k pcs) -- OSL calibration against R5 absorbs the value error.
    ("4R7", "R_0805_2012Metric"): ("TC0525B470KT5G", "C21571495"),
    ("USBLC6-2SC6", "SOT-23-6"): ("USBLC6-2SC6", "C2827654"),
    ("USB-C", "USB_C_Receptacle_HRO_TYPE-C-31-M-12"):
        ("TYPE-C-31-M-12", "C165948"),
    # PROTOTYPE SUBSTITUTION: standard G6K-2F-Y instead of the $9.3 RF
    # version. Same land pattern per Omron datasheets (rows 3.2/5.4/7.6,
    # contact pads 1.8x0.8 on 7.0 mm span; the -Y coil terminal is
    # narrower than the RF-S pad, which is fine), same 5 V/237R coil,
    # same DPDT arrangement. Loses 50R impedance control -- irrelevant
    # below 50 MHz. Production board: G6K-2F-RF-S DC5 (C2750987).
    ("G6K-2F-RF-S DC5", "Relay_Omron_G6K-2F-RF-S"):
        ("G6K-2F-Y-TR DC5", "C47190"),
    ("100R 0.1%", "R_0805_2012Metric"): ("TNPW0805100RBEEA", "C2073390"),
    ("1k 0.1%", "R_0805_2012Metric"): ("TNPW08051K00BEEA", "C2073519"),
    # Vishay TNPW080547R0BEEA shows 1 pc; same grade from Yageo (0.1 %, 25 ppm)
    ("47R 0.1%", "R_0805_2012Metric"): ("RT0805BRD0747RL", "C136921"),
    ("STM32F072CBT6", "LQFP-48_7x7mm_P0.5mm"): ("STM32F072CBT6", "C81720"),
    ("TBD62003APG", "SOIC-16_3.9x9.9mm_P1.27mm"):
        ("TBD62003AFG(Z,EL)", "C163227"),
    ("AP2112K-3.3", "SOT-23-5"): ("AP2112K-3.3TRG1", "C23380830"),
    # rev C additions (numbers confirmed on jlcpcb.com part pages,
    # 2026-10-11; anything not listed here is left to JLC's matcher)
    ("10nF", "C_0402_1005Metric"): ("CL05B103KB5NNNC", "C15195"),
    ("4.7uF", "C_0805_2012Metric"): ("CL21A475KAQNNNE", "C1779"),
    ("1k 0.1%", "R_0603_1608Metric"): ("RT0603BRD071KL", "C110776"),
    ("600R@100MHz", "L_0603_1608Metric"): ("GZ1608D601TF", "C1002"),
    # checked against LCSC's product database 2026-10-12 (code -> MPN, value,
    # package); JLC stock still to be confirmed in its matching report
    ("10uF", "C_0805_2012Metric"): ("CL21A106KAYNNNE", "C15850"),
    ("1uF", "C_0402_1005Metric"): ("CL05A105KA5NQNC", "C52923"),
    ("1uF", "C_0603_1608Metric"): ("CL10A105KB8NNNC", "C15849"),
    ("100nF", "C_0402_1005Metric"): ("CL05B104KO5NNNC", "C1525"),
    ("47uF", "C_1206_3216Metric"): ("CL31A476MPHNNNE", "C96123"),
    ("BAV99", "SOT-23"): ("BAV99,215", "C2500"),
    ("100k", "R_0402_1005Metric"): ("0402WGF1003TCE", "C25741"),
    ("100R", "R_0402_1005Metric"): ("0402WGF1000TCE", "C25076"),
    ("49.9R", "R_0402_1005Metric"): ("0402WGF499JTCE", "C25120"),
    ("18k", "R_0402_1005Metric"): ("0402WGF1802TCE", "C25762"),
    ("0R", "R_0603_1608Metric"): ("0603WAF0000T5E", "C21189"),
    # LEDs: XINGLIGHT XL-1608 (the Everlight 19-217/19-213 codes show 0-1 pcs);
    # yellow-green for PWR/RUN (Vf 2.2 V: an emerald 3.3 V part would barely
    # light from +3V3), yellow stands in for amber
    ("RUN", "LED_0603_1608Metric"): ("XL-1608SYGC-06", "C965805"),
    ("PWR", "LED_0603_1608Metric"): ("XL-1608SYGC-06", "C965805"),
    ("KELVIN", "LED_0603_1608Metric"): ("XL-1608UBC-04", "C965807"),
    ("BUSY", "LED_0603_1608Metric"): ("XL-1608UYC-06", "C965802"),
    ("TMP117AIDRV", "WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm"): ("TMP117AIDRVR", "C699536"),
    ("OPA356", "SOT-23-5"): ("OPA356AIDBVR", "C183100"),
    ("TPS2051CDBV", "SOT-23-5"): ("TPS2051CDBVR", "C129581"),
    ("SMF5.0A", "D_SOD-123F"): ("SMF5.0A", "C151296"),
    ("220R@100MHz 2A", "L_0805_2012Metric"): ("BLM21PG221SN1D", "C85840"),
    # -T002 (the BOM part) shows 0 at JLC; -T001 is the same choke on another reel
    ("90R CM", "L_CommonModeChoke_Coilank_ACM2012"): ("ACM2012-900-2P-T001", "C307642"),
    # CNC Tech 3220-10-0300-00 shows 0; same land pattern (pad-for-pad match)
    ("SWD", "PinHeader_2x05_P1.27mm_Vertical_SMD"): ("HX PZ1.27-2x5P TP", "C41376037"),
    ("1R 0.5%", "R_1206_3216Metric"): ("RT1206DRD071RL", "C111291"),
    # +/-2 % (0.2 pF) is the tightest 10 pF C0G 0603 in stock; the check
    # standard's pass band is 10 %
    ("10pF C0G", "C_0603_1608Metric"): ("GRM1885C1H100GA01D", "C2987939"),
}

#: Options that are not fitted (USB shield bleed / RF bond): no BOM line, no placement.
NOT_FITTED = {"C31", "R36"}


def jlc_footprint(tail):
    """JLC's matcher parses sizes out of the footprint name and reads the
    KiCad metric suffix (C_0402_1005Metric) as an 01005 -- feed it plain
    imperial names like its own sample (R0603) instead."""
    m = re.match(r"^(C|R|L|LED)_(\d{4})_\d{4}Metric$", tail)
    if m:
        return m.group(1) + m.group(2)
    for prefix in ("SOIC-16", "LQFP-48"):
        if tail.startswith(prefix):
            return prefix
    return tail


def jlc_comment(value, part):
    """Bare MPN only: JLC strips spaces from comments, so a manufacturer
    prefix ('ST USBLC6-2SC6' -> 'STUSBLC6-2SC6') kills the part search."""
    if value == "PWR":
        return "LTST-C190GKT"      # generic green 0603 LED
    if part:
        return part.split(" ", 1)[1] if " " in part else part
    return value


#: Hand-soldered through-hole parts, not for the SMT line.
HAND_SOLDER = {f"J{n}" for n in range(10, 18)} | {   # DUT test clips (Grayhill 02-21)
               "J6", "J7", "J9"}         # BNC jacks (J8 SWD is SMD now: JLC places it)


def load_components():
    root = ET.parse(os.path.join(HERE, "relay_board.xml")).getroot()
    components = {}
    for comp in root.find(".//components"):
        reference = comp.get("ref")
        # TP* are bare copper pads, NT* net ties are copper: nothing to place.
        if (reference.startswith("#") or reference in HAND_SOLDER or reference in NOT_FITTED
                or reference.startswith(("TP", "NT"))):
            continue
        value = comp.find("value").text if comp.find("value") is not None else ""
        footprint_node = comp.find("footprint")
        footprint = footprint_node.text if footprint_node is not None else ""
        components[reference] = (value, footprint.split(":")[-1])
    return components


def sort_key(reference):
    return (reference.rstrip("0123456789"),
            int("".join(filter(str.isdigit, reference)) or 0))


def write_bom(components):
    groups = defaultdict(list)
    for reference, key in components.items():
        groups[key].append(reference)

    wb = openpyxl.Workbook()
    ws = wb.active
    ws.append(["Comment", "Designator", "Footprint",
               "JLCPCB Part #（optional）"])
    for (value, tail), references in sorted(groups.items(),
                                            key=lambda kv: sorted(
                                                kv[1], key=sort_key)[0]):
        references.sort(key=sort_key)
        part = PART_NUMBERS.get((value, tail), ("", ""))[0]
        comment, lcsc = JLC_BASIC.get(
            (value, tail), (jlc_comment(value, part), ""))
        ws.append([comment, ",".join(references),
                   jlc_footprint(tail), lcsc])

    destination = os.path.join(HERE, "relay_board_JLC_BOM.xlsx")
    wb.save(destination)
    print(f"JLC BOM: {destination} ({len(groups)} line items, "
          f"{len(components)} placements)")


#: JLC places each part in its own library footprint's zero orientation, which
#: is not KiCad's. footprint -> (degrees added to the KiCad rotation, offset of
#: JLC's footprint origin from ours in footprint coordinates, mm). Derived by
#: overlaying JLC's library footprint (EasyEDA API, per LCSC code) on ours pad
#: for pad, 2026-10-12; anything not listed needs neither. Without this the
#: relays, MCU, drivers and SOT-23s arrive turned (rev B: turned by hand in
#: JLC's placement viewer).
CPL_FIX = {
    "Relay_Omron_G6K-2F-RF-S": (270, (0.0, 0.0)),
    "LQFP-48_7x7mm_P0.5mm": (270, (0.0, 0.0)),
    "SOIC-16_3.9x9.9mm_P1.27mm": (270, (0.0, 0.0)),
    "SOT-23-6": (270, (0.0, 0.0)),
    "SOT-23-5": (180, (0.0, 0.0)),
    "SOT-23": (180, (0.0, 0.0)),
    "PinHeader_2x05_P1.27mm_Vertical_SMD": (270, (0.0, 0.0)),
    "USB_C_Receptacle_HRO_TYPE-C-31-M-12": (0, (0.0, -1.58)),
}


def jlc_placement(tail, x, y, rotation):
    """KiCad position file values (y up) -> JLC's for the same part."""
    turn, (dx, dy) = CPL_FIX.get(tail, (0, (0.0, 0.0)))
    angle = math.radians(rotation)
    x += dx * math.cos(angle) + dy * math.sin(angle)
    y -= -dx * math.sin(angle) + dy * math.cos(angle)
    return x, y, (rotation + turn) % 360


def write_cpl(components):
    positions = os.path.join(HERE, "fab", "relay_board_pos.csv")
    if not os.path.exists(positions):
        raise SystemExit("fab/relay_board_pos.csv missing -- run export_fab.py")

    wb = openpyxl.Workbook()
    ws = wb.active
    ws.append(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
    count = 0
    with open(positions, newline="") as handle:
        for row in csv.DictReader(handle):
            reference = row["Ref"]
            if reference not in components:
                continue
            x, y, rotation = jlc_placement(components[reference][1], float(row["PosX"]),
                                           float(row["PosY"]), float(row["Rot"]))
            ws.append([reference, f"{x:.4f}mm", f"{y:.4f}mm",
                       "Top" if row["Side"] == "top" else "Bottom", rotation])
            count += 1

    missing = set(components) - {
        row[0].value for row in ws.iter_rows(min_row=2)}
    if missing:
        raise SystemExit(f"components missing from pos file: {sorted(missing)}")

    destination = os.path.join(HERE, "relay_board_JLC_CPL.xlsx")
    wb.save(destination)
    print(f"JLC CPL: {destination} ({count} placements, all Top)")


def main():
    components = load_components()
    write_bom(components)
    write_cpl(components)


if __name__ == "__main__":
    main()
