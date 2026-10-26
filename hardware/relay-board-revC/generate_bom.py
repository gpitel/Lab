"""
Generate the BOM and assembly data from the exported netlist.

    python generate_bom.py

Everything derives from relay_board.xml -- the same file verify_board.py
simulates -- so the BOM cannot describe a different board than the one
verified (which is exactly what happened to rev A, whose hand-written BOM
listed connectors the schematic never had).
"""

import os
import xml.etree.ElementTree as ET
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))

#: Orderable part numbers keyed by (value, footprint-tail). Anything not
#: listed ships as generic passives per value/size.
PART_NUMBERS = {
    ("G6K-2F-RF-S DC5", "Relay_Omron_G6K-2F-RF-S"):
        ("Omron G6K-2F-RF-S DC5", "high-frequency DPDT relay, 5 V coil"),
    ("STM32F072CBT6", "LQFP-48_7x7mm_P0.5mm"):
        ("ST STM32F072CBT6",
         "Cortex-M0, 128K flash, crystal-less USB (HSI48 + CRS)"),
    ("TBD62003APG", "SOIC-16_3.9x9.9mm_P1.27mm"):
        ("Toshiba TBD62003AFG", "7ch DMOS sink driver (SOIC-16 = AFG suffix)"),
    ("AP2112K-3.3", "SOT-23-5"):
        ("Diodes AP2112K-3.3TRG1", "3.3 V 600 mA LDO, 250 mV dropout"),
    ("USBLC6-2SC6", "SOT-23-6"):
        ("ST USBLC6-2SC6", "USB 2.0 ESD array, IEC 61000-4-2 level 4"),
    ("USB-C", "USB_C_Receptacle_HRO_TYPE-C-31-M-12"):
        ("HRO TYPE-C-31-M-12", "USB-C receptacle, USB 2.0, 16 pin"),
    ("100R 0.1%", "R_0805_2012Metric"):
        ("Vishay TNPW0805100RBEEA", "LOAD standard: 100R 0.1% 25 ppm thin film"),
    ("1k 0.1%", "R_0805_2012Metric"):
        ("Vishay TNPW08051K00BEEA", "bridge RV1: 1k 0.1% 25 ppm thin film"),
    ("47R 0.1%", "R_0805_2012Metric"):
        ("Vishay TNPW080547R0BEEA",
         "bridge RV2/RC2: 47R 0.1% 25 ppm thin film"),
    ("4R7", "R_0805_2012Metric"):
        ("UNI-ROYAL TC0525B470KT5G (LCSC C21571495)",
         "bridge RC1 shunt half: 4R7 0.1% 25 ppm thin film, JLC stock "
         "(two in parallel = 2R35); equivalent: Vishay TNPW08054R70BEEA"),
    ("BNC SOURCE", "BNC_Amphenol_B6252HB-NPP3G-50_Horizontal"):
        ("Amphenol B6252HB-NPP3G-50", "50R right-angle BNC jack"),
    ("BNC CH1", "BNC_Amphenol_B6252HB-NPP3G-50_Horizontal"):
        ("Amphenol B6252HB-NPP3G-50", "50R right-angle BNC jack"),
    ("BNC CH2", "BNC_Amphenol_B6252HB-NPP3G-50_Horizontal"):
        ("Amphenol B6252HB-NPP3G-50", "50R right-angle BNC jack"),
    ("BOOT", "SW_SPST_TL3342"):
        ("C&K TL3342F160QG (or XKB TS-1187A-B-A-B)",
         "SMD tact switch: hold at power-up for ROM DFU"),
    ("RESET", "SW_SPST_TL3342"):
        ("C&K TL3342F160QG (or XKB TS-1187A-B-A-B)", "SMD tact switch: NRST"),
    ("PWR", "LED_0603_1608Metric"): ("0603 LED green", "status: power"),
    ("RUN", "LED_0603_1608Metric"): ("0603 LED green", "status: firmware running / USB host"),
    ("BUSY", "LED_0603_1608Metric"): ("0603 LED amber", "status: relays switching"),
    ("KELVIN", "LED_0603_1608Metric"): ("0603 LED blue", "status: Kelvin mode"),
    ("PROG", "LED_0603_1608Metric"): ("0603 LED red (KT-0603R)", "status: DFU / firmware update"),
    ("SWD", "PinHeader_2x05_P1.27mm_Vertical_SMD"):
        ("CNC Tech 3220-10-0300-00", "1.27 mm 2x5 Cortex debug header, SMD"),
    ("OPA356", "SOT-23-5"):
        ("TI OPA356AIDBVR", "200 MHz RRO op-amp, single 2.5-5.5 V (Kelvin front-end)"),
    ("BAV99", "SOT-23"): ("Nexperia BAV99,215", "front-end input clamp"),
    ("90R CM", "L_CommonModeChoke_Coilank_ACM2012"):
        ("TDK ACM2012-900-2P-T002", "USB CM choke, 90R @100 MHz; windings 1-4 / 2-3"),
    ("220R@100MHz 2A", "L_0805_2012Metric"):
        ("Murata BLM21PG221SN1D", "VBUS ferrite, 2 A"),
    ("600R@100MHz", "L_0603_1608Metric"):
        ("Sunlord GZ1608D601TF", "ferrite, 200 mA (VDDA, +5VA)"),
    ("1k 0.1%", "R_0603_1608Metric"):
        ("Yageo RT0603BRD071KL", "front-end difference stage, 0.1 % 25 ppm"),
    ("47uF", "C_1206_3216Metric"):
        ("X5R 10 V 1206", "front-end output DC block"),
}

CLAMP_NOTE = ("Grayhill 02-21 miniature test clip (banana plug)",
              "Kelvin pair per terminal (F/S, 8 mm apart); plug straight into the plated 4.0 mm hole, "
              "hand-fitted (solder for a permanent fixture)")

#: Mechanical parts that are not components: (item, quantity, part, note)
HARDWARE = [
    ("Standoff M3 x 25 mm, F-F", 5, "Wurth 970250321 (WA-SSTII, steel; or any M3 hex standoff >= 25 mm)",
     "legs under H1-H5: the 02-21 banana plugs stand ~22 mm below the board"),
    ("Screw M3 x 6 mm", 5, "any", "board to standoffs"),
]


def main():
    root = ET.parse(os.path.join(HERE, "relay_board.xml")).getroot()
    groups = defaultdict(list)
    for comp in root.find(".//components"):
        reference = comp.get("ref")
        if reference.startswith(("#", "NT", "TP")):
            continue                    # net ties / test pads: copper only
        value = comp.find("value").text if comp.find("value") is not None else ""
        footprint_node = comp.find("footprint")
        footprint = footprint_node.text if footprint_node is not None else ""
        tail = footprint.split(":")[-1]
        groups[(value, tail)].append(reference)

    lines = [
        "# OpenMagnetics Relay Board rev C -- Bill of Materials",
        "# GENERATED by generate_bom.py from relay_board.xml -- do not edit.",
        "",
        "| Refs | Qty | Value | Part number | Notes |",
        "|------|-----|-------|-------------|-------|",
    ]
    total = 0
    for (value, tail), references in sorted(groups.items(),
                                            key=lambda kv: kv[1][0]):
        references.sort(key=lambda r: (r.rstrip("0123456789"),
                                       int("".join(filter(str.isdigit, r)) or 0)))
        part, note = PART_NUMBERS.get((value, tail), ("-", tail))
        if value.startswith("DUT "):
            part, note = CLAMP_NOTE
        refs = ",".join(references) if len(references) <= 6 else (
            f"{references[0]}-{references[-1]}")
        lines.append(f"| {refs} | {len(references)} | {value} | {part} | {note} |")
        total += len(references)

    lines += [
        "",
        f"Total placements: {total}",
        "",
        "Hardware (not in the netlist):",
        "",
        "| Item | Qty | Part | Notes |",
        "|------|-----|------|-------|",
    ] + [f"| {item} | {quantity} | {part} | {note} |" for item, quantity, part, note in HARDWARE] + [
        "",
        "Notes:",
        "- All components on the top side; single-sided assembly.",
        "- R5 is the calibration LOAD standard: substitutions must hold 0.1%",
        "  and low TCR or every calibrated measurement inherits the error.",
        "- Relays: G6K-2F-RF-S DC5 (5 V coil suits the USB-derived +5 V rail",
        "  through the TBD62003's ~0.15 V drop). ~26 week lead time -- order first.",
        "- USB VID/PID are pid.codes test values; register before distributing.",
    ]

    destination = os.path.join(HERE, "relay_board_BOM.md")
    with open(destination, "w") as handle:
        handle.write("\n".join(lines) + "\n")
    print(f"BOM written: {destination} ({total} placements, {len(groups)} line items)")

    # CSV for the fab/assembly house: fully-enumerated designators, one
    # line item per (value, footprint) group.
    import csv
    csv_destination = os.path.join(HERE, "relay_board_BOM.csv")
    with open(csv_destination, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["Comment", "Designator", "Footprint", "Quantity",
                         "Manufacturer Part", "Description"])
        for (value, tail), references in sorted(groups.items(),
                                                key=lambda kv: kv[1][0]):
            part, note = PART_NUMBERS.get((value, tail), ("", tail))
            if value.startswith("DUT "):
                part, note = CLAMP_NOTE
            writer.writerow([value, ",".join(references), tail,
                             len(references), part, note])
    print(f"BOM CSV written: {csv_destination}")


if __name__ == "__main__":
    main()
