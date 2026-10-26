# Faraday (OpenConverters) -- user feedback log

Feedback from using [Faraday](https://github.com/OpenConverters/Faraday) (commit cloned
2026-10-11, `faraday_cli`) on the OpenMagnetics relay board: rev B (KiCad 9, 4 layers,
routed fab board) and rev C (KiCad 10, 6 layers, JLC06161H-3313). Entry point here:
`python faraday_review.py [board]`. New items are appended at the end with a date.

## What works well

- Findings are actionable: mechanism, a number, a confidence statement, a remediation.
  The calibration caveat ("closed form ~6.5 dB optimistic, ranking reliable") is honest
  and exactly what a reviewer needs.
- It found real issues on rev C that our own audits missed: 96 mm open stubs on the
  sense rails (400 MHz resonators), decoupling caps reaching ground through 13-15 mm,
  the USB ESD clamp 5 mm from the pin (~122 V overshoot), and connector ground spread.
- On rev B it independently confirmed the problems rev C was redesigned for (no
  reference plane, USB clamp 12 mm away, USB pair skew 17 mm, decoupling 13 mm away).
- Strict stackup validation and no silent default stackup.
- Fast: a full 6-layer, 117-part board in seconds.
- Bundle reporting (24 GPIO lines as one group with summed NEXT) is better than
  276 pairwise findings.

## Problems and suggestions

1. **Does not build with MSVC.** `M_PI` is used in GerberImporter, KicadImporter,
   Tline, Screener and Fixes; MSVC only defines it with `_USE_MATH_DEFINES`. Built here
   with `-DCMAKE_CXX_FLAGS="/D_USE_MATH_DEFINES /EHsc /utf-8"`. Suggest
   `std::numbers::pi` (the code is already C++20) and a Windows CI job.
2. **KiCad 10 boards fail with an opaque error** (`faraday: SExpr: not a number:
   '/+3V3'`). KiCad 10 (file version 20260206) writes `(net "/NAME")` with no numeric
   code and no top-level net table. Suggest keying nets by name (works for v5-v10), or
   at least detecting the version and saying "KiCad 10 is not supported yet". Workaround:
   `faraday_review.py` writes a KiCad-9-shaped copy (numbers every net, restores the table).
3. **`faraday_cli --help` is read as a board path** ("cannot open --help"). A usage
   message for `--help`/`-h`/no arguments would help.
4. **BNC cable entries missed by `connector-ground-spread` on rev B.** It paired USB-C
   with the SWD header but ignored the three BNCs, whose shells sit on a net named
   `AGND`. Coax shell pads (and AGND/PGND/CHASSIS/SHIELD nets) should count as cable
   grounds; on rev B the AGND-to-GND split *is* the cable-CM problem the rule exists for.
5. **Only the nearest plane counts as a reference.** On rev C the In2 sense rails are
   referenced by a GND strip on In4 (0.67 mm away) while In1 (0.55 mm, voided there) is
   nearer; the earlier run flagged "In1 does not reach there". Considering every plane
   within a few dielectrics, weighted by distance, would be more faithful.
6. **No way to declare intentional voids from the CLI.** Measurement fixtures void the
   planes under high-impedance nodes on purpose (here: DUT arms, terminal buses, links,
   for Cps accuracy). They rank as HIGH return-path breaks. The GUI has per-finding
   dismissal, but CI needs a waiver file (net patterns or named rule areas + a reason).
   Workaround: `faraday_review.py` applies such a list and prints the reasons.
7. **Findings carry net codes, not names** (`netA`/`netB` are integers), so every script
   has to join `board.nets`. Including the names in each finding would help.
8. **`severity` is a float score** while `severityLabel` is the category; the README
   calls them severities. Worth documenting the score's meaning and range.

## Domain notes (relay matrix / impedance fixture use)

- The return-path rule is right for the force rails: their loop is the radiating
  structure, and a GND strip under them is calibrated out because they always sit at
  the HI or LO potential. A fixture-aware mode could distinguish "nodes at a fixed
  potential" (planes fine) from "floating / intermediate-potential nodes" (planes cost
  accuracy), which is the real trade-off on this class of board.

## 2026-10-12 -- rev C routed board (4 layers, JLC04161H-7628), Faraday d73ca92 built on Linux (WSL)

- Builds cleanly with g++ / CMake / Ninja on Ubuntu 22.04 (`-DFARADAY_BUILD_TESTS=OFF`, target `faraday_cli`).
- **Unfilled zones read as "no reference plane"** on every layer (4 HIGH findings). A board saved
  before zone fill is a common state for generated boards; a hint ("zones are not filled") instead
  of a plane finding would save a wrong conclusion.
- **Grouped findings carry no net ids** ("Return-path breaks on B.Cu: 63 nets", `netA`/`netB` absent),
  so per-net waivers no longer match: `faraday_review.py` waived 0 of 135. A `nets` array on grouped
  findings would let a waiver list apply.
- New useful findings on this board: VBUS TVS 10 mm from the USB-C pin (~243 V overshoot estimate);
  C1 12 mm from the coil switch; 43 open stubs (unused tails of fine-pitch escape stubs).
- `esd-clamp-distance` picked D9 (TVS) and did not count the USBLC6's VBUS pin as a clamp on that net.
- The relay-drive lanes (23 static lines) are reported as coupled bundles with 3W violations; a way
  to mark nets as static (DC during the measurement) would move them out of the way.

## 2026-10-13 -- final rev C board (two signal layers, solid inner GND with openings under floating nodes)

- **Open stubs: 96 reported, 91 of them T-junctions** (a track end on the body of another track of
  its net; KiCad DRC had 0 dangling). After splitting the tracks at every junction: 0. The rule
  should accept an end that lies in the copper of another segment of its net.
- **Grouped plane-crossing length** is the length of every segment that touches an opening, not
  the uncovered length (2660 mm reported vs 1283 mm uncovered on B.Cu; 330 vs 1190 mm on F.Cu).
- **Rings of plane openings**: KiCad removes the enclosed ground as an island; Faraday showed the
  result (a third of the outer copper uncovered) without pointing at the cause. A hint when a
  large plane area is missing next to a keepout ring would have saved a routing iteration.
- **diff-skew** reported 9 mm on the front-end input pair; the signal paths are 2 mm each, the rest
  is a branch to a 100 k bias resistor.
- **esd-clamp-distance** still does not count the USBLC6's VBUS pin; **decoupling-distance** still
  flags the bulk capacitors of ICs that have a capacitor at the pin; relay-drive lanes and coil
  returns (static) still fill the bundle / 3W findings.
- Result on the final board: 92 findings (3 high: two grouped plane-crossings that are the
  intentional openings, one ESD distance), none an open stub.
