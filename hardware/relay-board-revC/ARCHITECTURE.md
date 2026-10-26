# Relay board rev C -- architecture and review response

Status: design in progress (2026-10-11). This file is the source of intent for
`generate_schematic.py`, `verify_board.py`, `generate_pcb.py`, the firmware relay
map and `scripts/RelayBoardController.py`.

## 1. What changes from rev B, and why

Rev B measures two-wire: the bridge sees the DUT plus everything between the OSL
plane (iso relay contacts) and the DUT leads, and every path has its own bus,
relays and rail length. The bench bring-up (TODO.md) and N. Rosano's review both
land on the same root cause: **series path impedance and fixture capacitance that
the calibration cannot see**. Rev C attacks it the way commercial transformer
testers do (Voltech AT series: Kelvin force/sense to every DUT pin through the
switching matrix, plus fixture compensation):

| Change | Rev B | Rev C |
|---|---|---|
| Kelvin | 2-wire to the clamps | **force + sense per terminal**: clamp pad 1 = F, pad 2 = S. Every signal relay is DPDT: pole 1 switches force, pole 2 switches the same terminal's sense. Sense rails go to an on-board differential front-end that can drive CH1 |
| 2-wire mode | -- | tie relay K22 (de-energized = tied) joins SNS_HI/SNS_LO to RAIL_HI/RAIL_LO: sense becomes a parallel force path, so 2-wire mode halves contact R and arm L and keeps the B-WIC bridge exactly as in rev B |
| Links | LINK rail + 4 column relays, 14.5 pF to GND, 39-94 nH loops | 4 direct bus-to-bus relays A-B, C-D, B-C, B-D (covers every link config in use: 2, 4, 5, 6, 18, 19) |
| SHORT standard | closes the HI terminal's LO relay: skips the LO terminal's bus + contact | unchanged on board (an exact Kelvin zero on this topology); the residual is removed by clamp-plane fixture compensation. Four SHORT relays were designed, simulated and dropped (section 3) |
| Rail feeds | both near the middle: HI+LO = 88/44/44/88 mm | HI fed at the A end, LO at the D end: equal for every column |
| Standards | R5 100R 2-wire | standards column: K19..K21 select 100R (LOAD), 1R and 10 pF (check standards), each Kelvin-sensed at its pads |
| CH1 source | divider only | K23 selects divider (2-wire, default) or front-end output (Kelvin) |
| Relays | 18 | 23 (8 matrix, 2 standards column, 4 iso, 4 link, 3 standard select, tie, CH1 select) |
| Stackup | 4 layers | 4 layers JLC04161H-7628, SIG/GND/GND/SIG: tracks on the outer layers only; the inner layers are solid GND except openings that follow the floating nodes (terminal buses, arms, clips) |

Relay map (bit n-1 drives K n):

| K | function | pole 1 (force) | pole 2 (sense) |
|---|---|---|---|
| 1..8 | A-HI, A-LO, B-HI, B-LO, C-HI, C-LO, D-HI, D-LO | F bus -> RAIL_HI/LO | S bus -> SNS_HI/LO |
| 9, 10 | standards column E -> HI, LO | CAL_E/F -> RAIL_HI/LO | CAL_ES/FS -> SNS_HI/LO |
| 11..14 | iso A..D (energized = isolated) | NC: clamp F, COM: F bus | NC: clamp S, COM: S bus |
| 15..18 | link AB, CD, BC, BD | F bus - F bus | S bus - S bus |
| 19..21 | select 100R / 1R / 10 pF | CAL_E -> standard pad 1 | CAL_ES -> standard sense tie |
| 22 | 2-wire tie (energized = Kelvin) | NC: SNS_HI = RAIL_HI | NC: SNS_LO = RAIL_LO |
| 23 | CH1 select (energized = Kelvin) | COM: CH1 BNC, NC: divider, NO: front-end | unused |

## 2. Kelvin front-end

CH2 still measures current through the RC1/RC2 shunt on the LO force rail
(ground-referenced, unchanged). In Kelvin mode CH1 measures `SNS_HI - SNS_LO`
through a differential amplifier, so the force path (relay contacts that move
on every actuation, buses, arms) drops out of the voltage. The front-end is a
three-op-amp instrumentation amplifier on a single 5 V analog rail (filtered
VBUS), AC-coupled at the inputs (bias to mid-supply, corner < 2 Hz) and at
the output (into the Bode 50 ohm input), gain set by 0.1 % resistors. Its gain
and phase sit inside the Kelvin OSL (below), so they need to be stable, not
exact.

## 3. Calibration

Three layers, from fast and automatic to one-time and manual:

1. **Per-path OSL at the iso-relay plane** (automatic, DUT stays clamped, every
   configuration in both modes): OPEN = iso energized; SHORT = iso + the first HI
   terminal's LO relay -- force rails bridged through that bus, and in Kelvin mode
   both sense rails meet on its current-free sense bus, an exact zero; LOAD = 100R
   0.1 % through the standards column, sensed at its pads.
2. **Verification after every OSL** (automatic, rev C): the 1R and 10 pF check
   standards are read through the fresh correction and recorded in the calibration
   sidecar with pass/fail (1R: 1 % Kelvin, 15 % 2-wire; 10 pF: 10 %).
   `MagneticCharacterizer._verify_check_standards`.
3. **Clamp-plane fixture compensation** (one time per board, operator: nothing in the
   clamps, then a shorting bar across all four clamps; `bench.py
   fixture-compensation`): Keysight open/short compensation per signal path,
   applied to every sweep (`FixtureCompensation.py`). It removes what the iso
   plane cannot see -- the arms, the clamps and the SHORT path's residual -- and
   supersedes rev B's board-path correction for configs 2/4.

**Why no SHORT relays.** A version with four SHORT relays on the iso NO throws
(a short through both used buses to a Kelvin star) was built and evaluated:

| Residual left by the on-board SHORT | rev B style | SHORT relays |
|---|---|---|
| 2-wire, A-B, lumped model (`simulate_calibration.py`) | +100 mohm, +29 nH | -100 mohm, -31 nH, -89 fF on Cps |
| 2-wire loop L, A-B / A-C / C-D (`kelvin_coupling.py`) | +68 / +25 / +48 nH | -16 / -98 / -24 nH |
| Kelvin force-sense coupling, A-B / A-C / C-D | +2.9 / +3.1 / +13.3 nH | +5.7 / +16.6 / -3.5 nH |

Neither on-board short reaches the DUT plane. Commercial practice is a
clamp-plane short (Keysight handbook 4.3: "Short measurement is performed by
connecting the test fixture terminals directly together"; Voltech AT: a dummy
header with shorting links, compensation stored per test), which removes either
residual completely. With that in place the four relays only added open-contact
capacitance and cross-bay star copper, so they were dropped and their slots went
to the check standards.

## 4. Response to the review (N. Rosano, 2026-10)

| # | Point | Rev B status | Rev C |
|---|---|---|---|
| 1 | DUT soldered, 4 THT terminals, no CT/planar | Rev B already uses spring clamps (no soldering). Still 4 terminals | Kelvin clamps (F/S) per terminal accept Kelvin clip leads or a DUT carrier board. **Open:** 6 terminals for CT/3-winding parts costs +6 relays and board area -- decision deferred to rev D unless asked |
| 2 | Contact R 50-100 mOhm, moves, no 4-wire: Rac no good | Agreed: rev B Rac is not trustworthy below ~1 ohm | Full Kelvin through pole 2: contacts carry no sense current, measurement plane at the clamp |
| 3 | Several nH, different per path: < 10 nH unresolved | Measured: -39/-33/+4 nH column asymmetry, 39-94 nH link loops | Kelvin removes series path L from the voltage; end-fed rails and direct link relays equalize/shorten what remains (2-wire) |
| 4 | Open relays 0.5-1 pF each add up; fixture dominates Cps | Measured: ~14.5 pF on LINK, ~2 pF per isolated clamp | LINK net gone; floating terminals isolated; fixture OPEN compensation; RF-S relays (lower open C) |
| 5 | Stubs/long tracks: resonances, few-MHz bandwidth? | Checked: SPICE vs measured within 0.2-2.5 % to 50 MHz on 13/16 configs | Shorter buses (no LINK rail), parasitic report gates every path |
| 6 | Each path needs its own cal; joint at DUT never calibrated | Each path does have its own automatic OSL | Kelvin plane at the DUT contact; clamp-plane open/short compensation per path; 1R and 10 pF check standards verify every OSL (section 3) |
| 7 | BNC not fitted: where is the cal plane? | Rev B integrates the B-WIC bridge with three BNCs: no adapter | Unchanged, plus the Kelvin front-end |
| 8 | No keepout under DUT? | Rev B voids all planes under the bay and signal region | Kept; the 4-layer stack keeps planes only outside the measurement void |

## 5. EMC pre-compliance and layout rules

Targets (EU): EN IEC 61326-1:2021 (laboratory and measurement equipment, the product-family
standard that takes precedence over EN 55032/55035) -- emissions to CISPR 11 / EN 55011 Group 1
**Class B** with 6 dB margin; immunity ESD (IEC 61000-4-2, 4 kV contact / 8 kV air, criterion B),
radiated RF (IEC 61000-4-3, 3 V/m basic / 10 V/m industrial, criterion A) and power-frequency
magnetic field (IEC 61000-4-8). Requirements, design checklist, open points, pre-compliance plan
and CE paperwork: [EMC_COMPLIANCE.md](EMC_COMPLIANCE.md).

| Rule | Implementation | Source |
|---|---|---|
| Radiation comes from RF voltage between cable shields (USB vs BNC): no common-mode voltage between them | one unified ground; rev B's AGND island + 0R tie removed; USB-C and all three BNCs on the bottom edge over continuous GND planes | H. Ott, "Dipoles for Dummies" (IEEE SCV EMC 2012); B. Olney, "Split planes" (2015) |
| ESD on a BNC shell or clamp must not cross an inductive ground bridge next to the MCU | same as above; shells and USB shield bonded straight to the planes | TI ESD layout guidance |
| Board noise off the USB cable | VBUS ferrite FB1 (2 A) at the connector, CM choke L1 on D+/D-, USBLC6 within a few mm of J1 | Wurth ANP007a; eeNews USB EMC |
| USB inrush (<= 10 uF equivalent) | C1 10 uF, C2 4.7 uF; relays only after enumeration (firmware) | USB 2.0 7.2.4.1 |
| Coil switching di/dt | TBD62003 COM on +5V (internal clamps), coils energized one at a time 2 ms apart, <= 16 energized (500 mA budget) | Keithley Switching Handbook 4; firmware 2.0.0 |
| Coil feed loop area | +5V coil feed and coil returns on the outer layers, over solid GND wherever they do not cross a floating-node opening; coils are DC during a measurement and switched one at a time | Keithley Switching Handbook 5.3 |
| Plane edges and cavity resonance | GND stitching vias every 4 mm along the board edge and around the measurement void (lambda/20 at 1 GHz = 7 mm); 20H rule NOT used | LearnEMC (Hubing), "EMC design myths" |
| Quiet measurement region | no outer-layer pour in the measurement area; the inner GND planes are opened only under the nodes that float at an intermediate potential (terminal buses, arms, clips: ground C there is a DUT-dependent error) and are solid under the rails, the standards column, the bridge band and the BNC launches | Keysight Impedance Measurement Handbook 2.4.7/3.4.2; TI OPA656 layout |
| MCU | 100 nF at every supply pin + 4.7 uF, VDDA ferrite + 1 uF + 10 nF, NRST 100 nF, no crystal (HSI48 + CRS), outputs at low slew (reset default), unused pins analog | ST AN4325, STM32F072 datasheet |
| Analog front-end immunity | 100 R + clamp at each input, AC-coupled, 0.1 % difference stage, ferrite-filtered +5VA | TI/ADI op-amp layout notes |
| Ground loop PC-USB-board-BNC-Bode | the Kelvin difference stage rejects it (60 dB); a USB isolator is the documented option if pre-compliance or low-Z measurements need more | Signal Integrity Journal, "2-port shunt-thru ground loop"; Omicron B-LCM |

Stackup JLC04161H-7628, 4 layers (decided 2026-10-11 after comparing with 6): L1 F.Cu /
0.2104 mm 7628 (er 4.4) / L2 In1 GND / 1.065 mm core (er 4.6) / L3 In2 GND / 0.2104 mm 7628 /
L4 B.Cu, written into the board file. SIG/GND/GND/SIG: every signal layer is 0.21 mm from a solid
GND plane, and the inner layers carry no tracks at all (decided 2026-10-13). The planes are
opened only under the floating nodes, following their routed copper with a 0.75 mm margin, with
a 1 mm ground bridge through every ring of openings so no ground is left as an island
(`route_pcb.float_node_voids`). Six layers bought no routing room and only
stripline margin for a board whose fastest signal is USB full-speed. Every signal via outside the
void gets a GND stitching via within 1.5 mm (return current changes planes there); decoupling
caps get their GND via beside the pad at 90 degrees to the pad axis, facing their IC (smallest
pad-via-plane loop).

**Layer use and coupling (routing rules, maze_route.py).** Every net is on F.Cu or B.Cu; both
inner layers are GND only. Ground under copper that always sits at the HI or LO potential (rails,
feeds, standards column) costs nothing: the bridge senses current in LO only and the per-path
OSL absorbs it. Under the nodes that float at an intermediate potential (terminal buses, arms,
clips) the planes are opened, because ground capacitance there enters as Cg*v*(v-1), a
DUT-dependent error (rev B measured 14.5 pF on its link net). Until 2026-10-13 the whole
measurement area was open and the +5V coil feed and coil returns ran on In1/In2 inside it,
0.21 mm under the measurement tracks. Crosstalk between measurement copper
and anything that switches or carries supply (coils, supplies, digital) is a routing cost: each
step costs 1 + 6K, K = 1/(1 + (d/H)^2) (parallel-line coupling coefficient, H. Johnson), d the
3-D distance to the other class's copper on the same layer or across the 0.21 mm prepreg
(broadside coupling, the strongest case). Parallel runs and broadside overlaps are avoided; where
they must meet, the cheapest path is a short 90-degree crossing (orthogonal routing of adjacent
layers). At 3W spacing K ~ 0.15, beyond 2 mm K < 0.06. Sources: Altium, "Crosstalk analysis,
reduction and elimination"; LearnEMC CH06 transmission-line crosstalk; RF Essentials, crosstalk.

**MCU pin assignment and routing without an autorouter (2026-10-12).** Every board net is routed by
our own router (`maze_route.py`: coarse 0.25 mm pass for measurement/supply/coil nets, then a fine
0.125 mm pass with exact-geometry fan-out of the 0.5 mm-pitch packages, exact clearance verification
of every emitted segment and via, and rip-up-and-reroute that never touches the generator's
pre-routes, listed in `relay_board_prerouted.json`). To make the MCU routable, all 23 relay-drive
pins sit on the package sides that face the drivers (north row PA0-PA2, PC13-PC15, PF0; west
PA3-PB11; east PB5-PB7, PA15, PB3, PB4); none wraps around the package through the USB/SWD corner.
The south row carries what goes south or east: COIL_EN PB12, COIL_FLT PB13 (coil switch U11 is
south-west), LED_PROG PB14, LED_KELVIN PB15, LED_BUSY PA8. PROG avoids PA9/PA10/PA14/PA15, which
the ROM bootloader drives (AN2606). The BOOT0 strap parts sit beside the BOOT button, south-east of the SWD
header and out of the east lane fan (BOOT0 leaves its pin inward, under the package, on B.Cu);
the SWD header is turned so its SWDIO/SWCLK column faces the MCU; other parts' GND vias keep
2.2 mm off every fine-pitch pin row.

**Placement rules learned while routing without an autorouter (2026-10-12).** Relay-drive pins
are assigned in lane order (see `generate_schematic.GPIO_PINS`): every driver input sits on one
line, so north-row pins (innermost lanes) take the nearest inputs, east-column pins the next,
west-column pins reach the farthest from inside the driver packages -- no two lanes cross. The
MCU's north side holds only what must be there, each part over its own pin: the VDDA filter stacked
over pin 9, NRST's R/C over pin 7, and one 100 nF in the NE corner notch decoupling VDD 48 and VBAT
1 together (adjacent +3V3 pins; scoped courtyard rule in relay_board.kicad_dru). GND pins of the
LQFP may drop their via under the package body. Bring-up test pads sit beside the circuit they
probe (a 100 mm FE_OUT stub to an edge row was an antenna and a wall). The TMP117 keeps a coarse-pass
access pocket so measurement lines cannot seal its I2C pins. USB: VBUS runs under the receptacle on
B.Cu (both pad pairs, D2's VBUS pin and FB1 joined by a pre-routed trunk between the shell legs),
the +5V_IN feed crosses the USB chain on B.Cu, and the pair stays on F.Cu without vias.

## 6. DUT interface, housekeeping and protection (added 2026-10-11)

**DUT interface.** Eight Grayhill 02-21 miniature test clips (the banana-plug style; the other
Series 02 styles are obsolete), a Kelvin pair per terminal, F inner and S outer, 8 mm apart. Each
clip's standard banana plug sits straight in a plated 4.0 mm hole (the bore of a 4 mm socket: the
plug's spring grips the barrel; solder it for a permanent fixture) -- no separate jack. The DUT
lead passes through the F clip into the S clip, so sense contacts the lead itself, at 8 mm
effectively the same point. The clip is 5.54 mm thick along the lead and 10.31 mm wide across it
(Grayhill Series 02 datasheet): 2.5 mm between the clips of a pair, 13.7 mm between the two
terminals of a side. The bay between the F clips is 41 mm wide (F to F 54 mm): the clip pairs and
their iso relays sit as far out as the link relays K15/K16 allow. The plug is 23.8 mm long and
stands ~22 mm below the board: 25 mm standoffs under the five mounting holes. Clip contact
resistance sits only in the force path; in 2-wire mode the clip is fixed geometry inside the
clamp-plane fixture compensation (compensate with the clips fitted).

**Housekeeping.** Status LEDs (PWR, RUN, BUSY, KELVIN, PROG). TMP117 (+/-0.1 C) beside the
standards column: every calibration records the fixture temperature and is re-acquired when it
drifts > 2 C. The MCU's 96-bit UID is the board serial (`SYST:SER?`), stored with every
calibration. Per-relay actuation counters in the last flash page (`RELAY:COUNT?`). Test pads
on +5V, +5V_IN, +3V3, +5VA, VMID, FE_OUT, GND.

**Power and protection.** VBUS -> SMF5.0A TVS -> 2 A ferrite -> +5V_IN (LDO / MCU) ->
TPS2051C (soft start, short-circuit limit, /FLT) -> +5V (coils, drivers, front-end), enabled by
the firmware only after USB enumeration, with 10 uF per driver behind it. USB shield on its own
net: 0R to GND fitted, 1M || 4.7 nF footprints for the bench to choose.

**Enclosure-ready.** Five plated M3 holes bonded to GND (one mid cable edge) and an exposed GND
strip along the cable edge between the connectors, for a lid or enclosure that bonds to the
plane where the cables enter.

**Track corners.** 90-degree corners are electrically irrelevant at these frequencies (excess
corner capacitance is femtofarads; Johnson & Graham, Montrose); the generated pre-routes are
chamfered to 45 degrees for tidiness, not for signal integrity.
