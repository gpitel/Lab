# Rev C -- what we learned routing it ourselves

Hard-won lessons from getting the board routed without an autorouter (2026-10-11/12). Read
before changing the placement, the pin assignment or maze_route.py.

## Floorplan beats router cleverness

Most "routing failures" were placement or pin-assignment mistakes; rip-up only made them thrash.
Before touching the router, render the stuck area (`devtools/`) and ask what put copper there.

- **Pin assignment is free -- use it.** The firmware relay map is generated from the schematic,
  so any MCU pin can drive any relay. Assign in *lane order*: all driver inputs sit on one line
  (x = 181.5, facing east); north-row pins are the innermost lanes and take the nearest inputs,
  east-column pins the next, west-column pins the farthest (`generate_schematic.GPIO_PINS`).
  Min-sum matching (straight-line or geodesic) was *useless* here: every MCU pin is ~7 mm from
  the others, the drivers 20-80 mm away, so all orders score within 0.3 %.
- **Functions go on the side facing their destination.** I2C on the east pins crossed the whole
  relay corridor (moved to I2C2 PB10/PB11, west, facing the TMP117). LEDs east of the USB pins
  had to cross USB (moved to a column SW of the MCU with the coil-switch lines; RUN moved from
  PC13 on the north row to PA9 on the south row).
- **Static nets that must cross a lane fan go under it.** BOOT0 sits between relay pins and must
  run to R4/R17: it escapes inward (via under the package) and runs on B.Cu (`INWARD_FIRST`).
- **Lane groups that must cross each other get different layers.** West-column relay lanes run
  on B.Cu, north/east lanes on F.Cu (`LAYER_COST`).
- **Support parts sit over their own pin, not "near the MCU".** AN4325 "at the pin" means the
  pin it decouples: the VDDA stack over pin 9, NRST R/C over pin 7. One cap in the NE corner notch
  serves VDD48 + VBAT (adjacent +3V3 pins). Anything else in the band between the MCU and the
  drivers blocks lanes.
- **Test pads beside the circuit they probe.** A row of test pads at the board edge dragged
  FE_OUT/VMID/+5VA 70-100 mm across the board: an antenna on the measurement output and a wall.
- **Reserve access around small ICs routed late** (`ACCESS_POCKETS`): the coarse pass otherwise
  walls them in with nets the fine pass may not move.
- **Crossings that are unavoidable: pick the layer deliberately.** The VBUS feed has to cross the
  USB chain -- supply on B.Cu, the USB pair on F.Cu without vias (`LAYER_COST`).

## What finally made the fine pass converge (2026-10-12)

Shortest-span ordering plus rip-up never converged; every fix below is placement or ordering.
Find the cause with one fine pass WITHOUT rip-up, saved and rendered: only five nets were ever
stuck, each with a visible reason.

- **Route lane bundles as one block, in river order** (`maze_route.LANE_ORDER`): each lane hugs
  the one before it and none seals a later pad. Shortest-first let lanes rip each other for ever.
- **River order depends on which side the bundle reaches its targets from.** PA0..PA2 sit west of
  the VDDA parts and reach the drivers from the west, under U6: their order is the mirror of the
  east-channel order. A pin with no outward escape (PF1, under the NRST parts) is the wrong pin:
  PC13 was free.
- **Connectors face their pins.** The SWD header had SWDIO/SWCLK on its far column; the two
  tracks looped around it through the east lane fan. Turned 180 deg.
- **Lanes that dive to the other layer get deterministic staggered vias** (`VIA_ESCAPE`). Left to
  the grid, eight vias stacked in one column, ran out of room and the runs spiralled around the
  package corner over the I2C exit.
- **Static nets that cross a lane fan go first and underneath** (NRST, like BOOT0:
  `INWARD_FIRST`, `BEFORE_LANES`, `LAYER_COST`).
- **Two-pad parts: the pad that goes to the far net faces it.** The LED series resistors had the
  MCU-side pad toward the LED; each LED track wrapped around and sealed it. Same failure with any
  part dropped in a lane fan (BOOT0 strap resistors) or beside a pin row (C9 over the I2C pins).
- **A pad counts as reached only if copper touches it.** The grid can stop a track short of a
  round or small pad; `route_pcb.close_pad_gaps` joins the ends DRC reports.
- **pcbnew scripts: do not edit a board while reading it.** After Remove/Add from Python, SWIG
  wrappers lose their types in that process (`board.Tracks()` not iterable, points without `.x`),
  even for a re-loaded board. Read into plain Python first, edit at the end, or use a process per stage.
- **`pad.HitTest(point)` needs the accuracy argument**; without it nothing hits.

- **A rip-up victim that was still waiting in the queue was dropped from it** (never routed, and
  the pass still printed "0 nets unrouted"). Trust DRC's unconnected count, not the router's.
- **Two layers are enough for everything**, the 23 coil returns and the +5 V coil feed included
  (2026-10-13): the inner layers carry no tracks. The coarse pass leaves three short
  standards-column nets to the fine pass.

- **A ring of plane openings deletes the ground it encloses** (the zone filler removes islands):
  the hole grows to the whole enclosed area. Cut a narrow ground bridge through each ring
  (`route_pcb.float_node_voids`).
- **Floorplan symmetry removes bus crossings**: outer iso relay over the outer column on both
  sides (column order A B E D C).
- **Dead copper**: `cleanup_dangling.py` cuts each track back to the span between its outermost
  contacts and removes one-layer vias; a one-layer via can still be the bridge between two track
  ends lying in its pad. Its DRC guard restored the board four times while the contact test was
  wrong -- keep the guard.
- **T-junctions**: KiCad joins a track end to the body of another track; other tools only match
  end points. Splitting at the junction costs nothing and removes the ambiguity.
- **A net that fails in the fine pass often routes once the others have settled** (final retry);
  nets left with long detours by rip-up are re-routed alone afterwards (`--polish`).

- **Vias in SMD pads**: forbidding them in the router (via ring off every SMD pad) left 18 nets
  unrouted -- the fine-pitch escapes and the 0402s at the MCU depend on dropping straight down.
  Moving them afterwards works for 84 of 88 (`vias_out_of_pads.py`): nearest free spot, a stub on
  each layer, and a spoke from every track the old via ring was joining (without the spokes four
  nets came apart). `maze_route.py --no-via-in-pad` is for single-net re-routes only.
  The last 4 were boxed in by a neighbour's track. Giving them room BEFORE routing (locked escape
  stubs, then R3/C15 turned with a top-layer link) cost two full runs: both left the fine pass
  with unrouted nets at the MCU (a locked via or a displaced GND fan-out via beside the lane fan
  is enough). What worked is a patch of the routed board, `fix_boxed_pads.py`: move the
  neighbour's track, every new piece checked with `optimize_tracks.build_obstacles`, then fill +
  DRC. Minutes instead of hours. Do not touch the placement or the pre-routes around U1 to fix a
  local problem.
- **A via saved without a track on its net becomes GND when the file is read** (KiCad takes it
  for a free via and gives it the net of the zone it stands in). Move a via and add its tracks
  in the same process, before saving.
- **The router's rip-up crashes on a finished board** (SWIG: 'SwigPyObject' has no attribute 'Pads'
  after the first rip). Rip-up only on the pre-finish board.
- **Git Bash rewrites `/NET` arguments into paths**: set MSYS_NO_PATHCONV=1, and then pass Windows
  paths for everything else.

## JLC assembly files

- **JLC's library footprints have their own zero orientation.** `export_jlcpcb.CPL_FIX` holds the
  correction per footprint (relays, LQFP, SOIC, SOT-23-6 and the SWD header +270, SOT-23-5 and
  SOT-23 +180, USB-C origin 1.58 mm off). Derive it, do not guess: fetch the library footprint for
  the LCSC code (EasyEDA API), place it with the CPL row, compare every pad with the board.
- **Polarity is not pad numbers.** KT-0603R numbers anode 1; our LED footprint numbers cathode 1.
- **Stock moves.** Check each pinned code against JLC's parts search before an order; the
  community jlcsearch index was wrong for several parts.

## Datasheet traps

- **TDK ACM2012 CM choke: windings are 1-4 and 2-3**, pads facing each other across the body
  (TDK equivalent-circuit library), not 1-2 / 3-4 as the generic KiCad symbol suggests. The first
  netlist swapped D+/D- at the connector. Check every 4-pad CM choke's winding pairs.
- **STM32 ROM bootloader** drives PA9/PA10 (USART1), PA14/PA15 (USART2), PA11/12 (USB) in DFU:
  a "PROG" LED must not sit on those. Relay pins there are safe only because the coil supply
  switch is held off by its pull-down through reset.
- **F0 I2C2** is clocked from PCLK (48 MHz), I2C1 from HSI by default: the timing argument differs.
- **USB-C (HRO TYPE-C-31-M-12)**: VBUS pads sit second from each end of the 0.5 mm row, between
  GND and CC/SBU, with NPTH pegs 1.45 mm below. Escape VBUS straight down each pad's inner edge
  (0.2 mm, exactly 0.2 to the neighbour, 0.69 mm centre-to-peg) to vias, trunk on B.Cu between
  the shell legs.

## Router internals that bit us (maze_route.py)

- **Grid quantisation cannot resolve 0.5 mm pitch.** Fine-pitch pads get exact-geometry escape
  stubs first (fanned outward, else inward to a via under the package, else a dog-bone), then the
  grid router starts from the stub ends.
- **Pad side from the pad's long axis**, never from its position relative to the package centre
  (corner pins got sideways escapes into their neighbours).
- **Verify every emitted segment and via against exact geometry**; on refusal block that cell and
  re-search. Then the grid can allow diagonal moves freely.
- **Net ties:** the bridge copper belongs to *both* tied nets; modelling it as foreign copper made
  every net-tie pad unreachable (`Obstacles.add(..., net2=)`).
- **A plane on an inner layer blocks tracks, not vias** -- treating the GND planes as obstacles for
  vias made every driver-side net unroutable.
- **Rip-up and reroute:** rip along the path the stuck net *wants* (planned with other nets'
  copper passable at 12x cost -- PathFinder-style congestion), radius rings only as a fallback;
  priority classes (measurement > supplies/coils/USB > digital); never rip the generator's
  pre-routes (`relay_board_prerouted.json`) or the fan-out escapes (a ripped escape is never
  recreated and the pad stays sealed).
- **Weighted A\*** (heuristic x2) on the fine grid: the crosstalk cost (up to 7x) otherwise makes the
  search explode.
- **Footprints added by script need a library ID**, else freerouting writes `(component ` with no
  name and KiCad's SES import fails (relevant only if freerouting is ever used again).
- **Mask openings over copper must be pads with a net** (bonding strip), else every GND via under
  them is a solder_mask_bridge.

## Workflow

- A full run is generate (1 min) -> coarse maze (~15 min) -> fine maze (~35 min) -> route_pcb
  --no-freerouting -> parasitic_report (optimize_tracks currently makes the board worse: skip it). Keep a copy of the coarse output and
  re-run only the fine pass while iterating on it.
- When a net fails, first run it alone (`devtools/astar_test.py NET`). Routes alone = ordering or
  congestion; fails alone = geometry (render it).
- Stop a run as soon as the log shows the same nets ripping each other: that is a floorplan
  problem, and every minute after that is wasted.
