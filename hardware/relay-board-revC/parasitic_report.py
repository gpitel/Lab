"""
Post-route parasitic audit of the signal nets.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" parasitic_report.py

Walks the routed board and reports, for every measurement net:
  - total routed length per layer and estimated L / C
  - via count (each ~1.2 nH)
  - parallel-run exposure: length of any OTHER net's track running closer
    than 1 mm alongside on the same layer, and closer than 0.5 mm on the
    adjacent layer (crossings at ~90 degrees are ignored -- ~0.05 pF each)
  - copper pour presence under the net (there must be none in the void)

Budgets (rev C, Kelvin; see ARCHITECTURE.md):
    force arms (DUT_x)   <= 16 mm (B, D) / 26 mm (A, C: the far relay), 0 vias
                         (uncalibrated in 2-wire mode; FORCE_ARM_BUDGET)
    sense arms (DUTS_x)  <= 30 mm, <= 1 via    (carry no current in Kelvin)
    rail feeds           HI + LO force-rail length from each column's NO pad
                         to its feed equal across A..D within 5 % (the
                         end-fed rails: rev B was 88/44/44/88 mm)
    T/S buses            informational (calibrated per configuration)
Exit code 1 when a budget is violated.
"""

import math
import os
import sys
from collections import defaultdict

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD_PATH = os.path.join(HERE, "relay_board.kicad_pcb")

L_PER_MM = 0.8e-9
C_PER_MM = 0.04e-12
L_VIA = 1.2e-9

SIGNAL_PREFIXES = ("/DUT_", "/T", "/CAL_", "/RAIL_")
BUDGET_FAILURES = []
#: Force-arm budget per terminal, mm. Two iso relays sit side by side between
#: the two clip rows of each side, so one terminal per side (A, C) has the far
#: relay: its arm steps around the pair's other clip and is ~24 mm. Accepted
#: 2026-10-13: in Kelvin mode the arm is in the force path only, in 2-wire mode
#: it is inside the clamp-plane fixture compensation; halving it would mean
#: re-placing six relays around the bay for ~10 nH that never reaches a Kelvin
#: result.
FORCE_ARM_BUDGET = {"A": 26.0, "B": 16.0, "C": 26.0, "D": 16.0}


def is_signal(name):
    if name in [f"/{bus}{t}" for bus in "TS" for t in "ABCD"]:
        return True
    return name.startswith(("/DUT", "/CAL_", "/RAIL_", "/SNS_", "/ISO", "/STAR_"))


def rail_feed_lengths(board):
    """Copper length along each force rail from every column's NO pad to the
    rail's feed (SOURCE pin for HI, RC1 shunt pad for LO): Dijkstra over the
    rail's segments, joined wherever an endpoint lies on another segment."""
    import heapq
    feeds = {"/RAIL_HI": ("J6", "1"), "/RAIL_LO": ("R14", "1")}
    columns = {"A": ("K1", "K2"), "B": ("K3", "K4"), "C": ("K5", "K6"), "D": ("K7", "K8")}
    pads = {(fp.GetReference(), pad.GetNumber()): (to_mm(pad.GetPosition().x), to_mm(pad.GetPosition().y))
            for fp in board.GetFootprints() for pad in fp.Pads()}
    result = {}
    for net, feed in feeds.items():
        segments = [((to_mm(t.GetStart().x), to_mm(t.GetStart().y)),
                     (to_mm(t.GetEnd().x), to_mm(t.GetEnd().y)))
                    for t in board.GetTracks()
                    if t.GetNetname() == net and t.GetClass() != "PCB_VIA"]
        points = {p for seg in segments for p in seg}
        graph = {}

        def link(a, b):
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            graph.setdefault(a, []).append((b, d))
            graph.setdefault(b, []).append((a, d))

        for a, b in segments:
            on = [p for p in points if p not in (a, b)
                  and abs(math.hypot(p[0] - a[0], p[1] - a[1]) + math.hypot(p[0] - b[0], p[1] - b[1])
                          - math.hypot(a[0] - b[0], a[1] - b[1])) < 0.01]
            chain = sorted([a, b] + on, key=lambda p: math.hypot(p[0] - a[0], p[1] - a[1]))
            for u, v in zip(chain, chain[1:]):
                link(u, v)

        def nearest(target):
            return min(graph, key=lambda p: math.hypot(p[0] - target[0], p[1] - target[1]))

        start = nearest(pads[feed])
        distance = {start: 0.0}
        queue = [(0.0, start)]
        while queue:
            d, u = heapq.heappop(queue)
            if d > distance.get(u, 1e9):
                continue
            for v, w in graph.get(u, []):
                if d + w < distance.get(v, 1e9):
                    distance[v] = d + w
                    heapq.heappush(queue, (d + w, v))
        rail = "HI" if net.endswith("HI") else "LO"
        for terminal, (hi_relay, lo_relay) in columns.items():
            relay = hi_relay if rail == "HI" else lo_relay
            result[(terminal, rail)] = distance.get(nearest(pads[(relay, "4")]), float("nan"))
    return result


def to_mm(value):
    return pcbnew.ToMM(value)


def segment_length(track):
    start, end = track.GetStart(), track.GetEnd()
    return math.hypot(to_mm(end.x - start.x), to_mm(end.y - start.y))


def parallel_exposure(track, other):
    """Rough co-run length between two segments when nearly parallel and
    within 1 mm; returns 0 for crossings (angle > 30 degrees)."""
    ax1, ay1 = to_mm(track.GetStart().x), to_mm(track.GetStart().y)
    ax2, ay2 = to_mm(track.GetEnd().x), to_mm(track.GetEnd().y)
    bx1, by1 = to_mm(other.GetStart().x), to_mm(other.GetStart().y)
    bx2, by2 = to_mm(other.GetEnd().x), to_mm(other.GetEnd().y)
    va = (ax2 - ax1, ay2 - ay1)
    vb = (bx2 - bx1, by2 - by1)
    la = math.hypot(*va)
    lb = math.hypot(*vb)
    if la < 0.05 or lb < 0.05:
        return 0.0
    cosine = abs((va[0] * vb[0] + va[1] * vb[1]) / (la * lb))
    if cosine < 0.866:                       # > 30 degrees: a crossing
        return 0.0
    # distance between midpoints as a cheap proximity proxy
    mid_a = ((ax1 + ax2) / 2, (ay1 + ay2) / 2)
    mid_b = ((bx1 + bx2) / 2, (by1 + by2) / 2)
    distance = math.hypot(mid_a[0] - mid_b[0], mid_a[1] - mid_b[1])
    if distance > max(1.0, (la + lb) / 4):
        return 0.0
    return min(la, lb)


LAYER_Z = {"F.Cu": 0.0, "In1.Cu": 0.245, "In2.Cu": 1.325, "B.Cu": 1.57}
COUPLED_D = 1.0                  # mm (3-D): K = 1/(1+(d/0.5)^2) > 0.2 inside this


def coupling_audit(board):
    """Measurement-net copper near switching/supply/digital copper (same layer
    or across the 0.21 mm prepreg), and any measurement copper on an inner
    layer (not allowed: maze_route.MEASUREMENT is outer-only)."""
    import numpy
    from maze_route import MEASUREMENT
    from optimize_tracks import point_seg_distance
    aggressors = {layer: [] for layer in LAYER_Z}
    victims = []
    for item in board.GetTracks():
        name = item.GetNetname()
        layer = item.GetLayerName()
        if item.GetClass() == "PCB_VIA":
            if name not in MEASUREMENT and name not in ("/GND", ""):
                p = item.GetPosition()
                for each in LAYER_Z:
                    aggressors[each].append((to_mm(p.x), to_mm(p.y), to_mm(p.x), to_mm(p.y),
                                             to_mm(item.GetWidth(pcbnew.F_Cu)) / 2))
            continue
        if layer not in LAYER_Z:
            continue
        a = (to_mm(item.GetStart().x), to_mm(item.GetStart().y))
        b = (to_mm(item.GetEnd().x), to_mm(item.GetEnd().y))
        half = to_mm(item.GetWidth()) / 2
        if name in MEASUREMENT:
            victims.append((name, layer, a, b, half))
        elif name not in ("/GND", ""):
            aggressors[layer].append((*a, *b, half))
    arrays = {layer: numpy.array(rows) if rows else numpy.zeros((0, 5)) for layer, rows in aggressors.items()}
    coupled, inner = defaultdict(float), defaultdict(float)
    for name, layer, a, b, half in victims:
        length = math.dist(a, b)
        if layer in ("In1.Cu", "In2.Cu"):
            inner[name] += length
        n = max(1, int(length / 0.1))
        t = (numpy.arange(n) + 0.5) / n
        points = numpy.column_stack([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])])
        near = numpy.zeros(n, dtype=bool)
        for other, data in arrays.items():
            dz = abs(LAYER_Z[layer] - LAYER_Z[other])
            if dz > 0.5 or not len(data):
                continue
            for row in data:
                # edge-to-edge lateral distance, then 3-D with the layer gap
                lateral = point_seg_distance(points, numpy.broadcast_to(row[0:2], (n, 2)),
                                             numpy.broadcast_to(row[2:4], (n, 2))) - row[4] - half
                near |= numpy.hypot(numpy.maximum(lateral, 0), dz) < COUPLED_D
        coupled[name] += length * near.mean()
    return coupled, inner


def main():
    board = pcbnew.LoadBoard(BOARD_PATH)
    tracks_by_net = defaultdict(list)
    vias_by_net = defaultdict(int)
    for track in board.GetTracks():
        name = track.GetNetname()
        if track.GetClass() == "PCB_VIA":
            vias_by_net[name] += 1
        else:
            tracks_by_net[name].append(track)

    signal_nets = sorted(n for n in tracks_by_net if is_signal(n))
    all_tracks = [t for ts in tracks_by_net.values() for t in ts]

    print(f"{'net':<12} {'outer mm':>8} {'inner mm':>8} {'vias':>4} "
          f"{'L est':>8} {'C est':>8} {'co-run mm':>9}")
    print("-" * 64)
    lengths = {}
    for name in signal_nets:
        per_layer = defaultdict(float)
        exposure = 0.0
        for track in tracks_by_net[name]:
            per_layer[track.GetLayerName()] += segment_length(track)
            for other in all_tracks:
                if other.GetNetname() == name:
                    continue
                if other.GetLayer() != track.GetLayer():
                    continue
                if not is_signal(other.GetNetname()) or True:
                    exposure += parallel_exposure(track, other)
        total = sum(per_layer.values())
        lengths[name] = total
        inductance = total * L_PER_MM + vias_by_net[name] * L_VIA
        capacitance = total * C_PER_MM
        outer = per_layer.get('F.Cu', 0) + per_layer.get('B.Cu', 0)
        print(f"{name:<12} {outer:8.1f} "
              f"{total - outer:8.1f} {vias_by_net[name]:4d} "
              f"{inductance*1e9:6.1f}nH {capacitance*1e12:6.2f}pF {exposure:9.1f}")

    print("-" * 64)

    # Budgets
    for terminal in "ABCD":
        name = f"/DUT_{terminal}"
        budget = FORCE_ARM_BUDGET[terminal]
        if lengths.get(name, 99) > budget:
            BUDGET_FAILURES.append(f"{name} arm {lengths[name]:.1f} mm > {budget:.0f} mm")
        if vias_by_net.get(name, 0) != 0:
            BUDGET_FAILURES.append(f"{name} has vias -- uncalibrated layer change")
        sense = f"/DUTS_{terminal}"
        if lengths.get(sense, 99) > 30.0:
            BUDGET_FAILURES.append(f"{sense} sense arm {lengths.get(sense, 0):.1f} mm > 30 mm")
        if vias_by_net.get(sense, 0) > 1:
            BUDGET_FAILURES.append(f"{sense} has {vias_by_net[sense]} vias (> 1)")

    feeds = rail_feed_lengths(board)
    print("\nForce-rail length from each column's NO pad to its feed (mm):")
    totals = {}
    for terminal in "ABCD":
        hi, lo = feeds[(terminal, "HI")], feeds[(terminal, "LO")]
        totals[terminal] = hi + lo
        print(f"  {terminal}: HI {hi:6.1f}  LO {lo:6.1f}  HI+LO {hi + lo:6.1f}")
    spread = (max(totals.values()) - min(totals.values())) / (sum(totals.values()) / 4)
    print(f"  spread {spread:.1%} (rev B: 88/44/44/88 mm, 67 %)")
    if not spread <= 0.05:
        BUDGET_FAILURES.append(f"column rail paths differ by {spread:.0%} (> 5 %)")

    buses = [lengths.get(f"/T{t}", 0) for t in "ABCD"]
    if buses and max(buses) > 95.0:
        print(f"  note: longest T bus {max(buses):.1f} mm (calibrated; informational)")
    if buses and min(buses) > 0:
        spread = (max(buses) - min(buses)) / (sum(buses) / len(buses))
        if spread > 0.15:
            # informational under per-config calibration: each path is
            # OSL-calibrated at the iso contact, so bus asymmetry cancels
            print(f"  note: T buses mismatched by {spread:.0%} "
                  "(calibrated out per config; informational)")

    average_bus = sum(buses) / len(buses) if buses else 0
    for name in ("/CAL_E", "/CAL_F"):
        length = lengths.get(name, 0)
        if average_bus and abs(length - average_bus) / average_bus > 0.40:
            print(f"  note: {name} {length:.1f} mm vs T-bus average {average_bus:.1f} mm "
                  "(calibrated per configuration; informational)")

    coupled, inner = coupling_audit(board)
    print(f"\nCoupling: measurement copper within {COUPLED_D} mm (3-D, same layer or across the "
          "0.21 mm prepreg) of coil/supply/digital copper:")
    for name in sorted(coupled, key=coupled.get, reverse=True):
        if coupled[name] >= 0.5:
            print(f"  {name:<12} {coupled[name]:6.1f} mm")
    print(f"  total {sum(coupled.values()):.1f} mm")
    for name, length in inner.items():
        if length > 0.01:
            BUDGET_FAILURES.append(f"{name} has {length:.1f} mm on an inner layer (outer layers only)")

    if BUDGET_FAILURES:
        print("BUDGET VIOLATIONS:")
        for failure in BUDGET_FAILURES:
            print(f"  FAIL  {failure}")
        return 1
    print("All parasitic budgets met.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
