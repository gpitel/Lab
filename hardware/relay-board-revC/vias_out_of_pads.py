"""
Move every via that sits in an SMD pad to the nearest free spot beside it.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" vias_out_of_pads.py [board]

Why: an open via in a pad drains the solder joint (JLC fills via-in-pad only
as a paid option). maze_route.py ends a route with a via on the pad it is
reaching when that is the cheapest way down; forbidding it in the router left
18 nets without a route (the fine-pitch escapes and the 0402s at the MCU rely
on it), so the vias are moved afterwards instead.

Each via goes to the closest point where its ring is off every SMD pad and
clear of all other copper (exact check, optimize_tracks.build_obstacles), and
a stub from the old position to the new one is added on every layer the via
was joining. Vias that find no spot are listed and left alone.
"""

import math
import os
import sys

import pcbnew

import optimize_tracks as exact

BOARD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "relay_board.kicad_pcb")
PAD_GAP = 0.05                 # via ring to an SMD pad of its own net (mm)
VIA_PITCH = 0.85               # centre to centre of two vias (0.6 ring, 0.25 between)
STEP = 0.125
REACH = 4.0                    # search radius (mm)
STUB_WIDTHS = (0.25, 0.2)      # tried in this order when the track's own width does not fit
SKIP = ("H", "BOND")           # mounting holes, bonding strip: copper meant to carry vias


def closest(p, a, b):
    """Point of segment a-b nearest to p."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    length2 = vx * vx + vy * vy
    t = 0.0 if length2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / length2))
    return (a[0] + t * vx, a[1] + t * vy)


def main():
    path = next((a for a in sys.argv[1:] if not a.startswith("--")), BOARD)
    board = pcbnew.LoadBoard(path)
    mm = pcbnew.ToMM

    # plain-Python copy first: SWIG wrappers lose their type once items are added
    pads = []
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue
            box = pad.GetBoundingBox()
            pads.append((footprint.GetReference(), pad.GetNumber(), pad,
                         (mm(box.GetLeft()), mm(box.GetTop()), mm(box.GetRight()), mm(box.GetBottom()))))
    vias, tracks = [], []
    for item in board.GetTracks():
        if item.GetClass() == "PCB_VIA":
            p = item.GetPosition()
            vias.append({"item": item, "at": (mm(p.x), mm(p.y)), "net": item.GetNetCode(),
                         "name": item.GetNetname(), "r": mm(item.GetWidth(pcbnew.F_Cu)) / 2, "uid": exact.uid(item)})
        elif item.GetClass() == "PCB_TRACK":
            a, b = item.GetStart(), item.GetEnd()
            tracks.append((item.GetLayer(), (mm(a.x), mm(a.y)), (mm(b.x), mm(b.y)), mm(item.GetWidth())))

    obstacles = exact.build_obstacles(board)
    offsets = sorted(((i * STEP, j * STEP) for i in range(-int(REACH / STEP), int(REACH / STEP) + 1)
                      for j in range(-int(REACH / STEP), int(REACH / STEP) + 1)
                      if 0 < math.hypot(i, j) * STEP <= REACH), key=lambda o: math.hypot(*o))

    def off_pads(q, r):
        return not any(x0 - r - PAD_GAP < q[0] < x1 + r + PAD_GAP and y0 - r - PAD_GAP < q[1] < y1 + r + PAD_GAP
                       for _, _, _, (x0, y0, x1, y1) in pads)

    # GND vias that join no track are plane stitching: one may be given up
    # when it is what boxes a pad in
    for via in vias:
        via["free"] = via["name"] == "/GND" and not any(
            math.dist(via["at"], closest(via["at"], a, b)) <= via["r"] for _, a, b, _ in tracks)
    spare = {via["uid"] for via in vias if via["free"]}
    moved, stuck, new_tracks, dropped = 0, [], [], []
    for via in vias:
        p = via["at"]
        hit = next(((ref, number) for ref, number, pad, _ in pads
                    if not ref.startswith(SKIP) and pad.HitTest(pcbnew.VECTOR2I(pcbnew.FromMM(p[0]), pcbnew.FromMM(p[1])), 0)), None)
        if hit is None:
            continue
        # copper the via joins: the pad, and on each layer the tracks of its
        # net lying in the via's ring. The new via may tap those anywhere.
        joined, own = {pcbnew.F_Cu: []}, 0.0
        for layer, a, b, width in tracks:
            if layer in exact.OUTER and math.dist(p, closest(p, a, b)) <= via["r"] + width / 2 - 0.01:
                joined.setdefault(layer, []).append((a, b, width))
                if layer != pcbnew.F_Cu:
                    own = max(own, width)
        own = own or STUB_WIDTHS[0]

        def route(layer, q, width, ignore=()):
            """Shortest clear stub (straight or one bend) from this layer's copper to q."""
            starts = [closest(q, a, b) for a, b, _ in joined[layer]] + [p]
            for s in sorted(starts, key=lambda s: math.dist(s, q)):
                for bend in (None, (q[0], s[1]), (s[0], q[1])):
                    points = [s] + ([bend] if bend else []) + [q]
                    if all(obstacles.clear(layer, a, b, width / 2, via["net"], ignore) for a, b in zip(points, points[1:])):
                        return points
            return None

        target = None
        for ignore in ((), spare):                        # second round: stitching vias may go
            for width in dict.fromkeys((own,) + tuple(w for w in STUB_WIDTHS if w < own)):
                for dx, dy in offsets:
                    q = (round(p[0] + dx, 4), round(p[1] + dy, 4))
                    if not off_pads(q, via["r"]):
                        continue
                    if any(other is not via and other["uid"] not in ignore and other.get("at") is not None
                           and math.dist(q, other["at"]) < VIA_PITCH for other in vias):
                        continue
                    if not all(obstacles.clear(layer, q, q, via["r"], via["net"], ignore) for layer in exact.OUTER):
                        continue
                    routes = {layer: route(layer, q, width, ignore) for layer in joined}
                    if all(routes.values()):
                        target = (q, width, routes)
                        break
                if target:
                    break
            if target:
                break
        if target is None:
            stuck.append((via["name"], hit, p))
            continue
        q, width, routes = target
        if ignore:                                        # give up the stitching vias now in the way
            for other in vias:
                if other["free"] and other.get("at") is not None and not other.get("gone"):
                    o = other["at"]
                    near = math.dist(q, o) < VIA_PITCH or any(
                        math.dist(o, closest(o, a, b)) < other["r"] + width / 2 + exact.CLEARANCE
                        for points in routes.values() for a, b in zip(points, points[1:]))
                    if near and via["name"] != "/GND":
                        other["gone"] = True
                        dropped.append(other)
            for other in dropped:
                other["at"] = None
        via["at"] = q
        via["to"] = q
        for layer in exact.OUTER:
            obstacles.add(layer, q, q, via["r"], exact.CLEARANCE, via["net"])
        # the via's ring was what joined those tracks to each other: a spoke
        # from each one to the old centre keeps them joined once it has moved
        for layer, group in joined.items():
            for a, b, w in group:
                c = closest(p, a, b)
                if math.dist(c, p) > 1e-3:
                    new_tracks.append((layer, c, p, min(w, 2 * via["r"]), via["net"]))
        for layer, points in routes.items():
            for a, b in zip(points, points[1:]):
                if math.dist(a, b) > 1e-3:
                    new_tracks.append((layer, a, b, width, via["net"]))
                    obstacles.add(layer, a, b, width / 2, exact.CLEARANCE, via["net"])
        obstacles.freeze()
        moved += 1
        print(f"  {via['name']:<12} {hit[0]}.{hit[1]}: ({p[0]:.2f}, {p[1]:.2f}) -> ({q[0]:.2f}, {q[1]:.2f}), "
              f"{math.dist(p, q):.2f} mm, {width} mm stub on {len(routes)} layer(s)")

    for other in dropped:
        board.Remove(other["item"])
    for via in vias:
        if "to" in via:
            via["item"].SetPosition(exact.vec(*via["to"]))
    for layer, a, b, width, net in new_tracks:
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(exact.vec(*a))
        track.SetEnd(exact.vec(*b))
        track.SetLayer(layer)
        track.SetWidth(pcbnew.FromMM(width))
        track.SetNetCode(net)
        track.SetLocked(True)
        board.Add(track)
    pcbnew.SaveBoard(path, board)
    print(f"vias out of pads: {moved} moved ({len(dropped)} stitching vias given up), {len(stuck)} without room")
    for name, hit, p in stuck:
        print(f"  !! {name} at {hit[0]}.{hit[1]} ({p[0]:.2f}, {p[1]:.2f})")
    return 1 if stuck else 0


if __name__ == "__main__":
    sys.exit(main())
