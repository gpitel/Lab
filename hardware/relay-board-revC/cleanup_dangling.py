"""Post-route sweep of dead copper: stub tails and one-layer vias.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" cleanup_dangling.py [--dry-run]

Runs inside `route_pcb.py --no-freerouting`, after the pad gaps are closed.
The router leaves copper that carries nothing: the unused tail of a
fine-pitch escape stub (the route joined it part-way, or the pin escaped
through its via instead), fragments of ripped-up routes, and vias that end up
joining one layer only. Each is an open stub on its net -- an antenna, and on
a measurement net extra stray capacitance.

Every track is cut back to the span between its outermost contacts; a track
with fewer than two contacts, or a via with fewer than two things attached, is
removed. This repeats until nothing changes (removing a tail can expose the
next one). A contact is what KiCad's own connectivity counts: an end point
or via centre lying inside another item's copper, or inside a pad of the net.

Never touched: GND, and the generator's pre-routes
(relay_board_prerouted.json) -- they count as contacts but are not edited.

The whole board is read into plain Python first and edited once at the end
(pcbnew's SWIG wrappers lose their types when items are removed while others
are still being read). DRC runs before and after; if the sweep costs a single
connection the original board is put back.
"""

import json
import math
import os
import re
import shutil
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD = os.path.join(HERE, "relay_board.kicad_pcb")
KICAD_CLI = r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
TOLERANCE = pcbnew.FromMM(0.005)
MIN_SPAN = pcbnew.FromMM(0.05)


def drc_counts(path):
    report = os.path.join(HERE, "relay_board_drc.txt")
    subprocess.run([KICAD_CLI, "pcb", "drc", "--format", "report", "--severity-all", "-o", report, path],
                   capture_output=True, text=True)
    text = open(report, encoding="utf-8", errors="replace").read()
    unconnected = int(re.search(r"Found (\d+) unconnected", text).group(1))
    return unconnected, len(re.findall(r"\[(?:track|via)_dangling\]", text))


def project(point, a, b):
    """(distance from point to segment a-b, parameter 0..1 of the nearest point)"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if not length2 else max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy), t


def closest(a, b, c, d):
    """(distance between segments a-b and c-d, parameter 0..1 on a-b of its nearest point)"""
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    d1, d2, d3, d4 = cross(c, d, a), cross(c, d, b), cross(a, b, c), cross(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and (d1 - d2):
        return 0.0, d1 / (d1 - d2)                    # they intersect
    best = (float("inf"), 0.0)
    for point in (c, d):
        best = min(best, project(point, a, b))
    for t, point in ((0.0, a), (1.0, b)):
        best = min(best, (project(point, c, d)[0], t))
    return best


def sweep(board):
    protected = set()
    listing = os.path.join(HERE, "relay_board_prerouted.json")
    if os.path.exists(listing):
        protected = set(json.load(open(listing)))
    pads = [pad for footprint in board.GetFootprints() for pad in footprint.Pads() if pad.GetNetCode() > 0]
    pads_by_net = {}
    for pad in pads:
        pads_by_net.setdefault(pad.GetNetCode(), []).append(pad)

    records = []
    for item in board.GetTracks():
        net = item.GetNetCode()
        via = item.GetClass() == "PCB_VIA"
        record = {"item": item, "via": via, "net": net, "alive": True,
                  "fixed": item.m_Uuid.AsString() in protected or item.GetNetname() in ("", "/GND")}
        if via:
            position = item.GetPosition()
            record["a"] = record["b"] = (position.x, position.y)
            record["half"] = item.GetWidth(pcbnew.F_Cu) / 2
            record["pad_layers"] = {layer for pad in pads_by_net.get(net, ())
                                    if pad.HitTest(position, int(record["half"]))
                                    for layer in (pcbnew.F_Cu, pcbnew.B_Cu) if pad.IsOnLayer(layer)}
        else:
            start, end = item.GetStart(), item.GetEnd()
            record["a"], record["b"] = (start.x, start.y), (end.x, end.y)
            record["half"] = item.GetWidth() / 2
            record["layer"] = item.GetLayer()
            # where pads of the net touch this track. KiCad joins a track to a pad
            # when their copper overlaps anywhere, not only at the track's ends:
            # sample the centreline and test each point against the pad grown by
            # the track's half-width.
            touches = []
            length = math.hypot(end.x - start.x, end.y - start.y)
            steps = max(1, int(length / pcbnew.FromMM(0.1)))
            x0, x1 = min(start.x, end.x), max(start.x, end.x)
            y0, y1 = min(start.y, end.y), max(start.y, end.y)
            reach = int(record["half"] + TOLERANCE)
            for pad in pads_by_net.get(net, ()):
                if not pad.IsOnLayer(record["layer"]):
                    continue
                box = pad.GetBoundingBox()
                if (box.GetRight() + reach < x0 or box.GetLeft() - reach > x1
                        or box.GetBottom() + reach < y0 or box.GetTop() - reach > y1):
                    continue
                for k in range(steps + 1):
                    t = k / steps
                    point = pcbnew.VECTOR2I(int(start.x + t * (end.x - start.x)), int(start.y + t * (end.y - start.y)))
                    if pad.HitTest(point, reach):
                        touches.append(t)
            record["pad_t"] = touches
        records.append(record)

    by_net = {}
    for record in records:
        by_net.setdefault(record["net"], []).append(record)

    def contacts(record):
        """Parameters 0..1 along the track where live copper of its net meets it."""
        found = list(record["pad_t"])
        for other in by_net[record["net"]]:
            if other is record or not other["alive"]:
                continue
            if other["via"]:
                distance, t = project(other["a"], record["a"], record["b"])
                if distance <= max(record["half"], other["half"]) + TOLERANCE:
                    found.append(t)
                continue
            if other["layer"] != record["layer"]:
                continue
            # nearest approach of the two centrelines: covers an end of one lying
            # in the other's copper and two tracks that cross part-way along both
            distance, t = closest(record["a"], record["b"], other["a"], other["b"])
            if distance <= max(record["half"], other["half"]) + TOLERANCE:
                found.append(t)
            for point in (other["a"], other["b"]):          # each of its ends inside our copper
                distance, t = project(point, record["a"], record["b"])
                if distance <= record["half"] + TOLERANCE:
                    found.append(t)
            for t, point in ((0.0, record["a"]), (1.0, record["b"])):   # our ends inside its copper
                if project(point, other["a"], other["b"])[0] <= other["half"] + TOLERANCE:
                    found.append(t)
        return found

    def via_dead(record):
        """A via is dead when at most one thing hangs on it. A via joining one
        layer only can still be the copper that bridges two track ends lying
        inside its pad without touching each other: that one stays."""
        joined = len(record["pad_layers"])
        for other in by_net[record["net"]]:
            if other["via"] or not other["alive"]:
                continue
            if project(record["a"], other["a"], other["b"])[0] <= max(record["half"], other["half"]) + TOLERANCE:
                joined += 1
        return joined < 2

    cut = removed = 0
    for _ in range(60):
        changed = False
        for record in records:
            if record["fixed"] or not record["alive"]:
                continue
            if record["via"]:
                if via_dead(record):
                    record["alive"] = False
                    removed += 1
                    changed = True
                continue
            found = contacts(record)
            length = math.hypot(record["b"][0] - record["a"][0], record["b"][1] - record["a"][1])
            low, high = (min(found), max(found)) if found else (0.0, 0.0)
            if not found or (high - low) * length < MIN_SPAN:
                record["alive"] = False
                removed += 1
                changed = True
            elif low * length > MIN_SPAN or (1.0 - high) * length > MIN_SPAN:
                a, b = record["a"], record["b"]
                record["a"] = (a[0] + low * (b[0] - a[0]), a[1] + low * (b[1] - a[1]))
                record["b"] = (a[0] + high * (b[0] - a[0]), a[1] + high * (b[1] - a[1]))
                record["pad_t"] = [min(1.0, max(0.0, (t - low) / (high - low))) for t in record["pad_t"]]
                record["moved"] = True
                cut += 1
                changed = True
        if not changed:
            # one-layer vias that only bridge track ends lying inside their pad:
            # bring those ends to the via centre (inside copper that is already
            # there) and drop the via
            for record in records:
                if not record["via"] or record["fixed"] or not record["alive"] or record["pad_layers"]:
                    continue
                attached = [other for other in by_net[record["net"]]
                            if not other["via"] and other["alive"]
                            and project(record["a"], other["a"], other["b"])[0]
                            <= max(record["half"], other["half"]) + TOLERANCE]
                if len(attached) < 2 or len({other["layer"] for other in attached}) != 1                         or any(other["fixed"] for other in attached):
                    continue
                for other in attached:
                    for end in ("a", "b"):
                        gap = math.hypot(other[end][0] - record["a"][0], other[end][1] - record["a"][1])
                        if TOLERANCE < gap <= record["half"] + TOLERANCE:
                            other[end] = record["a"]
                            other["moved"] = True
                record["alive"] = False
                removed += 1
                changed = True
        if not changed:
            break

    for record in records:
        if not record["alive"]:
            board.Remove(record["item"])
        elif record.get("moved"):
            record["item"].SetStart(pcbnew.VECTOR2I(int(round(record["a"][0])), int(round(record["a"][1]))))
            record["item"].SetEnd(pcbnew.VECTOR2I(int(round(record["b"][0])), int(round(record["b"][1]))))
    return cut, removed


def remove_flagged(board, text, protected):
    """Second net: the tracks KiCad itself still reports as dangling on the
    filled board (ends on a net-tie pad and the like, which the geometric sweep
    reads as a contact). Whole-track removal; the caller's DRC guard decides."""
    flagged = re.findall(r"\[track_dangling\][^@]*@\((-?[\d.]+) mm, (-?[\d.]+) mm\): Track \[([^\]]+)\] on (\S+?), "
                         r"length ([\d.]+) mm", text)
    doomed = []
    for x, y, name, layer, length in flagged:
        if name in ("", "/GND"):
            continue
        spot = (pcbnew.FromMM(float(x)), pcbnew.FromMM(float(y)))
        for item in board.GetTracks():
            if (item.GetClass() != "PCB_TRACK" or item.GetNetname() != name
                    or board.GetLayerName(item.GetLayer()) != layer or item.m_Uuid.AsString() in protected
                    or abs(pcbnew.ToMM(item.GetLength()) - float(length)) > 0.002):
                continue
            ends = [(point.x, point.y) for point in (item.GetStart(), item.GetEnd())]
            if min(math.hypot(e[0] - spot[0], e[1] - spot[1]) for e in ends) < pcbnew.FromMM(0.01):
                doomed.append(item)
                break
    for item in doomed:
        board.Remove(item)
    return len(doomed)


def main_flagged():
    report = os.path.join(HERE, "relay_board_drc.txt")
    before = drc_counts(BOARD)
    text = open(report, encoding="utf-8", errors="replace").read()
    if "[track_dangling]" not in text:
        return 0
    protected = set()
    listing = os.path.join(HERE, "relay_board_prerouted.json")
    if os.path.exists(listing):
        protected = set(json.load(open(listing)))
    backup = BOARD.replace(".kicad_pcb", "_presweep.kicad_pcb")
    shutil.copyfile(BOARD, backup)
    board = pcbnew.LoadBoard(BOARD)
    count = remove_flagged(board, text, protected)
    if not count:
        os.remove(backup)
        return 0
    board.Save(BOARD)
    after = drc_counts(BOARD)
    print(f"flagged by DRC: {count} dangling tracks removed; unconnected {before[0]} -> {after[0]}")
    if after[0] > before[0]:
        shutil.copyfile(backup, BOARD)
        print("that cost a connection: board restored")
        return 1
    os.remove(backup)
    return 3                                 # changed: the caller refills the zones


def split_junctions(board):
    """Make every T-junction explicit: where a track end lies on the body of
    another track of its net (same layer), move the end onto that track's
    centreline and split the other track there, so that all three segments end
    at one point. The copper does not change. KiCad joins a T either way;
    tools that only match end points to end points (Faraday's open-stub rule)
    read the end as floating."""
    snap = pcbnew.FromMM(0.001)
    tracks = [item for item in board.GetTracks() if item.GetClass() == "PCB_TRACK"]
    records = [{"item": item, "net": item.GetNetCode(), "layer": item.GetLayer(), "half": item.GetWidth() / 2,
                "a": (item.GetStart().x, item.GetStart().y), "b": (item.GetEnd().x, item.GetEnd().y),
                "cuts": [], "locked": item.IsLocked()} for item in tracks]
    by_key = {}
    for record in records:
        by_key.setdefault((record["net"], record["layer"]), []).append(record)
    # an end that sits in a via or on a pad is already terminated: leave it
    vias = {}
    for item in board.GetTracks():
        if item.GetClass() == "PCB_VIA":
            position = item.GetPosition()
            vias.setdefault(item.GetNetCode(), []).append((position.x, position.y, item.GetWidth(pcbnew.F_Cu) / 2))
    pads = {}
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            pads.setdefault(pad.GetNetCode(), []).append(pad)
    for record in records:
        record["terminated"] = {}
        for end, point in (("a", record["item"].GetStart()), ("b", record["item"].GetEnd())):
            record["terminated"][end] = (
                any(math.hypot(point.x - x, point.y - y) <= radius + TOLERANCE for x, y, radius in vias.get(record["net"], ()))
                or any(pad.IsOnLayer(record["layer"]) and pad.HitTest(point, int(record["half"]))
                       for pad in pads.get(record["net"], ())))
    moved = 0
    for group in by_key.values():
        if group[0]["net"] <= 0:
            continue
        for record in group:
            for end in ("a", "b"):
                point = record[end]
                if record["terminated"][end]:
                    continue
                # already an end point of another track of the net there?
                if any(other is not record and (math.hypot(point[0] - other["a"][0], point[1] - other["a"][1]) <= snap
                                                or math.hypot(point[0] - other["b"][0], point[1] - other["b"][1]) <= snap)
                       for other in group):
                    continue
                # end caps that overlap without either end lying in the other's
                # copper (a thin neck): bring ours onto that end point
                near = [(math.hypot(point[0] - other[e][0], point[1] - other[e][1]), other[e])
                        for other in group if other is not record for e in ("a", "b")]
                near = [entry for entry in near if entry[0] <= record["half"] * 2 + TOLERANCE]
                if near:
                    record[end] = min(near)[1]
                    record["moved"] = True
                    moved += 1
                    continue
                best = None
                for other in group:
                    if other is record:
                        continue
                    distance, t = project(point, other["a"], other["b"])
                    if distance <= other["half"] + TOLERANCE and (best is None or distance < best[0]):
                        best = (distance, t, other)
                if best is None:
                    continue
                _, t, other = best
                a, b = other["a"], other["b"]
                junction = (int(round(a[0] + t * (b[0] - a[0]))), int(round(a[1] + t * (b[1] - a[1]))))
                if min(math.hypot(junction[0] - a[0], junction[1] - a[1]),
                       math.hypot(junction[0] - b[0], junction[1] - b[1])) <= other["half"] + TOLERANCE:
                    # within that track's own end cap: the two ends overlap without
                    # coinciding -- bring ours onto its end point
                    junction = a if t < 0.5 else b
                else:
                    other["cuts"].append((t, junction))
                if junction != point:
                    record[end] = junction
                    record["moved"] = True
                moved += 1
    split = 0
    for record in records:
        item = record["item"]
        if record.get("moved"):
            item.SetStart(pcbnew.VECTOR2I(int(record["a"][0]), int(record["a"][1])))
            item.SetEnd(pcbnew.VECTOR2I(int(record["b"][0]), int(record["b"][1])))
    for record in records:
        if not record["cuts"]:
            continue
        item = record["item"]
        points = [record["a"]] + [junction for _, junction in sorted(set(record["cuts"]))] + [record["b"]]
        item.SetStart(pcbnew.VECTOR2I(int(points[0][0]), int(points[0][1])))
        item.SetEnd(pcbnew.VECTOR2I(int(points[1][0]), int(points[1][1])))
        for start, end in zip(points[1:], points[2:]):
            if math.hypot(end[0] - start[0], end[1] - start[1]) <= snap:
                continue
            piece = pcbnew.PCB_TRACK(board)
            piece.SetStart(pcbnew.VECTOR2I(int(start[0]), int(start[1])))
            piece.SetEnd(pcbnew.VECTOR2I(int(end[0]), int(end[1])))
            piece.SetLayer(record["layer"])
            piece.SetWidth(int(record["half"] * 2))
            piece.SetNetCode(record["net"])
            piece.SetLocked(record["locked"])
            board.Add(piece)
            split += 1
    return moved, split


def main_split():
    before = drc_counts(BOARD)
    backup = BOARD.replace(".kicad_pcb", "_presweep.kicad_pcb")
    shutil.copyfile(BOARD, backup)
    board = pcbnew.LoadBoard(BOARD)
    moved, split = split_junctions(board)
    board.Save(BOARD)
    after = drc_counts(BOARD)
    print(f"T-junctions made explicit: {moved} track ends, {split} tracks split; "
          f"unconnected {before[0]} -> {after[0]}, dangling {before[1]} -> {after[1]}")
    if after[0] > before[0]:
        shutil.copyfile(backup, BOARD)
        drc_counts(BOARD)
        print("that cost a connection: board restored")
        return 1
    os.remove(backup)                    # an overhang past a junction now shows as dangling:
    return 0                             # the --flagged pass that follows removes it


def main():
    before = drc_counts(BOARD)
    if not before[1]:
        print("no dangling copper")
        return 0
    backup = BOARD.replace(".kicad_pcb", "_presweep.kicad_pcb")
    shutil.copyfile(BOARD, backup)
    board = pcbnew.LoadBoard(BOARD)
    cut, removed = sweep(board)
    if "--dry-run" in sys.argv:
        print(f"dry run: would cut back {cut} tracks and remove {removed} tracks/vias")
        return 0
    board.Save(BOARD)
    after = drc_counts(BOARD)
    print(f"dead copper: {cut} tracks cut back, {removed} tracks/vias removed; "
          f"dangling {before[1]} -> {after[1]}, unconnected {before[0]} -> {after[0]}")
    if after[0] > before[0]:
        shutil.copyfile(backup, BOARD)
        drc_counts(BOARD)
        print("the sweep cost a connection: original board restored")
        return 1
    os.remove(backup)
    return 0


if __name__ == "__main__":
    sys.exit(main_flagged() if "--flagged" in sys.argv else main_split() if "--split" in sys.argv else main())
