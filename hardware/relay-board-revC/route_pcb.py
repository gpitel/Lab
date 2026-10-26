"""
Route the rev C board: freerouting DSN/SES round trip, zone fill, DRC.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" route_pcb.py

Programmatic trace-by-trace routing was abandoned in rev A (see
ROUTING_GUIDE.md there); the supported path is the Specctra round trip with
the freerouting jar vendored in tools/.  This script drives it end to end:

    relay_board.dsn -> freerouting -> relay_board.ses -> ImportSpecctraSES
    -> fill zones -> save -> kicad-cli pcb drc
"""

import math
import os
import subprocess
import sys

import pcbnew

import generate_pcb as layout          # geometry + DSN plane-layer patch

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD_PATH = os.path.join(HERE, "relay_board.kicad_pcb")
DSN = os.path.join(HERE, "relay_board.dsn")
SES = os.path.join(HERE, "relay_board.ses")
JAR = os.path.join(HERE, "..", "..", "tools", "freerouting-1.9.0.jar")
KICAD_CLI = r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"

# freerouting 1.9 needs Java 17+; a portable Adoptium JRE is vendored in
# tools/ next to the freerouting jar.
_TOOLS_JAVA = next(iter(sorted(__import__("glob").glob(
    os.path.join(HERE, "..", "..", "tools", "jdk-*-jre", "bin", "java.exe")))), "")
JAVA = _TOOLS_JAVA if os.path.exists(_TOOLS_JAVA) else "java"


def patch_dsn_power_layers():
    """Mark the plane layers (In1/In3/In4) as power in the DSN so freerouting
    never routes there (must run after EVERY ExportSpecctraDSN)."""
    count = layout.patch_dsn_power_layers(DSN)
    if count:
        print(f"  DSN: {count} plane layers set to power")


def in_plane_region(x, y):
    """True where the GND planes (and the strip's +5V plane) exist."""
    return (layout.BOARD_X0 + 1.0 < x < layout.BOARD_X1 - 1.0
            and layout.BOARD_Y0 + 1.0 < y < layout.BOARD_Y1 - 1.0
            and not layout.point_in_polygon(x, y, layout.VOID))


def run_freerouting(passes=12):
    if not os.path.exists(DSN):
        raise SystemExit("relay_board.dsn missing -- run generate_pcb.py first")
    print(f"freerouting: {os.path.basename(DSN)} -> {os.path.basename(SES)}")
    command = [JAVA, "-jar", JAR, "-de", DSN, "-do", SES,
               "-mp", str(passes), "-us", "greedy"]   # global: >30 min on the 6-layer board
    result = subprocess.run(command, capture_output=True, text=True,
                            cwd=HERE, timeout=7200)
    tail = (result.stdout or "").strip().splitlines()[-8:]
    for line in tail:
        print("   ", line)
    if not os.path.exists(SES):
        print(result.stderr[-2000:] if result.stderr else "(no stderr)")
        raise SystemExit("freerouting produced no session file")


def mm(value):
    return pcbnew.FromMM(value)


def plane_fanout(board):
    """Deterministic via fanout for GND and +5V SMD pads.

    The DSN describes the zones as planes, so freerouting treats plane-net
    pads as already connected and skips them; the planes are on In1/In2/B.Cu
    while the pads are on F.Cu.  For every such pad without copper already
    arriving (a locked feed), drop a stub + via at the first collision-free
    candidate offset.
    """
    plane_nets = {"/GND": "/GND"}         # +5V is routed in rev C
    obstacles = []
    for track in board.GetTracks():
        obstacles.append(track)

    def collides(x, y, net_code):
        position = pcbnew.VECTOR2I(mm(x), mm(y))
        hit = pcbnew.PCB_VIA(board)
        hit.SetPosition(position)
        hit.SetDrill(mm(0.3))
        hit.SetWidth(mm(0.6))
        for item in obstacles:
            if item.GetNetCode() == net_code:
                continue
            if item.HitTest(position, mm(0.55)):
                return True
        for footprint in board.GetFootprints():
            for pad in footprint.Pads():
                if pad.GetNetCode() == net_code:
                    continue
                if pad.HitTest(position, mm(0.45)):
                    return True
        return False

    added, skipped = 0, 0
    candidates = ((1.3, 0), (-1.3, 0), (0, 1.3), (0, -1.3),
                  (1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0))
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            net_name = pad.GetNetname()
            if net_name not in plane_nets:
                continue
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue            # through-hole pads reach the planes
            position = pad.GetPosition()
            px, py = pcbnew.ToMM(position.x), pcbnew.ToMM(position.y)
            if net_name == "/+5V" and px < layout.STRIP_X0:
                continue            # relay coils: pre-routed In3 tree
            already = any(t.GetNetCode() == pad.GetNetCode()
                          and t.GetClass() == "PCB_VIA"
                          and abs(pcbnew.ToMM(t.GetPosition().x) - px) < 2.2
                          and abs(pcbnew.ToMM(t.GetPosition().y) - py) < 2.2
                          for t in board.GetTracks())
            if already:
                continue
            placed = False
            for dx, dy in candidates:
                x, y = px + dx, py + dy
                if not in_plane_region(x, y):
                    continue        # via must land inside the plane region
                if net_name == "/+5V" and x < layout.STRIP_X0 + 0.5:
                    continue        # +5V plane only in the control strip
                if any(collides(px + dx * f, py + dy * f, pad.GetNetCode())
                       for f in (0.4, 0.7, 1.0)):
                    continue
                track = pcbnew.PCB_TRACK(board)
                track.SetStart(position)
                track.SetEnd(pcbnew.VECTOR2I(mm(x), mm(y)))
                track.SetNet(pad.GetNet())
                track.SetLayer(pcbnew.F_Cu)
                track.SetWidth(mm(0.25))
                board.Add(track)
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
                via.SetDrill(mm(0.3))
                via.SetWidth(mm(0.6))
                via.SetNet(pad.GetNet())
                via.SetViaType(pcbnew.VIATYPE_THROUGH)
                board.Add(via)
                obstacles.append(track)
                obstacles.append(via)
                added += 1
                placed = True
                break
            if not placed:
                skipped += 1
                print(f"  fanout: no room at {footprint.GetReference()}"
                      f".{pad.GetNumber()} ({px:.1f},{py:.1f})")
    print(f"plane fanout: {added} vias added, {skipped} pads without room")


def repair_locked_clearance(board, margin_mm=0.0):
    """Remove freerouted (unlocked) copper that lands too close to the locked
    pre-routes -- freerouting's clearance checks against fixed wires are
    unreliable. Returns the set of net names whose geometry was culled."""
    all_items = [t for t in board.GetTracks()]
    locked = [t for t in all_items if t.IsLocked()]
    fiducials = [pad.GetPosition() for footprint in board.GetFootprints()
                 if footprint.GetReference().startswith("FID")
                 for pad in footprint.Pads()]
    culled_nets = set()
    to_remove = []
    for item in all_items:
        if item.IsLocked():
            continue
        sx, sy = item.GetStart().x, item.GetStart().y
        ex, ey = item.GetEnd().x, item.GetEnd().y
        anchors = ((sx, sy), (ex, ey), ((sx + ex) // 2, (sy + ey) // 2))
        offender = False
        for fixed in locked:
            if fixed.GetNetCode() == item.GetNetCode():
                continue
            if item.GetClass() != "PCB_VIA" and fixed.GetClass() != "PCB_VIA"                     and fixed.GetLayer() != item.GetLayer():
                continue
            for ax, ay in anchors:
                if fixed.HitTest(pcbnew.VECTOR2I(int(ax), int(ay)),
                                 mm(margin_mm + 0.14)):
                    offender = True
                    break
            if offender:
                break
        if not offender:
            for fid in fiducials:
                if item.HitTest(fid, mm(1.25)):
                    offender = True
                    break
        if offender:
            to_remove.append(item)
            culled_nets.add(item.GetNetname())
    for item in to_remove:
        board.Delete(item)
    if to_remove:
        print(f"repair: culled {len(to_remove)} freerouted items too close to "
              f"locked copper (nets: {sorted(culled_nets)})")
    return culled_nets


def return_path_stitching(board, reach=1.5):
    """A GND via within `reach` of every signal via outside the measurement
    void. A signal moving F.Cu <-> B.Cu changes reference plane (In1 <-> In2);
    its return current crosses between the planes at the nearest GND via, so
    that via must be right there or the return loop opens up (Faraday's
    return-path rule; Ott ch. 16)."""
    gnd = board.FindNet("/GND").GetNetCode()
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    gnd_xy = [(pcbnew.ToMM(v.GetPosition().x), pcbnew.ToMM(v.GetPosition().y))
              for v in vias if v.GetNetCode() == gnd]
    for footprint in board.GetFootprints():          # THT GND pads count too
        for pad in footprint.Pads():
            if pad.GetNetCode() == gnd and pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                gnd_xy.append((pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y)))
    pads = [pad for fp in board.GetFootprints() for pad in fp.Pads()]

    def spot_free(x, y):
        position = pcbnew.VECTOR2I(mm(x), mm(y))
        for item in board.GetTracks():
            if item.GetClass() == "PCB_VIA":
                q = item.GetPosition()
                if ((pcbnew.ToMM(q.x) - x) ** 2 + (pcbnew.ToMM(q.y) - y) ** 2) ** 0.5 < 0.85:
                    return False
            if item.GetNetCode() != gnd and item.HitTest(position, mm(0.55)):
                return False
        for pad in pads:
            if pad.GetNetCode() != gnd and pad.HitTest(position, mm(0.55)):
                return False
            # not in or against a GND SMD pad either: no via in a solder joint
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD and pad.HitTest(position, mm(0.35)):
                return False
        return True

    added, missing = 0, 0
    for via in vias:
        if via.GetNetCode() == gnd:
            continue
        x, y = pcbnew.ToMM(via.GetPosition().x), pcbnew.ToMM(via.GetPosition().y)
        if layout.point_in_polygon(x, y, layout.VOID):
            continue                                  # no planes there
        if any((gx - x) ** 2 + (gy - y) ** 2 < reach ** 2 for gx, gy in gnd_xy):
            continue
        for radius in (0.9, 1.2, 1.5):
            spots = [(x + radius * __import__("math").cos(a), y + radius * __import__("math").sin(a))
                     for a in [k * 3.14159265 / 6 for k in range(12)]]
            spot = next(((sx, sy) for sx, sy in spots
                         if not layout.point_in_polygon(sx, sy, layout.VOID) and spot_free(sx, sy)), None)
            if spot:
                stitch = pcbnew.PCB_VIA(board)
                stitch.SetPosition(pcbnew.VECTOR2I(mm(spot[0]), mm(spot[1])))
                stitch.SetDrill(mm(0.3))
                stitch.SetWidth(mm(0.6))
                stitch.SetNetCode(gnd)
                stitch.SetViaType(pcbnew.VIATYPE_THROUGH)
                stitch.SetLocked(True)
                board.Add(stitch)
                gnd_xy.append(spot)
                added += 1
                break
        else:
            missing += 1
    print(f"return-path stitching: {added} GND vias added"
          + (f", {missing} signal vias without room" if missing else ""))


def sweep_dangling(board):
    """Iteratively remove unlocked track segments with a floating end
    (geometric test: nothing else touches the endpoint)."""
    removed_total = 0
    for _ in range(12):
        tracks = [t for t in board.GetTracks()]
        pads = [pad for footprint in board.GetFootprints()
                for pad in footprint.Pads()]
        removed = []
        for item in tracks:
            if item.IsLocked() or item.GetClass() == "PCB_VIA":
                continue
            if item.GetLength() < mm(0.01):
                removed.append(item)
                continue
            for anchor in (item.GetStart(), item.GetEnd()):
                touched = False
                for other in tracks:
                    if other is item:
                        continue
                    if other.GetClass() != "PCB_VIA"                             and other.GetLayer() != item.GetLayer():
                        continue
                    if other.HitTest(anchor, mm(0.01)):
                        touched = True
                        break
                if not touched:
                    for pad in pads:
                        if pad.HitTest(anchor, mm(0.01)):
                            touched = True
                            break
                if not touched:
                    removed.append(item)
                    break
        if not removed:
            break
        for item in removed:
            board.Delete(item)
        removed_total += len(removed)
    if removed_total:
        print(f"swept {removed_total} dangling fragments")


def import_session():
    board = pcbnew.LoadBoard(BOARD_PATH)
    before = len([t for t in board.GetTracks()])
    if not pcbnew.ImportSpecctraSES(board, SES):
        raise SystemExit("ImportSpecctraSES failed")
    after = len([t for t in board.GetTracks()])
    print(f"session imported: {before} -> {after} tracks+vias")

    # purge unlocked inner-layer tracks: only locked pre-routes belong on
    # the planes (freerouting briefly routed there while the DSN power-type
    # patch was broken, and that junk otherwise survives every import)
    inner = (pcbnew.In1_Cu, pcbnew.In2_Cu)          # the plane layers
    junk = [t for t in board.GetTracks()
            if not t.IsLocked() and t.GetClass() != "PCB_VIA"
            and t.GetLayer() in inner]
    for item in junk:
        board.Delete(item)
    if junk:
        print(f"purged {len(junk)} unlocked inner-layer tracks")
    culled = repair_locked_clearance(board)
    sweep_dangling(board)
    plane_fanout(board)
    return_path_stitching(board)
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    board.Save(BOARD_PATH)
    print("zones filled, board saved")
    return board, culled


def run_drc():
    report = os.path.join(HERE, "relay_board_drc.txt")
    result = subprocess.run([KICAD_CLI, "pcb", "drc", "--format", "report",
                             "--severity-all", "-o", report, BOARD_PATH],
                            capture_output=True, text=True)
    print(result.stdout.strip() or result.stderr.strip())
    counts = {}
    if os.path.exists(report):
        for line in open(report, encoding="utf-8", errors="replace"):
            if line.startswith("["):
                key = line.split("]")[0][1:]
                counts[key] = counts.get(key, 0) + 1
            if line.startswith("** Found"):
                print("  " + line.strip())
    for key, count in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"    {count:4d}  {key}")
    return counts



def _end_touched(board, item, anchor, tracks, pads):
    """Does anything of the same net meet this track end?"""
    for other in tracks:
        if other is item or other.GetNetCode() != item.GetNetCode():
            continue
        if other.GetClass() != "PCB_VIA" and other.GetLayer() != item.GetLayer():
            continue
        if other.HitTest(anchor, mm(0.01)):
            return True
    return any(pad.GetNetCode() == item.GetNetCode() and pad.IsOnLayer(item.GetLayer())
               and pad.HitTest(anchor, mm(0.01)) for pad in pads)


def close_pad_gaps(board, reach=2.5):
    """The grid router counts a pad as reached when it enters the pad's raster
    cells, which on round or small pads can stop the track just short of the
    copper (seen on a 1.5 mm test pad and a BNC pin). For every pad DRC reports
    as unconnected, join the floating track ends of its net around it to the pad
    centre with a straight segment, if that segment is clear."""
    import re
    import optimize_tracks as exact
    report = os.path.join(HERE, "relay_board_drc.txt")
    subprocess.run([KICAD_CLI, "pcb", "drc", "--format", "report", "--severity-all", "-o", report, BOARD_PATH],
                   capture_output=True, text=True)
    text = open(report, encoding="utf-8", errors="replace").read()
    wanted = set(re.findall(r"\[unconnected_items\].*?(?:PTH pad|Pad) (\S+) \[[^\]]+\] of (\S+)", text, re.S))
    if not wanted:
        return
    obstacles = exact.build_obstacles(board)
    pads = [pad for footprint in board.GetFootprints() for pad in footprint.Pads()]
    tracks = list(board.GetTracks())
    closed = 0
    for number, reference in sorted(wanted):
        footprint = board.FindFootprintByReference(reference)
        for pad in [p for p in footprint.Pads() if p.GetNumber() == number]:
            ends = []
            for item in tracks:
                if item.GetClass() != "PCB_TRACK" or item.GetNetCode() != pad.GetNetCode()                         or not pad.IsOnLayer(item.GetLayer()):
                    continue
                for anchor in (item.GetStart(), item.GetEnd()):
                    gap = math.hypot(pad.GetPosition().x - anchor.x, pad.GetPosition().y - anchor.y)
                    if gap <= mm(reach) and not _end_touched(board, item, anchor, tracks, pads):
                        ends.append((gap, item, anchor))
            joined = False               # every floating end there: two branches can stop short of one pad
            for gap, item, anchor in sorted(ends, key=lambda entry: entry[0]):
                a = (pcbnew.ToMM(anchor.x), pcbnew.ToMM(anchor.y))
                b = (pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y))
                if not obstacles.clear(item.GetLayer(), a, b, pcbnew.ToMM(item.GetWidth()) / 2, item.GetNetCode()):
                    continue
                track = pcbnew.PCB_TRACK(board)
                track.SetStart(anchor)
                track.SetEnd(pad.GetPosition())
                track.SetLayer(item.GetLayer())
                track.SetWidth(item.GetWidth())
                track.SetNetCode(item.GetNetCode())
                track.SetLocked(True)
                board.Add(track)
                tracks.append(track)
                closed += 1
                joined = True
                print(f"  joined {item.GetNetname()} to {reference}.{number} ({pcbnew.ToMM(gap):.2f} mm)")
            if not joined:
                print(f"  {reference}.{number} [{pad.GetNetname()}]: no floating end within {reach} mm with a clear segment")
    if closed:
        print(f"closed {closed} track-to-pad gaps")


#: Nodes that sit at an intermediate potential in some configuration (a
#: floating or linked terminal): terminal force/sense buses and the clip arms.
#: The rails, their feeds, the standards column and the front-end are always at
#: the HI or LO potential (or behind a buffer): ground under them costs nothing.
FLOATING = tuple(f"/{kind}{t}" for kind in ("T", "S", "DUT_", "DUTS_") for t in "ABCD")
#: mm beyond the copper: 3.5x the 0.21 mm prepreg, where the fringe field to the
#: plane edge is a few percent of the parallel-plate value. At 1.5 mm the
#: openings of neighbouring buses merged into one area and a third of the
#: outer-layer copper (rails, coil wiring) lost its plane.
FLOAT_VOID_MARGIN = 0.75
FLOAT_BRIDGE = 1.0                     # mm: width of a ground bridge through a ring of openings
FLOAT_BRIDGE_MIN_AREA = 25.0           # mm2: only islands this large get a bridge


def float_node_voids(board):
    """Open the In1/In2 GND planes under the copper of the floating nodes
    only: one rule area that follows their routed tracks, vias and pads."""
    # (the previous opening, if any, is removed at the very end: after a Remove,
    # pcbnew hands back untyped wrappers and board.GetTracks() fails)
    stale = [zone for zone in board.Zones() if zone.GetZoneName() == "float_node_void"]
    margin = FLOAT_VOID_MARGIN
    union = pcbnew.SHAPE_POLY_SET()

    def add(points):
        piece = pcbnew.SHAPE_POLY_SET()
        piece.NewOutline()
        for x, y in points:
            piece.Append(int(x), int(y))
        try:
            union.BooleanAdd(piece)
        except TypeError:
            union.BooleanAdd(piece, pcbnew.SHAPE_POLY_SET.PM_FAST)

    def capsule(a, b, radius, sides=12):
        """Polygon around the segment a-b at the given distance."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        ux, uy = ((b[0] - a[0]) / length, (b[1] - a[1]) / length) if length else (1.0, 0.0)
        heading = math.atan2(uy, ux)
        points = []
        for k in range(sides + 1):                       # around b
            angle = heading - math.pi / 2 + math.pi * k / sides
            points.append((b[0] + radius * math.cos(angle), b[1] + radius * math.sin(angle)))
        for k in range(sides + 1):                       # around a
            angle = heading + math.pi / 2 + math.pi * k / sides
            points.append((a[0] + radius * math.cos(angle), a[1] + radius * math.sin(angle)))
        return points

    count = 0
    for item in board.GetTracks():
        if item.GetNetname() not in FLOATING:
            continue
        if item.GetClass() == "PCB_VIA":
            p = item.GetPosition()
            add(capsule((p.x, p.y), (p.x, p.y), item.GetWidth(pcbnew.F_Cu) / 2 + mm(margin)))
        else:
            start, end = item.GetStart(), item.GetEnd()
            add(capsule((start.x, start.y), (end.x, end.y), item.GetWidth() / 2 + mm(margin)))
        count += 1
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            if pad.GetNetname() in FLOATING:
                box = pad.GetBoundingBox()
                x0, y0, x1, y1 = box.GetLeft(), box.GetTop(), box.GetRight(), box.GetBottom()
                m = mm(margin)
                add([(x0 - m, y0 - m), (x1 + m, y0 - m), (x1 + m, y1 + m), (x0 - m, y1 + m)])
                count += 1
    # Ground enclosed by a ring of openings is an island: the zone filler
    # removes it, and the hole grows to everything the ring encloses (the east
    # side lost ~40 x 60 mm that way). Cut a narrow ground bridge through the
    # ring at its thinnest point, under one track: FLOAT_BRIDGE mm of track over
    # ground is ~0.1 pF, against a third of the outer-layer copper losing its plane.
    bridges = 0
    for _ in range(12):
        holes, every_hole = [], []
        for outline in range(union.OutlineCount()):
            for hole in range(union.HoleCount(outline)):
                chain = union.Hole(outline, hole)
                points = [(chain.CPoint(k).x, chain.CPoint(k).y) for k in range(chain.PointCount())]
                area = abs(sum(points[k][0] * points[(k + 1) % len(points)][1]
                               - points[(k + 1) % len(points)][0] * points[k][1]
                               for k in range(len(points)))) / 2 / 1e12
                every_hole.append(points)
                if area >= FLOAT_BRIDGE_MIN_AREA:        # a small island is not worth 0.1 pF
                    holes.append(points)
        if not holes:
            break
        step = mm(0.2)
        cuts = []

        def in_island(px, py):
            """Inside any enclosed island (a ray that comes out there has not reached the plane)."""
            for points in every_hole:
                inside = False
                for k in range(len(points)):
                    x1, y1 = points[k]
                    x2, y2 = points[(k + 1) % len(points)]
                    if (y1 > py) != (y2 > py) and px < x1 + (py - y1) * (x2 - x1) / (y2 - y1):
                        inside = not inside
                if inside:
                    return True
            return False

        for points in holes:
            best = None
            for x, y in points[::max(1, len(points) // 60)]:
                for k in range(24):
                    angle = 2 * math.pi * k / 24
                    dx, dy = math.cos(angle), math.sin(angle)
                    entered = False
                    for n in range(1, 60):
                        px, py = x + dx * step * n, y + dy * step * n
                        inside = union.Contains(pcbnew.VECTOR2I(int(px), int(py)))
                        if inside:
                            entered = True
                        elif entered:                     # through the ring, out the other side
                            if not in_island(px, py) and (best is None or n < best[0]):
                                best = (n, x, y, dx, dy)
                            break
                        elif n > 2:
                            break                         # this ray stays inside the island
            if best is not None:
                cuts.append(best)
        if not cuts:
            break
        half = mm(FLOAT_BRIDGE) / 2
        for n, x, y, dx, dy in cuts:
            a = (x - dx * mm(0.4), y - dy * mm(0.4))
            b = (x + dx * (step * n + mm(0.4)), y + dy * (step * n + mm(0.4)))
            piece = pcbnew.SHAPE_POLY_SET()
            piece.NewOutline()
            for px, py in ((a[0] - dy * half, a[1] + dx * half), (b[0] - dy * half, b[1] + dx * half),
                           (b[0] + dy * half, b[1] - dx * half), (a[0] + dy * half, a[1] - dx * half)):
                piece.Append(int(px), int(py))
            try:
                union.BooleanSubtract(piece)
            except TypeError:
                union.BooleanSubtract(piece, pcbnew.SHAPE_POLY_SET.PM_FAST)
            bridges += 1
    if bridges:
        print(f"  {bridges} ground bridges ({FLOAT_BRIDGE} mm) cut through rings of openings")
    zone = pcbnew.ZONE(board)
    zone.SetIsRuleArea(True)
    zone.SetDoNotAllowZoneFills(True)
    zone.SetDoNotAllowTracks(False)
    zone.SetDoNotAllowVias(False)
    zone.SetDoNotAllowPads(False)
    zone.SetDoNotAllowFootprints(False)
    zone.SetZoneName("float_node_void")
    layers = pcbnew.LSET()
    layers.AddLayer(pcbnew.In1_Cu)
    layers.AddLayer(pcbnew.In2_Cu)
    zone.SetLayerSet(layers)
    outline = zone.Outline()
    try:
        outline.BooleanAdd(union)
    except TypeError:
        outline.BooleanAdd(union, pcbnew.SHAPE_POLY_SET.PM_FAST)
    area = outline.Area() / 1e12
    areas = outline.OutlineCount()
    for old_zone in stale:
        board.Remove(old_zone)
    board.Add(zone)
    print(f"plane openings under floating nodes: {count} copper items, {areas} areas, "
          f"{area:.0f} mm2 of {FLOAT_VOID_MARGIN} mm-margin keep-out on In1/In2")


def finish_without_autorouter():
    """Everything is maze-routed (maze_route.py, coarse + --fine): no
    freerouting. Clean up, fan GND out, stitch return paths, fill, DRC."""
    # in its own process: once items have been added from Python, pcbnew's SWIG
    # layer starts handing back untyped wrappers in that process
    # (board.Tracks() "SwigPyObject is not iterable"), even for a re-loaded board
    subprocess.run([sys.executable, os.path.abspath(__file__), "--stage-gaps"], check=True)
    # dead copper (stub tails, rip-up fragments, one-layer vias): its own DRC guard
    subprocess.run([sys.executable, os.path.join(HERE, "cleanup_dangling.py")])
    # vias the router left in SMD pads: move them beside the pad; a net whose
    # via has no room is routed again on its own with vias kept off the pads
    mover = [sys.executable, os.path.join(HERE, "vias_out_of_pads.py")]
    result = subprocess.run(mover, capture_output=True, text=True)
    print(result.stdout.strip().splitlines()[-1] if result.stdout.strip() else result.stderr[-300:])
    stuck = sorted({line.split()[1] for line in result.stdout.splitlines() if line.startswith("  !! ")})
    for name in stuck:
        again = subprocess.run([sys.executable, os.path.join(HERE, "maze_route.py"), "--fine", "--polish",
                                f"--polish-net={name}", "--polish-force", "--no-via-in-pad"],
                               capture_output=True, text=True)
        print(([l for l in again.stdout.splitlines() if l.startswith(("  ~", "  ="))] or [f"  ? {name}"])[-1])
    if stuck:
        result = subprocess.run(mover, capture_output=True, text=True)
        print(chr(10).join(l for l in result.stdout.splitlines() if l.startswith(("vias out", "  !! "))))
        subprocess.run([sys.executable, os.path.join(HERE, "cleanup_dangling.py")])
    subprocess.run([sys.executable, os.path.abspath(__file__), "--stage-voids"], check=True)
    board = pcbnew.LoadBoard(BOARD_PATH)
    sweep_dangling(board)
    plane_fanout(board)
    return_path_stitching(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(BOARD_PATH)
    print("zones filled, board saved")
    # every T-junction as coinciding end points; what overhangs a junction is then
    # reported as dangling and removed by the loop below
    subprocess.run([sys.executable, os.path.join(HERE, "cleanup_dangling.py"), "--split"])
    counts = run_drc()
    for _ in range(4):                   # leftovers only KiCad's own check sees, on the filled board
        if not counts.get("track_dangling"):
            break
        if subprocess.run([sys.executable, os.path.join(HERE, "cleanup_dangling.py"), "--flagged"]).returncode != 3:
            break
        subprocess.run([sys.executable, os.path.abspath(__file__), "--stage-fill"], check=True)
        counts = run_drc()
    sys.exit(1 if counts.get("unconnected_items") or counts.get("clearance")
             or counts.get("shorting_items") or counts.get("tracks_crossing") else 0)


if __name__ == "__main__":
    if "--stage-fill" in sys.argv:
        stage_board = pcbnew.LoadBoard(BOARD_PATH)
        pcbnew.ZONE_FILLER(stage_board).Fill(stage_board.Zones())
        stage_board.Save(BOARD_PATH)
        sys.exit(0)
    if "--stage-gaps" in sys.argv or "--stage-voids" in sys.argv:
        stage_board = pcbnew.LoadBoard(BOARD_PATH)
        (close_pad_gaps if "--stage-gaps" in sys.argv else float_node_voids)(stage_board)
        stage_board.Save(BOARD_PATH)
        sys.exit(0)
    if "--no-freerouting" in sys.argv:
        finish_without_autorouter()
    if "--skip-route" not in sys.argv:
        run_freerouting()
    board, culled = import_session()
    if culled:
        # Second freerouting pass over the updated board: the culled nets
        # re-route against the now-committed remainder, which changes the
        # landscape enough to avoid the previous collision in practice.
        print("re-running freerouting for the culled nets...")
        board = pcbnew.LoadBoard(BOARD_PATH)
        pcbnew.ExportSpecctraDSN(board, DSN)
        patch_dsn_power_layers()
        run_freerouting(passes=16)   # different depth -> different landscape
        board, culled = import_session()
        if culled:
            print(f"WARNING: nets still conflicting after repair: {culled}")
    counts = run_drc()
    extra_round = 0
    while counts.get("unconnected_items") and extra_round < 12:
        extra_round += 1
        print(f"stragglers remain -- extra freerouting round {extra_round}...")
        board = pcbnew.LoadBoard(BOARD_PATH)
        pcbnew.ExportSpecctraDSN(board, DSN)
        patch_dsn_power_layers()
        run_freerouting(passes=12 + 4 * extra_round)
        import_session()
        counts = run_drc()
    sys.exit(1 if counts.get("unconnected_items") or counts.get("clearance")
             or counts.get("shorting_items") or counts.get("tracks_crossing")
             else 0)
