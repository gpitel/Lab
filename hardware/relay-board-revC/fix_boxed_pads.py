"""
Take the last four vias out of SMD pads on the routed board (2026-10-13).

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" fix_boxed_pads.py

A one-shot patch of the routed relay_board.kicad_pcb, not a routing stage:
vias_out_of_pads.py found no room beside four pads because a neighbour's
track closed the top layer around them, and the two attempts to give them
room before routing (locked escapes, turned parts) left the fine pass with
unrouted nets at the MCU. Here the neighbour is moved aside instead:

  R25.2 /FE_AMP    /FE_DM looped around the pad to reach R25.1; it now goes
                   straight between R24 and R25, and /FE_AMP runs on top to
                   R28.1 (its via and bottom-layer link are gone)
  R3.2  /NRST      the +3V3 feed of R3.1 moves 0.5 mm east, the via beside it
  U4.14 /K17_COIL  /K18_COIL's top-layer run moves 0.55 mm west, the via
                   between it and the pad
  R40.1 /COIL_EN   R40 moves 0.1 mm west (generate_pcb.py has the same
                   position), the via between it and U11

Every track named below must exist exactly as written, and every new piece is
checked against all other copper (optimize_tracks.build_obstacles) before
anything is saved: on a board routed differently the script stops and leaves
the file alone. Afterwards: route_pcb.py --stage-fill, then DRC.
"""

import math
import os
import shutil
import subprocess
import sys

import pcbnew

import optimize_tracks as exact

BOARD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "relay_board.kicad_pcb")
F, B = pcbnew.F_Cu, pcbnew.B_Cu
TOLERANCE = 0.01

MOVE_PARTS = [("R40", -0.1, 0.0)]
REMOVE_TRACKS = [
    (F, "/FE_DM", (100.5, 121.25), (101.5, 121.75)),
    (F, "/FE_DM", (101.5, 121.75), (101.5, 123.25)),
    (F, "/FE_DM", (101.5, 123.25), (100.75, 123.75)),
    (F, "/FE_DM", (100.75, 123.75), (99.5, 123.75)),
    (F, "/FE_DM", (99.5, 123.75), (99.0, 123.0)),
    (B, "/FE_AMP", (102.25, 121.75), (100.5, 122.5)),
    (B, "/FE_AMP", (102.25, 121.75), (102.5, 121.75)),
    (B, "/FE_AMP", (102.5, 117.0), (102.5, 121.75)),
    (F, "/+3V3", (180.0, 109.5), (179.625, 110.75)),
    (F, "/+3V3", (179.625, 110.75), (179.125, 111.25)),
    (F, "/+3V3", (179.125, 111.25), (179.0, 111.25)),
    (B, "/NRST", (179.875, 111.875), (178.875, 110.25)),
    (F, "/K18_COIL", (176.75, 86.75), (175.0, 86.75)),
    (F, "/K18_COIL", (175.0, 86.75), (175.0, 89.875)),
    (B, "/COIL_EN", (169.25, 128.25), (166.75, 128.25)),
]
REMOVE_VIAS = [("/FE_AMP", (100.5, 122.5))]
MOVE_VIAS = [
    ("/NRST", (178.875, 110.25), (179.45, 110.2)),
    ("/K17_COIL", (175.75, 88.0), (175.15, 88.1)),
    ("/COIL_EN", (166.75, 128.25), (167.2, 128.15)),
]
ADD_TRACKS = [
    (F, "/FE_DM", [(100.1, 121.3), (98.9, 122.3)], 0.3),
    (F, "/FE_AMP", [(100.32, 122.6), (102.49, 122.0)], 0.3),
    (B, "/FE_AMP", [(102.5, 117.0), (102.5, 121.25)], 0.3),
    (F, "/+3V3", [(180.0, 109.5), (180.15, 110.0), (180.15, 110.9), (179.6, 111.45), (178.9, 111.4)], 0.3),
    (F, "/NRST", [(178.75, 110.29), (179.45, 110.2)], 0.2),
    (B, "/NRST", [(179.875, 111.875), (179.45, 110.2)], 0.2),
    (F, "/K18_COIL", [(176.75, 86.75), (174.45, 86.75), (174.45, 89.325), (175.0, 89.875)], 0.3),
    (F, "/K17_COIL", [(175.75, 87.9), (175.15, 88.1)], 0.3),
    (B, "/K17_COIL", [(175.625, 88.25), (175.15, 88.1)], 0.3),
    (F, "/COIL_EN", [(166.5, 128.11), (167.2, 128.15)], 0.2),
    (B, "/COIL_EN", [(169.25, 128.25), (167.2, 128.15)], 0.2),
]


def same(p, q):
    return math.dist(p, q) <= TOLERANCE


def at(point):
    return (pcbnew.ToMM(point.x), pcbnew.ToMM(point.y))


def take_out(board):
    """Stage 1: parts moved, old copper removed. False if the board is not
    the one this patch was written for."""
    tracks = [(t, t.GetLayer(), t.GetNetname(), at(t.GetStart()), at(t.GetEnd()))
              for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [(v, v.GetNetname(), at(v.GetPosition())) for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]
    doomed, missing = [], []
    for layer, net, a, b in REMOVE_TRACKS:
        found = [t for t, l, n, s, e in tracks if l == layer and n == net
                 and ((same(s, a) and same(e, b)) or (same(s, b) and same(e, a)))]
        if len(found) != 1:
            missing.append(f"track {net} {a}-{b}: {len(found)} found")
        doomed += found
    for net, p in REMOVE_VIAS:
        found = [v for v, n, q in vias if n == net and same(p, q)]
        if len(found) != 1:
            missing.append(f"via {net} {p}: {len(found)} found")
        doomed += found
    for net, p, q in MOVE_VIAS:
        found = [v for v, n, old in vias if n == net and same(p, old)]
        if len(found) != 1:
            missing.append(f"via {net} {p}: {len(found)} found")
    if missing:
        print("not the board this patch was written for:\n  " + "\n  ".join(missing))
        return False
    for reference, dx, dy in MOVE_PARTS:
        footprint = board.FindFootprintByReference(reference)
        position = footprint.GetPosition()
        footprint.SetPosition(pcbnew.VECTOR2I(position.x + pcbnew.FromMM(dx), position.y + pcbnew.FromMM(dy)))
    for item in doomed:
        board.Remove(item)
    return True


def offenders(obstacles, layer, a, b, half, net):
    """What is in the way of a-b, as text (for the failure message)."""
    data, nets, _ = obstacles.arrays[layer]
    other = (nets != net) & (obstacles.second[layer] != net)
    short = half + data[:, 4] + data[:, 5] - exact.seg_seg_distance(a, b, data[:, 0:2], data[:, 2:4])
    return "; ".join(f"({r[0]:.3f}, {r[1]:.3f})-({r[2]:.3f}, {r[3]:.3f}) r{r[4]:.3f} short by {d:.3f}"
                     for r, d, o in zip(data, short, other) if o and d > 0)


def put_in(board):
    """Stage 2 (own process, freshly loaded board): vias moved and new copper
    added, each piece checked. The vias move here, together with their
    tracks: a via saved without a track on its net is taken for a free via
    and given the net of the zone it stands in (GND) when the file is read."""
    obstacles = exact.build_obstacles(board)
    codes = {name: board.FindNet(name).GetNetCode() for _, name, _, _ in ADD_TRACKS}
    blocked = []
    for net, _, q in MOVE_VIAS:
        code = board.FindNet(net).GetNetCode()
        if not all(obstacles.clear(layer, q, q, 0.3, code) for layer in exact.OUTER):
            blocked += [f"via {net} at {q} on {board.GetLayerName(layer)}: {offenders(obstacles, layer, q, q, 0.3, code)}"
                        for layer in exact.OUTER if not obstacles.clear(layer, q, q, 0.3, code)]
    for layer, net, points, width in ADD_TRACKS:
        for a, b in zip(points, points[1:]):
            if not obstacles.clear(layer, a, b, width / 2, codes[net]):
                blocked.append(f"track {net} {a}-{b} on {board.GetLayerName(layer)}: "
                               + offenders(obstacles, layer, a, b, width / 2, codes[net]))
    if blocked:
        print("no clearance for:\n  " + "\n  ".join(blocked))
        return False
    for net, p, q in MOVE_VIAS:
        for via in board.GetTracks():
            if via.GetClass() == "PCB_VIA" and via.GetNetname() == net and same(at(via.GetPosition()), p):
                via.SetPosition(exact.vec(*q))
    for layer, net, points, width in ADD_TRACKS:
        for a, b in zip(points, points[1:]):
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(exact.vec(*a))
            track.SetEnd(exact.vec(*b))
            track.SetLayer(layer)
            track.SetWidth(pcbnew.FromMM(width))
            track.SetNetCode(codes[net])
            track.SetLocked(True)
            board.Add(track)
    return True


def main():
    if "--stage-add" in sys.argv:            # own process: SWIG loses types after Remove
        board = pcbnew.LoadBoard(BOARD)
        if not put_in(board):
            return 1
        pcbnew.SaveBoard(BOARD, board)
        return 0
    backup = BOARD + ".before_fix"
    shutil.copyfile(BOARD, backup)
    board = pcbnew.LoadBoard(BOARD)
    if not take_out(board):
        os.remove(backup)
        return 1
    pcbnew.SaveBoard(BOARD, board)
    if subprocess.run([sys.executable, os.path.abspath(__file__), "--stage-add"]).returncode:
        shutil.copyfile(backup, BOARD)
        os.remove(backup)
        return 1
    os.remove(backup)
    print(f"boxed-in pads: {len(MOVE_VIAS)} vias moved, {len(REMOVE_VIAS)} removed, "
          f"{len(REMOVE_TRACKS)} tracks replaced by {sum(len(p) - 1 for _, _, p, _ in ADD_TRACKS)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
