"""
Deterministic maze routing of the measurement-critical and power nets.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" maze_route.py

Runs after generate_pcb.py and before route_pcb.py. The autorouter is not
trusted with the nets that decide the measurement: the terminal buses, the
standards column, the bridge and the Kelvin front-end, the USB pair and the
supplies are routed here, in a fixed order, by an A* maze router on a 0.25 mm
grid over F.Cu and B.Cu, plus In1/In2 inside the measurement void only (they
are GND planes everywhere else). Every pad, track, via, the DUT-bay keepout and the board edge are
obstacles inflated by the JLC clearance plus half the track width. Vias cost
extra so a net changes layer only when it must; each path is then
straightened (line-of-sight) into few segments. Everything routed here is
locked, so freerouting only finishes GPIO, coil returns, LEDs and I2C.

Re-exports relay_board.dsn with the plane layers marked power.
"""

import heapq
import math
import os
import sys

import numpy
import pcbnew

import generate_pcb as layout

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD = os.path.join(HERE, "relay_board.kicad_pcb")

#: --fine: second pass on a 0.125 mm grid for everything still unrouted
#: (digital, USB, supplies on the MCU's 0.5 mm-pitch pins): no autorouter.
FINE = "--fine" in sys.argv
GRID = 0.125 if FINE else 0.25
CLEARANCE = 0.2
#: Grid quantization margin: straightened segments run between cell centres,
#: so obstacles are inflated by this much more than the DRC clearance
#: (worst case ~GRID/sqrt(2)/... ; 0.08 on the fine grid, 0.12 on the coarse).
SAFETY = 0.08 if FINE else 0.12
INFLATE = CLEARANCE + SAFETY
VIA_DIAMETER, VIA_DRILL = 0.6, 0.3
VIA_COST = 6.0                         # in mm of track
LAYERS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
FREE, CONFLICT = -1, -2

#: (net, width mm) in routing order: measurement first, then USB, then power.
NETS = (
    [(n, 0.4) for n in ("/TA", "/SA", "/TB", "/SB", "/TC", "/SC", "/TD", "/SD")]
    + [(n, 0.4) for n in ("/CAL_E", "/CAL_ES", "/CAL_F", "/CAL_FS", "/STD1_F", "/STD1_S",
                          "/STD2_F", "/STD2_S", "/STD3_F", "/STD3_S")]
    + [(n, 0.5) for n in ("/RAIL_HI", "/RAIL_LO")]
    + [(n, 0.3) for n in ("/SNS_HI", "/SNS_LO", "/CH1_NODE", "/CH1_BNC", "/CH2_NODE",
                          "/FE_INP", "/FE_INN", "/FE_P", "/FE_N", "/FE_BP", "/FE_BN",
                          "/FE_DP", "/FE_DM", "/FE_AMP", "/FE_R", "/FE_OUT", "/VMID",
                          "/VMID_DIV")]
    + [(n, 0.5) for n in ("/+5V_IN", "/+5V", "/+5VA", "/USB_SHIELD")]
    # coil returns last, mostly on In1 beside the +5V coil tree: feed and
    # return side by side (smallest switching loop), and no new stray C under
    # the measurement nets (In1 there already carries +5V = AC ground). On
    # the outer layers they could not cross the buses to the driver column.
    + [(f"/K{k}_COIL", 0.3) for k in range(1, 24)]
)
#: Measurement nets: outer layers only (F.Cu / B.Cu). The inner layers inside
#: the void are for the +5V coil feed and the coil returns.
MEASUREMENT = {name for name, _ in NETS
               if not name.startswith(("/+5V", "/USB_")) and not name.endswith("_COIL")}

# Coupling (crosstalk) cost between measurement copper and everything that
# switches or carries supply (coils, supplies, digital): each grid step costs
# 1 + COUPLING_BETA * K, with K = 1 / (1 + (d / COUPLING_H)^2) the parallel-line
# coupling coefficient (H. Johnson; d = 3-D distance to the nearest copper of
# the other class on the same layer or on the layer across the thin 0.21 mm
# prepreg -- broadside coupling is the strongest). Parallel runs, including
# broadside overlaps, become expensive; a short 90-degree crossing stays cheap
# (orthogonal routing of adjacent layers). Beyond COUPLING_RADIUS K < 0.06.
COUPLING_H = 0.5                       # mm: no reference plane inside the void
COUPLING_BETA = 6.0
COUPLING_RADIUS = 2.0                  # mm
LAYER_Z = (0.0, 0.245, 1.325, 1.57)    # JLC04161H-7628 copper heights, mm

#: reference -> mm of free board kept around it by the coarse pass
ACCESS_POCKETS = {"U12": 1.5, "C36": 0.8}

#: Per-net layer preference (F, In1, In2, B); the inner layers carry no tracks at all.
DEFAULT_LAYER_COST = (1.0, 1.15, 1.15, 1.0)
LAYER_COST = {}
# The VBUS feed has to cross the USB chain (J1 -> D2 -> L1 -> MCU runs north
# between FB1 and U5/U11): the supply crosses underneath on B.Cu, the USB pair
# stays on F.Cu, unbroken and via-free.
LAYER_COST.update({name: (3.0, 1.15, 1.15, 1.0) for name in ("/+5V_IN", "/VBUS")})
LAYER_COST.update({name: (1.0, 1.15, 1.15, 3.0) for name in
                   ("/USB_DM", "/USB_DP", "/USB_DM_ESD", "/USB_DP_ESD", "/USB_DM_CON", "/USB_DP_CON")})
# Relay-drive lanes by MCU side (generate_schematic.GPIO_PINS): the west-column
# group (PA3..PB2 = GPIO14, GPIO1..7) must cross the north-row and east-column
# lanes on its way to the far drivers -- it runs on B.Cu, they run on F.Cu,
# so the crossings cost nothing but the vias at the lane ends.
LAYER_COST.update({f"/GPIO{k}": (3.0, 1.15, 1.15, 1.0) for k in (1, 2, 3, 4, 5, 6, 7, 14)})
LAYER_COST["/NRST"] = (3.0, 1.15, 1.15, 1.0)      # static: under the lane fans (BEFORE_LANES)
LAYER_COST.update({f"/GPIO{k}": (1.0, 1.15, 1.15, 3.0)
                   for k in range(8, 24) if k != 14})
#: Left to freerouting on purpose: nets that land on 0.5 mm-pitch pins (MCU
#: supplies, USB after the pre-routed entry, VBUS at the connector), where a
#: 0.25 mm grid cannot keep the clearance; freerouting sees the exact pads.


class Grid:
    def __init__(self, board):
        self.board = board
        self.x0, self.y0 = layout.BOARD_X0, layout.BOARD_Y0
        self.nx = int((layout.BOARD_X1 - self.x0) / GRID) + 1
        self.ny = int((layout.BOARD_Y1 - self.y0) / GRID) + 1
        self.xs = self.x0 + GRID * numpy.arange(self.nx)
        self.ys = self.y0 + GRID * numpy.arange(self.ny)
        # owner: which net's copper (inflated by CLEARANCE) covers a cell
        self.owner = numpy.full((len(LAYERS), self.ny, self.nx), FREE, dtype=numpy.int32)
        # copper: actual copper (no inflation), for connectivity
        self.copper = numpy.full((len(LAYERS), self.ny, self.nx), FREE, dtype=numpy.int32)
        self.via_cells = {}          # (iy, ix) -> net code (through vias / THT)
        self._build()

    # ------------------------------------------------------------ rasterize
    def _window(self, x0, y0, x1, y1):
        i0 = max(0, int((x0 - self.x0) / GRID) - 1)
        i1 = min(self.nx, int((x1 - self.x0) / GRID) + 2)
        j0 = max(0, int((y0 - self.y0) / GRID) - 1)
        j1 = min(self.ny, int((y1 - self.y0) / GRID) + 2)
        return j0, j1, i0, i1

    def _mark(self, grid, layer_index, mask, window, net):
        j0, j1, i0, i1 = window
        view = grid[layer_index, j0:j1, i0:i1]
        free = view == FREE
        view[mask & free] = net
        view[mask & ~free & (view != net)] = CONFLICT

    def add_rect(self, net, layers, cx, cy, hw, hh, inflate=INFLATE):
        for grid, extra in ((self.owner, inflate), (self.copper, 0.0)):
            window = self._window(cx - hw - extra, cy - hh - extra, cx + hw + extra, cy + hh + extra)
            j0, j1, i0, i1 = window
            X, Y = numpy.meshgrid(self.xs[i0:i1], self.ys[j0:j1])
            dx = numpy.maximum(numpy.abs(X - cx) - hw, 0)
            dy = numpy.maximum(numpy.abs(Y - cy) - hh, 0)
            mask = numpy.hypot(dx, dy) <= extra + 1e-9
            for layer in layers:
                self._mark(grid, layer, mask, window, net)

    def add_segment(self, net, layer, x1, y1, x2, y2, half, inflate=INFLATE):
        for grid, extra in ((self.owner, inflate), (self.copper, 0.0)):
            r = half + extra
            window = self._window(min(x1, x2) - r, min(y1, y2) - r, max(x1, x2) + r, max(y1, y2) + r)
            j0, j1, i0, i1 = window
            X, Y = numpy.meshgrid(self.xs[i0:i1], self.ys[j0:j1])
            vx, vy = x2 - x1, y2 - y1
            length2 = vx * vx + vy * vy or 1e-12
            t = numpy.clip(((X - x1) * vx + (Y - y1) * vy) / length2, 0, 1)
            d = numpy.hypot(X - (x1 + t * vx), Y - (y1 + t * vy))
            self._mark(grid, layer, d <= r + 1e-9, window, net)

    def add_circle(self, net, x, y, radius, layers=range(len(LAYERS)), inflate=INFLATE):
        for layer in layers:
            self.add_segment(net, layer, x, y, x, y, radius, inflate)

    # ------------------------------------------------------------ coupling
    def set_classes(self, nets_by_name):
        """'meas' for measurement nets, None for GND, 'agg' for the rest."""
        self.meas_codes = [n.GetNetCode() for name, n in nets_by_name.items() if name in MEASUREMENT]
        self.agg_codes = [n.GetNetCode() for name, n in nets_by_name.items()
                          if name not in MEASUREMENT and name not in ("/GND", "") and n.GetNetCode() > 0]

    def _distance(self, mask):
        """Lateral distance (mm) from every cell to the nearest True cell of
        `mask` (2-D), capped at COUPLING_RADIUS."""
        cap = COUPLING_RADIUS
        distance = numpy.full(mask.shape, cap)
        if not mask.any():
            return distance
        reach = int(cap / GRID)
        ny, nx = mask.shape
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                d = math.hypot(dx, dy) * GRID
                if d >= cap:
                    continue
                src = mask[max(0, -dy):ny - max(0, dy), max(0, -dx):nx - max(0, dx)]
                view = distance[max(0, dy):ny - max(0, -dy), max(0, dx):nx - max(0, -dx)]
                numpy.minimum(view, numpy.where(src, d, cap), out=view)
        return distance

    def coupling_cost(self, net):
        """Per-layer cost multiplier for routing `net` (>= 1)."""
        if net in self.meas_codes:
            others = self.agg_codes
        elif net in self.agg_codes:
            others = self.meas_codes
        else:
            return numpy.ones(self.owner.shape)
        mask = numpy.isin(self.copper, others)
        lateral = [self._distance(mask[l]) for l in range(len(LAYERS))]
        cost = numpy.ones(self.owner.shape)
        for l in range(len(LAYERS)):
            k = numpy.zeros(mask.shape[1:])
            for m in range(len(LAYERS)):
                dz = abs(LAYER_Z[l] - LAYER_Z[m])
                if dz > 0.5:
                    continue
                d2 = lateral[m] ** 2 + dz ** 2
                k_m = 1.0 / (1.0 + d2 / COUPLING_H ** 2)
                k_m[lateral[m] >= COUPLING_RADIUS] = 0.0
                k = numpy.maximum(k, k_m)
            cost[l] = 1.0 + COUPLING_BETA * k
        return cost

    def _build(self):
        board = self.board
        for footprint in board.GetFootprints():
            for pad in footprint.Pads():
                net = pad.GetNetCode() or -10 - pad.GetPadToDieLength()   # unconnected pads: unique
                box = pad.GetBoundingBox()
                cx, cy = pcbnew.ToMM(box.Centre().x), pcbnew.ToMM(box.Centre().y)
                hw, hh = pcbnew.ToMM(box.GetWidth()) / 2, pcbnew.ToMM(box.GetHeight()) / 2
                if pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                    layers = range(len(LAYERS))
                    if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                        net = CONFLICT
                else:
                    layers = [i for i, l in enumerate(LAYERS) if pad.IsOnLayer(l)]
                if layers:
                    self.add_rect(net if net >= 0 else CONFLICT, layers, cx, cy, hw, hh)
        # net-tie bridges (footprint copper with no net): no via within reach
        # of them -- tracks keep clear through their pads' own inflation, but
        # a via hole needs 0.25 mm to any copper
        self.via_block = numpy.zeros((self.ny, self.nx), dtype=bool)
        for footprint in board.GetFootprints():
            if not footprint.GetReference().startswith("NT"):
                continue
            box = footprint.GetBoundingBox(False)
            x0, x1 = pcbnew.ToMM(box.GetLeft()) - 0.75, pcbnew.ToMM(box.GetRight()) + 0.75
            y0, y1 = pcbnew.ToMM(box.GetTop()) - 0.75, pcbnew.ToMM(box.GetBottom()) + 0.75
            j0, j1, i0, i1 = self._window(x0, y0, x1, y1)
            self.via_block[j0:j1, i0:i1] = True
        # --no-via-in-pad: no via in or overlapping an SMD pad, the net's own
        # pads included (an open via in a pad drains the solder joint). Only for
        # single-net re-routes: with it on for the whole fine pass 18 nets found
        # no route. vias_out_of_pads.py moves the vias afterwards and
        # route_pcb.py re-routes the nets it could not help with this flag.
        reach = VIA_DIAMETER / 2
        for footprint in (board.GetFootprints() if "--no-via-in-pad" in sys.argv else ()):
            for pad in footprint.Pads():
                if pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                    continue
                box = pad.GetBoundingBox()
                j0, j1, i0, i1 = self._window(pcbnew.ToMM(box.GetLeft()) - reach, pcbnew.ToMM(box.GetTop()) - reach,
                                              pcbnew.ToMM(box.GetRight()) + reach, pcbnew.ToMM(box.GetBottom()) + reach)
                self.via_block[j0:j1, i0:i1] = True
        for item in board.GetTracks():
            net = item.GetNetCode()
            if item.GetClass() == "PCB_VIA":
                p = item.GetPosition()
                self.add_circle(net, pcbnew.ToMM(p.x), pcbnew.ToMM(p.y),
                                pcbnew.ToMM(item.GetWidth(pcbnew.F_Cu)) / 2)
            else:
                layer = item.GetLayer()
                if layer not in LAYERS:
                    continue
                s, e = item.GetStart(), item.GetEnd()
                self.add_segment(net, LAYERS.index(layer), pcbnew.ToMM(s.x), pcbnew.ToMM(s.y),
                                 pcbnew.ToMM(e.x), pcbnew.ToMM(e.y), pcbnew.ToMM(item.GetWidth()) / 2)
        # In1/In2 are GND planes outside the measurement void: route on them
        # only inside the void, and never over the In2 guard strips
        inner = [LAYERS.index(pcbnew.In1_Cu), LAYERS.index(pcbnew.In2_Cu)]
        X, Y = numpy.meshgrid(self.xs, self.ys)
        inside = numpy.zeros(X.shape, dtype=bool)
        polygon = layout.VOID
        for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
            crosses = (y1 > Y) != (y2 > Y)
            xc = x1 + (Y - y1) * (x2 - x1) / ((y2 - y1) or 1e-12)
            inside ^= crosses & (X < xc)
        # not marked in the owner map: a through via may pass a plane (the
        # plane clears around it); only tracks are kept off (free_for)
        self.plane_only = ~inside
        for x0, y0, x1, y1 in layout.Board.guard_strips():
            self.add_rect(CONFLICT, [LAYERS.index(pcbnew.In2_Cu)], (x0 + x1) / 2, (y0 + y1) / 2,
                          (x1 - x0) / 2, (y1 - y0) / 2)
        # access pockets (coarse pass only): small ICs routed in the fine pass
        # keep a margin of free board around them so the coarse nets cannot
        # wall them in (the TMP117 beside the standards was boxed in by
        # measurement lines that the fine pass may not displace)
        if not FINE:
            for footprint in board.GetFootprints():
                margin = ACCESS_POCKETS.get(footprint.GetReference())
                if margin is None:
                    continue
                box = footprint.GetBoundingBox(False)
                x0, x1 = pcbnew.ToMM(box.GetLeft()) - margin, pcbnew.ToMM(box.GetRight()) + margin
                y0, y1 = pcbnew.ToMM(box.GetTop()) - margin, pcbnew.ToMM(box.GetBottom()) + margin
                self.add_rect(CONFLICT, range(len(LAYERS)), (x0 + x1) / 2, (y0 + y1) / 2,
                              (x1 - x0) / 2, (y1 - y0) / 2, inflate=0.0)
        # DUT bay: nothing on any layer
        x0, y0, x1, y1 = layout.BAY
        self.add_rect(CONFLICT, range(len(LAYERS)), (x0 + x1) / 2, (y0 + y1) / 2,
                      (x1 - x0) / 2, (y1 - y0) / 2, inflate=0.0)
        # board edge margin
        margin = int(math.ceil(0.7 / GRID))
        for grid in (self.owner,):
            grid[:, :margin, :] = CONFLICT
            grid[:, -margin:, :] = CONFLICT
            grid[:, :, :margin] = CONFLICT
            grid[:, :, -margin:] = CONFLICT

    # --------------------------------------------------------------- route
    def free_for(self, net, half):
        """Cells where a track of half-width `half` of `net` may run, per layer:
        the owner map is inflated by INFLATE only, so erode by `half`."""
        ok = (self.owner == FREE) | (self.owner == net)
        steps = int(math.ceil(half / GRID))
        for _ in range(steps):
            shrunk = ok.copy()
            shrunk[:, 1:, :] &= ok[:, :-1, :]
            shrunk[:, :-1, :] &= ok[:, 1:, :]
            shrunk[:, :, 1:] &= ok[:, :, :-1]
            shrunk[:, :, :-1] &= ok[:, :, 1:]
            ok = shrunk
        # copper of the net itself is always walkable (starts/targets inside pads)
        ok |= self.copper == net
        self.via_layers = ok.copy()            # vias may cross the GND planes
        for layer in (LAYERS.index(pcbnew.In1_Cu), LAYERS.index(pcbnew.In2_Cu)):
            ok[layer] &= ~self.plane_only
        return ok

    def via_ok(self, ok_track, net):
        """Through via (radius 0.3) fits: track-free on every layer within one
        extra cell (radius 0.3 > track half-widths used here)."""
        all_layers = numpy.logical_and.reduce(self.via_layers, axis=0) & ~self.via_block
        if FINE:
            # square (8-neighbour) erosion by the via radius: conservative
            steps = int(math.ceil((VIA_DIAMETER / 2) / GRID))
            ok = all_layers
            for _ in range(steps):
                shrunk = ok.copy()
                for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    shifted = numpy.zeros_like(ok)
                    shifted[max(0, dj):ok.shape[0] + min(0, dj), max(0, di):ok.shape[1] + min(0, di)] =                         ok[max(0, -dj):ok.shape[0] + min(0, -dj), max(0, -di):ok.shape[1] + min(0, -di)]
                    shrunk &= shifted
                ok = shrunk
            return ok
        dilated = all_layers.copy()
        dilated[1:, :] &= all_layers[:-1, :]
        dilated[:-1, :] &= all_layers[1:, :]
        dilated[:, 1:] &= all_layers[:, :-1]
        dilated[:, :-1] &= all_layers[:, 1:]
        return dilated

    def components(self, net):
        """Connected copper islands of a net (4-neighbour, vias join layers)."""
        cells = numpy.argwhere(self.copper == net)
        label = {}
        component = 0
        cell_set = set(map(tuple, cells))
        via_xy = {xy for xy, n in self.via_cells.items() if n == net}
        for start in map(tuple, cells):
            if start in label:
                continue
            stack = [start]
            label[start] = component
            while stack:
                l, j, i = stack.pop()
                # 8-neighbour: a diagonal track only covers its own cells
                neighbours = [(l, j + dj, i + di) for dj in (-1, 0, 1) for di in (-1, 0, 1)
                              if dj or di]
                if (j, i) in via_xy:
                    neighbours += [(m, j, i) for m in range(len(LAYERS))]
                for nb in neighbours:
                    if nb in cell_set and nb not in label:
                        label[nb] = component
                        stack.append(nb)
            component += 1
        groups = {}
        for cell, c in label.items():
            groups.setdefault(c, []).append(cell)
        return list(groups.values())

    def astar(self, net, sources, targets, ok, via_ok, layer_cost=DEFAULT_LAYER_COST, coupling=None):
        target_set = set(targets)
        # quick reject: if the target's free neighbourhood is a small pocket
        # that never meets the source, the full search would only burn time
        source_set = set(sources)
        seen = set(targets)
        frontier = list(targets)
        while frontier and len(seen) < 60000:
            l, j, i = frontier.pop()
            if (l, j, i) in source_set:
                break
            nbs = [(l, j + dj, i + di) for dj in (-1, 0, 1) for di in (-1, 0, 1) if dj or di]
            if via_ok[j, i]:
                nbs += [(m, j, i) for m in range(len(LAYERS)) if m != l]
            for nb in nbs:
                L, J, I = nb
                if 0 <= J < self.ny and 0 <= I < self.nx and nb not in seen and ok[L, J, I]:
                    seen.add(nb)
                    frontier.append(nb)
        else:
            if not frontier:
                return None                    # enclosed pocket: no path at this width
        tj = numpy.array([t[1] for t in targets])
        ti = numpy.array([t[2] for t in targets])
        centre = (float(tj.mean()), float(ti.mean()))

        def h(j, i):
            dy, dx = abs(j - centre[0]), abs(i - centre[1])
            return (max(dx, dy) + 0.414 * min(dx, dy)) * GRID * (2.0 if FINE else 0.5)   # weighted A* on the fine grid

        open_heap = []
        best = {}
        parent = {}
        for s in sources:
            best[s] = 0.0
            heapq.heappush(open_heap, (h(s[1], s[2]), 0.0, s))
        moves = [(0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        expanded = 0
        while open_heap:
            f, g, node = heapq.heappop(open_heap)
            if g > best.get(node, 1e18):
                continue
            if node in target_set:
                path = [node]
                while path[-1] in parent:
                    path.append(parent[path[-1]])
                return path[::-1]
            expanded += 1
            if expanded > (4_000_000 if FINE else 6_000_000):
                return None
            l, j, i = node
            for dj, di, step in moves:
                nj, ni = j + dj, i + di
                if 0 <= nj < self.ny and 0 <= ni < self.nx and ok[l, nj, ni]:
                    if dj and di and not FINE and not (ok[l, j + dj, i] and ok[l, j, i + di]):
                        continue                       # no corner cutting (fine grid: exact check)
                    ng = g + step * GRID * layer_cost[l] * (coupling[l, nj, ni] if coupling is not None else 1.0)
                    nb = (l, nj, ni)
                    if ng < best.get(nb, 1e18):
                        best[nb] = ng
                        parent[nb] = node
                        heapq.heappush(open_heap, (ng + h(nj, ni), ng, nb))
            if via_ok[j, i]:
                for m in range(len(LAYERS)):
                    if m != l and ok[m, j, i]:
                        nb = (m, j, i)
                        ng = g + VIA_COST
                        if ng < best.get(nb, 1e18):
                            best[nb] = ng
                            parent[nb] = node
                            heapq.heappush(open_heap, (ng + h(j, i), ng, nb))
        return None

    def clear_line(self, ok_layer, a, b):
        (j0, i0), (j1, i1) = a, b
        n = int(max(abs(j1 - j0), abs(i1 - i0)) * 2) + 1
        for k in range(n + 1):
            t = k / n
            j = int(round(j0 + (j1 - j0) * t))
            i = int(round(i0 + (i1 - i0) * t))
            if not ok_layer[j, i]:
                return False
        return True

    def to_mm(self, j, i):
        return float(self.x0 + i * GRID), float(self.y0 + j * GRID)


def route_net(board, grid, name, width, nets_by_name):
    """Route one net; on failure remove everything added for it (no partial
    locked copper for freerouting to work around)."""
    owner, copper, vias = grid.owner.copy(), grid.copper.copy(), dict(grid.via_cells)
    added = []
    done, tracks, via_count = _route_net(board, grid, name, width, nets_by_name, added)
    if not done:
        for item in added:
            board.Remove(item)
        grid.owner, grid.copper, grid.via_cells = owner, copper, vias
        return False, 0, 0
    exact = getattr(grid, "exact", None)
    if exact is not None and added:
        import optimize_tracks as ex
        net = nets_by_name[name].GetNetCode()
        for item in added:
            if item.GetClass() == "PCB_VIA":
                p = item.GetPosition()
                c = (pcbnew.ToMM(p.x), pcbnew.ToMM(p.y))
                for l in (pcbnew.F_Cu, pcbnew.B_Cu):
                    exact.add(l, c, c, VIA_DIAMETER / 2, ex.CLEARANCE, net, ex.uid(item))
                    exact.add(l, c, c, VIA_DRILL / 2, ex.HOLE_CLEARANCE, net, ex.uid(item))
            elif item.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu):
                a, b = item.GetStart(), item.GetEnd()
                exact.add(item.GetLayer(), (pcbnew.ToMM(a.x), pcbnew.ToMM(a.y)),
                          (pcbnew.ToMM(b.x), pcbnew.ToMM(b.y)),
                          pcbnew.ToMM(item.GetWidth()) / 2, ex.CLEARANCE, net, ex.uid(item))
        exact.freeze()
    return True, tracks, via_count


def _route_net(board, grid, name, width, nets_by_name, added):
    net = nets_by_name[name].GetNetCode()
    full_width = width
    added_tracks, added_vias = 0, 0
    coupling = grid.coupling_cost(net)          # other-class copper only: fixed per net
    extra = getattr(grid, "extra_cost", None)
    if extra is not None:
        coupling = coupling * extra
    widths = [full_width] + [w for w in (0.3, 0.2) if w < full_width]
    narrowed = 0
    blocked, via_blocked = set(), set()       # cells refused by the exact check
    exact = getattr(grid, "exact", None)      # fine pass: exact clearance oracle
    for attempt in range(400):
        width = widths[narrowed]
        half = width / 2
        islands = grid.components(net)
        if len(islands) <= 1:
            return True, added_tracks, added_vias
        islands.sort(key=len, reverse=True)
        ok = grid.free_for(net, half)
        via_ok = grid.via_ok(ok, net)
        # no net routes on the inner layers: they are for planes and guard copper
        # only (the +5 V coil tree and the coil returns used to run on In1/In2
        # inside the void, 0.21 mm under the measurement tracks on F.Cu)
        for layer in (LAYERS.index(pcbnew.In1_Cu), LAYERS.index(pcbnew.In2_Cu)):
            ok[layer] &= grid.copper[layer] == net
        for l, j, i in blocked:
            ok[l, j, i] = False
        for j, i in via_blocked:
            via_ok[j, i] = False
        tree = islands[0]
        # nearest other island by centroid
        tree_c = numpy.mean([(c[1], c[2]) for c in tree], axis=0)
        others = sorted(islands[1:], key=lambda isl: numpy.hypot(*(numpy.mean([(c[1], c[2]) for c in isl], axis=0) - tree_c)))
        target = others[0]
        path = grid.astar(net, tree, target, ok, via_ok, LAYER_COST.get(name, DEFAULT_LAYER_COST), coupling)
        if path is None:
            if narrowed + 1 < len(widths):
                narrowed += 1          # this connection only: tight pad entry
                print(f"  .. {name}: retrying a connection at {widths[narrowed]} mm")
                continue
            refused = sorted(blocked) + [(None,) + c for c in sorted(via_blocked)]
            where = ", ".join(f"{'via' if l is None else pcbnew.LayerName(LAYERS[l])}"
                              f"@({grid.to_mm(j, i)[0]:.2f},{grid.to_mm(j, i)[1]:.2f})"
                              for l, j, i in refused[:4])
            print(f"  !! {name}: no path for an island of {len(target)} cells"
                  + (f" ({len(refused)} cells refused by the exact check: {where})" if refused else ""))
            return False, added_tracks, added_vias
        # split into runs per layer, straighten each run (every segment and
        # via verified against the exact geometry on the fine pass), emit
        runs, current = [], [path[0]]
        for node in path[1:]:
            if node[0] != current[-1][0]:
                runs.append(current)
                current = [node]
            else:
                current.append(node)
        runs.append(current)
        def exact_ok(layer, a, b):
            if exact is None or LAYERS[layer] not in (pcbnew.F_Cu, pcbnew.B_Cu):
                return True
            return exact.clear(LAYERS[layer], grid.to_mm(*a), grid.to_mm(*b), half, net)

        plan, refused = [], False
        for index, run in enumerate(runs):
            layer = run[0][0]
            cells = [(c[1], c[2]) for c in run]
            points = [cells[0]]
            k = 0
            while k < len(cells) - 1:
                far = len(cells) - 1
                while far > k + 1 and not (grid.clear_line(ok[layer], cells[k], cells[far])
                                           and exact_ok(layer, cells[k], cells[far])):
                    far -= 1
                if far == k + 1 and not exact_ok(layer, cells[k], cells[far]):
                    blocked.add((layer,) + cells[far])
                    refused = True
                    break
                points.append(cells[far])
                k = far
            if refused:
                break
            via = None
            if index < len(runs) - 1:
                j, i = run[-1][1], run[-1][2]
                via = (j, i)
                if exact is not None:
                    x, y = grid.to_mm(j, i)
                    if not all(exact.clear(l, (x, y), (x, y), VIA_DIAMETER / 2, net)
                               for l in (pcbnew.F_Cu, pcbnew.B_Cu)):
                        via_blocked.add((j, i))
                        refused = True
                        break
            plan.append((layer, points, via))
        if refused:
            continue                   # re-search this connection without the refused cell
        narrowed = 0                   # next connection starts at full width again
        for layer, points, via in plan:
            mm_points = [grid.to_mm(j, i) for j, i in points]
            for (x1, y1), (x2, y2) in zip(mm_points, mm_points[1:]):
                track = pcbnew.PCB_TRACK(board)
                track.SetStart(layout.vec(x1, y1))
                track.SetEnd(layout.vec(x2, y2))
                track.SetLayer(LAYERS[layer])
                track.SetWidth(layout.mm(width))
                track.SetNetCode(net)
                track.SetLocked(True)
                board.Add(track)
                added.append(track)
                grid.add_segment(net, layer, x1, y1, x2, y2, half)
                added_tracks += 1
            if via is not None:
                j, i = via
                x, y = grid.to_mm(j, i)
                item = pcbnew.PCB_VIA(board)
                item.SetPosition(layout.vec(x, y))
                item.SetWidth(layout.mm(VIA_DIAMETER))
                item.SetDrill(layout.mm(VIA_DRILL))
                item.SetNetCode(net)
                item.SetViaType(pcbnew.VIATYPE_THROUGH)
                item.SetLocked(True)
                board.Add(item)
                added.append(item)
                grid.add_circle(net, x, y, VIA_DIAMETER / 2)
                grid.via_cells[(j, i)] = net
                added_vias += 1
        blocked.clear()
        via_blocked.clear()
    return False, added_tracks, added_vias


#: Fan-out of fine-pitch packages (LQFP 0.5 mm, USB-C 0.5 mm): exact-geometry
#: escape stubs from each pad's toe, fanned away from the side's centre so the
#: stub ends sit ~0.9 mm apart -- room for the grid router. Adjacent stubs
#: diverge, so they are never closer than at the pads.
FANOUT_PITCH = 0.8                     # packages with a pad pitch below this
FANOUT_WIDTH = 0.15
FANOUT_SPREAD = 1.8                    # end pitch / pad pitch
FANOUT_LENGTHS = (3.0, 2.5, 2.0, 1.5, 1.0)


#: UUIDs of the fan-out escape stubs and vias: protected from rip-up like
#: the generator's pre-routes (a ripped escape is not recreated by a normal
#: re-route, which left its pad sealed in for good)
ESCAPES = set()

#: Nets whose fine-pitch pin escapes inward (via under the package) before
#: trying outward: BOOT0 sits between east-side relay pins and must cross
#: all their lanes to reach its strap resistors; it is static, a via is free.
INWARD_FIRST = {"/BOOT0", "/NRST"}

#: Lanes that run on B.Cu (the west-column relay pins, see LAYER_COST): their
#: escape ends in a via. Left to the grid router, the eight vias landed in one
#: column, drifted south until the last had no room, and the B.Cu runs wound
#: around the MCU's south-west corner over the I2C exit.
VIA_ESCAPE = {f"/GPIO{k}" for k in (14, 1, 2, 3, 4, 5, 6, 7)}


def escape_fine_pitch(board, grid, nets_by_name):
    import optimize_tracks as exact
    obstacles = exact.build_obstacles(board)
    pad_count = {}
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            pad_count[pad.GetNetname()] = pad_count.get(pad.GetNetname(), 0) + 1
    attached = set()
    for item in board.GetTracks():
        if item.GetClass() == "PCB_TRACK":
            for end in (item.GetStart(), item.GetEnd()):
                attached.add((item.GetNetCode(), end.x, end.y))
    added = skipped = vias = 0
    stranded = []
    for footprint in board.GetFootprints():
        pads = [p for p in footprint.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
        if len(pads) < 6:
            continue
        xy = [(pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y)) for p in pads]
        pitch = min(math.dist(a, b) for k, a in enumerate(xy) for b in xy[k + 1:])
        if pitch >= FANOUT_PITCH:
            continue
        cx = sum(x for x, _ in xy) / len(xy)
        cy = sum(y for _, y in xy) / len(xy)
        for pad, (px, py) in zip(pads, xy):
            name = pad.GetNetname()
            if name in ("", "/GND") or pad_count.get(name, 0) < 2:
                continue
            # already connected by a pre-route (USB entry, maze coarse pass)?
            if any(pad.HitTest(pcbnew.VECTOR2I(x, y)) for n, x, y in attached if n == pad.GetNetCode()):
                continue
            box = pad.GetBoundingBox()
            hw, hh = pcbnew.ToMM(box.GetWidth()) / 2, pcbnew.ToMM(box.GetHeight()) / 2
            if hw > hh:                                        # pad long axis = escape axis: left/right side
                n = (1.0 if px > cx else -1.0, 0.0)
                toe = (px + n[0] * hw, py)
                offset = (0.0, py - cy)
            else:                                              # top/bottom side
                n = (0.0, 1.0 if py > cy else -1.0)
                toe = (px, py + n[1] * hh)
                offset = (px - cx, 0.0)
            net = pad.GetNetCode()
            half = FANOUT_WIDTH / 2
            via_r = VIA_DIAMETER / 2
            heel = (px - n[0] * hw, py) if hw > hh else (px, py - n[1] * hh)   # inner pad end
            candidates = []
            # 0. B.Cu lanes: the fan stub ends in a via, alternate pins at
            # alternate lengths (two staggered via columns)
            if name in VIA_ESCAPE:
                odd = pad.GetNumber().isdigit() and int(pad.GetNumber()) % 2
                for length in ((3.0, 2.0, 2.5, 1.5) if odd else (2.0, 3.0, 1.5, 2.5)):
                    shift = (FANOUT_SPREAD - 1.0) * length / FANOUT_LENGTHS[0]
                    end_point = (toe[0] + n[0] * length + offset[0] * shift,
                                 toe[1] + n[1] * length + offset[1] * shift)
                    candidates.append(([(px, py), toe, end_point], end_point))
            # 1. outward fan, no via
            for length in FANOUT_LENGTHS:
                shift = (FANOUT_SPREAD - 1.0) * length / FANOUT_LENGTHS[0]
                end_point = (toe[0] + n[0] * length + offset[0] * shift,
                             toe[1] + n[1] * length + offset[1] * shift)
                candidates.append(([(px, py), toe, end_point], None))
            # 2. inward under the package body to a via (B.Cu under an LQFP is free)
            for length in (0.7, 1.2, 1.7, 2.2, 2.7):
                via = (heel[0] - n[0] * length, heel[1] - n[1] * length)
                candidates.append(([(px, py), via], via))
            # 3. short outward dog-bone
            for length in (0.6, 1.0, 1.4):
                via = (toe[0] + n[0] * length, toe[1] + n[1] * length)
                candidates.append(([(px, py), toe, via], via))
            if name in INWARD_FIRST:
                # static nets that must cross a lane fan: under the package
                # to a via first, so their B.Cu run passes under the lanes
                candidates.sort(key=lambda c: 0 if (c[1] is not None and len(c[0]) == 2) else 1)
            for points, via in candidates:
                segments = list(zip(points, points[1:]))
                if not all(obstacles.clear(pcbnew.F_Cu, a, b, half, net) for a, b in segments):
                    continue
                if via is not None and not all(obstacles.clear(layer, via, via, via_r, net)
                                               for layer in (pcbnew.F_Cu, pcbnew.B_Cu)):
                    continue
                if via is not None and grid.via_block[int(round((via[1] - grid.y0) / GRID)),
                                                      int(round((via[0] - grid.x0) / GRID))]:
                    continue
                for a, b in segments:
                    if math.dist(a, b) < 1e-3:
                        continue
                    track = pcbnew.PCB_TRACK(board)
                    track.SetStart(layout.vec(*a))
                    track.SetEnd(layout.vec(*b))
                    track.SetLayer(pcbnew.F_Cu)
                    track.SetWidth(layout.mm(FANOUT_WIDTH))
                    track.SetNetCode(net)
                    track.SetLocked(True)
                    board.Add(track)
                    ESCAPES.add(exact.uid(track))
                    grid.add_segment(net, 0, a[0], a[1], b[0], b[1], half)
                    obstacles.add(pcbnew.F_Cu, a, b, half, exact.CLEARANCE, net, exact.uid(track))
                if via is not None:
                    item = pcbnew.PCB_VIA(board)
                    item.SetPosition(layout.vec(*via))
                    item.SetWidth(layout.mm(VIA_DIAMETER))
                    item.SetDrill(layout.mm(VIA_DRILL))
                    item.SetNetCode(net)
                    item.SetViaType(pcbnew.VIATYPE_THROUGH)
                    item.SetLocked(True)
                    board.Add(item)
                    ESCAPES.add(exact.uid(item))
                    grid.add_circle(net, via[0], via[1], via_r)
                    grid.via_cells[(int(round((via[1] - grid.y0) / GRID)),
                                    int(round((via[0] - grid.x0) / GRID)))] = net
                    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
                        obstacles.add(layer, via, via, via_r, exact.CLEARANCE, net, exact.uid(item))
                        obstacles.add(layer, via, via, VIA_DRILL / 2, exact.HOLE_CLEARANCE, net, exact.uid(item))
                    vias += 1
                obstacles.freeze()
                added += 1
                break
            else:
                skipped += 1
                stranded.append(f"{footprint.GetReference()}.{pad.GetNumber()}({pad.GetNetname()})")
    print(f"fan-out: {added} escapes on fine-pitch packages ({vias} through a via), {skipped} without room"
          + (f": {', '.join(stranded)}" if stranded else ""))
    return obstacles


PATH_RIP_MARGIN = 0.6                  # mm beyond the two widths and the clearance


def priority(name):
    """Rip-up order: a net may only displace nets of equal or lower priority.
    Measurement nets (calibrated paths, equal-length rails) outrank supplies
    and coil returns, which outrank digital lines."""
    if name in MEASUREMENT:
        return 2
    if name.startswith(("/+", "/VBUS", "/USB_")) or name.endswith("_COIL"):
        return 1                       # supplies, coil returns, the USB pair
    return 0


#: cost multiplier for crossing another net's (rippable) copper while
#: planning the preferred path: passable but expensive (PathFinder-style
#: congestion cost), so the plan follows the emptiest corridor
CONGESTION_PENALTY = 12.0


def rip_up_along_path(board, name, width, nets_by_name, protected, grid=None):
    """Plan `name` with other nets' router-made copper passable but costly
    (the path it wants through the emptiest corridor), then remove exactly the
    router-made copper of other nets that sits on that path. Returns the names
    of the nets that lost copper."""
    import optimize_tracks as ex
    net = nets_by_name[name].GetNetCode()
    movable = [item for item in board.GetTracks()
               if item.GetNetCode() != net and item.GetNetname() not in ("/GND", "")
               and item.m_Uuid.AsString() not in protected
               and priority(item.GetNetname()) <= priority(name)]
    for item in movable:
        board.Remove(item)
    scratch = build_grid(board, nets_by_name)
    scratch.exact = ex.build_obstacles(board)
    if grid is not None:
        # cells that only the movable copper occupies: soft, not free
        soft = (grid.owner != FREE) & (scratch.owner == FREE)
        scratch.extra_cost = 1.0 + CONGESTION_PENALTY * soft
    trial = []
    done, _, _ = _route_net(board, scratch, name, width, nets_by_name, trial)
    ideal = []
    for item in trial:
        if item.GetClass() == "PCB_VIA":
            p = item.GetPosition()
            c = (pcbnew.ToMM(p.x), pcbnew.ToMM(p.y))
            ideal.append((None, c, c, VIA_DIAMETER / 2))
        else:
            ideal.append((item.GetLayer(), (pcbnew.ToMM(item.GetStart().x), pcbnew.ToMM(item.GetStart().y)),
                          (pcbnew.ToMM(item.GetEnd().x), pcbnew.ToMM(item.GetEnd().y)),
                          pcbnew.ToMM(item.GetWidth()) / 2))
        board.Remove(item)
    for item in movable:
        board.Add(item)
    if not done or not ideal:
        return []
    ripped, names = [], set()
    for item in movable:
        if item.GetClass() == "PCB_VIA":
            p = item.GetPosition()
            a = b = (pcbnew.ToMM(p.x), pcbnew.ToMM(p.y))
            half, layers = VIA_DIAMETER / 2, None
        else:
            a = (pcbnew.ToMM(item.GetStart().x), pcbnew.ToMM(item.GetStart().y))
            b = (pcbnew.ToMM(item.GetEnd().x), pcbnew.ToMM(item.GetEnd().y))
            half, layers = pcbnew.ToMM(item.GetWidth()) / 2, item.GetLayer()
        for layer, c, d, own_half in ideal:
            if layer is not None and layers is not None and layer != layers:
                continue
            gap = ex.seg_seg_distance(a, b, numpy.array([c]), numpy.array([d]))[0]
            if gap < half + own_half + CLEARANCE + PATH_RIP_MARGIN:
                ripped.append(item)
                names.add(item.GetNetname())
                break
    for item in ripped:
        board.Remove(item)
    print(f"  <> {name}: ripped {len(ripped)} items of {sorted(names)} along its preferred path")
    return sorted(names)


def route_fine(board, grid, nets_by_name):
    """Fine pass with rip-up and reroute: a net that cannot reach a pad
    removes the router-made copper of other nets around that pad, routes
    first, and the ripped nets are re-queued (they reconnect as islands)."""
    import json
    import optimize_tracks as ex
    protected = set()
    prerouted = os.path.join(HERE, "relay_board_prerouted.json")
    if os.path.exists(prerouted):
        protected = set(json.load(open(prerouted)))
    protected |= ESCAPES
    widths = dict(fine_nets(board, nets_by_name))
    queue = [name for name, _ in fine_nets(board, nets_by_name)]
    rips = {}
    failed = []
    while queue:
        name = queue.pop(0)
        width = widths.get(name, dict(NETS).get(name, FINE_WIDTHS.get(name, 0.2)))
        done, tracks, vias = route_net(board, grid, name, width, nets_by_name)
        print(f"  {name:12s} {'ok ' if done else 'FAIL'} {tracks:3d} segments {vias:2d} vias")
        if done:
            if name in failed:
                failed.remove(name)
            continue
        if rips.get(name, 0) >= MAX_RIPS_PER_NET or not protected:
            if name not in failed:
                failed.append(name)
            continue
        rips[name] = rips.get(name, 0) + 1
        victims = []
        if rips[name] == 1:
            # first: clear the path the net wants (blockers mid-route)
            victims = rip_up_along_path(board, name, width, nets_by_name, protected, grid)
        while not victims and rips[name] <= MAX_RIPS_PER_NET:
            # escalate: the pad's surroundings first, then a wider corridor
            victims = rip_up_around(board, grid, name, nets_by_name, protected,
                                    radius=RIP_RADIUS * rips[name])
            if not victims:
                rips[name] += 1
        if not victims:
            if name not in failed:
                failed.append(name)
            continue
        # rebuild the raster and the exact oracle from the board as it now is
        grid = build_grid(board, nets_by_name)
        grid.exact = ex.build_obstacles(board)
        victims.sort(key=lambda v: LANE_ORDER.index(v) if v in LANE_ORDER else -1)
        # victims go right behind the net that displaced them, whether or not they
        # were still waiting (a waiting victim used to be dropped from the queue
        # altogether: never routed, and the pass still reported 0 unrouted)
        queue = [name] + victims + [q for q in queue if q not in victims and q != name]
    # last word for the nets that ran out of rip-ups: with every other net
    # settled, the corridor they needed is often free again
    for name in list(failed) + [n for n in sys.argv if n.startswith("--retry=")]:
        name = name.split("=", 1)[-1]
        if name not in nets_by_name:
            continue
        grid = build_grid(board, nets_by_name)
        grid.exact = ex.build_obstacles(board)
        width = widths.get(name, dict(NETS).get(name, FINE_WIDTHS.get(name, 0.2)))
        done, tracks, vias = route_net(board, grid, name, width, nets_by_name)
        print(f"  {name:12s} {'ok ' if done else 'FAIL'} {tracks:3d} segments {vias:2d} vias  (final retry)")
        if done and name in failed:
            failed.remove(name)
    board.Save(BOARD)
    print(f"fine pass: {len(failed)} nets unrouted" + (f": {failed}" if failed else ""))
    return 1 if failed else 0


#: Relay-drive lanes in river order (each lane hugs the one routed before it,
#: so none seals a later pad -- shortest-span order let them rip each other
#: forever). Three bundles, see generate_schematic.GPIO_PINS: the north-row
#: pins west of the VDDA parts reach U6/U4 from the west under U6 (inner
#: lane = eastmost pin first); the east channel, nearest input first; the
#: west column on B.Cu from staggered escape vias (VIA_ESCAPE), northmost
#: pin first: parallel diagonals under the MCU.
LANE_ORDER = [f"/GPIO{k}" for k in (22, 23, 15, 16, 17, 18, 19, 20, 21, 8, 9, 10, 11, 12, 13,
                                     14, 1, 2, 3, 4, 5, 6, 7)]
#: Nets routed just ahead of the lane block. NRST runs from the north row to
#: the SWD header and the reset button, across the whole east lane fan: it is
#: static, so it dives under the package (INWARD_FIRST), runs on B.Cu
#: (LAYER_COST) and is in place before the lanes. (I2C ahead of the lanes
#: made things worse; with the west lanes on staggered vias it fits after.)
BEFORE_LANES = ("/NRST",)

#: Fine-pass widths (mm); anything not listed is 0.2 (JLC minimum 0.15).
FINE_WIDTHS = {"/+3V3": 0.3, "/+3V3A": 0.3, "/VBUS": 0.4, "/USB_SHIELD": 0.3}


def fine_nets(board, nets_by_name):
    """Every net except GND, shortest pad span first (short nets claim the
    space next to their pins before long ones sweep across)."""
    widths = dict(NETS)
    spans = {}
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            name = pad.GetNetname()
            if name in ("", "/GND") or name not in nets_by_name:
                continue
            p = pad.GetPosition()
            spans.setdefault(name, []).append((pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)))
    order = []
    for name, points in spans.items():
        if len(points) < 2:
            continue
        xs, ys = zip(*points)
        order.append((max(xs) - min(xs) + max(ys) - min(ys), name))
    names = [name for _, name in sorted(order)]
    # the relay-drive lanes go as one block, in river order, where the first
    # of them stood: the short nets around the MCU pins are already in place
    lanes = [name for name in LANE_ORDER if name in names]
    if lanes:
        first = min(names.index(name) for name in lanes)
        ahead = [name for name in names[first:] if name in BEFORE_LANES]
        names = [name for name in names[:first] if name not in lanes] + ahead + lanes             + [name for name in names[first:] if name not in lanes and name not in ahead]
    return [(name, widths.get(name, FINE_WIDTHS.get(name, 0.2))) for name in names]


RIP_RADIUS = 2.0                       # mm around an unreachable pad, first attempt;
MAX_RIPS_PER_NET = 4                   # each further attempt widens it by RIP_RADIUS


def build_grid(board, nets_by_name):
    grid = Grid(board)
    for item in board.GetTracks():
        if item.GetClass() == "PCB_VIA":
            p = item.GetPosition()
            j = int(round((pcbnew.ToMM(p.y) - grid.y0) / GRID))
            i = int(round((pcbnew.ToMM(p.x) - grid.x0) / GRID))
            grid.via_cells[(j, i)] = item.GetNetCode()
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pad.GetNetCode() > 0:
                p = pad.GetPosition()
                j = int(round((pcbnew.ToMM(p.y) - grid.y0) / GRID))
                i = int(round((pcbnew.ToMM(p.x) - grid.x0) / GRID))
                grid.via_cells[(j, i)] = pad.GetNetCode()
    grid.set_classes(nets_by_name)
    return grid


def rip_up_around(board, grid, name, nets_by_name, protected, radius=RIP_RADIUS):
    """Remove other nets' router-made copper (never the generator's
    pre-routes, never GND) within RIP_RADIUS of the failed net's stranded
    islands. Returns the names of the nets that lost copper."""
    import optimize_tracks as ex
    net = nets_by_name[name].GetNetCode()
    islands = sorted(grid.components(net), key=len, reverse=True)[1:]
    centres = [grid.to_mm(*numpy.mean([(c[1], c[2]) for c in isl], axis=0)) for isl in islands]
    if not centres:
        return []
    ripped, names = [], set()
    for item in list(board.GetTracks()):
        other = item.GetNetname()
        if (item.GetNetCode() == net or other in ("/GND", "") or item.m_Uuid.AsString() in protected
                or priority(other) > priority(name)):
            continue
        if item.GetClass() == "PCB_VIA":
            p = item.GetPosition()
            a = b = (pcbnew.ToMM(p.x), pcbnew.ToMM(p.y))
        else:
            a = (pcbnew.ToMM(item.GetStart().x), pcbnew.ToMM(item.GetStart().y))
            b = (pcbnew.ToMM(item.GetEnd().x), pcbnew.ToMM(item.GetEnd().y))
        near = ex.point_seg_distance(numpy.array(centres), numpy.array([a] * len(centres)),
                                     numpy.array([b] * len(centres)))
        if near.min() <= radius:
            ripped.append(item)
            names.add(other)
    for item in ripped:
        board.Remove(item)
    print(f"  <> {name}: ripped {len(ripped)} items of {sorted(names)} within {radius:.0f} mm")
    return sorted(names)


#: polish: a net is re-routed when its copper is this much longer than the
#: half-perimeter of its pads (rip-up and reroute joins islands in whatever
#: order they come up: one relay line ran 150 mm around the MCU for a 70 mm hop)
POLISH_DETOUR = 1.6
POLISH_MIN_LENGTH = 25.0               # mm


def polish(board, nets_by_name):
    """After the fine pass: take each net with a long detour off the board
    (its escape stubs and the generator's pre-routes stay), route it again on
    its own with every other net in place, and keep the result only if it is
    shorter. Run as `maze_route.py --fine --polish`."""
    import json
    import optimize_tracks as ex
    protected = set()
    prerouted = os.path.join(HERE, "relay_board_prerouted.json")
    if os.path.exists(prerouted):
        protected = set(json.load(open(prerouted)))
    widths = dict(fine_nets(board, nets_by_name))
    spans = {}
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            p = pad.GetPosition()
            spans.setdefault(pad.GetNetname(), []).append((pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)))

    def cost(items):
        return sum(pcbnew.ToMM(item.GetLength()) if item.GetClass() == "PCB_TRACK" else VIA_COST for item in items)

    def movable(name):
        net = nets_by_name[name].GetNetCode()
        tracks = [item for item in board.GetTracks() if item.GetNetCode() == net
                  and item.m_Uuid.AsString() not in protected]
        stubs = [item for item in tracks if item.GetClass() == "PCB_TRACK"
                 and abs(pcbnew.ToMM(item.GetWidth()) - FANOUT_WIDTH) < 1e-3]
        ends = [(pcbnew.ToMM(point.x), pcbnew.ToMM(point.y)) for item in stubs
                for point in (item.GetStart(), item.GetEnd())]
        keep = {id(item) for item in stubs}
        for item in tracks:                              # the via an escape stub ends in
            if item.GetClass() == "PCB_VIA":
                p = item.GetPosition()
                if any(math.dist((pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)), end) < 0.01 for end in ends):
                    keep.add(id(item))
        return [item for item in tracks if id(item) not in keep]

    candidates = []
    for name, points in spans.items():
        if name in ("", "/GND") or name not in nets_by_name or name in MEASUREMENT or len(points) < 2:
            continue
        xs, ys = zip(*points)
        direct = (max(xs) - min(xs)) + (max(ys) - min(ys))
        length = sum(pcbnew.ToMM(item.GetLength()) for item in movable(name) if item.GetClass() == "PCB_TRACK")
        if length > POLISH_MIN_LENGTH and length > POLISH_DETOUR * max(direct, 1.0):
            candidates.append((length / max(direct, 1.0), name, length))
    only = [a.split("=", 1)[1] for a in sys.argv if a.startswith("--polish-net=")]
    if not only:
        # one process per net: pcbnew's SWIG wrappers lose their types after a
        # few remove / add-back cycles on one board object
        import subprocess
        improved = 0
        for ratio, name, length in sorted(candidates, reverse=True):
            result = subprocess.run([sys.executable, os.path.abspath(__file__), "--fine", "--polish",
                                     f"--polish-net={name}"], capture_output=True, text=True)
            line = [l for l in result.stdout.splitlines() if l.startswith(("  ~", "  ="))]
            print(line[-1] if line else f"  ? {name}: {result.stderr.strip().splitlines()[-1:]}", flush=True)
            improved += bool(line and line[-1].startswith("  ~"))
        print(f"polish: {improved} of {len(candidates)} detouring nets re-routed shorter")
        return 0
    name = only[0]
    ratio = dict((n, r) for r, n, _ in candidates).get(name, 0.0)
    old = movable(name)
    before = cost(old)
    for item in old:
        board.Remove(item)
    grid = build_grid(board, nets_by_name)
    grid.exact = ex.build_obstacles(board)
    added = []
    width = widths.get(name, dict(NETS).get(name, FINE_WIDTHS.get(name, 0.2)))
    done, _, _ = _route_net(board, grid, name, width, nets_by_name, added)
    after = cost(added)
    # --polish-force: keep any complete route (used to re-route a net whose
    # via could not be moved out of a pad, see vias_out_of_pads.py)
    if done and (after < before - 1.0 or "--polish-force" in sys.argv):
        board.Save(BOARD)
        print(f"  ~ {name:12s} {before:6.1f} -> {after:6.1f} mm-equivalent (was {ratio:.1f}x its pad span)")
    else:                                 # nothing saved: the board on disk is untouched
        print(f"  = {name:12s} kept ({'no shorter route' if done else 'no route alone'}; {ratio:.1f}x its pad span)")
    return 0


def main():
    board = pcbnew.LoadBoard(BOARD)
    nets_by_name = {n.GetNetname(): n for n in board.GetNetsByNetcode().values()}
    grid = build_grid(board, nets_by_name)
    nets_by_name = {n.GetNetname(): n for n in board.GetNetsByNetcode().values()}
    grid.set_classes(nets_by_name)
    failed = []
    if FINE and "--polish" in sys.argv:
        return polish(board, nets_by_name)
    if FINE:
        grid.exact = escape_fine_pitch(board, grid, nets_by_name)
        return route_fine(board, grid, nets_by_name)
    for name, width in NETS:
        if name not in nets_by_name:
            print(f"  (skip {name}: not on the board)")
            continue
        done, tracks, vias = route_net(board, grid, name, width, nets_by_name)
        print(f"  {name:12s} {'ok ' if done else 'FAIL'} {tracks:3d} segments {vias:2d} vias")
        if not done:
            failed.append(name)
    board.Save(BOARD)
    if FINE:
        print(f"fine pass: {len(failed)} nets unrouted" + (f": {failed}" if failed else ""))
        return 1 if failed else 0
    destination = BOARD.replace(".kicad_pcb", ".dsn")
    if pcbnew.ExportSpecctraDSN(board, destination):
        layout.patch_dsn_power_layers(destination)
    print(f"maze-routed {len(NETS) - len(failed)}/{len(NETS)} nets"
          + (f"; left to freerouting: {failed}" if failed else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
