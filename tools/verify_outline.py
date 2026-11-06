#!/usr/bin/env python3
"""Rev 7 board-outline DRC: zero-overhang Edge.Cuts + M2.5 mounting-hole audit.

Checks every manufacturing representation of the board envelope against the
single source of truth in tools/board_geometry.py:

  * radxa-cm4-nano-carrier.kicad_pcb  (Edge.Cuts, holes, keep-outs, overhang)
  * gerbers/*-Edge_Cuts.gm1            (profile as sent to the fab)
  * gerbers/*.drl                      (NPTH drill hits as sent to the fab)
  * board_outline.dxf                  (enclosure / mechanical hand-off)
  * kicad-cli DRC (+ mutation tests proving the .kicad_dru rules fire)

Usage:  python3 tools/verify_outline.py [--no-kicad] [--report artifacts/rev7_outline_drc.md]
Exit status is non-zero if any check fails.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import board_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
PCB = ROOT / "radxa-cm4-nano-carrier.kicad_pcb"
GERBER_EDGE = ROOT / "gerbers" / "radxa-cm4-nano-carrier-Edge_Cuts.gm1"
DRILL_FILES = sorted((ROOT / "gerbers").glob("*.drl"))
DXF = ROOT / "board_outline.dxf"
MOUNT_LIB_ID = "CM4Carrier:MountingHole_M2.5_NPTH_KO6"


# --------------------------------------------------------------------------
# Minimal S-expression reader (sufficient for KiCad 8 files)
# --------------------------------------------------------------------------
_TOKEN = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))')


def parse_sexpr(text: str):
    stack: list[list] = [[]]
    pos = 0
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        if not m:
            if text[pos:].strip() == "":
                break
            raise ValueError(f"S-expression parse error at offset {pos}")
        pos = m.end()
        if m.group(1):
            stack.append([])
        elif m.group(2):
            done = stack.pop()
            stack[-1].append(done)
        elif m.group(3) is not None:
            stack[-1].append(m.group(3))
        elif m.group(4) is not None:
            stack[-1].append(m.group(4))
    if len(stack) != 1:
        raise ValueError("unbalanced parentheses")
    return stack[0][0]


def children(node, name):
    return [c for c in node if isinstance(c, list) and c and c[0] == name]


def child(node, name):
    c = children(node, name)
    return c[0] if c else None


def xy(node):
    return float(node[1]), float(node[2])


# --------------------------------------------------------------------------
# Result bookkeeping
# --------------------------------------------------------------------------
@dataclass
class Check:
    cid: str
    desc: str
    ok: bool
    detail: str = ""


@dataclass
class Report:
    checks: list[Check] = field(default_factory=list)

    def add(self, cid, desc, ok, detail=""):
        self.checks.append(Check(cid, desc, bool(ok), detail))

    @property
    def ok(self):
        return all(c.ok for c in self.checks)


def close(a, b, tol=G.POS_TOL):
    return abs(a - b) <= tol


# --------------------------------------------------------------------------
# Outline helpers
# --------------------------------------------------------------------------
def loop_closed(segments, tol=G.POS_TOL):
    """True if segments form exactly one closed loop (every vertex degree 2,
    single connected component)."""
    if not segments:
        return False, "no Edge.Cuts segments"
    verts: list[tuple[float, float]] = []

    def vid(p):
        for i, v in enumerate(verts):
            if math.dist(v, p) <= tol:
                return i
        verts.append(p)
        return len(verts) - 1

    adj: dict[int, list[int]] = {}
    for a, b in segments:
        ia, ib = vid(a), vid(b)
        adj.setdefault(ia, []).append(ib)
        adj.setdefault(ib, []).append(ia)
    bad = [verts[i] for i, n in adj.items() if len(n) != 2]
    if bad:
        return False, f"open/branching vertices at {bad}"
    seen, todo = set(), [0]
    while todo:
        v = todo.pop()
        if v in seen:
            continue
        seen.add(v)
        todo.extend(adj[v])
    if len(seen) != len(verts):
        return False, f"{len(verts) - len(seen)} vertices in a second loop"
    return True, f"{len(segments)} segments, {len(verts)} vertices, single loop"


def polygon_area(segments):
    return 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in segments))


def bbox(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def check_envelope(rep, tag, segments, y_sign=1):
    pts = [p for s in segments for p in s]
    ok, why = loop_closed(segments)
    rep.add(f"{tag}-closed", f"{tag}: Edge.Cuts forms one closed loop", ok, why)
    x0, y0, x1, y1 = bbox(pts)
    if y_sign < 0:
        y0, y1 = -y1, -y0
    want = (0.0, 0.0, G.BOARD_W, G.BOARD_H)
    got = (x0, y0, x1, y1)
    rep.add(f"{tag}-bbox", f"{tag}: extents exactly (0,0)-({G.BOARD_W},{G.BOARD_H}) mm",
            all(close(a, b) for a, b in zip(got, want)),
            "got (%.4f,%.4f)-(%.4f,%.4f)" % got)
    area = polygon_area(segments)
    rep.add(f"{tag}-area", f"{tag}: enclosed area = {G.BOARD_W * G.BOARD_H:.1f} mm^2 (rectangular, no notches)",
            close(area, G.BOARD_W * G.BOARD_H, 1e-3), f"{area:.4f} mm^2")
    axis = all(close(a[0], b[0]) or close(a[1], b[1]) for a, b in segments)
    rep.add(f"{tag}-ortho", f"{tag}: all edges axis-aligned", axis)


# --------------------------------------------------------------------------
# PCB checks
# --------------------------------------------------------------------------
def rotate(px, py, deg):
    r = math.radians(-deg)  # KiCad: positive angle = CCW on screen (Y down)
    return px * math.cos(r) - py * math.sin(r), px * math.sin(r) + py * math.cos(r)


def footprint_extents(fp):
    """Yield (layer, x0, y0, x1, y1) for every geometric item in a footprint."""
    at = child(fp, "at")
    ox, oy = float(at[1]), float(at[2])
    rot = float(at[3]) if len(at) > 3 else 0.0

    def tf(x, y):
        rx, ry = rotate(x, y, rot)
        return ox + rx, oy + ry

    for item in fp:
        if not isinstance(item, list) or not item:
            continue
        kind = item[0]
        layer = child(item, "layer")
        layer = layer[1] if layer else None
        stroke = child(item, "stroke")
        w = float(child(stroke, "width")[1]) / 2 if stroke and child(stroke, "width") else 0.0
        if kind == "fp_circle":
            cx, cy = tf(*xy(child(item, "center")))
            ex, ey = xy(child(item, "end"))
            cx0, cy0 = xy(child(item, "center"))
            r = math.dist((cx0, cy0), (ex, ey)) + w
            yield layer, cx - r, cy - r, cx + r, cy + r
        elif kind in ("fp_line", "fp_rect"):
            a = tf(*xy(child(item, "start")))
            b = tf(*xy(child(item, "end")))
            yield layer, min(a[0], b[0]) - w, min(a[1], b[1]) - w, max(a[0], b[0]) + w, max(a[1], b[1]) + w
        elif kind == "fp_poly":
            pts = [tf(*xy(p)) for p in child(item, "pts")[1:]]
            x0, y0, x1, y1 = bbox(pts)
            yield layer, x0 - w, y0 - w, x1 + w, y1 + w
        elif kind == "pad":
            px, py = tf(*xy(child(item, "at")))
            sx, sy = (float(v) for v in child(item, "size")[1:3])
            r = math.hypot(sx, sy) / 2 if child(item, "at")[3:] or rot else max(sx, sy) / 2
            yield "pad", px - r, py - r, px + r, py + r


def board_item_extents(pcb):
    for kind in ("gr_line", "gr_rect", "gr_circle", "gr_poly", "segment", "via", "zone"):
        for it in children(pcb, kind):
            layer = child(it, "layer")
            layer = layer[1] if layer else kind
            if layer == "Edge.Cuts":
                continue
            pts = []
            if kind == "via":
                x, y = xy(child(it, "at"))
                r = float(child(it, "size")[1]) / 2
                pts = [(x - r, y - r), (x + r, y + r)]
            elif kind == "gr_circle":
                c = xy(child(it, "center"))
                r = math.dist(c, xy(child(it, "end")))
                pts = [(c[0] - r, c[1] - r), (c[0] + r, c[1] + r)]
            elif kind in ("zone", "gr_poly"):
                poly = child(it, "polygon") or it
                pts = [xy(p) for p in child(poly, "pts")[1:]]
            else:
                pts = [xy(child(it, "start")), xy(child(it, "end"))]
            if pts:
                yield f"{kind}:{layer}", *bbox(pts)


def check_pcb(rep):
    pcb = parse_sexpr(PCB.read_text())
    rep.add("PCB-parse", "kicad_pcb parses as a single S-expression", pcb[0] == "kicad_pcb")

    # --- Edge.Cuts -------------------------------------------------------
    segs, other_edge = [], []
    for kind in ("gr_line", "gr_arc", "gr_circle", "gr_rect", "gr_poly"):
        for it in children(pcb, kind):
            if child(it, "layer") and child(it, "layer")[1] == "Edge.Cuts":
                if kind == "gr_line":
                    segs.append((xy(child(it, "start")), xy(child(it, "end"))))
                else:
                    other_edge.append(kind)
    for fp in children(pcb, "footprint"):
        for it in fp:
            if isinstance(it, list) and child(it, "layer") and child(it, "layer")[1] == "Edge.Cuts":
                other_edge.append(f"{fp[1]}:{it[0]}")
    rep.add("PCB-edge-only-lines", "Edge.Cuts contains only the outer rectangle (no cut-outs, arcs or footprint edges)",
            not other_edge, ", ".join(other_edge) or "4 gr_line only")
    check_envelope(rep, "PCB", segs)

    # --- Mounting holes --------------------------------------------------
    holes = {}
    for fp in children(pcb, "footprint"):
        if fp[1] != MOUNT_LIB_ID:
            continue
        ref = next(p[2] for p in children(fp, "property") if p[1] == "Reference")
        at = child(fp, "at")
        pad = child(fp, "pad")
        holes[ref] = dict(x=float(at[1]), y=float(at[2]), type=pad[2],
                          drill=float(child(pad, "drill")[1]),
                          pad_size=float(child(pad, "size")[1]))
    rep.add("HOLE-count", "exactly 4 M2.5 mounting holes (H1..H4)",
            sorted(holes) == sorted(G.HOLES), f"found {sorted(holes)}")
    for ref, (ex, ey) in G.HOLES.items():
        h = holes.get(ref)
        if not h:
            rep.add(f"HOLE-{ref}-pos", f"{ref} present", False)
            continue
        rep.add(f"HOLE-{ref}-pos", f"{ref} centre at ({ex:.2f}, {ey:.2f}) = {G.HOLE_INSET} mm inset",
                close(h["x"], ex) and close(h["y"], ey), f"({h['x']:.4f}, {h['y']:.4f})")
        rep.add(f"HOLE-{ref}-npth", f"{ref} is NPTH, drill {G.HOLE_DRILL} mm, no annular ring",
                h["type"] == "np_thru_hole" and close(h["drill"], G.HOLE_DRILL) and h["pad_size"] <= h["drill"] + G.POS_TOL,
                f"{h['type']} drill={h['drill']} pad={h['pad_size']}")
        edge = min(h["x"], h["y"], G.BOARD_W - h["x"], G.BOARD_H - h["y"])
        web = edge - h["drill"] / 2
        rep.add(f"HOLE-{ref}-web", f"{ref} drill wall to edge >= {G.MIN_HOLE_WEB} mm (1x board thickness)",
                web >= G.MIN_HOLE_WEB - G.POS_TOL, f"{web:.3f} mm")
        ko = edge - G.HOLE_KEEPOUT_D / 2
        rep.add(f"HOLE-{ref}-keepout", f"{ref} Ø{G.HOLE_KEEPOUT_D} keep-out inside outline with >= {G.MIN_KEEPOUT_EDGE} mm margin",
                ko >= G.MIN_KEEPOUT_EDGE - G.POS_TOL, f"{ko:.3f} mm")
        so = edge - G.standoff_corner_radius()
        rep.add(f"HOLE-{ref}-standoff", f"{ref} {G.STANDOFF_AF} mm A/F hex standoff (any rotation) does not overhang",
                so >= -G.POS_TOL, f"{so:.3f} mm margin (corner R {G.standoff_corner_radius():.3f})")
    if len(holes) == 4:
        px = holes["H2"]["x"] - holes["H1"]["x"]
        py = holes["H3"]["y"] - holes["H1"]["y"]
        sym = (close(holes["H4"]["x"] - holes["H3"]["x"], px) and close(holes["H4"]["y"] - holes["H2"]["y"], py))
        rep.add("HOLE-pitch", f"hole pitch {G.HOLE_PITCH_X} x {G.HOLE_PITCH_Y} mm, rectangular pattern",
                close(px, G.HOLE_PITCH_X) and close(py, G.HOLE_PITCH_Y) and sym, f"{px:.4f} x {py:.4f}")

    # --- Copper keep-out rule areas around every hole ---------------------
    kos = [z for z in children(pcb, "zone") if child(z, "keepout") is not None]
    for ref, (hx, hy) in G.HOLES.items():
        hit = None
        for z in kos:
            pts = [xy(p) for p in child(child(z, "polygon"), "pts")[1:]]
            rmin = min(math.dist(p, (hx, hy)) for p in pts)
            ko = child(z, "keepout")
            rules = {r[0]: r[1] for r in ko[1:] if isinstance(r, list)}
            layers = child(z, "layers")[1:]
            if rmin >= G.HOLE_KEEPOUT_D / 2 * math.cos(math.pi / len(pts)) - G.POS_TOL and \
               bbox(pts)[0] <= hx <= bbox(pts)[2] and bbox(pts)[1] <= hy <= bbox(pts)[3]:
                hit = (rules, layers, rmin)
        ok = bool(hit) and all(hit[0].get(k) == "not_allowed" for k in ("tracks", "vias", "copperpour")) and \
            ("F&B.Cu" in hit[1] or "*.Cu" in hit[1]) and ("*.Cu" in hit[1] or {"In1.Cu", "In2.Cu"} <= set(hit[1]))
        rep.add(f"KO-{ref}", f"{ref} has a Ø{G.HOLE_KEEPOUT_D} copper keep-out (tracks/vias/pour) on all 4 layers",
                ok, f"layers={hit[1]} rules={hit[0]}" if hit else "missing")

    # --- Zero overhang: nothing outside the outline ---------------------
    offenders = []
    for fp in children(pcb, "footprint"):
        ref = next((p[2] for p in children(fp, "property") if p[1] == "Reference"), fp[1])
        for layer, x0, y0, x1, y1 in footprint_extents(fp):
            if x0 < -G.POS_TOL or y0 < -G.POS_TOL or x1 > G.BOARD_W + G.POS_TOL or y1 > G.BOARD_H + G.POS_TOL:
                offenders.append(f"{ref}/{layer} ({x0:.3f},{y0:.3f})-({x1:.3f},{y1:.3f})")
    for name, x0, y0, x1, y1 in board_item_extents(pcb):
        if x0 < -G.POS_TOL or y0 < -G.POS_TOL or x1 > G.BOARD_W + G.POS_TOL or y1 > G.BOARD_H + G.POS_TOL:
            offenders.append(f"{name} ({x0:.3f},{y0:.3f})-({x1:.3f},{y1:.3f})")
    rep.add("OVERHANG-zero", "zero overhang: every footprint/courtyard/pad/zone/track lies within 55 x 40 mm",
            not offenders, "; ".join(offenders) or f"{len(children(pcb, 'footprint'))} footprints, "
            f"{len(children(pcb, 'zone'))} zones checked")

    # --- Title block revision ------------------------------------------
    tb = child(pcb, "title_block")
    rev = child(tb, "rev")[1] if tb and child(tb, "rev") else None
    rep.add("PCB-rev", "title block revision is 7.0", rev == "7.0", str(rev))


# --------------------------------------------------------------------------
# Gerber / Excellon / DXF checks
# --------------------------------------------------------------------------
def check_gerber(rep):
    if not GERBER_EDGE.exists():
        rep.add("GBR-exists", "Edge_Cuts Gerber exists", False, str(GERBER_EDGE))
        return
    txt = GERBER_EDGE.read_text()
    fmt = re.search(r"%FSLAX(\d)(\d)Y(\d)(\d)\*%", txt)
    scale = 10 ** int(fmt.group(2)) if fmt else 1e6
    rep.add("GBR-units", "Edge_Cuts Gerber is metric with Profile file function",
            "%MOMM*%" in txt and "FileFunction,Profile" in txt)
    segs, cur = [], (0.0, 0.0)
    for m in re.finditer(r"(?:X(-?\d+))?(?:Y(-?\d+))?D0([12])\*", txt):
        x = int(m.group(1)) / scale if m.group(1) else cur[0]
        y = int(m.group(2)) / scale if m.group(2) else cur[1]
        if m.group(3) == "1":
            segs.append((cur, (x, y)))
        cur = (x, y)
    arcs = re.search(r"G0[23]", txt)
    rep.add("GBR-no-arcs", "Edge_Cuts Gerber has no arcs (square-corner CM4 envelope)", not arcs)
    check_envelope(rep, "GBR", segs, y_sign=-1)


def check_drill(rep):
    hits = []
    for f in DRILL_FILES:
        tools, cur = {}, None
        for line in f.read_text().splitlines():
            m = re.match(r"T(\d+)C([\d.]+)", line)
            if m:
                tools[m.group(1)] = float(m.group(2))
                continue
            m = re.match(r"T(\d+)$", line)
            if m:
                cur = m.group(1)
                continue
            m = re.match(r"X(-?[\d.]+)Y(-?[\d.]+)", line)
            if m and cur:
                hits.append((tools[cur], float(m.group(1)), -float(m.group(2)), f.name))
    rep.add("DRL-count", "Excellon drill data contains exactly 4 holes",
            len(hits) == 4, f"{len(hits)} hits in {[f.name for f in DRILL_FILES]}")
    for ref, (ex, ey) in G.HOLES.items():
        m = [h for h in hits if close(h[1], ex, 0.005) and close(h[2], ey, 0.005)]
        rep.add(f"DRL-{ref}", f"Excellon hit for {ref} at ({ex}, {ey}) Ø{G.HOLE_DRILL}",
                len(m) == 1 and close(m[0][0], G.HOLE_DRILL, 0.005),
                str(m[0][:3]) if m else "missing")
    npth = [f.name for f in DRILL_FILES if "NPTH" in f.name]
    rep.add("DRL-npth-file", "mounting holes delivered in a dedicated NPTH drill file",
            bool(npth) and all("NPTH" in h[3] for h in hits), ", ".join(npth) or "no NPTH file")


def check_dxf(rep):
    try:
        import ezdxf
    except ImportError:
        rep.add("DXF-ezdxf", "ezdxf available for DXF audit", False, "pip install ezdxf")
        return
    doc = ezdxf.readfile(DXF)
    msp = doc.modelspace()
    segs = []
    edge_circles = []
    for e in msp:
        if e.dxf.layer != "EDGE_CUTS":
            continue
        if e.dxftype() == "LWPOLYLINE":
            pts = [(float(p[0]), float(p[1])) for p in e.get_points("xy")]
            if e.closed and pts[0] != pts[-1]:
                pts.append(pts[0])
            segs += list(zip(pts, pts[1:]))
        elif e.dxftype() == "LINE":
            segs.append(((e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)))
        else:
            edge_circles.append(e.dxftype())
    rep.add("DXF-edge-only-outline", "DXF EDGE_CUTS layer holds only the outer profile (no keep-out circles routed as cut-outs)",
            not edge_circles, ", ".join(edge_circles) or "outline only")
    check_envelope(rep, "DXF", segs)  # DXF is drawn Y-up from the same (0,0) corner
    holes = [e for e in msp.query("CIRCLE") if e.dxf.layer == "HOLES_NPTH"]
    for ref, (ex, ey) in G.HOLES.items():
        dy = G.BOARD_H - ey  # Y-up
        m = [h for h in holes if close(h.dxf.center.x, ex) and close(h.dxf.center.y, dy)]
        rep.add(f"DXF-{ref}", f"DXF HOLES_NPTH circle for {ref} (Ø{G.HOLE_DRILL})",
                len(m) == 1 and close(m[0].dxf.radius * 2, G.HOLE_DRILL), "ok" if m else "missing")
    rep.add("DXF-hole-count", "DXF has exactly 4 NPTH hole circles", len(holes) == 4, str(len(holes)))
    bad = []
    for e in msp:
        try:
            box = ezdxf.bbox.extents([e])
        except Exception:
            continue
        if not box.has_data:
            continue
        if e.dxf.layer in ("DIMENSIONS", "TEXT"):
            continue
        if box.extmin.x < -G.POS_TOL or box.extmin.y < -G.POS_TOL or \
           box.extmax.x > G.BOARD_W + G.POS_TOL or box.extmax.y > G.BOARD_H + G.POS_TOL:
            bad.append(f"{e.dxftype()}@{e.dxf.layer}")
    rep.add("DXF-overhang", "no DXF geometry (keep-outs, connectors, components) outside the outline",
            not bad, ", ".join(bad) or "ok")


# --------------------------------------------------------------------------
# KiCad DRC + mutation tests
# --------------------------------------------------------------------------
def run_drc(pcb_path: Path):
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all",
                        "--schematic-parity", "-o", str(out), str(pcb_path)],
                       check=True, capture_output=True, text=True)
        d = json.loads(out.read_text())
    return d


def check_kicad(rep):
    if not shutil.which("kicad-cli"):
        rep.add("KICAD-cli", "kicad-cli available", False, "not on PATH")
        return
    d = run_drc(PCB)
    v = d.get("violations", [])
    rep.add("KICAD-drc", "kicad-cli DRC (all severities, custom .kicad_dru) reports 0 violations",
            not v, "; ".join(f"{x['type']}: {x['description']}" for x in v) or "0 violations")
    rep.add("KICAD-parity", "schematic <-> PCB parity clean",
            not d.get("schematic_parity") and not d.get("unconnected_items"),
            f"{len(d.get('schematic_parity', []))} parity, {len(d.get('unconnected_items', []))} unconnected")

    # Mutation tests: inject a defect into a scratch copy; DRC must catch it.
    src = PCB.read_text()
    mutations = {
        "via 0.2 mm from edge -> copper_edge_clearance":
            ("copper_edge_clearance",
             src.rstrip()[:-1] + '  (via (at 27.5 0.45) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 0) (uuid "00000000-0000-0000-0000-00000000a001"))\n)\n'),
        "via inside H1 keep-out -> items_not_allowed":
            ("items_not_allowed",
             src.rstrip()[:-1] + '  (via (at 5.5 3.5) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 0) (uuid "00000000-0000-0000-0000-00000000a002"))\n)\n'),
        "H1 moved to legacy 2.25 mm inset -> npth_edge_web (hole_clearance)":
            ("hole_clearance",
             src.replace("(at 3.5 3.5)\n    (path", "(at 2.25 2.25)\n    (path", 1)),
        "Edge.Cuts corner opened -> invalid_outline":
            ("invalid_outline",
             src.replace("(start 55 40)\n    (end 0 40)", "(start 54 40)\n    (end 0 40)", 1)),
    }
    for name, (want, text) in mutations.items():
        if text == src:
            rep.add(f"MUT-{want}", f"mutation '{name}' applied", False, "pattern not found")
            continue
        with tempfile.TemporaryDirectory() as td:
            for ext in (".kicad_pro", ".kicad_dru", ".kicad_sch"):
                s = PCB.with_suffix(ext)
                if s.exists():
                    shutil.copy(s, Path(td) / s.name)
            for t in ("fp-lib-table", "sym-lib-table"):
                shutil.copy(ROOT / t, Path(td) / t)
            shutil.copytree(ROOT / "lib", Path(td) / "lib")
            p = Path(td) / PCB.name
            p.write_text(text)
            d = run_drc(p)
        types = sorted({x["type"] for x in d.get("violations", [])})
        rep.add(f"MUT-{name.split(' -> ')[0].replace(' ', '_')}",
                f"mutation caught: {name}", want in types, f"DRC types: {types}")


# --------------------------------------------------------------------------
def write_report(rep, path: Path):
    lines = ["# Rev 7.0 Board Outline DRC Report", "",
             "Generated by `tools/verify_outline.py`. Source of truth: `tools/board_geometry.py`.", "",
             f"**Overall: {'PASS' if rep.ok else 'FAIL'}** "
             f"({sum(c.ok for c in rep.checks)}/{len(rep.checks)} checks passed)", "",
             "| ID | Check | Result | Detail |", "|---|---|---|---|"]
    for c in rep.checks:
        lines.append(f"| {c.cid} | {c.desc} | {'PASS' if c.ok else '**FAIL**'} | {c.detail.replace('|', '/')} |")
    path.write_text("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-kicad", action="store_true", help="skip kicad-cli DRC and mutation tests")
    ap.add_argument("--report", type=Path, help="write a Markdown report")
    a = ap.parse_args()
    rep = Report()
    check_pcb(rep)
    check_gerber(rep)
    check_drill(rep)
    check_dxf(rep)
    if not a.no_kicad:
        check_kicad(rep)
    w = max(len(c.cid) for c in rep.checks)
    for c in rep.checks:
        print(f"[{'PASS' if c.ok else 'FAIL'}] {c.cid:<{w}}  {c.desc}" + (f"  ({c.detail})" if c.detail else ""))
    print(f"\n{'PASS' if rep.ok else 'FAIL'}: {sum(c.ok for c in rep.checks)}/{len(rep.checks)} checks")
    if a.report:
        write_report(rep, a.report)
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
