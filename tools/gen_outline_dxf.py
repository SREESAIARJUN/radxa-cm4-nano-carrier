#!/usr/bin/env python3
"""Generate board_outline.dxf (enclosure / mechanical hand-off) from board_geometry.py.

DXF convention: Y-up, origin at the board's bottom-left corner, mm units.
(KiCad board coords are Y-down from the top-left; y_dxf = BOARD_H - y_kicad.)

Layers
  EDGE_CUTS     outer profile ONLY - this is what gets routed. Nothing else may live here.
  HOLES_NPTH    Ø2.7 mm M2.5 NPTH drills
  KEEPOUT       Ø6.0 mm copper/component keep-out rings (NOT cut)
  STANDOFF      5 mm A/F hex standoff envelope (circumscribed circle, NOT cut)
  CONNECTORS    DF40 B2B receptacle reference envelopes (carried over, see README R3)
  COMPONENTS    edge-connector reference envelopes (carried over)
  DIMENSIONS    reference dimensions
"""
import math
import sys
from pathlib import Path

import ezdxf
from ezdxf import units

sys.path.insert(0, str(Path(__file__).resolve().parent))
import board_geometry as G  # noqa: E402

OUT = Path(__file__).resolve().parent.parent / "board_outline.dxf"

# Reference envelopes carried over from the Rev 5/6 DXF (Y-up, mm). These are
# placement envelopes, not cut geometry; positions still require confirmation
# against the Radxa mechanical drawing (README finding R3).
CONNECTORS = [(15.1, 10.6, 39.9, 14.4), (15.1, 25.6, 39.9, 29.4)]
COMPONENTS = [(7.5, 0.0, 16.5, 7.5), (34.5, 0.0, 41.5, 6.5), (40.0, 12.0, 55.0, 28.0)]


def rect(msp, x0, y0, x1, y1, layer):
    msp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True,
                       dxfattribs={"layer": layer})


def main():
    doc = ezdxf.new("R2010", setup=True)
    doc.units = units.MM
    doc.header["$MEASUREMENT"] = 1
    for name, color in [("EDGE_CUTS", 1), ("HOLES_NPTH", 3), ("KEEPOUT", 6), ("STANDOFF", 8),
                        ("CONNECTORS", 5), ("COMPONENTS", 4), ("DIMENSIONS", 7)]:
        doc.layers.add(name, color=color)
    msp = doc.modelspace()

    rect(msp, 0.0, 0.0, G.BOARD_W, G.BOARD_H, "EDGE_CUTS")
    for ref, (x, yk) in G.HOLES.items():
        y = G.BOARD_H - yk
        msp.add_circle((x, y), G.HOLE_DRILL / 2, dxfattribs={"layer": "HOLES_NPTH"})
        msp.add_circle((x, y), G.HOLE_KEEPOUT_D / 2, dxfattribs={"layer": "KEEPOUT"})
        r = G.standoff_corner_radius()
        msp.add_lwpolyline([(x + r * math.cos(math.radians(30 + 60 * k)),
                             y + r * math.sin(math.radians(30 + 60 * k))) for k in range(6)],
                           close=True, dxfattribs={"layer": "STANDOFF"})
    for r in CONNECTORS:
        rect(msp, *r, "CONNECTORS")
    for r in COMPONENTS:
        rect(msp, *r, "COMPONENTS")

    h1 = (G.HOLE_INSET, G.HOLE_INSET)
    msp.add_linear_dim(base=(0, -4), p1=(0, 0), p2=(G.BOARD_W, 0),
                       dxfattribs={"layer": "DIMENSIONS"}).render()
    msp.add_linear_dim(base=(-4, 0), p1=(0, 0), p2=(0, G.BOARD_H), angle=90,
                       dxfattribs={"layer": "DIMENSIONS"}).render()
    msp.add_linear_dim(base=(0, G.BOARD_H + 4), p1=(h1[0], G.BOARD_H - h1[1]),
                       p2=(G.BOARD_W - h1[0], G.BOARD_H - h1[1]),
                       dxfattribs={"layer": "DIMENSIONS"}).render()
    msp.add_linear_dim(base=(G.BOARD_W + 4, 0), p1=(G.BOARD_W - h1[0], h1[1]),
                       p2=(G.BOARD_W - h1[0], G.BOARD_H - h1[1]), angle=90,
                       dxfattribs={"layer": "DIMENSIONS"}).render()
    doc.saveas(OUT)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
