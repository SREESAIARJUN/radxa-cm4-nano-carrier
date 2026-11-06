"""Single source of truth for the Rev 7 mechanical envelope.

All values in millimetres, KiCad board coordinates (origin top-left,
+X right, +Y down).  Gerber/Excellon output from KiCad negates Y.

Every generator (DXF) and verifier (verify_outline.py) imports from here so
the outline, hole pattern and keep-out rules cannot drift apart again.
"""

# --- Board envelope (CM4 form factor, zero overhang) ----------------------
BOARD_W = 55.0
BOARD_H = 40.0
BOARD_T = 1.6                 # FR4 finished thickness
EDGE_CUT_WIDTH = 0.05         # Edge.Cuts stroke (KiCad uses the centreline)

# --- M2.5 mounting holes ---------------------------------------------------
HOLE_INSET = 3.5              # hole centre to nearest two edges
HOLE_DRILL = 2.7              # ISO 273 "medium" clearance for M2.5, NPTH
HOLE_KEEPOUT_D = 6.0          # copper/component keep-out ring diameter
STANDOFF_AF = 5.0             # M2.5 hex standoff, across flats
SCREW_HEAD_D = 4.5            # ISO 4762 / DIN 912 M2.5 socket head

HOLES = {
    "H1": (HOLE_INSET, HOLE_INSET),
    "H2": (BOARD_W - HOLE_INSET, HOLE_INSET),
    "H3": (HOLE_INSET, BOARD_H - HOLE_INSET),
    "H4": (BOARD_W - HOLE_INSET, BOARD_H - HOLE_INSET),
}
HOLE_PITCH_X = BOARD_W - 2 * HOLE_INSET   # 48.0
HOLE_PITCH_Y = BOARD_H - 2 * HOLE_INSET   # 33.0

# --- DRC limits ------------------------------------------------------------
POS_TOL = 0.001               # 1 um placement / closure tolerance
MIN_HOLE_WEB = BOARD_T        # drill wall to board edge >= 1x thickness
MIN_KEEPOUT_EDGE = 0.30       # keep-out ring to board edge
MIN_COPPER_EDGE = 0.30        # copper to Edge.Cuts (matches .kicad_pro)


def standoff_corner_radius(af: float = STANDOFF_AF) -> float:
    """Circumradius of a hex standoff from its across-flats size."""
    return af / (3 ** 0.5)
