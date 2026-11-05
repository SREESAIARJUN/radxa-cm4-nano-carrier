# DRC Review: 55.0 × 40.0 mm Carrier Board

## Status: Review Blocked (No Design Data Received)

The design data from the latest iteration did not come through. I have no placement file, Gerbers, ODB++, IPC-2581, native CAD project, stackup, or netlist. I can't issue a pass/fail on a layout I haven't seen, and inventing clearance numbers for it would be worse than no review.

Below is what I can do without the files: findings that follow from the stated constraints alone, a numerical rule deck to run against the design, and a list of the inputs I need.

---

## 1. Board Outline and M2.5 Mounting Holes

### Pre-layout finding: the 2.25 mm margin is ambiguous and may be a real problem

**Interpretation A: hole center 2.25 mm from each edge**

Hole centers would be at (2.25, 2.25), (52.75, 2.25), (2.25, 37.75), (52.75, 37.75).

| Item | Diameter | Radius | Edge distance from board edge | Result |
|---|---|---|---|---|
| M2.5 clearance hole (medium fit) | 2.70 | 1.35 | 0.90 mm | **Marginal.** Many fabs require ≥ 1.0 mm hole-to-edge. |
| Screw head, ISO 7380 / DIN 912 | ~4.5 to 4.7 | 2.25 to 2.35 | 0 to −0.10 mm | **FAIL.** The head is flush with or overhangs the board edge. |
| Pan head, ISO 7045 | 5.0 | 2.50 | −0.25 mm | **FAIL.** Overhangs. |
| Washer, ISO 7089 | 6.0 | 3.00 | −0.75 mm | **FAIL.** Overhangs. |
| Standoff, hex 5.0 mm A/F | ~5.8 across corners | 2.9 | −0.65 mm | **FA