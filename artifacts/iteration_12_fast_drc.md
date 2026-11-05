# DRC Review: 55 × 40 mm Board

## Status: Review Blocked (No Design Data Received)

Your message references an engineering iteration, but no design data came through. There is no layout file, Gerbers, ODB++, IPC-2581, placement report, netlist, or screenshots. I can't issue pass/fail results on a layout I haven't seen, and inventing component coordinates or violations would be worse than no review at all.

Below are three things I *can* deliver now:
- The data I need to complete the review
- Issues I can already flag from the constraints you stated
- A ready-to-apply DRC criteria set

---

## 1. Required Inputs

| Item | Preferred format | Purpose |
|---|---|---|
| Board outline and hole locations | DXF or Gerber `Edge.Cuts` + drill file | Boundary and mounting-hole clearance |
| Placement data | Pick-and-place CSV (ref, X, Y, rot, side) | Edge and keepout checks |
| Component bodies and heights | STEP or a 3D-model height table | Top/bottom height and DF40 mating checks |
| Stackup | Layer count, dielectric thickness, Dk, copper weights | Impedance and skew verification |
| Netlist and net classes | KiCad/Altium project or IPC-356 | Diff-pair and length-match audit |
| Copper layers | Gerbers or native project | Thermal, via, and routing review |
| Fab capability | Min trace/space, min drill, annular ring | Sets DRC limits |

The native project file (`.kicad_pcb` or `.PcbDoc`) is ideal, since it carries all of the above.

---

## 2. Pre-Review Flags from Stated Constraints

### Flag A: Mounting-hole margin may conflict with the form factor
A 55 × 40 mm board with two 100-pin DF40 connectors strongly suggests a Raspberry Pi CM4-compatible footprint. The CM4 mechanical spec places the M2.5 holes at **3.5 mm** from the edges (48 × 33 mm pitch), not 2.25 mm.
- **If this is a CM4 carrier or clone:** 2.25 mm margins will not align with standard carriers or standoffs. Confirm the hole positions against the CM4 mechanical drawing before anything else.
- **If 2.25 mm is intentional:** see Flag B.

### Flag B: The 2.25 mm margin is geometrically tight
This assumes "margin" means the hole center sits 2.25 mm from each edge.

| Parameter | Typical value | Result at 2.25 mm center |
|---|---|---|
| M2.5 clearance drill | 2.7 mm (r = 1.35) | Hole edge to board edge = **0.90 mm** |
| Plated pad / annular copper | Ø 4.5–5.0 mm (r = 2.25–2.5) | Pad reaches or **overhangs the board edge**, so copper-to-edge fails |
| M2.5 pan-head / washer keepout | Ø 5.0–5.5 mm (r = 2.5–2.75) | Head **overhangs the board edge** by 0.25–0.5 mm |
| Component keepout | r ≈ 3.0 mm | No placement within ~**5.25 mm** of either edge at each corner