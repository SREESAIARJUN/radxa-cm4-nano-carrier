# DRC Review: 55.0 × 40.0 mm Carrier, Rev "Opus 5.5" Iteration

## ⚠️ Status: No Design Data Received. Review Cannot Be Signed Off.

Your message references the latest iteration, but **no design data was attached**. There are no Gerbers, ODB++, IPC-2581, KiCad/Altium files, placement CSV, stackup, or netlist. I won't invent placement coordinates or report findings on a layout I haven't seen. That would produce a fabricated DRC result, which is worse than none.

What I can give you now:

- Board geometry derived from your stated constraints
- Rule-based pre-checks that already show **four real problems** in the stated requirements
- Pass/fail criteria to run against the files once you send them

---

## 1. Board Envelope and Mounting Hole Geometry

**Assumption:** "2.25 mm margin" means the hole **center** is 2.25 mm from each edge. If it means hole-wall-to-edge, tell me and I'll recompute.

| Hole | Center (X, Y) mm |
|---|---|
| H1 | (2.25, 2.25) |
| H2 | (52.75, 2.25) |
| H3 | (2.25, 37.75) |
| H4 | (52.75, 37.75) |

Hole-to-hole pitch is 50.50 × 35.50 mm.

### 🔴 Finding 1-A: M2.5 hardware does not fit a 2.25 mm center margin

| Item | Typical dimension | Radius | Margin left to board edge |
|---|---|---|---|
| Drill (M2.5 clearance, ISO 273 medium) | 2.70 mm | 1.35 | 0.90 mm ✅ |
| Plated annular ring pad (common 4.5–5.0 mm) | 5.00 mm | 2.50 | **−0.25 mm ❌ (pad off board)** |
| Socket head cap screw (ISO 4762) | 4.50 mm | 2.25 | **0.00 mm ❌ (flush with edge)** |
| Button head (ISO 7380) | 4.70 mm | 2.35 | **−0.10 mm ❌ (over