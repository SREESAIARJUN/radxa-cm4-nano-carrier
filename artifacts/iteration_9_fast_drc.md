# DRC Review: 55 × 40 mm Board

## ⚠️ No Design Data Received

Your message references "the latest engineering iteration from Claude Opus 5.5," but nothing came through after the colon. I have no placement data, Gerbers, netlist, or layout to review. **I won't produce pass/fail verdicts on a design I haven't seen.** That would be a fabricated DRC report, which is worse than none.

Below I've done the work that is possible without the design: I derived the numerical acceptance criteria from your stated constraints and flagged several issues in the constraints themselves. When you send the data, I'll run each check against these limits.

### Please provide (any of these)
- KiCad (`.kicad_pcb`) or Altium/Eagle source, **or** Gerbers + drill file + IPC-D-356 netlist
- Pick-and-place / centroid file (X, Y, rotation, side)
- Stackup (layer count, dielectric thicknesses, copper weights)
- BOM with component heights
- The purpose of the DF40 interface (e.g., CM4-compatible module? Which high-speed buses?)

---

## 1. Board Outline & Mounting Holes: Pre-Analysis

### Constraint ambiguity to resolve first
"2.25 mm margins" can mean two different things:

| Interpretation | Hole center from edge | Hole edge to board edge |
|---|---|---|
| A: center at 2.25 mm | 2.25 mm | 0.90 mm (with 2.7 mm drill) |
| B: hole edge at 2.25 mm | 3.60 mm | 2.25 mm |

### 🔴 Likely iss