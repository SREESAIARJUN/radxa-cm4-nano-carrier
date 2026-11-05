# DRC Review: 55 × 40 mm Board

## Status: Review Blocked (No Design Data Received)

Your message references the latest iteration, but **no design data was attached**. I have no placement file, Gerbers, ODB++, IPC-2581, KiCad/Altium project, BOM, or stackup. I can't produce a real pass/fail verdict without them, and a fabricated one would be worse than none.

What I can do now:
1. Flag **two problems that are already visible in the stated requirements**, before any layout data.
2. Provide the **rule deck and numerical pass/fail criteria** I'll apply once you send the files.
3. List exactly what to send.

---

## Part 1: Issues Visible in the Stated Requirements

### ⚠️ Issue A: The 2.25 mm mounting-hole margin is too tight for M2.5 hardware

If "2.25 mm margin" means **hole center 2.25 mm from each board edge**, the hole centers are:

| Hole | X (mm) | Y (mm) |
|---|---|---|
| H1 | 2.25 | 2.25 |
| H2 | 52.75 | 2.25 |
| H3 | 2.25 | 37.75 |
| H4 | 52.75 | 37.75 |

Pitch: 50.50 × 35.50 mm.

| Feature | Typical diameter | Radius | Edge clearance at 2.25 mm center |
|---|---|---|---|
| M2.5 clearance drill (ISO 273 medium) | 2.70 | 1.35 | **0.90 mm** ✅ |
| Socket head cap (DIN 912) | 4.50 | 2.25 | **0.00 mm** ❌ flush with edge |
| Flat washer M2.5 | 5.00 | 2.50 | **−0.25 mm** ❌ overhangs board |
| Hex standoff, 5 mm A/F (across corners ≈ 5.77) | 5.77 | 2.89 | **−0.64 mm** ❌ overhangs board |
| Typical annular ring / keepout pad | 5.0–6.0 | 2.5–3.0 | **negative** ❌ |

**Correction options (pick one):**
- **Option 1 (preferred):** Move hole centers to **3.50 mm** from each edge. A 6.0 mm keepout then leaves 0.50 mm to the edge, and the 5.77 mm standoff stays inside the outline. New pitch: **48.0 × 33.0 mm**.
- **Option 2:** If 2.25 mm is fixed by a mating enclosure, use **M2.5 low-profile hardware with a ≤ 4.0 mm head/standoff** and a **4.2 mm max copper/keepout ring**. Acc