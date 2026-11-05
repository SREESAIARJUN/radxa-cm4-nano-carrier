# Radxa CM4 Nano Carrier (Cycle 1): DRC and Manufacturability Review

## Review Status: No Design Data Provided

I received only a text description. There are no Gerbers, ODB++, IPC-2581, KiCad or Altium project, or DRC report. I can't measure trace lengths, pour clearances, or via placements that I can't see, and inventing numbers to fill a PASS/FAIL table would give you false confidence before fabrication.

This review therefore covers three things:
1. Issues detectable from the stated specs and known module geometry (some are real risks already).
2. Conditional verdicts with the exact checks to run.
3. Ready-to-use DRC rules so your tool can produce the verified PASS/FAIL.

---

## Summary Table

| # | Rule | Status | Basis |
|---|------|--------|-------|
| 1 | M2.5 standoff/screw head clearance (2.25 mm) | ⚠️ **AT RISK. Margin likely insufficient for hex standoffs** | Geometric analysis below |
| 2 | Copper pour to edge cut ≥ 0.3 mm | ❓ **UNVERIFIED** (no layout data) | 0.3 mm is fab-minimum; recommend 0.5 mm |
| 3 | SY8089 thermal vias, no exposed bottom vias | ⚠️ **RULE PREMISE NEEDS CORRECTION** + thermal risk | SOT23-5 has no exposed pad |
| 4 | PCIe/HDMI intra-pair skew < 0.15 mm | ❓ **UNVERIFIED** (no layout data) | Tolerance is reasonable; method below |
| — | Additional findings | ⚠️ 3 items | See section 6 |

---

## 1. M2.5 Standoff and Screw Head Clearance: ⚠️ AT RISK

**Known geometry (CM4 form factor):**
- Mounting holes sit 3.5 mm from each board edge, on a 48 × 33 mm pitch.
- Your carrier is 55 × 40 mm, the same outline as the module, so the holes are 3.5 mm from the carrier edges too.

**Why 2.25 mm is not enough as a universal margin:**

| Hardware (M2.5) | Head/body size | Radius needed |
|---|---|---|
| Socket head (ISO 4762) | Ø4.5 mm | 2.25 mm. **Zero tolerance margin** |
| Button head (ISO 7380) | Ø4.4–4.7 mm | 2.2–2.35 mm. **May exceed 2.25** |
| Pan head (ISO 7045) | Ø5.0 mm | 2.5 mm. **FAIL** |
| Hex standoff, 5 mm A/F | ~5.77 mm across corners | **~2.9 mm. FAIL** |
| Washer (ISO 7089) | Ø5.0 mm | 2.5 mm. **FAIL** |

A 2.25 mm keep-out exactly equals the socket-head radius. It leaves nothing for hole position tolerance (typically ±0.1 mm), screw