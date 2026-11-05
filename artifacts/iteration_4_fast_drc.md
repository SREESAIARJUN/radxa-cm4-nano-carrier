# DRC & Manufacturability Review: Radxa CM4 Nano Carrier, Rev 4.0 (Cycle 4)

## Scope Limitations

1. **The document is truncated at R6.** Everything after "3.3 V @ 2 A and 1.8 V regulators may be redun…" is missing, including the stackup, placement, and routing sections.
2. **No layout data was supplied.** There are no Gerbers, no `.kicad_pcb` or `.brd` file, no DRC report, and no length-tuning report.

Rules that depend on geometry I don't have are marked **BLOCKED**, not PASS. A rule marked PASS without layout evidence would be a false sign-off. Where the spec itself guarantees a violation, the rule is marked **FAIL** regardless of layout.

## Summary

| # | Rule | Status | Basis |
|---|---|---|---|
| 1 | M2.5 head/standoff clearance at 2.25 mm inset | **FAIL** | Geometry guarantees overhang |
| 2 | Copper ≥ 0.3 mm from edge cut | **FAIL (conditional) / BLOCKED** | Fails if mounting holes are plated with ground pads. Otherwise needs layout. |
| 3 | SY8089 thermal vias, no unpadded bottom vias | **FAIL (rule premise) + thermal risk** | SOT23-5 has no exposed pad. 2 A load is thermally marginal. |
| 4 | PCIe/HDMI intra-pair skew < 0.15 mm | **Rule PASS with amendments / Verification BLOCKED** | Tolerance is acceptable but incomplete. No routing to check. |

---

## Rule 1: M2.5 Screw Head and Standoff Clearance at 2.25 mm Inset → **FAIL**

**Calculation.** The hole center sits 2.25 mm from the edge. The table gives the edge margin for each hardware option (negative = overhang).

| Hardware | Relevant Ø | Radius | Edge margin |
|---|---|---|---|
| Drill Ø2.7 (M2.5 clearance) | 2.7 | 1.35 | **+0.90 mm FR4 web** |
| ISO 1207 cheese head M2.5 | 4.5 | 2.25 | **0.00 mm** (tangent) |
| ISO 7380 button head M2.5 | 4.7 | 2.35 | **−0.10 mm** |
| ISO 7045 pan head M2.5 | 5.0 | 2.50 | **−0.25 mm** |
| M2.5 hex standoff, 5 mm AF (flats) | 5.0 | 2.50 | **−0.25 mm** |
| M2.5 hex standoff, across corners | 5.77 | 2.89 | **−0.64 mm** |
| M2.5 washer DIN 125 | 6.0 | 3.00 | **−0.75 mm** |

**Correction to R1 in the document.** R1 assumes an M2.5 pan head is Ø4.5 mm. ISO 7045 specifies dk = 5.0 mm max, so the real overhang is worse than R1 states. Ø4.5 mm applies only to cheese heads, and even that case is tangent to the edge, not clear of it.

**The proposed R1 fallback also fails:**
- **Ø4.0 low-profile head:** 2.25 − 2.0 = 0.25 mm. That is below the 0.3 mm keepout the document itself requires.
- **M2 hardware:**
  - The M2 pan head (Ø4.0) has the same 0.25 mm margin.
  - An M2 4 mm hex standoff overhangs by 0.06 mm across its corners.

**Other concerns:**
- A 0.9 mm FR4 web carrying screw torque risks cracking, particularly on 0.8–1.0 mm boards.
- A rotating hex standoff will sweep past the edge.

**Corrective actions, in order of preference:**
1. **Move the holes to a 3.5 mm inset (48 × 33 mm pitch).** This matches the CM4 module holes, so a single standoff stack clamps both boards.
   - Margin with an M2.5 pan head: 3.5 − 2.5 = **1.0 mm**.
   - Margin with standoff corners: 3.5 − 2.89 = **0.61 mm**.
   - Both pass.
2. If 2.25 mm is contractually fixed:
   - Use M2 with a round (not hex) standoff of Ø3.5 mm, giving 0.5 mm margin.
   - Use an M2 cheese head of Ø3.8 mm, giving 0.35 mm margin.
   - Make the hole NPTH with no copper.
   - Record a mechanical deviation, because the hole will no longer align with the module.
3. Add a courtyard or keepout polygon equal to the **largest** hardware outline (washer or standoff corners) to the footprint. That lets DRC catch the problem automatically.

---

## Rule 2: Copper Pour ≥ 0.3 mm From Edge Cut → **FAIL (conditional) / BLOCKED**

**Deterministic fail case: plated mounting holes at 2.25 mm inset.**
- The maximum pad diameter that keeps 0.3 mm to the edge is 2 × (2.25 − 0.3) = **Ø3.9 mm**.
- A typical M2.5 grounding pad is Ø5.0–6.0 mm. That puts copper 0.25–0.75 mm **past the edge**, so fabrication will either clip it or reject it.
- With a Ø2.7 drill, a Ø3.9 pad leaves a 0.6 mm annular ring. That passes fab minimums but is a weak ground contact.

**Items that need layout to verify:**
- Pour clearance on all layers. Inner planes need the same 0.3 mm (0.5 mm is preferred if the board is V-scored or routed from a panel).
- **Edge-mounted connectors:**
  - USB-C, micro-HDMI, and RJ45 shells often have tabs that intentionally reach the edge.
  - Each one needs an explicit, documented DRC exclusion, not a global relaxation