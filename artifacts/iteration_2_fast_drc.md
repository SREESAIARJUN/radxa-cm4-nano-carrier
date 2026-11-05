# DRC & Manufacturability Review: CM4-Form-Factor Carrier, Rev 2.0 (Cycle 2)

## Review Basis and Limitations

- **The package is truncated.** It ends mid-sentence in item 0.2, so I have no §1 pin table, stack-up, or layout data.
- **No Gerbers, ODB++, or board file were provided.** Any rule that needs as-built geometry is scored against the stated intent and marked **UNVERIFIED** where I could not measure it. A UNVERIFIED rule is not a pass.
- **Assumed hardware:**
  - M2.5 ISO 7380 / DIN 7985 pan head: Ø4.5–5.0 mm.
  - M2.5 hex standoff: 5.0 mm across flats, about 5.8 mm across corners.
  - M2.5 clearance drill: 2.7 mm.
  - The 2.25 mm-inset enclosure holes are also M2.5. If they are M2, the conclusion is the same.

## Summary

| # | Rule | Status | Severity |
|---|---|---|---|
| 1 | M2.5 standoff/screw head clearance vs 2.25 mm margin | **FAIL** | Blocker |
| 2 | Copper pour ≥ 0.3 mm from edge cut | **FAIL (rule inadequate) / UNVERIFIED (layout)** | Major |
| 3 | Thermal via stitching on SY8089 SOT23-5 | **FAIL (premise wrong) + thermal risk** | Major |
| 4 | PCIe/HDMI intra-pair skew < 0.15 mm | **PASS (rule) / UNVERIFIED (layout), incomplete** | Major |

---

## Rule 1: Screw Head Clearance at the 2.25 mm Margin. **FAIL**

**A. The two hole sets collide.**
- Hole centres are at (2.25, 2.25) and (3.5, 3.5).
- Centre-to-centre distance: √(1.25² + 1.25²) = **1.77 mm**.
- Sum of drill radii: 1.35 + 1.35 = 2.70 mm, which is greater than 1.77 mm. **The drills physically overlap.**
- With Ø5.0 mm head and standoff keepouts, the overlap is about 3.2 mm. Neither fastener can seat.
- The truncated note in §0.2 ("check that the keepouts don't collide") resolves to: **they do collide, at all four corners.**

**B. The 2.25 mm inset has zero edge margin.**
- Head radius is 2.25 mm (Ø4.5) to 2.5 mm (Ø5.0).
- At a 2.25 mm inset, the head is tangent to the board edge, or overhangs it by 0.25 mm.
- A hex standoff corner (r ≈ 2.9 mm) overhangs by about 0.65 mm.
- Typical fab and assembly practice wants head keepout + ≥0.5 mm to the edge. The required inset is therefore ≥ 2.75–3.0 mm.

**C. Module footprint consideration.**
- The carrier is 55 × 40 mm, the same outline as the module.
- Any head or standoff at the 3.5 mm positions sits directly under the module's own mounting points.
- Bottom-side screw heads must also clear any bottom components.

**Corrective action (choose one):**
1. **Preferred:** Delete the 2.25 mm holes. Use the 4× M2.5 holes at (3.5, 3.5), (51.5, 3.5), (51.5, 36.5), (3.5, 36.5) for both module and enclosure.
   - Use male-female standoffs: module screw on top, enclosure screw or standoff thread on the bottom.
2. If separate enclosure holes are mandatory, grow the board, for example to 61 × 46 mm, and place the enclosure holes ≥ 3.0 mm from the edge and ≥ 6 mm centre-to-centre from the module holes.
3. **Keepout around each M2.5 hole, all layers:**
   - Ø6.0 mm copper/component keepout on the head side.
   - Ø6.5 mm under hex standoffs.
   - Add a GND-plated ring if chassis grounding is wanted. Otherwise use NPTH with a 0.5 mm copper pullback.

---

## Rule 2: Copper Pour ≥ 0.3 mm from Edge Cut. **FAIL (rule) / UNVERIFIED (layout)**

**Why 0.3 mm is insufficient:**
- Routed-edge tolerance at typical prototype fabs is ±0.1–0.2 mm. A 0.3 mm pullback leaves 0.1–0.2 mm worst case. This risks exposed copper, burrs, and inner-plane shorts on 4-layer stack-ups.
- If panelised with V-score, most fabs require ≥ 0.4–0.5 mm.
- The minimum must apply to **inner planes as well**, not just outer pours. Inner GND/PWR planes reaching the edge are the usual shorting failure.

**Corrective action:**
1. Set board-edge clearance to **0.5 mm** for outer pours and inner planes.
   - Use 0.4 mm only if routed-only and confirmed with the fab.
   - Use ≥ 0.5 mm plus fab-specified distance if V-scored.
2. Apply ≥ 0.5 mm copper clearance to NPTH mounting holes. Apply ≥ 0.3 mm (annulus excluded) to the PTH hole edge where unconnected.
3. Optionally add a GND via fence (≈ 2–3 mm pitch) inside the pullback for EMI on the HDMI/PCIe edge.
4. Re-run DRC on the actual board file and attach the report. Status stays UNVERIFIED until then.

---

## Rule 3: Thermal Via Stitching on SY8089 SOT23-5. **FAIL**

**A. Wrong premise.**
- The SY8089 (SY8089AAAC) in SOT23-5 has **no exposed thermal pad**.
- Heat leaves through the leads, mainly GND (pin 2) and the LX/IN copper.
- There is no pad under the body to stitch. Vias placed under the body between pads are exactly the "unpadded bottom vias" the rule is meant to avoid.
