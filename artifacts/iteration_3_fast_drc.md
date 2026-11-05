# DRC and Manufacturability Review: Radxa CM4 Nano Carrier, Rev 3.0 (Cycle 3)

## Scope Limitation

The submitted document is **truncated**. It ends partway through Finding F1 ("holes inset 3.5 mm, at a pitch of…"). The review package contains no layout data: no Gerbers, no stackup, no placement, no length reports, no DRC rule file.

So none of the four rules can be confirmed against the actual board. Under a stringent review, an unverifiable rule cannot pass. Each rule below therefore gets two things:

1. **Status:** a verdict based on the evidence actually submitted.
2. **Engineering pre-check:** analysis against known form-factor and vendor facts, so the designer knows what will pass once data is supplied.

---

## Rule 1: M2.5 Standoff and Screw-Head Clearance (2.25 mm margin)

**Status: FAIL (blocking).** The hole coordinates and keepout geometry are missing, and F1 states the holes do not match the CM4 pattern.

**Reference geometry (CM4 form factor):**
- Module size: 55 × 40 mm.
- Holes: 4 × M2.5, inset 3.5 mm from each edge, so the pitch is 48 × 33 mm.
- The carrier must align its holes to this pattern, or the module cannot be secured.

**Clearance math.** The "2.25 mm" figure only works for one of the three fastener features:

| Feature | Typical size | Radius from hole centre | Meets 2.25 mm? |
|---|---|---|---|
| M2.5 socket or button head (ISO 7380 / DIN 912) | Ø4.4 to 4.5 mm | ~2.25 mm | Marginal: zero clearance |
| M2.5 pan head (ISO 7045) | Ø5.0 mm | 2.5 mm | **No** |
| M2.5 hex standoff, 5 mm across flats | 5.77 mm across corners | **2.89 mm** | **No** |

In other words, a 2.25 mm radius equals the head radius itself and leaves no assembly tolerance. It also ignores the standoff's hex corners on the opposite face.

**Corrective actions:**
1. Close F1 first. Place holes at a 48 × 33 mm pitch referenced to the module outline, and confirm the same pattern against the Radxa CM3 and CM5 mechanical drawings.
2. Set the component and copper keepout to at least **Ø6.5 mm, i.e. 3.25 mm radius**. This covers the hex corners (2.89 mm), adds 0.3 mm placement tolerance, and leaves wrench clearance.
3. If holes are plated and grounded, define the annular ring explicitly (for example Ø5.0 mm pad on a Ø2.7 mm drill). Otherwise use NPTH with a copper pullback of 0.5 mm or more.
4. Check component height inside the standoff zone on the **underside** of the module, not just on the carrier top.
5. Specify the actual fastener part number in the BOM so the keepout can be tied to a real head diameter.

---

## Rule 2: Copper Pour Clearance to Edge Cut (≥ 0.3 mm)

**Status: FAIL (blocking).** No outline layer or pour settings were submitted.

**Pre-check notes:**
- 0.3 mm is adequate for a **routed** edge at most fabs (typical minimum 0.2 to 0.25 mm).
- It is **not adequate** for:
  - **V-score panel edges.** Use 0.4 to 0.5 mm. V-scoring cuts into the board, so copper too close gets exposed or cracks.
  - **Mouse-bite tab regions.** Use 0.5 mm or more near breakout holes.
  - **Inner planes.** Pull these back at least 0.5 mm to avoid exposed planes and shorts after routing.

**Corrective actions:**
1. Set the edge-cut clearance rule on all layers: 0.3 mm (outer, routed), 0.5 mm (inner planes), 0.5 mm (V-score and tab edges).
2. Run DRC with "copper to board edge" enabled on **every** layer, including pours and plane layers. Polygon pours often bypass the track rule.
3. Confirm the panelisation method with the fab before freezing the rule.
4. If any connector overhangs the edge (USB, HDMI), check its shell and ground pads separately. These are frequently waived by mistake.

---

## Rule 3: Thermal Via Stitching on the SY8089 (SOT23-5)

**Status: FAIL (blocking).** No footprint or layout was submitted.

**Pre-check:** the rule as written is partly malformed.
- The SY8089 (for example the SY8089AAAC) in SOT23-5 has **no exposed thermal pad**. There is nothing to put "padded" vias into under the body.
- Heat leaves through the **leads**, mainly GND, VIN and LX, into the copper they attach to.
- A 2 A, 1.5 MHz buck in SOT23-5 is thermally marginal, so copper area matters.

**Corrective actions:**
1. **No vias under the package body.** Untented vias there can wick solder or create bridge and short risk under a low-standoff part.
2. **Expand copper on the GND pin** into a top-layer pour. Stitch this pour to the inner GND plane with 4 to 8 vias (0.3 mm drill), placed **beside** the pin, tented or plugged, and 0.5 mm or more from the pad edge to prevent solder wicking.
3. **Widen VIN and VOUT copper.** Place the input capacitor directly at VIN and GND, with its GND vias at the capacitor pad.
4. **Keep the LX node small**, with no stitching vias on it, to limit EMI. The inductor must sit adjacent to the IC.
5. Keep the feedback trace away from LX and the inductor.
6. Run a thermal estimate at worst-case load (CM4 module plus USB):
   - If the rail exceeds about 1.5 A sustained, consider a DFN or exposed-pad regulator.
   - Alternatively, split the load across rails.

---

## Rule 4: PCIe and HDMI Differential Pair Length Matching (< 0.15 mm intra-pair)

**Status: FAIL (blocking).** No length or skew report and no stackup or impedance data were submitted.

**Pre-check targets:**

| Interface | Impedance | Intra-pair (P/N) | Inter-pair |
|---|---|---|---|
| PCIe Gen2 (CM4) / Gen2–3 (Radxa CM5) | 85 Ω diff (90 Ω acceptable) | ≤ 0.15 mm (0.127 mm preferred for Gen3) | Not required (lanes independent) |
| HDMI TMDS (data and clock) | 100 Ω diff | ≤ 0.15 mm | Per module design guide (typically a few mm) |

The 0.15 mm limit is reasonable for both interfaces.

**Corrective actions:**
1. **Use module-side length data.** The CM4 datasheet publishes internal track lengths per pin, and carrier matching must include them. Obtain the equivalent tables for Radxa CM3 and CM5. If they differ, matching cannot be optimal for all three modules, so document the compromise.
2. Match intra-pair skew **close to the mismatch source**, for example near a bend or the connector. Do not add a single serpentine at the far end.
3. Use **small bumps** for skew compensation, not large serpentines. The bump amplitude should not exceed 2× the trace gap.
4. Keep a continuous reference plane under every pair, with no splits.
5. Place PCIe AC-coupling capacitors on TX with symmetric P/N placement.
6. Minimise layer changes. Where one is needed, place a GND return via within about 1 mm of each signal via pair.
7. Submit the following, verified with the fab's field solver:
   - the per-net length report;
   - the stackup;
   - the impedance calculation.

---

## Summary

| # | Rule | Status | Blocking issue |
|---|---|---|---|
| 1 | M2.5 head/standoff clearance (2.25 mm) | **FAIL** | F1 hole pattern mismatch. 2.25 mm is below the standoff corner radius (2.89 mm). Use a 3.25 mm keepout radius. |
| 2 | Copper to edge ≥ 0.3 mm | **FAIL** (unverified) | No outline or pour data. Raise to 0.5 mm for inner planes and V-score edges. |
| 3 | SY8089 th