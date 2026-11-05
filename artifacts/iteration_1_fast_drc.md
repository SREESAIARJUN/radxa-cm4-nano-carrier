# DRC & Manufacturability Review: Radxa CM4 Nano Carrier (Cycle 1)

## Important limitation

I received only a text summary of the design. There are no Gerbers, ODB++, KiCad/Altium files, stack-up, or length reports. **I cannot honestly issue measured PASS/FAIL results**, because doing so would mean inventing numbers.

Instead, this review provides:
- **Findings from the stated specs.** Some rules can be judged from the description alone, and some of the rules themselves have problems.
- **A verdict for each rule** that distinguishes "fails by specification" from "unverified."
- **Exact checks and data** needed to close each item in Cycle 2.

---

## Rule 1: M2.5 Standoff and Screw Head Clearance (2.25 mm margin)

**Status: ❌ FAIL (the rule is non-conservative as written). The layout itself is unverified.**

**Analysis:** If "2.25 mm margin" means a keep-out radius of 2.25 mm from the hole center, that is a 4.5 mm diameter. That only matches the smallest common M2.5 hardware, with zero tolerance:

| Hardware | Head / body Ø | Required radius | vs. 2.25 mm |
|---|---|---|---|
| DIN 912 socket head | 4.5 mm | 2.25 mm | 0 mm margin |
| ISO 7380 button head | 4.7 mm | 2.35 mm | ❌ violates |
| ISO 7045 pan head | 5.0 mm | 2.50 mm | ❌ violates |
| M2.5 hex standoff, 5 mm A/F (across corners) | 5.77 mm | 2.89 mm | ❌ violates |
| M2.5 washer (ISO 7089) | 5.0 mm | 2.50 mm | ❌ violates |

The hex standoff is the binding case on the **top side**, which faces the module. The screw head or washer is the binding case on the **bottom side**.

**Corrective action:**
1. Set the copper and component keep-out to **≥ 3.2 mm radius (6.4 mm Ø)** on both sides. This covers the standoff corners plus about 0.3 mm placement and rotation tolerance.
2. Decide whether the holes are plated and grounded:
   - **Grounded (preferred for EMI/ESD):** use a ring pad ≥ 6.0 mm Ø.
   - **Isolated:** pull pour back ≥ 0.5 mm beyond the keep-out.
3. Verify hole positions against the official Radxa CM4 mechanical drawing, not a Raspberry Pi CM4 assumption, since tolerances may differ. On a 55 × 40 mm board, holes sit about 3.5 mm from the edges. A 3.2 mm keep-out therefore leaves only about 0.3 mm to the edge, so keep the board outline corners free of copper there.
4. Check that no DF40 connector body, decoupling caps, or SY8089 inductor fall inside the standoff keep-out.

---

## Rule 2: Copper Pour to Edge Cut (≥ 0.3 mm)

**Status: ⚠️ UNVERIFIED (no layout data). The rule value is marginally acceptable.**

**Analysis:**
- 0.3 mm meets typical fab minimums for routed (milled) edges. For example, JLCPCB/PCBWay require about 0.2–0.3 mm.
- It is **insufficient for V-score panelization**, which needs ≥ 0.4–0.5 mm. On a board this small, V-score panels are likely.
- Router tolerance is about ±0.1–0.2 mm, so 0.3 mm can drop to roughly 0.1 mm in the worst case. That risks exposed copper and edge shorts between GND and inner-layer planes.

**Corrective action:**
1. Raise the design rule to **0.5 mm** for all layers, including inner planes, which are often missed because they are set separately.
2. Confirm the zone/polygon "board edge clearance" overrides the general clearance in your EDA tool. In KiCad, check both Board Setup → Constraints → *Copper to edge clearance* and the zone properties.
3. Run DRC with the edge-cut layer included, and visually inspect the inner-layer Gerbers.
4. Optionally, add a GND via fence along the edge (about 2–3 mm pitch) to reduce EMI near the HDMI connector.

---

## Rule 3: SY8089 Thermal Via Stitching (SOT23-5)

**Status: ⚠️ CONDITIONAL. The rule premise needs correction, and there is a thermal risk at 2 A.**

**Analysis:**
1. **SOT23-5 has no exposed pad.** "Thermal vias under the pad" do not apply in the usual sense. Heat leaves mainly through the leads, especially GND and the LX/SW pin, into the copper.
2. **Vias in the gap under the package body** are fine only if they are **tented or plugged** on the top side. Untented vias under a small leaded package can wick solder or trap flux. If "unpadded bottom vias" means untented, exposed vias on the bottom side, those should be tented or covered by solder mask.
3. **Thermal budget at 2 A is a real concern:**
