# Radxa CM4 Nano Carrier (Cycle 1): DRC and Manufacturability Review

## Scope

I received only a text description of the design, with no Gerbers, ODB++, KiCad/Altium project, stackup, or length report. I cannot measure clearances, via placement, or trace lengths. Each status below is therefore one of three kinds:

* **Spec-level**: The rule or design intent can be judged from the description alone.
* **Conditional**: The status depends on layout data I haven't seen.
* **Unverified**: Layout data is required.

Several rules have problems in the rule definitions themselves, and those are worth fixing before layout.

---

## Rule 1: M2.5 Standoff and Screw Head Clearance (2.25 mm margin)

**Status: FAIL (spec-level)**

**Findings**

* A 2.25 mm radius keep-out (Ø4.5 mm) equals the head diameter of a DIN 912 / ISO 4762 M2.5 socket cap screw. That leaves zero tolerance for screw-head runout or hole position tolerance.
* Other common hardware exceeds 2.25 mm outright:

| Hardware | Typical diameter | Radius | vs. 2.25 mm |
|---|---|---|---|
| M2.5 socket cap (DIN 912) | 4.5 mm | 2.25 mm | 0 mm margin |
| M2.5 pan head (ISO 7045) | ~5.0 mm | 2.5 mm | **Violates** |
| M2.5 hex standoff, 5 mm A/F (across corners) | ~5.77 mm | 2.89 mm | **Violates** |
| M2.5 nylon/steel washer | 5.0–6.0 mm | 2.5–3.0 mm | **Violates** |

* **Stack height**: A 55×40 mm carrier matches the CM4 module footprint, with mounting holes about 3.5 mm in from each edge. Standoff height must match the DF40 mated height for your connector variant, typically 1.5 mm or about 3.0 mm depending on DF40 part selection. Otherwise screw torque will load the 0.4 mm pitch connectors.

**Corrective action**

1. Increase the keep-out to **≥3.0 mm radius (Ø6.0 mm)** on both layers. This covers hex standoffs, washers, and fixture tolerance.
2. Apply the keep-out to copper, components, and traces on top and bottom layers. Use a plain annular ring or NPTH, and stitch the annular ring to GND only if chassis-ground mounting is intended.
3. Confirm hole drill size: **2.7 mm** for M2.5 clearance (ISO 273 medium fit).
4. Verify standoff length equals the DF40 mated stack height. Document the exact DF40 part numbers (receptacle and header) on the fab/assembly drawing.
5. Check that no DF40 body, decoupling cap, or SY8089 component sits inside the enlarged keep-outs, particularly at the corners near the connectors.

---

## Rule 2: Copper Pour to Edge Cut (≥0.3 mm)

**Status: CONDITIONAL PASS (marginal)**

**Findings**

* 0.3 mm is the common fab minimum for routed edges. It is not a robust design value.
* At 0.3 mm, router tolerance (±0.1–0.2 mm) can expose copper at the board edge. This risks shorts to an enclosure and GND burrs.
* V-score panelization typically requires **≥0.4–0.5 mm**, and some fabs specify more.
* The board edge equals the module outline, so copper near the edge also sits under the module edge.

**Corrective action**

1. Raise the zone-to-Edge.Cuts clearance to **0.5 mm** (0.4 mm absolute minimum if routing space forces it).
2. Apply the rule to **all copper layers, including inner planes**. Inner-plane pullback is often forgotten.
3. Add a GND via fence along the perimeter, about 0.5 mm in from the pullback, at a pitch of λ/20 or less. This improves EMI containment with the HDMI and PCIe signals present.
4. Confirm the panelization method (tab-route vs. V-score) with the fab and set the clearance accordingly.

---

## Rule 3: Thermal Via Stitching on SY8089 (SOT23-5)

**Status: FAIL (rule definition) / UNVERIFIED (layout)**

**Findings**

* **The SY8089 in SOT23-5 has no exposed thermal pad.** "Thermal via stitching under the device" does not apply. Heat leaves mainly through the leads, chiefly GND and the LX/SW pin, into copper.
* Vias placed **under the body or in pads** without tenting or filling cause solder wicking, voids, and tombstoning on a small leaded package. Your