# DRC and Manufacturability Review: Radxa CM4-Form-Factor Nano Carrier, Rev 1.0 (Cycle 1)

**Scope limitation:** The submitted document is cut off in Section 0, finding R1. I received no layout, Gerbers, stackup, or length report. Rules that need layout data are marked **UNVERIFIED** and cannot be signed off as PASS. Rules that can be decided from the stated geometry or datasheet facts are given a definite status.

---

## Summary

| # | Rule | Status | Severity |
|---|---|---|---|
| 1 | M2.5 standoff and screw-head clearance at 2.25 mm edge margin | **FAIL** | Blocker |
| 2 | Copper pour ≥0.3 mm from Edge.Cuts | **FAIL** (at the mounting holes); **UNVERIFIED** elsewhere | Blocker |
| 3 | Thermal via stitching on SY8089 (SOT23-5) | **FAIL**: the rule is mis-specified, and there is a thermal risk | Major |
| 4 | PCIe and HDMI intra-pair skew <0.15 mm | **UNVERIFIED**: the target is acceptable, but no length data was provided | Major |

**Gate: NOT release-for-fab.**

---

## 1. M2.5 Standoff and Screw-Head Clearance — FAIL

**Geometry check, with the hole center 2.25 mm from two edges:**

| Feature | Radius | Margin to edge (2.25 − r) | Required | Result |
|---|---|---|---|---|
| Drill, Ø2.7 clearance | 1.35 | 0.90 mm | ≥1.0 mm (hole to board edge, typical JLC guideline) | FAIL |
| Annular pad / keepout, Ø5.0 | 2.50 | **−0.25 mm** (overhangs the edge) | ≥0.3 mm | FAIL |
| Socket head, ISO 4762 (dk 4.5) | 2.25 | 0.00 mm | ≥0.25 mm, so the head does not overhang or catch the enclosure | FAIL |
| Button head, ISO 7380 (dk ≈4.7) | ≈2.35 | −0.10 mm | — | FAIL |
| Pan head, ISO 7045 (dk 5.0) | 2.50 | −0.25 mm | — | FAIL |
| Hex standoff, 5 mm across flats (≈5.77 mm across corners) | 2.89 | **−0.64 mm** | — | FAIL |

**What the failure causes:**
- The pad copper gets cut by the router, which exposes copper or burrs at the edge.
- The standoff overhangs the board outline, so it collides with enclosure walls.
- The thin 0.9 mm web cracks under screw torque.
- No standard M2.5 fastener head fits inside the outline.

**Corrective action:**
1. Move the hole centers to **3.5 mm from each edge**. This matches the CM4-form-factor module hole pattern, so module, carrier, and standoffs stack coaxially. It is also the most likely intent behind "CM4 form factor".
   - Drill wall to edge becomes 2.15 mm.
   - Ø5.0 pad edge becomes 1.0 mm from the edge.
   - Hex standoff corner becomes about 0.6 mm from the edge.
   - All of these pass.
2. If the mechanical constraint truly forces 2.25 mm, then do all of the following:
   - Use an NPTH hole with no pad.
   - Pull the copper keepout to Ø4.0.
   - Specify a low-profile M2.5 head no larger than Ø4.0, such as DIN 7984 or a wafer head.
   - Use **round** standoffs no larger than Ø4.0.
   - Accept the 0.9 mm web, with a waiver from the fab.
3. Add a courtyard and keepout in the footprint so DRC flags this condition automatically.

---

## 2. Copper Pour Clearance to Edge Cut (≥0.3 mm)

- **At the mounting holes: FAIL.** This follows directly from Rule 1, since the pad crosses the board edge.
- **Elsewhere: UNVERIFIED.** No layout was supplied.

**Corrective action and required settings (KiCad):**
- Board Setup → Constraints → *Copper to edge clearance*: **0.3 mm** minimum. I recommend **0.5 mm** if the board will be panelized with V-score, since V-cut needs about 0.4 mm or more.
- Zone properties should not override the edge clearance. Re-pour all zones and run DRC with "Test for parity between PCB and schematic" enabled.
- Check these specific areas, which most often violate the rule:
  - Connector shells and through-hole tabs at the edge (USB, HDMI). Edge-mounted connectors may intentionally sit at the edge, but their *pour* must not.
  - GND stitching vias along the outline. Keep each via annulus at least 0.3 mm from the edge.
  - Inner-layer planes on a 4-layer build. Pull them back ≥0.5 mm to avoid exposed copper or delamination when routed.
- Optional: add a GND guard ring with via fence at a 0.5 mm inset for EMI. This must not be applied at the mounting holes if they stay at 2.25 mm.

---

## 3. SY8089 Thermal Vias — FAIL (rule mis-specified, plus thermal risk)

**Problem with the rule:** The SY8089 in SOT23-5 has **no exposed pad**. "Thermal via stitching on the package" therefore has no pad to land on. The heat path runs through the leads, mainly GND (pin 2) and the SW and IN copper, into the surrounding copper area.

**Thermal sanity check (provide your actual operating point):**
- Assume a 5 V to 3.3 V conversion at 2 A with about 90% efficiency. Losses are then roughly 0.7 W.
- SOT23-5 θJA is typically in the high-100s to above 200 °C/W on JEDEC boards. With a good copper spread it may drop to about 100–130 °C/W.
- That gives a temperature rise of roughly 70–100 °C. At an ambient of 40 °C or more, the part will approach thermal shutdown.
- **The full 2 A rating is not usable continuously in this package without derating.**

**Corrective action:**
1. Rewrite the rule as follows: *"GND pin 