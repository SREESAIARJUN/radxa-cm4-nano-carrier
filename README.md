# Radxa CM4 Nano Carrier Board (Revision 5.0)

# Radxa CM4 Nano Carrier Board, Rev 5.0: Engineering Design Document

**Status:** Preliminary. Several specification conflicts must be resolved before layout (see Section 0).

---

## 0. Design Review Findings (Read First)

These issues were found in the input specification. Each one blocks layout or puts first-pass success at risk.

| # | Severity | Issue | Recommendation |
|---|---|---|---|
| R1 | **Blocker** | **Mounting holes at 2.25 mm inset are too close to the edge.** An M2.5 hole needs a Ø2.7 mm drill and a Ø4.5–5.0 mm keep-out pad for the screw head or standoff. A Ø5.0 mm pad at a 2.25 mm inset extends 0.25 mm past the board edge. A 5 mm A/F hex standoff (corner radius ≈ 2.89 mm) overhangs by up to 0.64 mm. This violates the 0.0 mm overhang rule. | Move holes to a ≥3.0 mm inset (pad fully on-board), or use Ø4.0 mm pads with a low-profile M2.5 head. |
| R2 | **Blocker** | **Hole pattern does not match the module.** The CM4 form factor (which Radxa CM-series modules follow) uses a 48.0 × 32.0 mm hole pitch at a 3.5 mm inset (verify against the Radxa mechanical drawing). The specified 50.5 × 35.5 mm pitch will not align with the module's holes. | Adopt the module hole pattern, at least for the shared holes. |
| R3 | **High** | **Connector spacing is unverified.** I cannot confirm that the 15.0 mm centre-to-centre spacing matches the module. A mismatch prevents the module from seating. | Take the connector X/Y origins directly from the Radxa CM4 mechanical DXF. |
| R4 | **High** | **The T