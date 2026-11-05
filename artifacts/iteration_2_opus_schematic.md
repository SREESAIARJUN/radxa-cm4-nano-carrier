# Carrier Board for a CM4-Form-Factor Module: Engineering Design Package (Rev 2.0, Preliminary)

**Scope:** 55.0 × 40.0 mm carrier for a DF40-based compute module (Radxa CM3/CM5 or RPi CM4 family)
**Status:** Preliminary. Sections marked ⚠ need verification before release.

---

## 0. Specification Problems to Resolve Before Layout

These issues block a correct design. Settle them first.

| # | Issue | Why It Matters | Recommendation |
|---|---|---|---|
| 0.1 | **No released Radxa "CM4 Nano" module datasheet is cited.** Radxa's DF40-based modules (CM3, CM3I, CM5) are footprint-compatible with RPi CM4 but differ in pin functions: one HDMI instead of two, different PCIe/SATA muxing, different camera lanes, different power-sequencing pins. | A hand-typed or memory-sourced 200-pin map will contain errors. | Generate the netlist from the vendor's schematic symbol (KiCad/Altium library from Radxa's hardware docs) and diff it against the table in §1. |
| 0.2 | **Mounting holes don't match the module.** CM4-format modules have holes inset 3.5 mm, on a 48.0 × 33.0 mm pitch. The spec places holes at 2.25 mm inset (50.5 × 35.5 pitch). | The module cannot be screwed to the carrier through these holes. | Keep the 4× 2.25 mm holes for enclosure mounting **and** add 4× M2.5 holes at (3.5, 3.5), (51.5, 3.5), (51.5, 36.5), (3.5, 36.5) for the module standoffs. Check that the two sets of keepouts don't collide (they are