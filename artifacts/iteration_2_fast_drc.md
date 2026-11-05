# Radxa CM4 Nano Carrier Board (Cycle 2): DRC and Manufacturability Review

## Scope

I received a text description of the design, not the design data itself (Gerbers, ODB++, KiCad/Altium source, stackup, or length-tuning report). I therefore can't measure actual clearances, via locations, or trace lengths, and reporting a measured PASS would be fabricating results.

This review does three things:
1. It evaluates the rules themselves against the hardware specs. Some rules fail at the spec level, regardless of layout.
2. It flags risks that follow from the stated architecture.
3. It gives exact verification steps and DRC rule settings so you can close each item when you run your tools.

**Status key:**
- **FAIL (spec):** The rule as written is inadequate and must change.
- **OPEN:** The rule is valid but needs layout data to verify.
- **RISK:** An architectural concern that should be addressed.

## Summary Table

| # | Rule | Status | Key finding |
|---|------|--------|-------------|
| 1 | M2.5 head clearance at 2.25 mm | **FAIL (spec)** | 2.25 mm is the bare head radius, with zero tolerance. It is too small for hex standoffs and washers. |
| 2 | Copper-to-edge at 0.3 mm or more | **OPEN** (rule acceptable for routed edges) | 0.3 mm is the fab minimum. 0.4–0.5 mm is recommended, and V-score needs more. |
| 3 | SY8089 thermal vias | **FAIL (spec) / RISK** | S