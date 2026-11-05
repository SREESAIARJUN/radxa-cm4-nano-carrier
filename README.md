# Radxa CM4 Nano Carrier Board (Revision 1.0)

# Radxa CM4-Form-Factor Nano Carrier Board, Rev 1.0: Engineering Design Document

**Status:** Draft for design review. This document is not yet release-for-fab.

---

## 0. Critical Review Findings (Read First)

Several items in the brief conflict with datasheets, physics, or each other. Each must be resolved before layout freeze. Where I made an engineering decision, it is stated here and used consistently in the rest of the document.

| # | Requirement as given | Issue | Disposition |
|---|---|---|---|
| R1 | M2.5 holes at 2.25 mm from the edges on a board with 0.0 mm overhang | An M2.5 clearance hole (Ø2.7 mm) with a Ø5.0 mm pad/keepout reaches 2.5 mm from center. The pad would extend 0.25 mm past the board edge, and the drill wall sits only 0.9 mm from the edge (below JLC's ≥1.0