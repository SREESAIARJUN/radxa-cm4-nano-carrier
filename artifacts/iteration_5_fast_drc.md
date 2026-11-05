# DRC Review: No Design Data Received

**No layout data was attached.** Your message references an iteration "from Claude Opus 5.5," but it contains no Gerbers, ODB++, native CAD files, placement data, or stackup. I can't issue pass/fail results on a design I can't see. Inventing component coordinates or violations would be worse than useless in a DRC sign-off.

What I can do is:
1. Flag issues in the stated requirements themselves.
2. Define the numerical pass/fail criteria the layout should be checked against.
3. List exactly what to send so I can run the review.

---

## Part 1: Issues in the Stated Requirements

### Flag A: The 2.25 mm M2.5 hole margin is likely too tight (if it means hole center to board edge)

| Parameter | Typical value | Clearance from edge with center at 2.25 mm |
|---|---|---|
| M2.5 clearance drill | 2.7 mm (r = 1.35) | 0.90 mm hole wall to edge ✅ |
| Plated pad / annular ring | 4.5–5.0 mm (r = 2.25–2.5) | 0.00 to **−0.25 mm** ❌ |
| ISO 7045 pan head / M2.5 washer | ~5.0 mm OD (r = 2.5) | **−0.25 mm (overhangs edge)** ❌ |
| ISO 7380 button head | ~4.7 mm (r = 2.35) | **−0.10 mm** ❌ |

**Recommended correction:** Clarify the intent of "2.25 mm margin":
- **If it means hole center inset:** move the centers to **≥ 3.0 mm** from both edges, giving holes at (3.0, 3.0), (52.0, 3.0), (3.0, 37.0), (52.0, 37.0). Alternatively, use an NPTH with no pad and a non-flanged standoff.
- **If it means keepout radius around the hole:** a 2.25 mm radius is smaller than most screw heads. Use **≥ 3.0 mm** radius (head radius plus 0.5 mm tool and tolerance margin) on both layers.

### Flag B: The SY8089 likely has no thermal pad to put vias under

The standard Silergy SY8089 (e.g., SY8089AAAC) is a **SOT23-5** package with **no exposed pad**. Please confirm the exact variant from your BOM. If it is SOT23-5, the requirement "thermal via array under the converter" should be rewritten.

Correct criteria for a SOT23-5 buck:
- **Heat path:** heat leaves through the leads, mainly GND (pin 2) and LX. Place **4–6 vias (0.3 mm drill / 0.6 mm pad)** in the GND copper directly adjacent to pin 2 and the C_IN/C_OUT ground returns, stitched to an internal GND plane.
- **No vias between pads under the body.** This avoids solder wicking and LX-to-GND/IN shorts at a 0.95 mm lead pitch.
- **GND pin connection: