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
| R4 | **High** | *(row truncated by the Rev 5 edit - the complete Rev 4 findings table is preserved in git commit `7eb46b7`)* | See commit `7eb46b7`. |

---

## Revision 9.0 Engineering Changelog

**Task:** Gigabit Ethernet PHY interface: 100-ohm differential routing for TRD0..TRD3 to an ultra-low-profile vertical RJ45.

**Result:** the full 1000BASE-T MDI chain is placed, routed, and verified. It runs from the CM4 module's on-board PHY (DF40 J1 pins 3-12) through discrete magnetics (T1) to a bottom-side vertical RJ45 (J3). The routing passes 4 x 100-ohm pairs on a field-solved stackup, with intra-pair skew of 0.000 mm and zero DRC violations attributable to Rev 9. `tools/ethernet_mdi.py verify` passes **26/26** checks (`artifacts/rev9_mdi_si_report.md`).

### 9.1 Changes implemented

| File | Change |
|---|---|
| `tools/mdi_impedance.py` (new) | The single source of truth for the stackup and the MDI geometry. It contains two independent impedance models: a 2-D finite-difference Laplace field solver (solder mask, finite copper thickness, discrete Gauss-law charge) and the Hammerstad-Jensen / Kirschning-Jansen closed forms. It also has a self-test, a width solver, and a fab-tolerance corner sweep. Run `--check` to exercise all three. |
| `tools/ethernet_mdi.py` (new) | Generator and verifier for the Rev 9 stage. It writes the footprints, places T1/J3/passives, routes all 16 MDI nets plus the centre-tap, Bob Smith, and shield networks, pours the In1/In2 planes, and writes the netclass and DRU rules. The `verify` command checks SI, polarity, and mechanics, runs DRC with mutation tests, and checks ERC and parity. `render` and `schematic` are also provided. |
| `lib/radxa-cm4-nano-carrier.pretty/` | New footprints: `Pulse_H5007NL_SMD-24_P1.27mm`, `Amphenol_RJE231881317T_RJ45_Vertical_SMD`, `C/R_0402_1005Metric`, `C_1206_3216Metric_HV`, `R_1206_3216Metric`. Land patterns are taken from the manufacturer drawings via the LCSC/EasyEDA pad data. |
| `radxa-cm4-nano-carrier.kicad_pcb` | Adds the JLC04161H-7628 stackup (`setup/stackup`), T1, J3, C10-C15, and R10-R14. It adds 4 PHY-side and 4 cable-side 100-ohm pairs, In1/In2 GND planes with an ETH_SHIELD island and a 2.0 mm hi-pot split, and GND/shield stitching. Every Rev 9 item belongs to group `REV9_ETH_MDI`. |
| `radxa-cm4-nano-carrier.kicad_pro` | Adds netclass `ETH_MDI_100R` (W 0.165 / gap 0.125 / via 0.50/0.30) with patterns `Ethernet_Pair*` and `ETH_MDI*`. The netclasses of other stages are preserved. |
| `radxa-cm4-nano-carrier.kicad_dru` | Adds a Rev 9 rule block for track width, diff-pair gap, uncoupled length, length, and via count. It adds `mdi_isolation` (0.30 mm to every other net's tracks, vias, and planes), `mdi_interpair` (0.30 mm), one skew rule (max 0.15 mm) for each of the 8 pair segments, and `eth_shield_split` (2.0 mm on the inner layers). Each rule is shown to be live by a mutation test. |
| `radxa-cm4-nano-carrier.kicad_sch`, `lib/CM4Carrier.kicad_sym` | Adds T1, J3, and C10-C15 / R10-R14 symbols with global labels whose net names are read back from the board. Schematic-to-PCB parity for the Rev 9 parts is clean. |
| `fp-lib-table` | **Bug fix:** the table is now merged rather than overwritten. It registers `CM4Carrier` (used by the schematic hole symbols), which removes the Rev 7 `lib_footprint_issues` DRC errors. |
| `README.md` | **Bug fix:** closed the findings-table row R4, which an earlier edit had truncated mid-sentence. |

### 9.2 Architecture

**Signal chain:** CM4 PHY (on-module) -> J1 DF40 pins 3-12 -> T1 Pulse H5007NL (1000BASE-T, 1CT:1CT, 1500 Vrms, LCSC C6384935) -> J3 Amphenol RJE231881317T (8P8C vertical SMT, shielded, 13.1 mm, LCSC C5384839).

**Placement:** the whole top side lies under the module and is limited to 1.2 mm part height, so T1, J3, and the 1206 HV parts are on **B.Cu**. On the top side there are only eight 0402s (0.55 mm tall) at the T1 centre and cable taps. T1 sits at (34.0, 20.0) with its PHY row facing east and pin 1 at the top. J3 sits at (11.0, 20.0) with its contacts facing T1, and the plug enters from below the carrier.

**Stackup and geometry** (`mdi_impedance.py`): JLC04161H-7628, L1/L4 microstrip over solid L2/L3 GND, 7628 prepreg 0.2104 mm, er 4.4, 10 um LPI mask. The chosen geometry is **W 0.165 mm / S 0.125 mm**, solved with the field solver to **100.03 ohm** (Zodd 50.0, Zcomm 44.2, 5.52 ps/mm).

| Model | Zdiff (ohm) |
|---|---|
| FD field solver, masked (sign-off) | 100.03 |
| FD field solver, bare copper | 104.71 |
| Kirschning-Jansen closed form, bare | 111.06 (its strip-thickness correction under-predicts the side-wall coupling across a 0.125 mm gap with 35 um copper; with thin copper it agrees with the FD solver to within 1.2-1.9 %) |
| Fab corners, one at a time (etch +/-0.02, prepreg +/-10 %, er +/-0.2) | 91.5 - 108.8 (inside +/-10 %) |
| All corners aligned worst case | 88.4 - 112.3, so order JLC impedance control at +/-10 % |

**Channel map and polarity.** On the bottom side both T1 and J3 are mirrored. Their two pad rows then always meet in *reversed* pair order, whatever the rotation. Two degrees of freedom absorb this without any pair crossing:

| CM4 pair | T1 channel (top -> bottom) | P lands on | Cable net | RJ45 pins (+/-) |
|---|---|---|---|---|
| Pair3 (TRD3) | ch1 | TD1- / MX1- | ETH_MDI3 | 7 / 8 |
| Pair1 (TRD1) | ch2 | TD2- / MX2- | ETH_MDI1 | 3 / 6 |
| Pair2 (TRD2) | ch3 | TD3- / MX3- | ETH_MDI2 | 4 / 5 |
| Pair0 (TRD0) | ch4 | TD4- / MX4- | ETH_MDI0 | 1 / 2 |

The four 1CT:1CT channels are identical, and swapping *both* ends of a winding preserves end-to-end polarity. The verifier proves this from the H5007NL dot convention (`MAP-polarity`) and checks the T568 pin map from the copper (`MAP-t568`).

**PHY-side topology:**
- The four pairs leave the 0.4 mm DF40 field. Pair3 and Pair2 come from the outer row through the inter-row gap. Pair1 and Pair0 come from the inner row on F.Cu.
- They run east as nested **F.Cu** bands over the In1 GND plane, then drop south on **B.Cu** over the In2 GND plane in a 43.4-46.8 mm corridor. This corridor is co-planned with the Rev 11 HDMI J4 and the Rev 13 buck. The pairs then turn west into the T1 pads.
- Bands and drops sit over separate solid planes, so they cross freely. **Every pair changes layer exactly once**, and Pair0 uses a compact two-layer via twist.
- GND stitching vias sit beside every layer transition.

**Cable side:** the pairs run almost straight across a 4.1 mm gap, 7.5-11.6 mm per net. Pair C hops to F.Cu with *both* legs, so via counts stay balanced, and passes over pair B's diagonal leg. That leg is the unavoidable T568 3/6 straddle.

**Planes and isolation:**
- In1/In2 are GND everywhere except an **ETH_SHIELD** island (x 0-33, y 11.8-31.5) under the cable side and the jack.
- A **2.0 mm split** runs under the T1 body. This is the 802.3 1500 Vrms functional isolation, enforced by DRU `eth_shield_split`.
- The J3 shield tabs land on the island. R14 (1 Mohm) in parallel with C15 (1 nF / 2 kV, 1206) ties the island to GND and can be tuned at EMC test.
- No MDI segment crosses the split (`REF-domain`), and every MDI segment has copper directly beneath it (`REF-plane`).

**Centre taps:**
- The module PHYs are voltage-mode. Each TCT gets its own **100 nF to GND** (C10-C13, on top under T1). The taps are never shorted together or tied to a supply.
- The cable side uses **Bob Smith** termination: 4 x 75 ohm (R10-R13) to node ETH_BS, then 1 nF / 2 kV (C14) to ETH_SHIELD.

**Pipeline coexistence:**
- Other revisions edit this same board concurrently. The pipeline order is `cm4_mechanical -> ethernet_mdi -> hdmi_tmds -> buck_3v3`.
- `ethernet_mdi.py generate` is **additive and idempotent**. It replaces only its own `REV9_ETH_MDI` group, its footprints, and the Ethernet-net copper and planes, and leaves other stages' groups untouched.
- DRC violations are attributed to Rev 9 by item UUID.

### 9.3 Validation status

| Check | Result |
|---|---|
| Impedance self-test (FD vs H-J / K-J, 8 cases) | PASS, agreement within -1.1 to -1.9 % |
| Nominal Zdiff / fab corners | 100.03 ohm; 91.5-108.8 ohm |
| T568 map, end-to-end polarity, J1 pinout | PASS |
| Intra-pair skew, 8 segments (track + 1.5512 mm per via, KiCad method) | 0.000 mm on all segments (limit 0.15 mm = 0.83 ps) |
| Length per side | max 42.95 mm (limit 50). J1-to-RJ45 totals: 50.9 / 36.5 / 51.5 / 33.6 mm |
| Uncoupled length per net | 3.8-8.8 mm (limit 10). Uncoupled 0.165 mm lines are about 141.7 ohm differential, and the worst case adds -39.6 dB return loss at 100 MHz (limit -30 dB) |
| Pair-to-pair spacing | min 0.495 mm (rule 0.30) |
| Reference planes | 0 voids under MDI copper; 0 split crossings |
| kicad-cli DRC, Rev 9 items | 0 violations, 0 unrouted |
| DRU mutation tests (width, isolation, inter-pair) | 3/3 caught |
| Schematic ERC / PCB parity (Rev 9 refs) | 0 unexpected / 0 parity items |
| CM4 mechanical base (`cm4_mechanical.py verify`) | 32/32 |

**Reproduce:** run `python3 tools/mdi_impedance.py --check`, then `python3 tools/ethernet_mdi.py generate`, `schematic`, `verify`, and `render`.

### 9.4 Open items and assumptions

- **PHY centre-tap mode:** the board assumes voltage-mode drivers (BCM54210PE on RPi CM4, RTL8211F on Radxa CM3). Confirm the Radxa CM4 (RK3576) PHY against its hardware design guide. If it needs a supply on the centre taps, R/C changes at C10-C13 are enough and no reroute is needed.
- **Link/activity LEDs:** J3 has no LEDs, and `Ethernet_nLED1..3` (J1 pins 15/17/19) are left unconnected. Discrete bottom-side LEDs are deferred to a later revision.
- **"Ultra-low-profile":** 13.1 mm is the minimum for a stocked vertical 8P8C with full plug retention. The jack is on the bottom side, so the enclosure needs 13.1 mm of clearance below the carrier.
- **Schematic:** the J1/J2 module connectors are not yet schematic symbols, so the 8 PHY-side `Ethernet_PairN` global labels are expected ERC "dangling" warnings.
- **DF40 pad field:** inside the 0.4 mm-pitch field, MDI stubs keep the netclass's 0.20 mm clearance to J1 pads. The 0.30 mm `mdi_isolation` rule exempts pads by design.
