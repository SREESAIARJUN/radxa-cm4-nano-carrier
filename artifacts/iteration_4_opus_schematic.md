# Radxa CM4 Nano Carrier Board, Rev 4.0: Engineering Design Document

**Status: Preliminary.** Section 0 lists spec conflicts that must be resolved before layout. Any value marked **⚠VERIFY** must be checked against the primary source before release. I have not invented pin assignments or distributor part numbers that I cannot stand behind, because a wrong DF40 pin-out or a wrong LCSC number produces a dead board.

---

## 0. Design Review Findings (Resolve Before Layout)

| # | Issue | Severity | Recommendation |
|---|---|---|---|
| R1 | **Mounting holes at 2.25 mm from the edge with M2.5.** A 2.7 mm drill leaves 0.9 mm of FR4 to the edge. A standard M2.5 pan head (Ø4.5 mm) or standoff (Ø5 mm hex, 5.8 mm across corners) reaches or overhangs the board edge, which violates the 0.0 mm overhang rule. The RPi-CM4 form factor uses holes at 3.5 mm inset (48 × 33 mm pitch). | **High** | Either move to 3.5 mm inset (CM4-compatible standoffs) or use M2 hardware. If 2.25 mm is mandatory, use Ø4.0 mm low-profile heads, NPTH with 0.3 mm keepout, and accept that the hole is not CM4-module-compatible. |
| R2 | **Board area equals the module area (55 × 40 mm).** The module covers the entire top side. USB-C, micro-HDMI, and RJ45 must sit on board edges, likely bottom-side, or under a module height-clearance zone. A HanRun HR911105A magjack is about 16 × 21 × 13.5 mm and consumes about 15 % of the board. | **High** | Do a mechanical feasibility pass (3D STEP) before schematic freeze. Consider a 10-pin 1.25 mm Ethernet header instead of an on-board RJ45. |
| R3 | **DF40 pitch "15.0 mm c-c".** This must match the module's plug positions exactly. I cannot confirm 15.0 mm from memory. | **High** | Take the X/Y of both connector centers and pin 1 orientation from the Radxa CM4 mechanical drawing (or RPi CM4 datasheet if pin-compatible). Do not use the spec value without confirming it. |
| R4 | **TPS22918 is a load switch, not a USB power-distribution switch.** It has no current limit, no fault flag, and no short-circuit protection meeting USB host requirements. | Medium | Use a current-limited switch, e.g. **TI TPS2553DBVR** (adjustable ILIM) or **Silergy SY6280AAC**. |
| R5 | **Micro-HDMI "FRL".** FRL (HDMI 2.1) needs SoC support plus HDMI 2.1-grade routing and ESD. Most CM4-class SoCs output TMDS (HDMI 2.0). | Medium | Confirm with the Radxa CM4 SoC datasheet. Design for TMDS 100 Ω and use ≤0.3 pF ESD arrays regardless. |
| R6 | **3.3 V @ 2 A and 1.8 V regulators may be redundant.** CM4-form-factor modules take +5 V input and export +3.3 V and +1.8 V rails to the carrier, at limited current (RPi CM4: about 300 mA each). | Info | Keep the SY8089 only if carrier 3.3 V loads (PCIe device, peripherals) exceed the module's export budget. Never back-drive the module's 3.3 V/1.8 V pins from carrier regulators. |
| R7 | **PCIe on a 55 × 40 board.** There is no room for an x1 slot. | Info | Use an M.2 Key-M/E (bottom side) or an FFC (e.g. 16-pin 0.5 mm, Pi-5 style). Place the 100 nF AC-coupling caps on TX near the module, plus a REFCLK pair. |
| R8 | **USB-C as sink only.** | Info | Place 5.1 kΩ Rd on CC1/CC2 (default 5 V). If OTG uses the same port, add a DRP controller (e.g. FUSB302) and a VBUS switch. |

---

## 1. Hirose DF40 Pin-to-Net Mapping (J1 / J2, 2 × 100 pins)

### 1.1 Approach

Radxa CM-series modules are designed to be pin-compatible with the **Raspberry Pi CM4 pin-out**. However, Radxa reassigns function-specific pins (e.g. extra USB 3.0, SATA/PCIe muxing, different GPIO alt functions), and these differ between Radxa CM3, CM3I, and CM4 SKUs.

I am deliberately **not** generating all 200 rows from memory. A single swapped differential pair or GND/+5 V error here destroys the module. Instead:

1. **Source of truth:** the Radxa CM4 hardware design guide/pin-out table and the Radxa reference IO-board schematic (KiCad/PDF from docs.radxa.com).
2. Import that table directly into the schematic symbol (CSV → KiCad symbol generator). Do not hand-type it.
3. Cross-check against the RPi CM4 datasheet Section 2 for common pins.

### 1.2 Template and Representative Rows (RPi CM4 Convention, ⚠VERIFY Against Radxa)

| Conn | Pin | Net (CM4 convention) | Net class | Carrier use |
|---|---|---|---|---|
| J1 | 1, 2 | GND | GND | Plane |
| J1 | 3 / 5 | ETH_TRD3_P / _N | DIFF100 | RJ45 magjack pair 3 |
| J1 | 4 / 6 | ETH_TRD1_P / _N | DIFF100 | Pair 1 |
| J1 | 7, 8 | GND | GND | |
| J1 | 9 / 11 | ETH_TRD2_N / _P | DIFF100 | Pair 2 |
| J1 | 10 / 12 | ETH_TRD0_N / _P | DIFF100 | Pair 0 |
| J1 | 13, 14 | GND | GND | |
| J1 | ~15–20 | ETH LEDs, PI_nLED_ACT, EEPROM_nWP | GPIO | Magjack LEDs |
| J1 | ~77–87 (odd) | **+5V_IN (module supply)** | PWR | USB-C VBUS after protection |
| J1 | ~84–88 | +3.3V / +1.8V **outputs** from module | PWR_OUT | Do not back-drive |
| J1 | — | GPIO0–27, SD/eMMC, nRPIBOOT, GLOBAL_EN, RUN_PG | GPIO | Per Radxa table |
| J2 | — | HDMI0 TMDS D0–D2, CLK, CEC, HPD, DDC | DIFF100 | Micro-HDMI |
| J2 | — | PCIe CLK_P/N, TX_P/N, RX_P/N, nRST, CLKREQ | DIFF100 | M.2/FFC |
| J2 | — | USB2_P/N (OTG), USB_OTG_ID | DIFF90 | USB-C D+/D−