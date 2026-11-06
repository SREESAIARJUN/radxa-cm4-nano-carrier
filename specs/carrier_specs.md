# Radxa CM4 Nano Carrier Board (Exact 55mm x 40mm Form Factor)

## 1. Mechanical Constraints
- Board Dimensions: Exactly 55.0 mm x 40.0 mm (0.0mm overhang beyond the CM4 module).
- Board Thickness: 1.6 mm, 4-layer or 6-layer FR4 (controlled impedance stackup).
- Mounting Holes: 4x M2.5 non-plated holes located at:
  - Hole 1: (x=2.25 mm, y=2.25 mm)
  - Hole 2: (x=52.75 mm, y=2.25 mm)
  - Hole 3: (x=2.25 mm, y=37.75 mm)
  - Hole 4: (x=52.75 mm, y=37.75 mm)
  - Hole pitch: 50.5 mm (X) x 35.5 mm (Y)

## 2. High-Density Board-to-Board Interface
- Receptacles: 2x Hirose DF40C-100DS-0.4V (0.4mm pitch, 100 pins each = 200 pins total).
- Connector Pitch between rows: 14.8 mm center-to-center.
- Connector Height: 1.5mm to 3.0mm mating height.

## 3. Power Distribution Network (PDN)
- Primary DC In: USB Type-C connector (5V @ 3.0A).
  - CC1 & CC2 pins tied to GND via independent 5.1kΩ ±1% pulldown resistors (ensuring 5V power from USB-C PD power supplies).
- Onboard Regulators:
  - 3.3V System Rail: High-efficiency synchronous step-down buck converter (e.g. Silergy SY8089AAAC, 2A continuous, 1.5MHz).
    - **Rev 13 correction:** the SY8089AAAC switches at **1.0 MHz typ** (Silergy AN_SY8089/SY8089A Rev 0.9A, Electrical Characteristics, FOSC), not 1.5 MHz. All ripple/inductor math uses 1.0 MHz with a +/-20 % corner. Thermal limit (SOT23-5, thetaJA 170 C/W, TJ <= 125 C): 1.44 A continuous at TA 70 C, so the carrier 3V3 load budget is 1.2 A (see tools/buck_3v3.py calc).
  - 1.8V VDD_IO Rail: Low-dropout linear regulator or secondary buck for low-noise MIPI/ADC rails.
- Power Sequencing: CM4 requires stable +5V rail; Global_EN and RUN pin control circuit.

## 4. Peripheral Breakouts (Within 55mm x 40mm Envelope)
- USB 2.0 / OTG: High-speed differential pair (D+/D-) routed with 90Ω differential impedance to USB-C receptacle with USBLC6-2SC6 ESD protection diode.
- Gigabit Ethernet:
  - RGMII or 1000BASE-T differential lanes (TRD0± to TRD3±, 100Ω differential) routed to ultra-low-profile vertical RJ45 or vertical 8-pin spring connector / high-density header.
- Video: Micro HDMI (Type D) or Mini HDMI connector with HDMI 2.0 100Ω differential pairs (TMDS D0±, D1±, D2±, CLK±) and TPD4E05U06 ESD array.
- Storage: MicroSD card slot (push-pull type) connected to SDMMC interface with 33Ω series termination and 0.1uF bypass capacitors.
- PCIe 2.0 x1: High-speed differential pairs (PCIe_TX±, PCIe_RX±, PCIe_CLK±, 85Ω / 100Ω differential) broken out to FPC connector.
- Camera / Display: 15-pin 1.0mm pitch FPC connector for 2-lane MIPI CSI camera.
- User Controls:
  - Reset / Recovery button (MaskROM recovery mode).
  - Status LEDs: Green (PWR 3.3V), Red (ACT / Disk activity).
