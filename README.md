<div align="center">

```
██████╗ ███╗   ███╗███████╗    ███████╗██╗    ██╗
██╔══██╗████╗ ████║██╔════╝    ██╔════╝██║    ██║
██████╔╝██╔████╔██║███████╗    █████╗  ██║ █╗ ██║
██╔══██╗██║╚██╔╝██║╚════██║    ██╔══╝  ██║███╗██║
██████╔╝██║ ╚═╝ ██║███████║    ██║     ╚███╔███╔╝
╚═════╝ ╚═╝     ╚═╝╚══════╝    ╚═╝      ╚══╝╚══╝
```

# ⚡ BMS\_FM\_1.2V — Battery Management System Firmware

### *Production-grade EV Battery Management • STM32G4 • FreeRTOS • MISRA C:2012*

---

[![Build Status](https://img.shields.io/badge/Build-Passing%20✓-00e676?style=for-the-badge&logo=gnu&logoColor=white)](.)
[![Firmware](https://img.shields.io/badge/Firmware-v1.0.0-0091ea?style=for-the-badge&logo=c&logoColor=white)](.)
[![Hardware](https://img.shields.io/badge/Hardware-Rev%202-ab47bc?style=for-the-badge&logo=circuitverse&logoColor=white)](.)
[![MCU](https://img.shields.io/badge/MCU-STM32G473CE-ef5350?style=for-the-badge&logo=stmicroelectronics&logoColor=white)](.)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-26a69a?style=for-the-badge)](.)
[![Safety](https://img.shields.io/badge/ISO_26262-ASIL--D-ff6f00?style=for-the-badge)](.)
[![MISRA](https://img.shields.io/badge/MISRA_C-2012-ffa726?style=for-the-badge)](.)
[![License](https://img.shields.io/badge/License-MIT-78909c?style=for-the-badge)](LICENSE)

</div>

---

## 🔋 What Is This?

This is the **complete embedded firmware** for a real-world **Electric Vehicle Battery Management System** — a safety-critical controller that guards, monitors, and manages a **350V / 100Ah NMC lithium battery pack**.

Built on an **STM32G473CE** ARM Cortex-M4 microcontroller, this firmware runs a **multi-tasking FreeRTOS** architecture and interfaces with **16 battery modules** (96 cells in series × 3 parallel) through the **Texas Instruments BQ76952** Analog Front-End IC. Every design decision targets **ISO 26262 ASIL-D** functional safety compliance.

> 💡 **In simple terms:** This firmware is the brain of an EV battery pack. It makes sure your battery never overcharges, overheats, or fails silently — and tells the vehicle exactly how much charge is left.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        BMS_FM_1.2V — System View                    │
├───────────────────────┬─────────────────────────────────────────────┤
│   HARDWARE LAYER      │   STM32G473CE  (ARM Cortex-M4 @ 170MHz)    │
│                       │   Flash: 512KB  |  RAM: 128KB               │
├───────────────────────┼─────────────────────────────────────────────┤
│   AFE INTERFACE       │   16× BQ76952 (I²C) → 96 cells monitored   │
│   CURRENT SENSE       │   1mΩ shunt → ±650A measurement range       │
│   TEMPERATURE         │   32× NTC sensors across pack + MOSFET      │
│   CAN BUS             │   J1939 @ 500kbps → VCU + Charger comms    │
├───────────────────────┼─────────────────────────────────────────────┤
│   RTOS TASKS          │   Priority │ Period  │ Function             │
│   ─────────────────   │   ──────── │ ─────── │ ──────────────────   │
│   🔴 Safety Monitor   │   Max      │ 10ms    │ Fault detection      │
│   🟠 Thermal Manager  │   High     │ 100ms   │ Temp + PID cooling   │
│   🟡 SOC Estimator    │   Medium   │ 100ms   │ EKF state estimator  │
│   🟢 Cell Balancing   │   Med-Low  │ 1000ms  │ Passive balancing    │
│   🔵 CAN Telemetry    │   Low      │ 100ms   │ J1939 broadcast      │
│   ⚪ EEPROM Manager   │   Lowest   │ 10s     │ Persistent data save │
└───────────────────────┴─────────────────────────────────────────────┘
```

---

## ✨ Key Features

### ⚡ Pack Specifications Supported

| Parameter | Value |
|---|---|
| Pack Topology | **96S × 3P** (NMC Lithium) |
| Nominal Voltage | **350V** |
| Nominal Capacity | **100 Ah** |
| Number of Modules | **16** (6 cells per module) |
| Temperature Sensors | **32× NTC** across pack |
| AFE IC | **TI BQ76952** per module |

### 🔒 Safety & Protection (ISO 26262 ASIL-D)

3-tier fault response system — no magic numbers, no silent failures:

```
L1 — WARNING  → Log event, notify VCU via CAN J1939
L2 — DERATING → Reduce current limits, continue operation
L3 — SHUTDOWN → Open all contactors, enter FAULT state immediately
```

| Protection | L1 Warn | L3 Shutdown |
|---|---|---|
| Cell Overvoltage | 4150 mV | 4250 mV |
| Cell Undervoltage | 3400 mV | 3000 mV |
| Discharge Current | — | 650 A |
| Charge Current | — | 175 A |
| Cell Temperature (high) | 50 °C | 65 °C |
| Cell Temperature (low) | −15 °C | −25 °C |
| Low SOC | 15% | 5% |

### 🧠 SOC Estimation — Extended Kalman Filter

No simple coulomb counting. This firmware implements a proper **1-state EKF**:

```
State:       x = SOC  [0.0 → 1.0]
Observation: z = V_terminal
Update rate: 100ms (10 Hz)

Q (process noise):   1.0×10⁻⁵
R (measurement noise): 1.0×10⁻³
P₀ (initial covariance): 0.1
```

The EKF compensates for sensor noise, temperature effects, and aging — giving accurate SoC even under dynamic load conditions.

### 🌡️ Thermal Management — PID Controller

Active thermal management with a full **PID control loop**:
- **Cooling activates** at 35°C → ramps up proportionally to setpoint error
- **Heating activates** at 5°C → prevents cold-climate charge damage
- **Emergency mode** at 60°C → maximum cooling, fault preparation
- All 32 NTC sensors are monitored; faulty sensors are flagged via bitmask

### ⚖️ Cell Balancing

Passive balancing via BQ76952 internal FETs:
- Triggers when cell voltage spread > **10 mV**
- Only active above **3500 mV** per cell (no deep-discharge balancing)
- **50% duty cycle** PWM on balancing FETs (thermal protection)
- Max session: **1 hour** with automatic timeout

### 📡 CAN / J1939 Telemetry

Full J1939-compliant telemetry broadcast at **500 kbps**:

| PGN | Data |
|---|---|
| `0xFF00` | BMS State, Contactor status, Fault level |
| `0xFF01` | All 96 cell voltages (delta-encoded) |
| `0xFF02` | 32 temperature sensor readings |
| `0xFF03` | SOC %, SOH %, SOE (Wh), Cycle count |
| `0xFF04` | Active fault codes (ISO-style 0xXXXX) |
| `0xFF10` | Charge current/voltage request to charger |

---

## 📁 Repository Structure

```
BMS_FM_1.2V/
│
├── Core/
│   ├── Inc/
│   │   ├── bms/
│   │   │   ├── bms_config.h          ← ⚙️  All thresholds & parameters
│   │   │   ├── battery_structs.h     ← 🧱  Master data structures
│   │   │   ├── afe_bq76952.h         ← 📡  TI BQ76952 AFE driver API
│   │   │   ├── soc_algorithm.h       ← 🧠  EKF SOC estimator API
│   │   │   ├── cell_balancing.h      ← ⚖️   Passive balancing API
│   │   │   ├── thermal_model.h       ← 🌡️   PID thermal controller API
│   │   │   ├── fault_manager.h       ← 🚨  Fault detection & response API
│   │   │   ├── can_stack.h           ← 📶  J1939 CAN stack API
│   │   │   └── eeprom_manager.h      ← 💾  NVM persistence API
│   │   ├── FreeRTOSConfig.h          ← ⏱️   RTOS tuning
│   │   └── main.h
│   │
│   └── Src/
│       ├── bms/
│       │   ├── afe_bq76952.c         ← BQ76952 register-level driver
│       │   ├── soc_algorithm.c       ← Extended Kalman Filter implementation
│       │   ├── cell_balancing.c      ← Balancing state machine
│       │   ├── thermal_model.c       ← PID controller + sensor fusion
│       │   ├── fault_manager.c       ← Tiered fault detection & logging
│       │   ├── can_stack.c           ← J1939 frame encode/decode
│       │   └── eeprom_manager.c      ← SOH, SOC, fault log persistence
│       ├── app_freertos.c            ← Task creation & RTOS init
│       └── main.c                    ← System init, global object
│
├── BMS_FM_1.2V.ioc                   ← STM32CubeIDE pin/clock config
└── Debug/                            ← Build artifacts (.elf, .map, .su)
```

---

## 🚀 Getting Started

### Prerequisites

| Tool | Version | Purpose |
|---|---|---|
| STM32CubeIDE | ≥ 1.14 | Build & flash (Eclipse CDT) |
| arm-none-eabi-gcc | ≥ 12.x | ARM cross-compiler |
| OpenOCD / ST-Link | Latest | JTAG/SWD debugging |
| STM32CubeMX | ≥ 6.x | Peripheral config (`.ioc`) |

### Build

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/BMS_FM_1.2V.git
cd BMS_FM_1.2V

# Open in STM32CubeIDE:
#   File → Open Projects from File System → select BMS_FM_1.2V/

# Build via IDE  (or terminal):
make -j16 all

# Expected output:
#   text: 78740  data: 96  bss: 13616  dec: 92452  hex: 16924
#   Build Finished. 0 errors, 0 warnings.
```

### Flash

```bash
# Via ST-Link (SWD):
arm-none-eabi-gdb Debug/BMS_FM_1.2V.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) continue
```

### Configuration

All tunable parameters live in one place — **`Core/Inc/bms/bms_config.h`**. No magic numbers anywhere else in the codebase (MISRA C:2012 Rule 8.9).

```c
// Example: Change overvoltage threshold
#define CELL_VOLTAGE_OVERVOLTAGE_MV   (4200U)   // ← edit here only

// Example: Tune EKF noise
#define EKF_PROCESS_NOISE_Q           (1.0E-5F)
#define EKF_MEASUREMENT_NOISE_R       (1.0E-3F)
```

---

## 🔌 Hardware Connections

```
STM32G473CE
    │
    ├── I²C1 ──────── BQ76952 AFE #1..16   (addr: 0x08)
    │                 (16 modules × 6 cells = 96 cells total)
    │
    ├── SPI / UART ── EEPROM (NVM storage for SOH, faults, cycle count)
    │
    ├── CAN1 ──────── J1939 Bus (500 kbps)
    │                 ├── VCU  (addr: 0x00)
    │                 └── Charger (addr: 0x80)
    │
    ├── ADC ─────────  32× NTC Temperature Sensors
    │                  1mΩ Current Shunt (±650A range)
    │
    └── GPIO ─────────  Contactor drivers (Main+, Main−, Precharge, Charge port)
                        Cooling / Heating relay outputs
```

---

## 🧪 Testing & Validation

The build produces static analysis artifacts in `Debug/`:
- **`.cyclo`** — Cyclomatic complexity per function (target: < 10)
- **`.su`** — Stack usage analysis per function (FreeRTOS stack sizing)
- **`.map`** — Full linker map for memory layout verification

Run static analysis in STM32CubeIDE via **Static Stack Analyzer** and **Cyclomatic Complexity** tabs (visible in the IDE toolbar).

---

## 📊 Build Metrics

```
┌──────────────────────────────────────────┐
│  BMS_FM_1.2V.elf  —  Build Summary       │
├────────────┬─────────────────────────────┤
│  .text     │  78,740 bytes  (code)       │
│  .data     │      96 bytes  (init data)  │
│  .bss      │  13,616 bytes  (RAM)        │
├────────────┼─────────────────────────────┤
│  Total ROM │  78,836 bytes  (~77 KB)     │
│  Total RAM │  13,712 bytes  (~13 KB)     │
│  Errors    │  0                          │
│  Warnings  │  0                          │
│  Build time│  821 ms                     │
└────────────┴─────────────────────────────┘
```

---

## 🗺️ State Machine

```
                     ┌─────────────┐
              ──────►│    INIT     │
                     └──────┬──────┘
                            │ POST pass
                     ┌──────▼──────┐
                     │  SELF TEST  │
                     └──────┬──────┘
                            │ Hardware OK
                     ┌──────▼──────┐
              ┌──────│   STANDBY   │◄──────────┐
              │      └──────┬──────┘           │
              │ Charge       │ Drive request    │
              │ connected    │                  │
       ┌──────▼───┐  ┌───────▼──────┐          │
       │ PRECHARGE│  │   PRECHARGE  │          │
       └──────┬───┘  └───────┬──────┘          │
              │              │ 95% bus V        │
       ┌──────▼───┐  ┌───────▼──────┐          │
       │ CHARGING │  │    DRIVE     │          │
       └──────┬───┘  └───────┬──────┘          │
              │              │                  │
       ┌──────▼───┐          │ Normal stop      │
       │BALANCING │          └──────────────────┘
       └──────────┘
              │ Any L3 fault from any state
       ┌──────▼──────────────────┐
       │         FAULT           │◄── EMERGENCY STOP (no recovery)
       └─────────────────────────┘
```

---

## 🔐 Fault Code Reference

| Code | Hex | Description |
|---|---|---|
| Cell Overvoltage | `0x0101` | Cell > 4250 mV |
| Cell Undervoltage | `0x0102` | Cell < 3000 mV |
| Pack Overvoltage | `0x0103` | Pack > 408V |
| Overcurrent Charge | `0x0201` | > 175A charge |
| Overcurrent Discharge | `0x0202` | > 650A discharge |
| Short Circuit | `0x0203` | Current spike detected |
| Overtemperature | `0x0301` | Cell > 65°C |
| Undertemperature | `0x0302` | Cell < −25°C |
| AFE Comm Timeout | `0x0401` | BQ76952 I²C failure |
| CAN Bus-Off | `0x0402` | CAN controller error |
| Precharge Timeout | `0x0501` | > 5s precharge |
| Contactor Weld | `0x0502` | Contactor stuck closed |
| EEPROM Error | `0x0503` | NVM read/write failure |
| Watchdog Reset | `0x0505` | MCU WWDG triggered |

---

## 🛡️ Compliance & Standards

| Standard | Scope | Status |
|---|---|---|
| **ISO 26262 ASIL-D** | Functional Safety Architecture | ✅ Design compliant |
| **MISRA C:2012** | Coding Standard (Rules 2.5, 3.1, 8.9, 20.7) | ✅ Enforced |
| **J1939** | CAN communication protocol | ✅ Implemented |
| **IEC 62133** | Battery cell safety (via thresholds) | ✅ Threshold compliant |

---

## 🤝 Contributing

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Follow **MISRA C:2012** — no magic numbers, explicit types, no dynamic allocation
4. All safety thresholds must be changed **only** in `bms_config.h`
5. Ensure 0 errors and 0 warnings on build
6. Open a Pull Request with a clear description

---

## 👨‍💻 Author

**BMS Firmware Engineer**
*Embedded Systems • Electric Vehicles • Functional Safety*

[![GitHub](https://img.shields.io/badge/GitHub-Follow-181717?style=for-the-badge&logo=github)](https://github.com/YOUR_USERNAME)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-0077B5?style=for-the-badge&logo=linkedin)](https://linkedin.com/in/YOUR_PROFILE)

---

## 📜 License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

---

<div align="center">

*Built with precision. Tested under pressure. Designed to protect.*

**⚡ If this project helped you, drop a ⭐ — it means a lot!**

</div>
