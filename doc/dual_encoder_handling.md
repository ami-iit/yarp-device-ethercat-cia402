# Dual-encoder handling (CiA-402 EtherCAT device)

This document describes how **`CiA402MotionControl`** and **`EthercatManager`** manage dual encoders per axis, how EtherCAT PDOs are mapped, and how values are converted into YARP’s expected units.
It is intended as a **developer guide to the code**, with emphasis on configuration, feedback/command pipelines, and validation logic.

---

## 1) EtherCAT PDO mapping

PDO mapping is configured in `EthercatManager::configurePDOMapping`.

### RxPDO 0x1600 (Master → Slave, fixed)

| Object             | Index     | Size | Standard / Vendor | Notes                       |
| ------------------ | --------- | ---- | ----------------- | --------------------------- |
| Controlword        | 0x6040:00 | 16   | **CiA-402**       | Drive state machine control |
| Modes of operation | 0x6060:00 | 8    | **CiA-402**       | Op-mode command             |
| Target torque      | 0x6071:00 | 16   | **CiA-402**       | CST torque target           |
| Target position    | 0x607A:00 | 32   | **CiA-402**       | CSP / PP position target    |
| Target velocity    | 0x60FF:00 | 32   | **CiA-402**       | CSV velocity target         |

---

### TxPDO 0x1A00… (Slave → Master, dynamic, rollover)

| Object                             | Index     | Size | Standard / Vendor       | Notes                      |
| ---------------------------------- | --------- | ---- | ----------------------- | -------------------------- |
| **Mandatory**                      |           |      |                         |                            |
| Statusword                         | 0x6041:00 | 16   | **CiA-402**             | Drive status bits          |
| OpMode display                     | 0x6061:00 | 8    | **CiA-402**             | Current op-mode            |
| Position actual value              | 0x6064:00 | 32   | **CiA-402**             | Position (counts)          |
| Velocity actual value              | 0x606C:00 | 32   | **CiA-402**             | Velocity (rpm)             |
| Torque actual value                | 0x6077:00 | 16   | **CiA-402**             | Torque (permille of rated) |
| Position error                     | 0x6065:00 | 32   | **CiA-402**             | Internal error signal      |
| **Optional (mapped if available)** |           |      |                         |                            |
| Timestamp                          | 0x20F0:00 | 32   | **Vendor (Synapticon)** | Drive µs timestamp         |
| STO status                         | 0x6621:01 | 8    | **Vendor (Synapticon)** | Safe Torque Off bit        |
| SBC status                         | 0x6621:02 | 8    | **Vendor (Synapticon)** | Safe Brake Control bit     |
| Enc1Pos                            | 0x2111:02 | 32   | **Vendor (Synapticon)** | Encoder 1 position         |
| Enc1Vel                            | 0x2111:03 | 32   | **Vendor (Synapticon)** | Encoder 1 velocity         |
| Enc2Pos                            | 0x2113:02 | 32   | **Vendor (Synapticon)** | Encoder 2 position         |
| Enc2Vel                            | 0x2113:03 | 32   | **Vendor (Synapticon)** | Encoder 2 velocity         |

**Notes:**

* **CiA-402 standard objects** are guaranteed across compliant drives.
* **Synapticon vendor objects** extend support for dual encoders, safety, and synchronization.
* TxPDOs roll over into 0x1A01, 0x1A02… if >8 entries are required.


## 2) Encoder mounting and drive loop sources

Configuration keys parsed in `CiA402MotionControl::open`:

| Key          | Allowed values           | Meaning                        |
| ------------ | ------------------------ | ------------------------------ |
| `enc1_mount` | `motor`, `joint`         | Physical location of encoder 1 |
| `enc2_mount` | `motor`, `joint`, `none` | Physical location of encoder 2 |

Drive reports its internal loop sources via SDOs:

| Loop                 | SDO index | Values                       |
| -------------------- | --------- | ---------------------------- |
| Position loop source | 0x2012:09 | 1=Enc1, 2=Enc2, else Unknown |
| Velocity loop source | 0x2011:05 | 1=Enc1, 2=Enc2, else Unknown |

These determine how the **standard CiA-402 objects** (6064/606C) should be interpreted.

---

## 3) Feedback source selection

Configurable per axis:

| Feedback       | Config key                | Options                      |
| -------------- | ------------------------- | ---------------------------- |
| Joint position | `position_feedback_joint` | `"6064"`, `"enc1"`, `"enc2"` |
| Motor position | `position_feedback_motor` | `"6064"`, `"enc1"`, `"enc2"` |
| Joint velocity | `velocity_feedback_joint` | `"606C"`, `"enc1"`, `"enc2"` |
| Motor velocity | `velocity_feedback_motor` | `"606C"`, `"enc1"`, `"enc2"` |

**Validation (`open()`):**

* If a source = `enc1`/`enc2`, then:

  * That encoder must be **mounted** and
  * Its PDO entry must be **mapped**.
    Otherwise: `invalid config: encX not mounted/mapped`.
* If a source = `6064`/`606C`, values are interpreted according to `posLoopSrc` / `velLoopSrc`.
  If loop source is `Unknown`, fallback = Enc1 if present, else Enc2.

---

## 4) Units and conversions

* **Resolutions:**

  * Enc1 CPR → 0x2110:03
  * Enc2 CPR → 0x2112:03

* **Gear ratio:**

  * Numerator 0x6091:01, denominator 0x6091:02
  * `gearRatio = motor_revs / load_revs`
  * `gearRatioInv = 1 / gearRatio`

* **Conversions:**

| Source                   | Native                         | Conversion                                            |
| ------------------------ | ------------------------------ | ----------------------------------------------------- |
| 0x6064, Enc1Pos, Enc2Pos | counts                         | × (360 / CPR) → degrees                               |
| 0x606C, Enc1Vel, Enc2Vel | rpm                            | × 360/60 → deg/s                                      |
| 0x6077                   | permille of rated motor torque | /1000 × ratedNm → motorNm; then × gearRatio → jointNm |

---

## 5) Shaft transformations

Transformation rules applied in `readFeedback`:

| From → To     | Formula            |
| ------------- | ------------------ |
| Motor → Motor | deg (unchanged)    |
| Motor → Joint | deg × gearRatioInv |
| Joint → Motor | deg × gearRatio    |
| Joint → Joint | deg (unchanged)    |

Same for velocities.

---

## 6) Feedback pipeline

Per axis:

1. **Select source** (config).
2. **Convert raw**: counts→deg or rpm→deg/s on the encoder’s own shaft.
3. **Transform** to requested shaft (motor/joint) via gear ratio + mount.
4. **Store** into `variables` (protected by mutex).

Mermaid diagram (position feedback):

```mermaid
flowchart LR
  subgraph PDO[TxPDO raw sources]
    A[6064 counts]
    B[Enc1Pos counts]
    C[Enc2Pos counts]
  end
  subgraph Own[Convert to deg on own shaft]
    A --> A1[deg, mount=loopSrc]
    B --> B1[deg, mount=enc1_mount]
    C --> C1[deg, mount=enc2_mount]
  end
  subgraph Tgt[Transform to target shaft]
    A1 --> T1[deg on joint/motor]
    B1 --> T2[deg on joint/motor]
    C1 --> T3[deg on joint/motor]
  end
  subgraph Out[Outputs]
    T1 --> JP[jointPositions[j]]
    T2 --> MP[motorEncoders[j]]
  end
```

Fallbacks:

* Unknown loop source → prefer Enc1 if mounted.
* Accelerations → always `0.0`.
* Safety bits (STO/SBC) and timestamp handled if PDO entries are present.

---

## 7) Command pipeline

Implemented in `setSetPoints`:

* **Torque mode (CST):**

  * YARP joint torque \[Nm] → motorNm = jointNm / gearRatio
  * TargetTorque = round(motorNm / ratedMotorTorqueNm × 1000) (permille)
  * First-cycle latch sends `0`.

* **Velocity mode (CSV):**

  * YARP joint velocity \[deg/s] → shaft velocity depends on `velLoopSrc` + mount.
  * shaft\_deg\_s × 60/360 → rpm.
  * TargetVelocity = round(rpm).
  * First-cycle latch sends `0`.

---

## 8) Valid config examples

1. **Motor encoder only**

   * `enc1_mount=motor, enc2_mount=none`
   * joint feedback derived from Enc1 + gearRatioInv.

2. **Joint encoder only**

   * `enc1_mount=none, enc2_mount=joint`
   * motor feedback derived via gearRatio.

3. **Dual (classical)**

   * `enc1_mount=motor, enc2_mount=joint`
   * position\_joint=enc2, position\_motor=enc1
   * velocity\_joint=606C (if velLoopSrc=Enc2), velocity\_motor=606C or enc1.

4. **Redundant/diagnostic**

   * both mounted; expose 6064/606C for drive loops, plus direct enc1/enc2 for cross-checks.

---

## 9) Edge cases & notes

* `gearRatio=0` → invalid; conversions return 0.
* Sign conventions: assumed consistent; compensation must be external if not.
* Backlash detection (motor vs joint encoder) is out of scope.
* PDO rollover handled automatically; always check `TxView::has()` before access.

---

## 10) Object reference

| Object            | Index     | Type   | Meaning           |
| ----------------- | --------- | ------ | ----------------- |
| Position actual   | 0x6064:00 | INT32  | counts            |
| Velocity actual   | 0x606C:00 | INT32  | rpm               |
| Torque actual     | 0x6077:00 | INT16  | permille of rated |
| Rated torque      | 0x6076:00 | UINT32 | mNm               |
| Max torque %      | 0x6072:00 | UINT16 | permille of rated |
| Enc1 resolution   | 0x2110:03 | UINT32 | counts/rev        |
| Enc2 resolution   | 0x2112:03 | UINT32 | counts/rev        |
| Position loop src | 0x2012:09 | UINT8  | 1=Enc1, 2=Enc2    |
| Velocity loop src | 0x2011:05 | UINT8  | 1=Enc1, 2=Enc2    |
| STO               | 0x6621:01 | UINT8  | safety input      |
| SBC               | 0x6621:02 | UINT8  | safety input      |
| Timestamp         | 0x20F0:00 | UINT32 | µs                |

---

## 11) Troubleshooting

* **Error:** `invalid config: encX not mounted/mapped`
  → Config requested an encoder not present in PDOs. Fix XML mapping or mounts.
* **Joint/motor look identical**
  → Check gear ratio values; or both encoders are on same shaft.
* **6064/606C look wrong**
  → Verify loop source SDOs 0x2012:09 / 0x2011:05.
