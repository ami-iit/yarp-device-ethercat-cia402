# Dual‑encoder handling (CiA‑402 EtherCAT device)

This document explains how the device handles two encoders per axis and how data flows between EtherCAT PDOs, the CiA‑402 control loops, and the YARP interfaces implemented by `CiA402MotionControl`. It also documents the configuration options and all the valid combinations.

- Encoder 1 (Enc1): typically motor‑mounted (pre‑gearbox)
- Encoder 2 (Enc2): typically joint/load‑mounted (post‑gearbox)
- Standard CiA‑402 objects: 0x6064 Position actual value, 0x606C Velocity actual value, 0x6077 Torque actual value, 0x6041 Statusword, 0x6061 OpMode display

The code involved lives in:
- `CiA402MotionControl.cpp` (feedback selection, shaft transformations, setpoint conversions)
- `EthercatManager.cpp` (PDO mapping, TxView accessors)


## 1) EtherCAT PDO mapping summary

RxPDO 0x1600 (fixed, master → slave):
- 0x6040 Controlword (16)
- 0x6060 Modes of operation (8)
- 0x6071 Target torque (16)
- 0x607A Target position (32)
- 0x60FF Target velocity (32)

TxPDO 0x1A00.. (dynamic, slave → master):
- Mandatory: 0x6041 Statusword (16), 0x6061 OpModeDisplay (8), 0x6064 Position (32), 0x606C Velocity (32), 0x6077 Torque (16), 0x6065 Position error (32)
- Optional (mapped if present): 0x20F0 Timestamp (32), 0x6621:01 STO (8), 0x6621:02 SBC (8), vendor encoders: 0x2111:02 Enc1Pos (32), 0x2111:03 Enc1Vel (32), 0x2113:02 Enc2Pos (32), 0x2113:03 Enc2Vel (32)

Mapping is built with automatic rollover to multiple TxPDOs to keep a conservative per‑PDO entry limit. If an optional object is not mappable, it is silently skipped and the system degrades gracefully.


## 2) Encoder mounting and loop sources

- Mounts per axis (configuration):
  - `enc1_mount`: "motor" | "joint"
  - `enc2_mount`: "motor" | "joint" | "none"

- Drive internal loop sources (SDO reads):
  - Position loop source (0x2012:09): 1 = Enc1, 2 = Enc2, other = unknown
  - Velocity loop source (0x2011:05): 1 = Enc1, 2 = Enc2, other = unknown

These loop sources determine how the drive itself computes 0x6064/0x606C. If unknown, the code will prefer Enc1 heuristically when interpreting 0x6064/0x606C.


## 3) Feedback source selection (per axis)

Each consumer can independently select its source:
- Position (joint): `position_feedback_joint`: "6064" | "enc1" | "enc2"
- Position (motor): `position_feedback_motor`: "6064" | "enc1" | "enc2"
- Velocity (joint): `velocity_feedback_joint`: "606C" | "enc1" | "enc2"
- Velocity (motor): `velocity_feedback_motor`: "606C" | "enc1" | "enc2"

Availability and validation at startup:
- If a source is `enc1`/`enc2`, the corresponding PDO entry must be mapped and the encoder must be mounted (not `none`). Otherwise, initialization fails with an explicit error.
- If a source is `6064`/`606C`, values are interpreted using the loop source SDOs to know which encoder those standard objects refer to internally.


## 4) Units and raw conversions

- Encoder resolutions are read via SDOs:
  - Enc1: 0x2110:03 (counts per revolution)
  - Enc2: 0x2112:03 (counts per revolution)
- Gear ratio is read via 0x6091:01 (numerator), 0x6091:02 (denominator) and cached as:
  - `gearRatio = motor_revs / load_revs` (i.e., joint = motor × gearRatioInv)
  - `gearRatioInv = 1 / gearRatio`
- Standardized units used in YARP side:
  - Positions: degrees
  - Velocities: degrees/second
  - Torque: joint Nm
- EtherCAT/CiA‑402 native units:
  - 0x606C and vendor encoder velocities are RPM and converted deg/s via 360/60.
  - 0x6064 and vendor encoder positions are counts and converted to degrees via counts × (360 / CPR).
  - 0x6077 torque is motor side, per‑thousand of rated motor torque (0x6076 in Nm).


## 5) Shaft transformations (motor ↔ joint)

Let the mount of the measured quantity be M ∈ {motor, joint}. To express the value on a target shaft T ∈ {motor, joint}:
- Position (deg):
  - M=motor → T=motor: deg
  - M=motor → T=joint: deg × gearRatioInv
  - M=joint → T=motor: deg × gearRatio
  - M=joint → T=joint: deg
- Velocity (deg/s): same rules as position

These transformations are applied consistently in both feedback and command paths.


## 6) Feedback pipeline

Per axis, for each requested feedback (joint position, motor position, joint velocity, motor velocity):
1) Pick source according to configuration (6064/606C or enc1/enc2)
2) Convert raw value to the encoder’s own shaft and units (counts→deg, rpm→deg/s)
3) Transform to target shaft (motor/joint) using gear ratio and mount
4) Store into the `variables` snapshot protected by a mutex

Mermaid flow for position feedback (velocity analogous):

```mermaid
flowchart LR
  subgraph PDO[TxPDO sources]
    A[6064 position]\n(int32 counts)
    B[Enc1Pos 0x2111:02]\n(int32 counts)
    C[Enc2Pos 0x2113:02]\n(int32 counts)
  end
  subgraph Own[Convert to degrees on own shaft]
    A --> A1[deg, mount = posLoopSrc]
    B --> B1[deg, mount = enc1_mount]
    C --> C1[deg, mount = enc2_mount]
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

Important fallbacks:
- For 6064/606C, if loop source SDO is unknown, prefer Enc1 mount if available, otherwise Enc2; else return 0.
- Accelerations are not provided by drives; they are reported as 0.0.
- Torque feedback: `motorNm = 0x6077/1000 × ratedMotorTorqueNm`; `jointNm = motorNm × gearRatio`.
- Safety bits (STO/SBC) and timestamp are used if present in PDOs.


## 7) Command pipeline

Currently implemented setpoints:
- CST (torque): YARP gives joint torque [Nm]
  - motorNm = jointNm / gearRatio
  - 0x6071 (Target torque) = round( motorNm / ratedMotorTorqueNm × 1000 )
  - First‑cycle latch sends 0 once upon entering CST
- CSV (velocity): YARP gives joint velocity [deg/s]
  - Choose loop shaft from `velLoopSrc` (Enc1 or Enc2 mount)
  - shaft_deg_s = joint_deg_s × (gearRatio or 1) depending on mount
  - 0x60FF (Target velocity) = round( shaft_deg_s × 60/360 ) [rpm]
  - First‑cycle latch sends 0 once upon entering CSV

Mermaid flow for CSV velocity command:

```mermaid
flowchart LR
  J[YARP joint vel (deg/s)] --> D{Loop src mount?}
  D -->|motor| M[shaft_deg_s = J * gearRatio]
  D -->|joint| N[shaft_deg_s = J]
  M --> R[rpm = shaft_deg_s * 60/360]
  N --> R
  R --> TX[RxPDO 0x60FF]
```


## 8) Valid configuration patterns and examples

1) Motor encoder only
- enc1_mount = motor, enc2_mount = none
- position_feedback_joint = "6064" or "enc1"
- position_feedback_motor = "enc1"
- velocity_feedback_* = "606C" or "enc1"
- Joint values are derived from motor values via gearRatioInv

2) Joint encoder only
- enc1_mount = none, enc2_mount = joint
- position_feedback_joint = "enc2"
- position_feedback_motor = "6064" or "enc2"
- velocity_feedback_* = "606C" or "enc2"
- Motor values derived via gearRatio

3) Dual encoders, classical
- enc1_mount = motor, enc2_mount = joint
- position_feedback_joint = "enc2"
- position_feedback_motor = "enc1"
- velocity_feedback_joint = "606C" (if velLoopSrc=Enc2), velocity_feedback_motor = "606C" or "enc1"

4) Redundant/diagnostic
- Both encoders present; choose 6064/606C to follow drive internal loops while optionally exposing direct enc1/enc2 on the other channel for cross‑checking.

Validation rules at open():
- If a source is `enc1`/`enc2`, that encoder must be mounted and the corresponding PDO entry must be present; otherwise open() fails.
- For `6064`/`606C`, the code relies on loop source SDOs; if unknown, it falls back to Enc1/Enc2 heuristics.


## 9) Edge cases and notes

- gearRatio = 0 is treated as invalid; conversions return 0 to avoid divide‑by‑zero.
- Sign conventions: the implementation assumes consistent sign across encoders; if needed, sign compensation should be added at configuration or via vendor objects.
- Backlash: having both motor and joint encoders allows detecting elastic effects; this is out of scope for this driver and left to higher‑level estimation.
- PDO size/rollover: optional fields may end up in 0x1A01/0x1A02; `TxView::has()` must be checked before accessing them.


## 10) Quick reference of objects used

- Position (std): 0x6064:00 INT32, counts
- Velocity (std): 0x606C:00 INT32, rpm
- Torque (std): 0x6077:00 INT16, per‑thousand of rated
- Rated motor torque: 0x6076:00 UINT32, mNm
- Max torque percent: 0x6072:00 UINT16, permille of rated
- Enc1 resolution: 0x2110:03 UINT32, counts/rev
- Enc2 resolution: 0x2112:03 UINT32, counts/rev
- Position loop source: 0x2012:09 UINT8 (1=Enc1, 2=Enc2)
- Velocity loop source: 0x2011:05 UINT8 (1=Enc1, 2=Enc2)
- Safety: 0x6621:01 (STO), 0x6621:02 (SBC)
- Timestamp: 0x20F0:00 UINT32 µs


## 11) Troubleshooting

- “invalid config: encX not mounted/mapped”: The chosen source requires a PDO that isn’t present or an encoder marked `none` — fix mapping or mount settings.
- Joint/motor numbers look identical: Verify gear ratio SDO 0x6091 (num/den) and mounts; if both encoders are on the same shaft, values will match (modulo noise).
- 6064/606C look wrong: Check loop source SDOs 0x2012:09 and 0x2011:05 and the drive’s internal configuration.
