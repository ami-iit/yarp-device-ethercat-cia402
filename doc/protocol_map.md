# EtherCAT protocol map (PDOs and SDOs)

## Summary
Concise reference of the EtherCAT objects the device uses, where they are mapped, and how they are interpreted. Use this as the single source for indices, types, and meanings.

## Quick reference

### RxPDO (master → slave)

| Name               | Index:Sub | Type   | Used in                 | Notes                                                |
|--------------------|-----------|--------|-------------------------|------------------------------------------------------|
| Controlword        | 0x6040:00 | UINT16 | CiA‑402 state machine   | Transitions (shutdown/switch on/enabled)             |
| Modes of operation | 0x6060:00 | INT8   | Op‑mode command         | 0=no mode, 1=PP, 8=CSP, 9=CSV, 10=CST                |
| Target torque      | 0x6071:00 | INT16  | CST (torque/current)    | Per‑thousand of rated motor torque (see 0x6076)      |
| Target position    | 0x607A:00 | INT32  | PP/CSP                  | Counts on loop shaft                                 |
| Target velocity    | 0x60FF:00 | INT32  | CSV                     | rpm on loop shaft                                    |

Notes:
- Only the target relevant to the active op‑mode is written.
- PP profiles use SDOs 0x6081 (velocity) and 0x6083 (acceleration).

### TxPDO (slave → master)

| Name                        | Index:Sub | Type   | Used in               | Notes                          |
|-----------------------------|-----------|--------|-----------------------|--------------------------------|
| Statusword                  | 0x6041:00 | UINT16 | CiA‑402 status        | Enabled/fault/target reached   |
| OpMode display              | 0x6061:00 | INT8   | Diagnostics           | Current op‑mode                |
| Position actual value       | 0x6064:00 | INT32  | Feedback (position)   | Counts                         |
| Velocity actual value       | 0x606C:00 | INT32  | Feedback (velocity)   | rpm                            |
| STO status (vendor)         | 0x6621:01 | UINT8  | Safety                | Safe Torque Off bit            |
| SBC status (vendor)         | 0x6621:02 | UINT8  | Safety                | Safe Brake Control bit         |
| Enc1 position (vendor)      | 0x2111:02 | INT32  | Feedback (position)   | If mapped                      |
| Enc1 velocity (vendor)      | 0x2111:03 | INT32  | Feedback (velocity)   | rpm, if mapped                 |
| Enc2 position (vendor)      | 0x2113:02 | INT32  | Feedback (position)   | If mapped                      |
| Enc2 velocity (vendor)      | 0x2113:03 | INT32  | Feedback (velocity)   | rpm, if mapped                 |

Notes:
- Additional entries may roll into 0x1A01/0x1A02.
- On non‑Synapticon drives, 0x211x/0x6621 may be unavailable.

### SDOs (init & configuration)

| Name                          | Index:Sub | Type    | Dir.  | Purpose                             | Values/Notes               |
|-------------------------------|-----------|---------|-------|-------------------------------------|----------------------------|
| Position loop source (vendor) | 0x2012:09 | UINT8   | Read  | Which encoder feeds position loop   | 1=Enc1, 2=Enc2             |
| Velocity loop source (vendor) | 0x2011:05 | UINT8   | Read  | Which encoder feeds velocity loop   | 1=Enc1, 2=Enc2             |
| Gear ratio numerator          | 0x6091:01 | UINT32  | Read  | Motor:Load gear ratio (numerator)   | Defaults to 1               |
| Gear ratio denominator        | 0x6091:02 | UINT32  | Read  | Motor:Load gear ratio (denominator) | Defaults to 1               |
| Profile velocity              | 0x6081:00 | INT32   | R/W   | PP profile parameter                | Units per drive             |
| Profile acceleration          | 0x6083:00 | INT32   | R/W   | PP profile parameter                | Units per drive             |
| Velocity feedback filter type | 0x2021:01 | UINT8   | R/W   | Enable/disable velocity LPF         | 0=off, 1=1st order          |
| Velocity feedback cutoff      | 0x2021:02 | UINT32  | R/W   | Velocity LPF cutoff frequency       | Hz, 5–2000, default 500     |
| Rated motor torque            | 0x6076:00 | INT16/* | Read  | Scale for 0x6071                    | Nm, vendor sizing may vary  |
| Error code                    | 0x603F:00 | UINT16  | Read  | Fault diagnostics                    | Decoded in driver           |

## Usage by mode
- CSV writes 0x60FF each cycle; reads 0x606C.
- CST writes 0x6071 each cycle; torque/current converted to per‑thousand of 0x6076.
- PP writes 0x607A on demand; uses 0x6081/0x6083 for profile; checks 0x6041 "target reached".

## Conversions

| Path                | Formula                                 |
|---------------------|-----------------------------------------|
| counts → deg        | deg = counts × (360 / CPR)              |
| rpm → deg/s         | deg/s = rpm × 360 / 60                  |
| joint ↔ motor shaft | apply gear ratio and mounting sign      |
| Nm → 0x6071         | (motorNm / ratedNm) × 1000 (clamped)    |

See dual_encoder_handling.md for encoder CPR, mounting, and shaft transforms.

## Related
- modes_and_setpoints.md — control modes and set‑point paths
- dual_encoder_handling.md — encoder mounts, source selection, and transforms
