# Control modes and set‑point paths

## Summary
How YARP control modes map to CiA‑402 operation modes and which targets are used, with brief notes on conversions.

## Scope
- Audience: users and developers issuing commands to the device
- Covers: mode mapping, which RxPDO target is written, where feedback comes from
- Excludes: protocol addresses (see protocol_map.md) and encoder mounting (see dual_encoder_handling.md)

## Quick reference

| YARP mode         | CiA‑402 op (6060) | Target written        | Feedback read                 | Notes                                  |
|-------------------|-------------------|-----------------------|-------------------------------|----------------------------------------|
| POSITION          | 1 (PP)            | Target position       | Position actual               | Uses 0x6081/0x6083 (profile params)    |
| POSITION_DIRECT   | 8 (CSP)           | Target position (cyc) | Position actual               | **Not Implemented**                 |
| VELOCITY          | 9 (CSV)           | Target velocity       | Velocity actual               | Units converted to rpm on loop shaft    |
| TORQUE            | 10 (CST)          | Target torque        | (optional) torque actual      | Joint Nm → motor Nm → per‑thousand     |
| CURRENT           | 10 (CST)          | Target torque        | (optional) torque actual      | A → motor Nm via torque const → per‑thousand |
| IDLE / FORCE_IDLE | 0                 | —                     | Statusword                    | Power stage disabled / reset            |

See protocol_map.md for exact object indices and data types.

## Usage by mode

- CSV (velocity):
  - velocityMove() writes the velocity target each cycle.
  - Joint deg/s are transformed to rpm on the loop shaft.
  - Feedback: velocity actual.

- CST (torque/current):
  - setRefTorque() or setRefCurrent() writes the torque target each cycle.
  - Torque path: joint Nm → motor Nm → per‑thousand of rated motor torque.
  - Current path: A → motor Nm via torque constant → per‑thousand.
  - On CST flavor or mode change, set‑points/latches are cleared to avoid stale outputs.

- PP (profile position):
  - positionMove()/relativeMove() write target position on demand.
  - Profile params set via ref speed/accel; completion via Statusword “target reached”.

- CSP (position direct):
  - Not Implemented.

## Conversions
- Unit formulas (counts↔deg, rpm↔deg/s, Nm↔per‑thousand) and gear‑ratio/mount transforms are listed in protocol_map.md and dual_encoder_handling.md.

## Related
- protocol_map.md — PDO/SDO reference and conversions
- dual_encoder_handling.md — encoder mounts, feedback selection, and shaft transforms
