# RUKA3: plan of the unified BLDC firmware

## Hardware mapping

| Joints | MCU | Firmware baseline | Role |
| --- | --- | --- | --- |
| 1--4 | STM32G474 | `bldc_joint` | Lower BLDC joints |
| 5--6 | STM32G431 | `VBDrive` | Upper BLDC joints |

One repository supplies both targets, but each target produces its own binary.
Joint selection is a build-time setting; calibration and runtime configuration
remain per device.

## Sources and boundaries

* `bldc_joint` supplies the G474 CubeMX platform, FOC/six-step control,
  `libvoltbro`, Cyphal transport and the current joint-calibration work.
* `VBDrive` supplies only the G431 board support and reusable motor, encoder,
  EEPROM, calibration and bootloader-integration code.  It is not a RUKA3
  dependency and its UART-centric application is not copied as-is.
* `RUKA2` supplies the externally visible behavior: Cyphal subjects,
  register names and semantics, safety behavior and the single robot-wide
  configuration model.  Its old TMC5160 motor implementation is not reused.
* `VBBoot` is pinned as `ThirdParty/VBBoot` and provides CAN-FD application
  updates.  Its G474 build options must be used for joints 1--4.

## Target layout

```
Boards/g474/     # STM32G474 board and application target, joints 1--4
Boards/g431/     # STM32G431 board and application target, joints 5--6
App/             # shared control, protocol and configuration (next step)
ThirdParty/VBBoot
```

The shared `App/` layer will own `robot_config.h`, the common joint state
machine and Cyphal protocol.  Board folders own CubeMX-generated code,
peripheral wiring and board-specific motor backends.

## Implementation order

1. Make both board CMake targets reproducible and select a joint with
   `RUKA3_JOINT_INDEX` (1--4 for G474; 5--6 for G431).
2. Introduce a single typed `robot_config.h` table: node IDs 101--106,
   Cyphal command and feedback subject IDs, gear ratios, signs, encoder type,
   limits and controller tuning.  Preserve known G474 calibration values;
   leave upper-joint values explicitly uncalibrated rather than guessing.
3. Extract the common `JointController` state machine:
   `boot -> unconfigured/calibrating -> ready -> armed -> fault`.
   Implement `LowerDrive` and `UpperDrive` behind the same interface.
4. Port the RUKA2 Cyphal contract first: heartbeat/GetInfo/register access,
   servo position, direct velocity, feedback and calibration/arm controls.
5. Port calibration storage to a versioned, CRC-protected record.  A missing
   or incompatible record must prevent arming.
6. Add the observer as a separate encoder-fusion module.  First publish its
   diagnostics and apply only bounded correction; make fault-on-mismatch an
   opt-in, tested policy.
7. Build bootloader + application images.  Application start is `0x08003000`.
   G431 ends at `0x08020000`; G474 uses VBBoot's G474 settings and its
   `0x08080000` flash boundary.
8. Add VS Code tasks for configure/build/full image/SWD flash/CAN flash and
   CAN monitoring.  CAN flashing calls the uploader shipped with VBBoot.

## Acceptance checks

* Both targets compile from a clean build directory.
* Each joint index resolves only to the compatible MCU target.
* The resulting application and full images pass address-range validation.
* A MoveIt/RUKA2 controller can command and receive feedback from all six
  nodes without per-board protocol branching.
* Invalid calibration or a hardware fault always leaves phase drive disabled.

## First flashing workflow

The first flash uses a `*_full.hex` image over SWD; it contains VBBoot and the
application.  Later CAN updates use the application-only `.hex`, never the
full image.

* G474: `build/g474-jointN/g474_full.hex` for SWD and
  `build/g474-jointN/g474.hex` for CAN.
* G431: `build/g431-jointN/VBDrive_full.hex` for SWD and
  `build/g431-jointN/VBDrive.hex` for CAN.

G431 builds generate Cyphal types from DSDL.  Install either `nnvg` or `uv`
before using the VS Code task; for example, `uv tool install nunavut==2.3.1`.
VBBoot uses the default command CAN ID `0x444`, so first CAN programming must
be done one controller at a time until per-node bootloader configuration is
added.
