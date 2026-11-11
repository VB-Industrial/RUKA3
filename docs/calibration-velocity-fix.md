# Calibration velocity control: pending J6 validation

The shared register handlers used HAL_GetTick() for manual calibration commands
and backlash calibration, whereas G431 updates the calibration service using
millis_32(). These clocks have independent origins. All shared calibration entry
points now use joint_uptime_ms(), supplied by each board from its service-loop
clock. boot_diag uptime uses this clock as well.

The calibration current slew previously delayed reductions and braking as well
as acceleration. Motoring current buildup remains limited to 1 A/s; reductions
and current opposing the measured velocity apply immediately within the existing
per-joint current limit. Electrical calibration, native FOC, EEPROM data, CAN
configuration and current limits are unchanged by this correction.

Validation: 17 host tests pass, including manual command expiry, zero-speed
braking, immediate overspeed braking and soft motoring startup for J4/J5/J6.
G431 J6 and G474 J4 firmware builds pass.

Hardware validation on J6:

- Raspberry became available after power-up. Before flashing, feedback and
  register clocks differed by about 2 s; after flashing, their sampled timestamps
  differed by at most 4 ms.
- CAN application update completed, 107348 padded bytes. Only J6 was flashed.
- Short manual-stage jogs now produce current and motion. Commands were reduced
  after initial motion, with displacement/current/speed guards and arm=0 cleanup.
- At requested +/-0.288 rad/s, observed speed extrema were -0.302/+0.324 rad/s
  for the negative run and -0.193/+0.355 rad/s for the positive run. Motor-angle
  changes across recorded samples were -0.0196/+0.0199 rad. Peak measured Iq
  was 0.299/0.320 A. Samples are asynchronous register reads, not simultaneous
  high-rate captures. A transient oscillation remains after reducing the target;
  these short tests do not establish sustained calibration-speed stability.
- The initial +0.479 rad/s test peaked at 0.671 rad/s and 0.359 A, then settled.
- Final CAN error count and active faults were zero; arm=0. No zero/limits/LUT
  were captured or saved; EEPROM save count stayed zero. Native motor calibration
  remains valid. J5 was explicitly disarmed and not moved or flashed.

Next: tune and validate sustained calibration velocity on free-spinning J6 before
attempting a complete encoder sweep or returning to loaded J5.

J5 was subsequently updated via CAN after the user reported buzzing without
motion during manual calibration. Its deployed register/service clocks differed
by approximately9.8s, confirming the older timestamp bug was still present there.
The current shared application was rebuilt forJ5 and flashed with per-frameACK,
flash_complete104998paddedbytes. After update the sampled clocks agreed within
10ms, PWMoff, arm0, active/latched faults0, can_err0; native motor calibration
valid. Calibration settings are1A and gain2A/(rad/s). No calibration or motion
was initiated by the agent. Unsaved points of the aborted attempt were lost on
reset; the old commissioned legacy zero/limits reloaded as expected. User must
repeat manualA/middle/B capture. The loaded J5 motion/torque problem is not yet
claimed resolved; verify actual current during the user's next attempt.
