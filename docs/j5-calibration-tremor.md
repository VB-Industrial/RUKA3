# J5 manual calibration tremor capture

User prepared manual stages1–4; man_cal5 was issued under observation.
Three asynchronous samples were collected in MoveToBStart(state23), error0.

| elapsed s | raw output | motor angle rad | motor velocity rad/s | measured native Iq A | diagnostic requested Iq A |
|---:|---:|---:|---:|---:|---:|
|0|10336|0.18010|0.423|-0.335|-0.393|
|0.034|10356|0.18559|-0.128|1.138|1.000|
|0.089|10428|0.17428|-2.343|-0.799|-1.000|

The speed guard exceeded1rad/s; arm0 was sent and acknowledged immediately.
The procedure was aborted intentionally, with no table save. Motor electrical
calibration was not modified. These asynchronous sparse samples demonstrate
current-command reversal/saturation but do not establish why measured velocity
oscillates. Do not infer instantaneous velocity from adjacent angle samples or
claim electrical offset valid from this alone. Further diagnosis should separate
native current/angle alignment from the calibration velocity feedback loop.

## History and constant-current probe

The saved chat contains the earlier loaded-J5 failure: after increasing the
calibration limit0.4→1A it could not leave the edge, then accelerated after a
manual push. Work moved to free-spinningJ6; there was no confirmed resolution
for loadedJ5. A separate older motor-calibration EEPROM-address problem was
also discussed, but is not evidence of the current electrical offset being bad.

A direct-current probe was explicitly authorized. Normal arm was initially
blocked by Aborted (blocks_normal_control includes failed/aborted states), so
J5 was reset to load saved legacy limits. No EEPROM writes were performed.
At fixed public current+0.4A for approximately0.15s, measured Iq was0.400–0.411A,
raw output changed10434→10425, public motor angle
increased0.000745rad, while reported motor velocity ranged+0.017 to-0.475rad/s.
The raw output direction was consistent with positive public motion. A+0.7A
probe was aborted at its first sample for estimated speed-1.863rad/s, measured
Iq0.629A. All exit paths sent arm0 and restored Kp200/Ki0. Asynchronous samples
cannot prove an incorrect instantaneous velocity, but the mismatch and previous
current-command reversals make calibration-speed feedback the next investigation.
Electrical calibration and native FOC were not modified. Current tracking alone
does not establish correct rotor electrical alignment or full useful torque.

## One-second constant-current test

User placed J5 centrally and authorized a1s motion pulse. Fixed public Iq+0.4A
was requested for1s;35 asynchronous pulse samples span0.999s. Speed-only guard
was removed for this diagnostic; displacement/current/fault guards remained.
No additional current amplitude was used. Test ended with arm0, Kp200, Ki0.

BeforePWM: reported velocity-0.0126..+0.0172rad/s, motor-angle peak-to-peak
0.000092rad. DuringPWM: reported velocity-3.1326..+2.6970rad/s, motor-angle
peak-to-peak0.02410rad, external raw peak-to-peak40/65536turn (0.003835rad).
Current mostly followed0.4A, sampled peak0.505A. Last pulse sample coincided
with command timeout/stop and measured nearzero; exclude it for steady current
statistics. PWM-off Iq values can be stale. Output did not execute a sustained
large travel. Plot and JSON are in diagnostics/calibration-tremor.

Thus calibration velocity-loop feedback is not the sole origin of fluctuations:
substantial velocity oscillation remains under fixed current without that loop.
Next distinguish raw motor encoder behaviour from estimator/current-loop effects
underPWM before adding smoothing or retuning the calibration velocity regulator.
Electrical offset and native FOC have not been modified.

## Raw encoder diagnostics prepared

encoder_diag retains12values; previously unused fields6/7/8/11 now expose motor
ticks before offset/LUT, corrected rotor angle before observer, observer electrical
angle, and motorCPR. Both board adapters implement this contract; control math
is unchanged. G431J5 and G474J4 builds and17host-tests passed. OnlyJ5 was flashed.
CAN updater reported flash_complete105280paddedbytes, but node105 did not return;
Pi remained ERROR-ACTIVE with rx/tx0. No new current pulse was applied. SWD onJ5
was requested to distinguish boot-vector/transport completion failure from an
application startup issue before repeating the experiment. New raw signals have
not yet been measured on hardware. See register-names.md for the exact layout.

## Confirmed motor LUT problem and RAM-only bypass

SWD read of calibration_data at0x200031b4 (8204bytes) found a valid header,
was_calibrated1, encoder_inverted0, electrical offset-298, TYPE_ID89ABCDEF.
2048motor correction entries ranged-806..2127ticks.382/2047adjacent corrected
steps were non-positive; largest adjacent correction jump573ticks. For example,
near bins775..780 correction changed-36→1134. Applying this loaded table to
logged raw motor ticks reproduces the logged pre-observer angle (within integer
truncation/float error). After software reset the entire8204bytes were identical,
confirming the same data reloads from EEPROM. No EEPROM writes were performed.

User explicitly approved RAM-only LUT bypass. GDB verified PWMoff and the
lookup_table pointer0x200031c0 at motor0x20000fa4 +offset0xa0, then set it toNULL.
The electrical offset remained-298. A0.4A fixed-current test was stopped by the
0.35rad displacement guard after~0.12s: motor angle0.146→0.635rad, external
angle-0.511→-0.097rad, motor speed0.586→8.586rad/s and measured Iq0.341..0.405A.
Motion was strong/accelerating, so the requested1s was intentionally not completed.
Both public angles increased: an initial suspicion of direction disagreement
from raw output ticks alone was incorrect; no signs were changed.

Final state: arm0, Kp200, Ki0, LUT bypass remains in RAM only; reset restores
stored faulty LUT. Native current-control math/offset and EEPROM were unchanged.
Do not resume full man_cal before deciding how to repair/regenerate motor LUT.

The diagnostic firmware initially exposed a register-handler stack overflow,
confirmed by HardFaultSP overlappingBSS. Removed a redundant ParameterValue
copy and prevented LTO inlining across shared read/write and G431 board-write
handlers. Codec frame fell1476→316bytes; firmware was recovered via SWD.
All17tests and both board builds passed; arm and new diagnostic reads exercised
onJ5. Debugger stack watermark showed deep usage; no permanent canary was added.

## Unresolved end-of-calibration incident

User reported vibration/weak torque duringman_cal5, then an impact against a
mechanical stop and heating; user sent arm0 and centred the drive. No active
motion was captured by the observer: it saw Complete/PWMoff and later a restart.
Readback after user stop confirmed arm0/PWMoff, Iq approximately0.001A, faults0.
The stored table being valid is not proof of correct motion or safe completion.

Code review confirmed a blocking-order defect: Saving calls EEPROM storage
before action.disable_driver is processed, so the last current target can persist
during the blocking write. A host reproducer asserted that storage sees outputs
off and failed on the current implementation. It is archived separately as
build/g431-joint5/diagnostics/saving-order/ruka3-calibration-saving-regression.cpp
with the failure log; it is NOT claimed fixed by the settings-only update.
The experimental regression was removed from the default service test so the
coefficient-only build's18passingtests are not a claim about Saving safety.
No firmware completion/hold/disarm policy was changed in that update.
