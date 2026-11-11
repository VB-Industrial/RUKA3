# J5 motor recalibration without an absolute-zero seek

Scope: motor electrical offset and rotor-encoder LUT only. External encoder
man_cal, output zero/limits and CAN configuration are not changed.

Changes in the G431 calibration routine:

- Removed return_to_zero, which rotated until raw motor ticks approached0.
  Electrical phase0 now aligns locally; bounded sweeps remain one rotor turn
  forward/back (approximately36 output degrees at gear10), plus small overshoots.
- Disabled the previous motor LUT while sampling.
- LUT generation and runtime now use the same raw-encoder index. Generation
  previously indexed raw-offset while runtime indexed raw.
- Offset samples are averaged modulo the electrical pole period to avoid averaging
  equivalent pole positions into a half-period phase error. Verification keeps
  the existing10%pole-period tolerance. Offset-point settling increased to100ms
  and initial alignment to500ms; no absolute-position seek was reintroduced.
- Before curve sampling, select the equivalent offset branch nearest the rotor
  position, keeping arbitrary full-pole displacement out of the correction curve.
- Verification uses its own sample array, not the first pass's array.
- Missing curve coverage and nonmonotonic LUT return failure; do_calibrate does
  not overwrite EEPROM on failure.

Tests:18host-tests passed, including electrical-period wrap/branch averaging and
LUT monotonicity. Cross-build passed. An initial packed-array const-reference
introduced an8KB stack temporary; that trial was stopped before persistence,
old EEPROM data verified identical. Validation now uses the existing aligned
array pointer; compiled FOC::calibrate local frame388bytes instead of8580.

A subsequent run rejected electrical verification before LUT generation and
kept the old record. SWD captured verification errors roughly-652..-861ticks
with average offset-777, motivating the pole-period averaging correction.
The updated procedure is under hardware validation; completion results below.

## Hardware validation so far

After electrical-period averaging and longer settling, electrical verification
passed: captured errors mostly4..51ticks, with one-31tick sample, within the
unchanged117tick bound. The generated curve was-37..58ticks, but had3nonmonotonic
steps near bins1713..1715 (-4,-14,-11 corrected ticks). It was rejected before
EEPROM write. A GDB breakpoint mapped to the common false return source line,
so its displayed line185 alone did NOT mean electrical verification failed.
The nonzero candidate LUT and valid verification samples establish the later
LUT validation failure.

Replaced the9tap block FIR/mask implementation with zero-phase triangular
smoothing over33bins, using the existing immutable source buffer. This removes
448bytes of static FIR buffers, reduces compiled calibration local frame to340B,
and preserves the final monotonicity check.18tests and the cross-build passed.
This build was written throughSWD, verifyOK. The next calibration request was
blocked BEFORE motor_calib because pose-1.0124rad did not meet the preflight
0.9rad margin inside legacy hard limits[-1.8312,1.9367]. User was asked to place
J5centrally. No new curve has yet been saved; motor EEPROM still contains the
old faulty table. Calibration remains incomplete until the final pass succeeds.

## Final successful run

The initial strict legacy-limit preflight still reported-1.012rad after the user
confirmed physical centering. For the expressly user-confirmed centred run, only
the host script's absolute-position preflight was bypassed; firmware limits for
normal commands were not changed. The bounded rotor calibration was then run.

Calibration completed and saved successfully: was_calibrated1, offset5561,
TYPE_ID89ABCDEF. Selecting an equivalent pole branch changes the numeric offset;
this is not a large commanded physical move. The2048entry LUT is-35..23ticks,
all2048periodic corrected steps7..9ticks (zero nonmonotonic transitions). After
reboot, all8204bytes matched the successful in-memory record exactly, SHA256
5c4fd5bf34ac8c1b888dab4b197d6c3defbffa7a58f86de13c1b985af323c62a.

A small0.15A verification pulse produced consistent positive acceleration:
reported velocities0.196,0.947,1.135,1.515,1.956rad/s with measuredIq0.132..0.148A.
The pulse was stopped early by the0.08rad displacement envelope, rather than
finishing the nominal0.25s; asynchronous monitoring recorded0.126rad of motor
movement across the sampled interval. No oscillatory sign reversals like those
with the old LUT appeared in this short test. It does not establish full-range
positioning accuracy or validate loaded external man_cal.

Final verification: node105online, arm0/PWM0, fault_act/latch0, can_err0,
motorcalibvalidtrue. External cal_data and limits matched the pre-run readback.
No external zero/limit/calibration commands were issued. Kp200/Kd0/Ki0 restored.
Both G431targets build; onlyJ5 flashed. A source-specific compiler frame limit
(1024B) now rejects accidental large local calibration copies. J5hex remained
byte-identical after rebuilding with this diagnostic guard.18host-tests passed.
