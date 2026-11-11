# J6 post-calibration checks

The externally initiated manual calibration completed with state Complete,
256 points, encoder span 60371 ticks, reference span 57929 ticks, backlash 97
reference ticks. Motor calibration remains valid.

Before the test, EEPROM diagnostics reported connected=0 and last_save_ok=0,
although an earlier table save succeeded (sequence=1, save_count=1). After a
software reset, EEPROM connected=1, slot A was valid at sequence 1, the same
cal_data and hard/soft limits loaded, and the angle at the unchanged physical
pose remained approximately 0.0855 rad. Thus the committed calibration survived
reset; the separate failed write remains unexplained. No write was retried and
no zero or calibration command was invoked during these tests.

Five set_pos commands were exercised with Kd=Ki=0. Goals were relative to the
initial angle except the final zero goal. Each was observed for two seconds.

| Kp | Goal rad | Final rad | Error rad | Peak measured Iq A |
|---:|---:|---:|---:|---:|
|50|0.15551|0.12262|-0.03289|0.199|
|50|0.01551|0.03233|0.01682|0.206|
|200|0.11551|0.10778|-0.00773|0.238|
|200|0.05551|0.06188|0.00637|0.206|
|200|0|0.00516|0.00516|0.197|

No active faults or node CAN errors occurred. EEPROM save count stayed zero
since reset. Final state: arm=0, Kp=200, Kd=Ki=0. J5 was not commanded.
The test establishes functional small-angle positioning and persistence, not
full-range accuracy, precise settling performance or calibration linearity.
