# zero_set: EEPROM readiness and register stack

The command is an empty read (`y r 106 zero_set`). It returns 1 only after a
successful save; on failure it returns 0 and preserves the previous zero.

Storage previously repeated address-only readiness probes immediately, including
before calling load, which performed its own probes. The STM32 HAL can return
BUSY between adjacent probes while the previous STOP is completing. Load now
uses a single readiness result; save uses the readiness result from loading the
old slots. No retries or peripheral resets were introduced.

After removing the redundant probes, the first hardware zero_set hung J6.
Stack pressure was reduced: zero calibration now updates just zero fields in
place, restoring those fields and sequence on failure, rather than copying the
whole table. The save-record buffer is static because all storage operations are
serialized in the application loop. The compiled register-handler frame fell
from 1580 to 1188 bytes, and the nested save frame lost a 560-byte record.
Stack overflow is a suspected cause of the hang, not proven by a fault capture.

All 17 host tests passed, including a regression for consecutive readiness probes
and zero/sequence/table preservation on save failure. Both target builds passed.
Only J6 was flashed via CAN. Hardware zero_set then returned 1, eeprom_diag was
[1,3,2,2,1,1], angle approximately -0.0000213 rad, CAN error count zero.
After a software reset, slot B/sequence 2 loaded; the angle remained within
0.0001 rad of zero. Table metadata and span were unchanged, motor calibration
valid, active faults and node CAN errors zero. Final state arm=0. No motion
commands were issued during this validation. J5 was not flashed or commanded.
