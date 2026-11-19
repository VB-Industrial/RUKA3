#pragma once
#ifdef RUKA3_FOC_TRACE
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void ruka3_foc_trace_init(void);
void ruka3_foc_trace_irq_begin(void);
void ruka3_foc_trace_irq_end(void);
#ifdef __cplusplus
}
#endif
#endif
