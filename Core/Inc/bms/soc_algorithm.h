#ifndef SOC_ALGORITHM_H
#define SOC_ALGORITHM_H
#include <stdint.h>
#include <stdbool.h>
#include "battery_structs.h"
void  Soc_Init(SocSohData_t *p_soc_soh, float initial_soc_pct, float soh_pct);
float Soc_Update(SocSohData_t *p_soc_soh, float terminal_voltage_mv, float current_a, float dt_s);
void  Soc_InitFromOcv(SocSohData_t *p_soc_soh, float ocv_mv);
void  Soc_OnFullChargeCycleComplete(SocSohData_t *p_soc_soh, float charged_ah);
float Soc_GetPercent(const SocSohData_t *p_soc_soh);
float Soc_GetSohPercent(const SocSohData_t *p_soc_soh);
#endif
