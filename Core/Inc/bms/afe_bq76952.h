#ifndef AFE_BQ76952_H
#define AFE_BQ76952_H
#include <stdbool.h>
#include "battery_structs.h"
bool AFE_Init(void);
bool AFE_ReadCellVoltages(uint8_t module_idx, CellModule_t *p_module);
bool AFE_ReadPackCurrent(uint8_t module_idx, int32_t *p_current_ma);
bool AFE_ReadDieTemperature(uint8_t module_idx, int16_t *p_temp_dc);
void AFE_SetBalancingFet(uint8_t module_idx, uint8_t cell_idx, bool enable);
bool AFE_ReadSafetyFlags(uint8_t module_idx, FaultCode_t *p_fault_code);
bool AFE_IsModuleOnline(uint8_t module_idx);
#endif
