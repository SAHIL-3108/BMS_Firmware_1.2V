#ifndef CELL_BALANCING_H
#define CELL_BALANCING_H
#include <stdint.h>
#include "battery_structs.h"
void     Balance_Init(void);
void     Balance_Run(BatteryPack_t *p_pack);
void     Balance_DisableAllCells(BatteryPack_t *p_pack);
uint32_t Balance_GetActiveCellCount(void);
uint32_t Balance_GetCellBalanceTimeMs(uint8_t module_idx, uint8_t cell_idx);
#endif
