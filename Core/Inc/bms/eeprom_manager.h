#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H
#include <stdint.h>
#include "battery_structs.h"
void Eeprom_LoadPackState(float *p_soc_pct, float *p_soh_pct, uint32_t *p_cycles);
void Eeprom_SavePackState(float soc_pct, float soh_pct, uint32_t cycles);
void Eeprom_SaveFaultLog(const FaultManager_t *p_fm);
#endif
