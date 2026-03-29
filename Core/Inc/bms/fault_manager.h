#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H
#include <stdbool.h>
#include "battery_structs.h"
void  Fault_Init(BatteryPack_t *p_pack);
void  Fault_RunSafetyChecks(BatteryPack_t *p_pack);
void  Fault_Assert(BatteryPack_t *p_pack, FaultCode_t code);
bool  Fault_Clear(BatteryPack_t *p_pack, FaultCode_t code, bool force_clear);
bool  Fault_IsActive(const BatteryPack_t *p_pack, FaultCode_t code);
#endif
