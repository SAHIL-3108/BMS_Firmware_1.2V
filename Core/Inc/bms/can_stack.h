#ifndef CAN_STACK_H
#define CAN_STACK_H
#include "battery_structs.h"
void Can_Init(BatteryPack_t *p_pack);
void Can_TransmitTelemetry(const BatteryPack_t *p_pack);
void Can_ProcessRxQueue(BatteryPack_t *p_pack);
#endif
