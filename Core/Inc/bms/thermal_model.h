#ifndef THERMAL_MODEL_H
#define THERMAL_MODEL_H
#include "battery_structs.h"
void  Thermal_Init(ThermalData_t *p_thermal);
void  Thermal_UpdateSensors(ThermalData_t *p_thermal);
void  Thermal_RunControl(ThermalData_t *p_thermal, BmsState_t pack_state);
float Thermal_GetMaxTempCelsius(const ThermalData_t *p_thermal);
#endif
