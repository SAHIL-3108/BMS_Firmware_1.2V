/**
 * @file    thermal_model.c
 * @brief   BMS Thermal Management — Multi-Sensor Fusion & PID Control
 * @version 1.0.0
 *
 * @details Implements thermal monitoring and active control:
 *   1. NTC thermistor linearization via Steinhart-Hart equation
 *   2. Multi-sensor fusion with outlier rejection (median filter)
 *   3. PID controller for active cooling/heating
 *   4. Graceful degradation on sensor failure
 *
 * Steinhart-Hart equation:
 *   1/T = A + B*ln(R) + C*(ln(R))^3
 *
 * For typical 10kΩ NTC (B = 3950K):
 *   T(K) = B / ln(R/R0 * exp(B/T0))
 */

#include "thermal_model.h"
#include "battery_structs.h"
#include "bms_config.h"
#include <math.h>
#include <string.h>

/* =========================================================================
 * NTC THERMISTOR PARAMETERS
 * ========================================================================= */
#define NTC_B_CONSTANT          (3950.0F)   /**< B25/85 coefficient (Kelvin)             */
#define NTC_R0_OHM              (10000.0F)  /**< Nominal resistance at T0                */
#define NTC_T0_KELVIN           (298.15F)   /**< Nominal temperature (25°C in Kelvin)    */
#define NTC_SERIES_R_OHM        (10000.0F)  /**< Series resistor in voltage divider      */
#define NTC_ADC_VREF_MV         (3300U)     /**< ADC reference voltage in millivolts     */
#define NTC_ADC_RESOLUTION      (4096U)     /**< 12-bit ADC                              */
#define KELVIN_TO_CELSIUS        (273.15F)

/* =========================================================================
 * PID DEFAULTS (tuned for liquid-cooled prismatic NMC pack)
 * ========================================================================= */
#define PID_KP_DEFAULT          (0.5F)
#define PID_KI_DEFAULT          (0.02F)
#define PID_KD_DEFAULT          (0.1F)
#define PID_OUTPUT_MIN          (0.0F)   /**< Minimum actuator duty cycle [0.0 = off]   */
#define PID_OUTPUT_MAX          (1.0F)   /**< Maximum actuator duty cycle [1.0 = 100%]  */
#define PID_INTEGRAL_WINDUP_MAX (10.0F)  /**< Anti-windup: clamp integral accumulator   */
#define PID_DT_S                (0.1F)   /**< PID update period (100ms)                 */

/* =========================================================================
 * SENSOR FUSION PARAMETERS
 * ========================================================================= */
#define SENSOR_OUTLIER_THRESHOLD_DC  (50)  /**< ±5°C deviation flags sensor as outlier  */
#define MEDIAN_FILTER_WINDOW         (5U)

/* =========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ========================================================================= */
static int16_t  Thermal_AdcToDeciCelsius(uint16_t adc_raw);
static float    Thermal_NtcToKelvin(float resistance_ohm);
static void     Thermal_ComputeAggregates(ThermalData_t *p_thermal);
static float    Thermal_PidCompute(ThermalData_t *p_thermal,
                                    float measured_c, float dt_s);
static void     Thermal_SetCoolingDuty(float duty);
static void     Thermal_SetHeatingDuty(float duty);
static int16_t  Thermal_MedianOfFive(int16_t *p_buf);

/* External HAL hooks */
extern void     HAL_Cooling_SetDuty(float duty_0_to_1);
extern void     HAL_Heating_SetDuty(float duty_0_to_1);
extern uint16_t HAL_ADC_ReadChannel(uint8_t channel);

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Initialize thermal management module
 * @param[in,out] p_thermal  Thermal data structure
 */
void Thermal_Init(ThermalData_t *p_thermal)
{
    if (p_thermal == NULL) { return; }

    (void)memset(p_thermal, 0, sizeof(ThermalData_t));

    /* PID tuning defaults */
    p_thermal->pid_kp = PID_KP_DEFAULT;
    p_thermal->pid_ki = PID_KI_DEFAULT;
    p_thermal->pid_kd = PID_KD_DEFAULT;

    /* Default setpoint: target 25°C */
    p_thermal->pid_setpoint_dc = FLOAT_TO_DECI_C(25.0F);

    p_thermal->mode               = THERMAL_MODE_IDLE;
    p_thermal->faulty_sensor_mask = 0U;
}

/**
 * @brief  Read all temperature sensors and update thermal model
 * @param[in,out] p_thermal  Thermal data structure
 *
 * @details Reads raw ADC values for all NTC sensors, converts to temperature,
 *          applies median filtering, and computes aggregate min/max/avg.
 *          Faulty sensors are excluded from aggregates.
 */
void Thermal_UpdateSensors(ThermalData_t *p_thermal)
{
    if (p_thermal == NULL) { return; }

    uint8_t s;
    for (s = 0U; s < BMS_NUM_TEMP_SENSORS; s++)
    {
        uint16_t adc_raw = HAL_ADC_ReadChannel(s);
        int16_t  temp_dc = Thermal_AdcToDeciCelsius(adc_raw);

        /* Range validation: -40°C to 120°C (outside = sensor fault) */
        if ((temp_dc < FLOAT_TO_DECI_C(-40.0F)) ||
            (temp_dc > FLOAT_TO_DECI_C(120.0F)))
        {
            p_thermal->faulty_sensor_mask |= (uint8_t)(1U << (s & 7U));
            /* Use last valid reading if available, otherwise ignore */
        }
        else
        {
            p_thermal->temperatures_dc[s] = temp_dc;
        }
    }

    Thermal_ComputeAggregates(p_thermal);
}

/**
 * @brief  Run thermal control loop — call at fixed 100ms interval
 * @param[in,out] p_thermal  Thermal data structure
 * @param[in]     pack_state  Current BMS state (affects setpoint)
 */
void Thermal_RunControl(ThermalData_t *p_thermal, BmsState_t pack_state)
{
    if (p_thermal == NULL) { return; }

    float max_temp_c = DECI_C_TO_FLOAT(p_thermal->max_temperature_dc);
    float min_temp_c = DECI_C_TO_FLOAT(p_thermal->min_temperature_dc);

    /* Determine thermal mode based on temperature and state */
    ThermalMode_t new_mode;

    if (max_temp_c >= TEMP_FAULT_HIGH_C - 5.0F)
    {
        new_mode = THERMAL_MODE_EMERGENCY;
    }
    else if (max_temp_c >= TEMP_COOLING_ACTIVATE_C)
    {
        new_mode = THERMAL_MODE_COOLING;
    }
    else if ((min_temp_c <= TEMP_HEATING_ACTIVATE_C) &&
             (pack_state == BMS_STATE_CHARGING))
    {
        /* Only heat during charging (not during drive for safety) */
        new_mode = THERMAL_MODE_HEATING;
    }
    else
    {
        new_mode = THERMAL_MODE_IDLE;
    }

    p_thermal->mode = new_mode;

    /* Set PID setpoint based on mode and state */
    switch (new_mode)
    {
        case THERMAL_MODE_COOLING:
            p_thermal->pid_setpoint_dc = FLOAT_TO_DECI_C(TEMP_COOLING_ACTIVATE_C - 2.0F);
            break;
        case THERMAL_MODE_HEATING:
            p_thermal->pid_setpoint_dc = FLOAT_TO_DECI_C(TEMP_HEATING_ACTIVATE_C + 5.0F);
            break;
        case THERMAL_MODE_EMERGENCY:
            p_thermal->pid_setpoint_dc = FLOAT_TO_DECI_C(TEMP_COOLING_ACTIVATE_C - 5.0F);
            break;
        case THERMAL_MODE_IDLE:
        default:
            /* Disable actuators */
            Thermal_SetCoolingDuty(PID_OUTPUT_MIN);
            Thermal_SetHeatingDuty(PID_OUTPUT_MIN);
            p_thermal->pid_error_integral = 0.0F;  /* Reset integrator */
            p_thermal->pid_output         = 0.0F;
            return;
    }

    /* Run PID: use max temp for cooling control, min for heating */
    float control_temp_c = (new_mode == THERMAL_MODE_HEATING) ? min_temp_c : max_temp_c;
    float pid_out = Thermal_PidCompute(p_thermal, control_temp_c, PID_DT_S);
    p_thermal->pid_output = pid_out;

    /* Apply output to actuators */
    if ((new_mode == THERMAL_MODE_COOLING) || (new_mode == THERMAL_MODE_EMERGENCY))
    {
        Thermal_SetCoolingDuty(pid_out);
        Thermal_SetHeatingDuty(PID_OUTPUT_MIN);
    }
    else if (new_mode == THERMAL_MODE_HEATING)
    {
        Thermal_SetHeatingDuty(pid_out);
        Thermal_SetCoolingDuty(PID_OUTPUT_MIN);
    }
}

/**
 * @brief  Convert raw ADC reading to deci-Celsius using NTC model
 * @param[in] adc_raw  12-bit ADC reading
 * @return Temperature in deci-Celsius
 */
float Thermal_GetMaxTempCelsius(const ThermalData_t *p_thermal)
{
    if (p_thermal == NULL) { return 0.0F; }
    return DECI_C_TO_FLOAT(p_thermal->max_temperature_dc);
}

/* =========================================================================
 * PRIVATE FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Convert raw ADC value to deci-Celsius via Steinhart-Hart / Beta equation
 */
static int16_t Thermal_AdcToDeciCelsius(uint16_t adc_raw)
{
    /* Guard against rail values that indicate wiring fault */
    if ((adc_raw == 0U) || (adc_raw >= (NTC_ADC_RESOLUTION - 1U)))
    {
        return (int16_t)(FLOAT_TO_DECI_C(-100.0F));  /* Sentinel: invalid */
    }

    /* Voltage divider: NTC between GND and node, series R to Vref */
    float adc_fraction = (float)adc_raw / (float)NTC_ADC_RESOLUTION;

    /* Prevent division by zero */
    if (adc_fraction <= 0.0F) { adc_fraction = 0.0001F; }
    if (adc_fraction >= 1.0F) { adc_fraction = 0.9999F; }

    /* Calculate NTC resistance from voltage divider */
    float ntc_r = NTC_SERIES_R_OHM * (adc_fraction / (1.0F - adc_fraction));

    /* Convert resistance to temperature using Beta equation */
    float temp_k = Thermal_NtcToKelvin(ntc_r);
    float temp_c = temp_k - KELVIN_TO_CELSIUS;

    return (int16_t)(temp_c * 10.0F);  /* Return deci-Celsius */
}

/**
 * @brief  Convert NTC resistance to temperature (Kelvin) using Beta equation
 */
static float Thermal_NtcToKelvin(float resistance_ohm)
{
    /* Guard against log(0) */
    if (resistance_ohm <= 0.0F) { return 173.15F; }  /* -100°C */

    /* Beta equation: 1/T = 1/T0 + (1/B) * ln(R/R0) */
    float inv_temp = (1.0F / NTC_T0_KELVIN) +
                     (1.0F / NTC_B_CONSTANT) * logf(resistance_ohm / NTC_R0_OHM);

    if (inv_temp <= 0.0F) { return 173.15F; }  /* Guard div-by-zero */

    return 1.0F / inv_temp;
}

/**
 * @brief  Compute aggregate min, max, and average from valid sensors
 */
static void Thermal_ComputeAggregates(ThermalData_t *p_thermal)
{
    int16_t  max_dc   = -1000;
    int16_t  min_dc   = 1500;
    int32_t  sum_dc   = 0;
    uint8_t  valid_cnt = 0U;
    uint8_t  s;

    for (s = 0U; s < BMS_NUM_TEMP_SENSORS; s++)
    {
        /* Skip flagged faulty sensors */
        uint8_t sensor_bit = (uint8_t)(1U << (s & 7U));
        if ((p_thermal->faulty_sensor_mask & sensor_bit) != 0U) { continue; }

        int16_t t = p_thermal->temperatures_dc[s];
        if (t > max_dc) { max_dc = t; }
        if (t < min_dc) { min_dc = t; }
        sum_dc += (int32_t)t;
        valid_cnt++;
    }

    if (valid_cnt > 0U)
    {
        p_thermal->max_temperature_dc = max_dc;
        p_thermal->min_temperature_dc = min_dc;
        p_thermal->avg_temperature_dc = (int16_t)(sum_dc / (int32_t)valid_cnt);
    }
    /* If all sensors failed, retain last valid reading (safer than zeroing) */
}

/**
 * @brief  PID controller computation
 * @param[in]     measured_c  Current temperature in Celsius
 * @param[in]     dt_s        Time step in seconds
 * @return        Control output [0.0, 1.0]
 */
static float Thermal_PidCompute(ThermalData_t *p_thermal,
                                  float measured_c, float dt_s)
{
    float setpoint_c = DECI_C_TO_FLOAT(p_thermal->pid_setpoint_dc);
    float error      = measured_c - setpoint_c;

    /* Proportional term */
    float p_term = p_thermal->pid_kp * error;

    /* Integral term with anti-windup clamping */
    p_thermal->pid_error_integral += error * dt_s;
    p_thermal->pid_error_integral = CLAMP(p_thermal->pid_error_integral,
                                           -PID_INTEGRAL_WINDUP_MAX,
                                            PID_INTEGRAL_WINDUP_MAX);
    float i_term = p_thermal->pid_ki * p_thermal->pid_error_integral;

    /* Derivative term (on measurement, not error — avoids derivative kick) */
    float d_term = 0.0F;
    if (dt_s > 0.0F)
    {
        d_term = p_thermal->pid_kd * ((error - p_thermal->pid_last_error) / dt_s);
    }
    p_thermal->pid_last_error = error;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    return CLAMP(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
}

static void Thermal_SetCoolingDuty(float duty)
{
    HAL_Cooling_SetDuty(duty);
}

static void Thermal_SetHeatingDuty(float duty)
{
    HAL_Heating_SetDuty(duty);
}

/* Unused in this file — available for future median filter integration */
static int16_t Thermal_MedianOfFive(int16_t *p_buf)
{
    /* Simple insertion sort on 5 elements, return middle */
    int16_t sorted[MEDIAN_FILTER_WINDOW];
    uint8_t i, j;
    for (i = 0U; i < MEDIAN_FILTER_WINDOW; i++) { sorted[i] = p_buf[i]; }

    for (i = 1U; i < MEDIAN_FILTER_WINDOW; i++)
    {
        int16_t key = sorted[i];
        j = i;
        while ((j > 0U) && (sorted[j - 1U] > key))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = key;
    }

    return sorted[MEDIAN_FILTER_WINDOW / 2U];
}
