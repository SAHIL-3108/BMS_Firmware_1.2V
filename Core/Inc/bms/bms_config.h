/**
 * @file    bms_config.h
 * @brief   BMS System Configuration & Safety Thresholds
 * @version 1.0.0
 * @date    2025
 *
 * @details Central configuration file for all BMS parameters.
 *          Adhere to MISRA C:2012 Rule 8.9 — no magic numbers in source files.
 *          All safety-critical thresholds are defined here per ISO 26262 ASIL-D.
 * MISRA C:2012 Compliance:
 *   - Rule 2.5: No unused macros
 *   - Rule 3.1: All comments are terminated
 *   - Rule 20.7: Macro parameters are parenthesized
**/

#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H


/* =========================================================================
 * SYSTEM IDENTIFICATION
 * ========================================================================= */
#define BMS_FW_VERSION_MAJOR        (1U)
#define BMS_FW_VERSION_MINOR        (0U)
#define BMS_FW_VERSION_PATCH        (0U)
#define BMS_HW_REVISION             (2U)

/* =========================================================================
 * PACK TOPOLOGY CONFIGURATION
 * ========================================================================= */
#define BMS_NUM_CELLS_SERIES        (96U)     /**< Cells in series (e.g., 96S for ~350V pack) */
#define BMS_NUM_CELLS_PARALLEL      (3U)      /**< Cells in parallel per group               */
#define BMS_NUM_MODULES             (16U)     /**< Number of battery modules                 */
#define BMS_CELLS_PER_MODULE        (6U)      /**< Cells per module (monitored by one AFE)   */
#define BMS_NUM_TEMP_SENSORS        (32U)     /**< Total NTC temperature sensors             */
#define BMS_NOMINAL_CAPACITY_AH     (100.0F)  /**< Nominal pack capacity in Ah               */
#define BMS_NOMINAL_VOLTAGE_V       (350.0F)  /**< Nominal pack voltage in Volts             */

/* =========================================================================
 * CELL VOLTAGE THRESHOLDS (Lithium NMC Chemistry)
 * ========================================================================= */
#define CELL_VOLTAGE_ABSOLUTE_MAX_MV    (4250U)  /**< L3 Fault: Hard shutdown              */
#define CELL_VOLTAGE_OVERVOLTAGE_MV     (4200U)  /**< L2 Fault: Derating above this        */
#define CELL_VOLTAGE_WARN_HIGH_MV       (4150U)  /**< L1 Fault: Warning threshold           */
#define CELL_VOLTAGE_NOMINAL_HIGH_MV    (4100U)  /**< Upper end of normal operation         */
#define CELL_VOLTAGE_NOMINAL_LOW_MV     (3600U)  /**< Lower end of normal operation         */
#define CELL_VOLTAGE_WARN_LOW_MV        (3400U)  /**< L1 Fault: Low voltage warning         */
#define CELL_VOLTAGE_UNDERVOLTAGE_MV    (3200U)  /**< L2 Fault: Derating below this         */
#define CELL_VOLTAGE_ABSOLUTE_MIN_MV    (3000U)  /**< L3 Fault: Hard shutdown               */
#define CELL_VOLTAGE_DEEP_DISCHARGE_MV  (2500U)  /**< Irreversible damage threshold         */

/* =========================================================================
 * PACK VOLTAGE THRESHOLDS
 * ========================================================================= */
#define PACK_VOLTAGE_MAX_MV     ((uint32_t)(CELL_VOLTAGE_ABSOLUTE_MAX_MV) * BMS_NUM_CELLS_SERIES)
#define PACK_VOLTAGE_MIN_MV     ((uint32_t)(CELL_VOLTAGE_ABSOLUTE_MIN_MV) * BMS_NUM_CELLS_SERIES)

/* =========================================================================
 * CURRENT THRESHOLDS
 * ========================================================================= */
#define PACK_CURRENT_CHARGE_MAX_A       (150.0F)  /**< Max continuous charge current        */
#define PACK_CURRENT_DISCHARGE_MAX_A    (300.0F)  /**< Max continuous discharge current     */
#define PACK_CURRENT_PEAK_DISCHARGE_A   (600.0F)  /**< Peak discharge (10s burst)           */
#define PACK_CURRENT_FAULT_CHARGE_A     (175.0F)  /**< L3: Overcurrent charge shutdown      */
#define PACK_CURRENT_FAULT_DISCHARGE_A  (650.0F)  /**< L3: Overcurrent discharge shutdown   */
#define PACK_CURRENT_REGEN_MAX_A        (120.0F)  /**< Max regenerative braking current     */

/* =========================================================================
 * TEMPERATURE THRESHOLDS (Celsius)
 * ========================================================================= */
#define TEMP_CHARGE_MAX_C           (45.0F)   /**< Max cell temp during charging           */
#define TEMP_CHARGE_MIN_C           (0.0F)    /**< Min cell temp for charging              */
#define TEMP_DISCHARGE_MAX_C        (60.0F)   /**< Max cell temp during discharge          */
#define TEMP_DISCHARGE_MIN_C        (-20.0F)  /**< Min cell temp for discharge             */
#define TEMP_WARN_HIGH_C            (50.0F)   /**< L1: High temp warning                  */
#define TEMP_FAULT_HIGH_C           (65.0F)   /**< L3: Overtemperature shutdown            */
#define TEMP_WARN_LOW_C             (-15.0F)  /**< L1: Low temp warning                   */
#define TEMP_FAULT_LOW_C            (-25.0F)  /**< L3: Undertemperature shutdown           */
#define TEMP_COOLING_ACTIVATE_C     (35.0F)   /**< Activate active cooling                */
#define TEMP_HEATING_ACTIVATE_C     (5.0F)    /**< Activate cell heating                  */

/* =========================================================================
 * SOC / SOH CONFIGURATION
 * ========================================================================= */
#define SOC_MAX_PERCENT             (100.0F)
#define SOC_MIN_PERCENT             (0.0F)
#define SOC_WARN_LOW_PERCENT        (15.0F)   /**< L1: Low SOC warning                    */
#define SOC_FAULT_LOW_PERCENT       (5.0F)    /**< L2: Critical low SOC                   */
#define SOH_REPLACE_THRESHOLD_PCT   (70.0F)   /**< Recommend pack replacement below 70%   */
#define SOH_WARN_THRESHOLD_PCT      (80.0F)   /**< SOH health warning threshold           */
#define SOC_INITIAL_PERCENT         (50.0F)   /**< Initial SOC estimate on cold boot       */

/* =========================================================================
 * KALMAN FILTER TUNING PARAMETERS
 * ========================================================================= */
#define EKF_PROCESS_NOISE_Q         (1.0E-5F)  /**< Process noise covariance               */
#define EKF_MEASUREMENT_NOISE_R     (1.0E-3F)  /**< Measurement noise covariance           */
#define EKF_INITIAL_COVARIANCE_P    (0.1F)     /**< Initial estimation error covariance    */
#define EKF_DT_SECONDS              (0.1F)     /**< EKF update interval (100ms)            */

/* =========================================================================
 * CELL BALANCING CONFIGURATION
 * ========================================================================= */
#define BALANCE_VOLTAGE_DIFF_MV     (10U)     /**< Trigger balancing if Vdiff > 10mV      */
#define BALANCE_MIN_CELL_MV         (3500U)   /**< Do not balance below this voltage      */
#define BALANCE_DUTY_CYCLE_PCT      (50U)     /**< Balancing FET duty cycle               */
#define BALANCE_TIMEOUT_S           (3600U)   /**< Max balancing session duration (1 hr)  */

/* =========================================================================
 * PRECHARGE CONFIGURATION
 * ========================================================================= */
#define PRECHARGE_TARGET_PCT        (95U)     /**< Precharge complete at 95% of pack V    */
#define PRECHARGE_TIMEOUT_MS        (5000U)   /**< Precharge timeout: 5 seconds           */
#define PRECHARGE_RESISTOR_OHM      (100U)    /**< Precharge resistor value               */

/* =========================================================================
 * RTOS TASK CONFIGURATION
 * ========================================================================= */
#define TASK_PRIORITY_SAFETY        (configMAX_PRIORITIES - 1U)  /**< Highest: ASIL-D    */
#define TASK_PRIORITY_THERMAL       (configMAX_PRIORITIES - 2U)  /**< High: Safety-adj.  */
#define TASK_PRIORITY_SOC           (configMAX_PRIORITIES - 3U)  /**< Medium             */
#define TASK_PRIORITY_BALANCING     (configMAX_PRIORITIES - 4U)  /**< Medium-Low         */
#define TASK_PRIORITY_CAN_TX        (configMAX_PRIORITIES - 5U)  /**< Low                */
#define TASK_PRIORITY_EEPROM        (configMAX_PRIORITIES - 6U)  /**< Lowest             */

#define TASK_PERIOD_SAFETY_MS       (10U)     /**< Safety task: 10ms (100Hz)              */
#define TASK_PERIOD_THERMAL_MS      (100U)    /**< Thermal task: 100ms (10Hz)             */
#define TASK_PERIOD_SOC_MS          (100U)    /**< SOC task: 100ms (10Hz)                 */
#define TASK_PERIOD_BALANCING_MS    (1000U)   /**< Balancing task: 1s (1Hz)               */
#define TASK_PERIOD_CAN_TX_MS       (100U)    /**< CAN telemetry: 100ms                   */
#define TASK_PERIOD_EEPROM_MS       (10000U)  /**< EEPROM save: 10s                       */

#define TASK_STACK_SAFETY_WORDS     (512U)
#define TASK_STACK_THERMAL_WORDS    (256U)
#define TASK_STACK_SOC_WORDS        (512U)
#define TASK_STACK_CAN_WORDS        (256U)

/* =========================================================================
 * CAN / J1939 CONFIGURATION
 * ========================================================================= */
#define CAN_BAUD_RATE_BPS           (500000U)  /**< 500kbps CAN 2.0B                      */
#define CAN_BMS_SOURCE_ADDRESS      (0xF4U)    /**< J1939 BMS Source Address               */
#define CAN_VCU_ADDRESS             (0x00U)    /**< Vehicle Control Unit address           */
#define CAN_CHARGER_ADDRESS         (0x80U)    /**< Charger node address                  */

/* J1939 PGN Definitions */
#define J1939_PGN_BMS_STATUS        (0xFF00U)  /**< BMS Status broadcast PGN              */
#define J1939_PGN_CELL_VOLTAGES     (0xFF01U)  /**< Cell voltage data PGN                 */
#define J1939_PGN_TEMPERATURES      (0xFF02U)  /**< Temperature data PGN                  */
#define J1939_PGN_SOC_SOH           (0xFF03U)  /**< SOC/SOH data PGN                      */
#define J1939_PGN_FAULT_STATUS      (0xFF04U)  /**< Fault codes PGN                       */
#define J1939_PGN_CHARGE_REQUEST    (0xFF10U)  /**< Charge current/voltage request PGN    */

/* =========================================================================
 * HARDWARE CONFIGURATION (AFE: BQ76952)
 * ========================================================================= */
#define AFE_I2C_ADDRESS             (0x08U)   /**< BQ76952 I2C address (ADDR pin = GND)   */
#define AFE_I2C_TIMEOUT_MS          (10U)     /**< I2C transaction timeout                */
#define AFE_ADC_RESOLUTION_UV       (190U)    /**< AFE ADC LSB in microvolts              */
#define AFE_CURRENT_SENSE_MOHM      (1U)      /**< Current sense resistor in milli-ohms   */

/* =========================================================================
 * EEPROM / NVM CONFIGURATION
 * ========================================================================= */
#define EEPROM_BASE_ADDRESS         (0x0000U)
#define EEPROM_SOH_ADDRESS          (0x0010U)
#define EEPROM_CYCLE_COUNT_ADDRESS  (0x0020U)
#define EEPROM_SOC_ADDRESS          (0x0030U)
#define EEPROM_FAULT_LOG_ADDRESS    (0x0100U)
#define EEPROM_FAULT_LOG_MAX_ENTRIES (32U)

/* =========================================================================
 * COMPILE-TIME ASSERTION HELPERS (MISRA C:2012 Rule 1.3)
 * ========================================================================= */
#define BMS_STATIC_ASSERT(cond, msg) typedef char static_assert_##msg[(cond) ? 1 : -1]

BMS_STATIC_ASSERT(BMS_NUM_CELLS_SERIES > 0U, cells_series_nonzero);
BMS_STATIC_ASSERT(BMS_NUM_MODULES > 0U, modules_nonzero);
BMS_STATIC_ASSERT(CELL_VOLTAGE_ABSOLUTE_MAX_MV > CELL_VOLTAGE_ABSOLUTE_MIN_MV, voltage_range_valid);

#endif /* BMS_CONFIG_H */
