/**
 * @file    battery_structs.h
 * @brief   Core BMS Data Structure Definitions
 * @version 1.0.0
 *
 * @details Defines all global data objects used across BMS firmware layers.
 *          Structures are designed for cache-alignment and MISRA compliance.
 *          All fields use fixed-width types per MISRA C:2012 Rule 4.6.
 *
 * Hierarchy:
 *   BatteryPack_t
 *     └── CellModule_t[BMS_NUM_MODULES]
 *           └── CellData_t[BMS_CELLS_PER_MODULE]
 */

#ifndef BATTERY_STRUCTS_H
#define BATTERY_STRUCTS_H

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

/* =========================================================================
 * ENUMERATION DEFINITIONS
 * ========================================================================= */

/**
 * @brief BMS Operating State Machine States
 */
typedef enum
{
    BMS_STATE_INIT              = 0U,  /**< Boot initialization                         */
    BMS_STATE_POWER_ON_SELF_TEST= 1U,  /**< POST: hardware and sensor validation         */
    BMS_STATE_STANDBY           = 2U,  /**< Idle, contactors open, monitoring active     */
    BMS_STATE_PRECHARGE         = 3U,  /**< Precharge resistor engaging DC bus           */
    BMS_STATE_DRIVE             = 4U,  /**< Main contactors closed, power delivery       */
    BMS_STATE_CHARGING          = 5U,  /**< AC/DC charger connected                     */
    BMS_STATE_BALANCING         = 6U,  /**< Active cell balancing (charger connected)    */
    BMS_STATE_FAULT             = 7U,  /**< Fault detected, safe state enforcement       */
    BMS_STATE_DEEP_SLEEP        = 8U,  /**< Ultra-low power, only wakeup logic active    */
    BMS_STATE_EMERGENCY_STOP    = 9U,  /**< Immediate contactor trip, no recovery        */
    BMS_STATE_COUNT             = 10U  /**< Sentinel — number of states                  */
} BmsState_t;

/**
 * @brief Fault severity levels per ISO 26262 tiered response
 */
typedef enum
{
    FAULT_LEVEL_NONE    = 0U,  /**< No active fault                                    */
    FAULT_LEVEL_1       = 1U,  /**< Warning: log and notify VCU, no action             */
    FAULT_LEVEL_2       = 2U,  /**< Derating: reduce current limits                    */
    FAULT_LEVEL_3       = 3U,  /**< Critical: open contactors, enter FAULT state        */
} FaultLevel_t;

/**
 * @brief Fault code enumeration — unique code per fault type
 */
typedef enum
{
    FAULT_NONE                  = 0x0000U,
    /* Voltage Faults (0x01xx) */
    FAULT_CELL_OVERVOLTAGE      = 0x0101U,
    FAULT_CELL_UNDERVOLTAGE     = 0x0102U,
    FAULT_PACK_OVERVOLTAGE      = 0x0103U,
    FAULT_PACK_UNDERVOLTAGE     = 0x0104U,
    FAULT_CELL_VOLTAGE_MISMATCH = 0x0105U,
    /* Current Faults (0x02xx) */
    FAULT_OVERCURRENT_CHARGE    = 0x0201U,
    FAULT_OVERCURRENT_DISCHARGE = 0x0202U,
    FAULT_SHORT_CIRCUIT         = 0x0203U,
    /* Temperature Faults (0x03xx) */
    FAULT_OVERTEMP_CELL         = 0x0301U,
    FAULT_UNDERTEMP_CELL        = 0x0302U,
    FAULT_OVERTEMP_MOSFET       = 0x0303U,
    FAULT_TEMP_SENSOR_OPEN      = 0x0304U,
    FAULT_TEMP_SENSOR_SHORT     = 0x0305U,
    /* Communication Faults (0x04xx) */
    FAULT_AFE_COMM_TIMEOUT      = 0x0401U,
    FAULT_CAN_BUS_OFF           = 0x0402U,
    FAULT_VCU_COMM_TIMEOUT      = 0x0403U,
    /* System Faults (0x05xx) */
    FAULT_PRECHARGE_TIMEOUT     = 0x0501U,
    FAULT_CONTACTOR_WELD        = 0x0502U,
    FAULT_EEPROM_ERROR          = 0x0503U,
    FAULT_SOC_ESTIMATE_ERROR    = 0x0504U,
    FAULT_WATCHDOG_RESET        = 0x0505U,
    FAULT_INTERNAL_ERROR        = 0x05FFU,
} FaultCode_t;

/**
 * @brief Contactor state enumeration
 */
typedef enum
{
    CONTACTOR_OPEN      = 0U,
    CONTACTOR_CLOSED    = 1U,
    CONTACTOR_WELDED    = 2U,  /**< Fault: contactor failed closed                     */
    CONTACTOR_UNKNOWN   = 3U,
} ContactorState_t;

/**
 * @brief Thermal management system operating mode
 */
typedef enum
{
    THERMAL_MODE_IDLE       = 0U,  /**< No active thermal management                  */
    THERMAL_MODE_COOLING    = 1U,  /**< Active cooling engaged                        */
    THERMAL_MODE_HEATING    = 2U,  /**< Cell heater engaged (cold climate)             */
    THERMAL_MODE_EMERGENCY  = 3U,  /**< Max cooling, fault imminent                   */
} ThermalMode_t;

/**
 * @brief Charger connection status
 */
typedef enum
{
    CHARGER_NOT_CONNECTED   = 0U,
    CHARGER_CONNECTED_AC    = 1U,
    CHARGER_CONNECTED_DC    = 2U,  /**< DC fast charge (CCS/CHAdeMO)                  */
} ChargerStatus_t;

/* =========================================================================
 * INDIVIDUAL CELL DATA STRUCTURE
 * ========================================================================= */

/**
 * @brief Per-cell measurement and status data
 */
typedef struct
{
    uint16_t    voltage_mv;         /**< Cell terminal voltage in millivolts           */
    int16_t     temperature_dc;     /**< Cell temperature in deci-Celsius (°C × 10)    */
    bool        is_balancing;       /**< True if balancing FET is active               */
    bool        is_faulty;          /**< True if cell has an active fault              */
    uint8_t     fault_count;        /**< Cumulative fault event counter                */
    uint8_t     _pad[1];            /**< Explicit padding for alignment                */
} CellData_t;

/* =========================================================================
 * MODULE DATA STRUCTURE (one AFE per module)
 * ========================================================================= */

/**
 * @brief Data for a single battery module monitored by one AFE IC
 */
typedef struct
{
    uint8_t         module_id;                          /**< Module index 0..N-1       */
    uint8_t         afe_address;                        /**< AFE I2C/SPI address       */
    bool            is_online;                          /**< AFE communication healthy */
    bool            is_balancing_active;                /**< Any cell balancing active */

    CellData_t      cells[BMS_CELLS_PER_MODULE];        /**< Per-cell data array       */

    uint16_t        module_voltage_mv;                  /**< Sum of cell voltages      */
    uint16_t        max_cell_voltage_mv;                /**< Highest cell in module    */
    uint16_t        min_cell_voltage_mv;                /**< Lowest cell in module     */
    uint16_t        cell_voltage_delta_mv;              /**< Max - Min cell voltage    */

    int16_t         max_temperature_dc;                 /**< Hottest cell deci-°C      */
    int16_t         min_temperature_dc;                 /**< Coldest cell deci-°C      */

    uint32_t        last_update_tick;                   /**< RTOS tick of last update  */
    FaultCode_t     active_fault;                       /**< Current module fault code */
} CellModule_t;

/* =========================================================================
 * EXTENDED KALMAN FILTER STATE STRUCTURE
 * ========================================================================= */

/**
 * @brief EKF internal state for SOC estimation
 * @details Implements 1-state EKF: state = SOC, observation = terminal voltage
 */
typedef struct
{
    float   soc_estimate;       /**< Current SOC estimate [0.0 - 1.0]                  */
    float   P;                  /**< Error covariance                                   */
    float   Q;                  /**< Process noise covariance                           */
    float   R;                  /**< Measurement noise covariance                       */
    float   last_current_a;     /**< Last measured current for prediction step          */
    float   dt_s;               /**< Time step in seconds                               */
    bool    is_initialized;     /**< EKF has been seeded with valid initial state       */
    uint8_t _pad[3];
} EkfState_t;

/* =========================================================================
 * SOC / SOH ESTIMATION DATA STRUCTURE
 * ========================================================================= */

/**
 * @brief SOC and SOH estimation result and state
 */
typedef struct
{
    float           soc_percent;            /**< State of Charge [0.0 - 100.0]          */
    float           soh_percent;            /**< State of Health [0.0 - 100.0]          */
    float           soe_wh;                 /**< State of Energy in Watt-hours          */
    float           remaining_range_km;     /**< Estimated range (requires VCU data)    */
    float           coulombs_accumulated;   /**< Accumulated Ah since last full charge  */
    uint32_t        cycle_count;            /**< Full charge-discharge cycle count      */
    EkfState_t      ekf;                    /**< EKF internal state                     */
    bool            is_estimate_valid;      /**< Estimate within confidence bounds      */
    uint8_t         _pad[3];
} SocSohData_t;

/* =========================================================================
 * FAULT MANAGEMENT STRUCTURE
 * ========================================================================= */

/**
 * @brief Active and historical fault tracking
 */
typedef struct
{
    FaultCode_t     active_faults[16];      /**< Ring buffer of active fault codes      */
    uint8_t         active_fault_count;     /**< Number of currently active faults      */
    FaultLevel_t    highest_severity;       /**< Highest severity level active          */

    FaultCode_t     fault_log[32];          /**< Non-volatile fault history log         */
    uint32_t        fault_log_timestamps[32]; /**< Timestamps for each logged fault     */
    uint8_t         fault_log_head;         /**< Log ring buffer write pointer          */

    uint32_t        fault_entry_tick;       /**< RTOS tick when fault state entered     */
    uint32_t        total_fault_events;     /**< Lifetime fault counter (from EEPROM)   */
} FaultManager_t;

/* =========================================================================
 * THERMAL MANAGEMENT STRUCTURE
 * ========================================================================= */

/**
 * @brief Thermal system state and PID controller data
 */
typedef struct
{
    int16_t         temperatures_dc[BMS_NUM_TEMP_SENSORS]; /**< All sensor readings    */
    int16_t         max_temperature_dc;     /**< System-wide maximum                   */
    int16_t         min_temperature_dc;     /**< System-wide minimum                   */
    int16_t         avg_temperature_dc;     /**< Average across all sensors            */

    ThermalMode_t   mode;                   /**< Current thermal operating mode         */

    /* PID Controller State */
    float           pid_setpoint_dc;        /**< Target temperature in deci-°C         */
    float           pid_error_integral;     /**< Integral accumulator                  */
    float           pid_last_error;         /**< Previous error for derivative term     */
    float           pid_output;             /**< PID output: cooling/heating duty [0,1] */

    /* PID Tuning */
    float           pid_kp;                 /**< Proportional gain                     */
    float           pid_ki;                 /**< Integral gain                         */
    float           pid_kd;                 /**< Derivative gain                       */

    uint8_t         faulty_sensor_mask;     /**< Bitmask of failed sensors             */
    uint8_t         _pad[3];
} ThermalData_t;

/* =========================================================================
 * CONTACTOR / POWER PATH STRUCTURE
 * ========================================================================= */

/**
 * @brief Contactor and power path state
 */
typedef struct
{
    ContactorState_t    main_positive;      /**< Main positive contactor state          */
    ContactorState_t    main_negative;      /**< Main negative contactor state          */
    ContactorState_t    precharge;          /**< Precharge contactor state              */
    ContactorState_t    charge_port;        /**< Charge port contactor state            */

    uint32_t            voltage_mv_bus;     /**< DC bus voltage after contactors        */
    uint32_t            precharge_start_tick; /**< Tick when precharge began            */
    bool                precharge_complete; /**< Precharge sequence finished            */
    uint8_t             _pad[3];
} PowerPath_t;

/* =========================================================================
 * CAN / COMMUNICATION STRUCTURE
 * ========================================================================= */

/**
 * @brief CAN bus and J1939 communication state
 */
typedef struct
{
    bool        is_bus_off;                 /**< CAN controller in BUS-OFF state        */
    bool        vcu_heartbeat_ok;           /**< VCU heartbeat received within timeout  */
    bool        charger_heartbeat_ok;       /**< Charger comms healthy                 */
    uint8_t     tx_error_count;
    uint8_t     rx_error_count;
    uint8_t     _pad[3];

    uint32_t    last_vcu_msg_tick;          /**< Tick of last VCU message received      */
    uint32_t    last_charger_msg_tick;      /**< Tick of last charger message           */
    uint32_t    messages_tx_total;          /**< Total CAN frames transmitted           */
    uint32_t    messages_rx_total;          /**< Total CAN frames received              */

    /* Charge control commands from charger */
    float       charger_requested_voltage_v;
    float       charger_requested_current_a;
    ChargerStatus_t charger_status;
    uint8_t     _pad2[3];
} CanCommsData_t;

/* =========================================================================
 * TOP-LEVEL BATTERY PACK STRUCTURE
 * ========================================================================= */

/**
 * @brief Master BMS data object — single global instance
 *
 * @details This structure is the single source of truth for all BMS state.
 *          Access must be controlled via mutexes in multi-task RTOS context.
 *          Write access restricted to owning task; read via getter functions.
 */
typedef struct
{
    /* Identity */
    uint32_t        serial_number;          /**< Pack serial number from EEPROM         */
    uint32_t        manufacture_date;       /**< YYYYMMDD format                        */

    /* State Machine */
    BmsState_t      state;                  /**< Current BMS operating state            */
    BmsState_t      previous_state;         /**< Last state (for transition logic)      */
    uint32_t        state_entry_tick;       /**< RTOS tick when state was entered       */

    /* Sub-modules */
    CellModule_t    modules[BMS_NUM_MODULES];  /**< Per-module data from AFEs           */
    SocSohData_t    soc_soh;               /**< SOC/SOH estimation data                */
    FaultManager_t  faults;                /**< Fault management subsystem             */
    ThermalData_t   thermal;               /**< Thermal management subsystem           */
    PowerPath_t     power_path;            /**< Contactor and bus state                */
    CanCommsData_t  comms;                 /**< CAN communication state                */

    /* Pack-level aggregates (computed from modules) */
    uint32_t        pack_voltage_mv;        /**< Total pack voltage                    */
    int32_t         pack_current_ma;        /**< Pack current (+ = discharge)          */
    int16_t         pack_max_temp_dc;       /**< Pack maximum temperature              */
    int16_t         pack_min_temp_dc;       /**< Pack minimum temperature              */
    uint16_t        max_cell_voltage_mv;    /**< Global maximum cell voltage           */
    uint16_t        min_cell_voltage_mv;    /**< Global minimum cell voltage           */
    uint16_t        cell_voltage_delta_mv;  /**< Pack-wide cell voltage spread         */

    /* Limits (can be derated by fault manager) */
    float           max_charge_current_a;   /**< Current maximum charge limit          */
    float           max_discharge_current_a;/**< Current maximum discharge limit       */

    /* Timestamps */
    uint32_t        uptime_seconds;         /**< System uptime since last power-on     */
    uint32_t        last_full_charge_tick;  /**< Tick of last 100% SOC achieved        */
} BatteryPack_t;

/* =========================================================================
 * EXTERN DECLARATION — Single global instance (defined in main.c)
 * ========================================================================= */
extern BatteryPack_t g_BatteryPack;

/* =========================================================================
 * HELPER MACROS
 * ========================================================================= */

/** @brief Convert deci-Celsius to float Celsius */
#define DECI_C_TO_FLOAT(x)      ((float)(x) * 0.1F)

/** @brief Convert float Celsius to deci-Celsius */
#define FLOAT_TO_DECI_C(x)      ((int16_t)((x) * 10.0F))

/** @brief Clamp a value between min and max */
#define CLAMP(val, lo, hi)      (((val) < (lo)) ? (lo) : (((val) > (hi)) ? (hi) : (val)))

#endif /* BATTERY_STRUCTS_H */
