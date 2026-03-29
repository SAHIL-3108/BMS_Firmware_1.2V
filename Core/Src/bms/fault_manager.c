/**
 * @file    fault_manager.c
 * @brief   BMS Fault Management System — ISO 26262 ASIL-D Safety Logic
 * @version 1.0.0
 *
 * @details Implements a tiered fault detection, classification, and response
 *          system. All fault checks run in the highest-priority RTOS task.
 *
 *          Fault Response Matrix:
 *          ┌─────────────┬──────────────────────────────────────────────┐
 *          │ Level       │ Action                                        │
 *          ├─────────────┼──────────────────────────────────────────────┤
 *          │ LEVEL 1     │ Log event, broadcast DTC on CAN              │
 *          │ LEVEL 2     │ Level 1 + derate current limits              │
 *          │ LEVEL 3     │ Level 2 + open contactors + enter FAULT state │
 *          └─────────────┴──────────────────────────────────────────────┘
 *
 *          Latching faults require explicit clear command from VCU or service
 *          tool after root cause is resolved.
 */

#include "fault_manager.h"
#include "battery_structs.h"
#include "bms_config.h"
#include <string.h>

/* =========================================================================
 * PRIVATE TYPE DEFINITIONS
 * ========================================================================= */

/**
 * @brief Fault definition entry in the fault catalog
 */
typedef struct
{
    FaultCode_t     code;
    FaultLevel_t    level;
    bool            is_latching;    /**< True = requires manual clear                 */
    bool            is_enabled;     /**< Can be disabled for calibration              */
    uint8_t         debounce_cycles; /**< Number of consecutive checks before tripping */
    const char     *p_description;  /**< Human-readable description for logging       */
} FaultDefinition_t;

/**
 * @brief Runtime fault tracking for debounce logic
 */
typedef struct
{
    FaultCode_t code;
    uint8_t     debounce_count;  /**< Consecutive cycles fault condition was true     */
    bool        is_active;
} FaultDebounceState_t;

/* =========================================================================
 * FAULT CATALOG — All detectable faults with their properties
 * ========================================================================= */
static const FaultDefinition_t k_FaultCatalog[] =
{
    /* code,                         level,          latching, enabled, debounce, description */
    { FAULT_CELL_OVERVOLTAGE,        FAULT_LEVEL_3,  true,     true,    3U,  "Cell overvoltage"            },
    { FAULT_CELL_UNDERVOLTAGE,       FAULT_LEVEL_3,  true,     true,    3U,  "Cell undervoltage"           },
    { FAULT_PACK_OVERVOLTAGE,        FAULT_LEVEL_3,  true,     true,    2U,  "Pack overvoltage"            },
    { FAULT_PACK_UNDERVOLTAGE,       FAULT_LEVEL_3,  true,     true,    2U,  "Pack undervoltage"           },
    { FAULT_CELL_VOLTAGE_MISMATCH,   FAULT_LEVEL_1,  false,    true,    5U,  "Cell imbalance warning"      },
    { FAULT_OVERCURRENT_CHARGE,      FAULT_LEVEL_3,  true,     true,    1U,  "Overcurrent during charge"   },
    { FAULT_OVERCURRENT_DISCHARGE,   FAULT_LEVEL_3,  true,     true,    1U,  "Overcurrent during discharge"},
    { FAULT_SHORT_CIRCUIT,           FAULT_LEVEL_3,  true,     true,    0U,  "Short circuit detected"      },
    { FAULT_OVERTEMP_CELL,           FAULT_LEVEL_3,  true,     true,    3U,  "Cell overtemperature"        },
    { FAULT_UNDERTEMP_CELL,          FAULT_LEVEL_2,  false,    true,    5U,  "Cell undertemperature"       },
    { FAULT_OVERTEMP_MOSFET,         FAULT_LEVEL_3,  true,     true,    3U,  "MOSFET overtemperature"      },
    { FAULT_TEMP_SENSOR_OPEN,        FAULT_LEVEL_1,  false,    true,    3U,  "Temp sensor open circuit"    },
    { FAULT_TEMP_SENSOR_SHORT,       FAULT_LEVEL_1,  false,    true,    3U,  "Temp sensor short circuit"   },
    { FAULT_AFE_COMM_TIMEOUT,        FAULT_LEVEL_3,  true,     true,    2U,  "AFE communication lost"      },
    { FAULT_CAN_BUS_OFF,             FAULT_LEVEL_2,  false,    true,    2U,  "CAN bus-off state"           },
    { FAULT_VCU_COMM_TIMEOUT,        FAULT_LEVEL_2,  false,    true,    5U,  "VCU communication timeout"   },
    { FAULT_PRECHARGE_TIMEOUT,       FAULT_LEVEL_3,  true,     true,    0U,  "Precharge sequence timeout"  },
    { FAULT_CONTACTOR_WELD,          FAULT_LEVEL_3,  true,     true,    0U,  "Contactor weld detected"     },
    { FAULT_EEPROM_ERROR,            FAULT_LEVEL_1,  false,    true,    2U,  "EEPROM read/write error"     },
    { FAULT_SOC_ESTIMATE_ERROR,      FAULT_LEVEL_1,  false,    true,    5U,  "SOC estimate diverged"       },
    { FAULT_WATCHDOG_RESET,          FAULT_LEVEL_3,  true,     true,    0U,  "Watchdog reset occurred"     },
    { FAULT_INTERNAL_ERROR,          FAULT_LEVEL_3,  true,     true,    0U,  "Internal firmware error"     },
};

#define FAULT_CATALOG_SIZE  ((uint8_t)(sizeof(k_FaultCatalog) / sizeof(k_FaultCatalog[0])))
#define MAX_DEBOUNCE_ENTRIES (32U)

/* Debounce state array — one entry per catalogued fault */
static FaultDebounceState_t s_DebounceStates[MAX_DEBOUNCE_ENTRIES];

/* =========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ========================================================================= */
static void         Fault_CheckVoltages(BatteryPack_t *p_pack);
static void         Fault_CheckCurrents(BatteryPack_t *p_pack);
static void         Fault_CheckTemperatures(BatteryPack_t *p_pack);
static void         Fault_CheckCommunications(BatteryPack_t *p_pack);
static void         Fault_CheckContactors(BatteryPack_t *p_pack);
static void         Fault_SetFault(BatteryPack_t *p_pack, FaultCode_t code);
static void         Fault_ClearFault(BatteryPack_t *p_pack, FaultCode_t code);
static void         Fault_ExecuteResponse(BatteryPack_t *p_pack,
                                           FaultCode_t code,
                                           FaultLevel_t level);
static bool         Fault_Debounce(FaultCode_t code, bool condition_met);
static FaultLevel_t Fault_GetLevel(FaultCode_t code);
static bool         Fault_IsLatching(FaultCode_t code);
static void         Fault_LogToNvm(FaultManager_t *p_fm, FaultCode_t code);
static void         Fault_ApplyCurrentDerating(BatteryPack_t *p_pack,
                                                float derating_factor);

/* External hooks — implemented in hardware driver layer */
extern void HAL_Contactor_OpenAll(void);
extern void HAL_Contactor_TripEmergency(void);
extern void HAL_Watchdog_Kick(void);
extern uint32_t HAL_GetTickMs(void);

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * @brief Initialize fault management system
 * @param[in,out] p_pack  Pointer to global battery pack structure
 */
void Fault_Init(BatteryPack_t *p_pack)
{
    if (p_pack == NULL) { return; }

    (void)memset(&p_pack->faults, 0, sizeof(FaultManager_t));
    (void)memset(s_DebounceStates, 0, sizeof(s_DebounceStates));

    /* Initialize debounce state codes */
    uint8_t i;
    for (i = 0U; (i < FAULT_CATALOG_SIZE) && (i < MAX_DEBOUNCE_ENTRIES); i++)
    {
        s_DebounceStates[i].code             = k_FaultCatalog[i].code;
        s_DebounceStates[i].debounce_count   = 0U;
        s_DebounceStates[i].is_active        = false;
    }

    /* Default current limits (may be derated by fault manager) */
    p_pack->max_charge_current_a    = PACK_CURRENT_CHARGE_MAX_A;
    p_pack->max_discharge_current_a = PACK_CURRENT_DISCHARGE_MAX_A;
}

/**
 * @brief  Main fault detection sweep — call from highest-priority RTOS task
 * @param[in,out] p_pack  Pointer to global battery pack
 *
 * @details Runs all fault checks. Each check applies debounce logic before
 *          asserting a fault. Deterministic execution: always runs all checks.
 *          Do NOT return early — all conditions must be evaluated.
 */
void Fault_RunSafetyChecks(BatteryPack_t *p_pack)
{
    if (p_pack == NULL) { return; }

    /* Kick hardware watchdog to prove safety task is running */
    HAL_Watchdog_Kick();

    /* Run all fault checks unconditionally */
    Fault_CheckVoltages(p_pack);
    Fault_CheckCurrents(p_pack);
    Fault_CheckTemperatures(p_pack);
    Fault_CheckCommunications(p_pack);
    Fault_CheckContactors(p_pack);

    /* Update highest severity level */
    uint8_t i;
    FaultLevel_t highest = FAULT_LEVEL_NONE;
    for (i = 0U; i < p_pack->faults.active_fault_count; i++)
    {
        FaultLevel_t lvl = Fault_GetLevel(p_pack->faults.active_faults[i]);
        if (lvl > highest) { highest = lvl; }
    }
    p_pack->faults.highest_severity = highest;

    /* Transition to FAULT state if Level 3 fault detected */
    if (highest == FAULT_LEVEL_3)
    {
        if (p_pack->state != BMS_STATE_FAULT)
        {
            p_pack->previous_state       = p_pack->state;
            p_pack->state                = BMS_STATE_FAULT;
            p_pack->faults.fault_entry_tick = HAL_GetTickMs();
            HAL_Contactor_OpenAll();
        }
    }
}

/**
 * @brief  Manually set a fault (e.g., from a driver callback or ISR via queue)
 * @param[in,out] p_pack  Battery pack
 * @param[in]     code    Fault code to assert
 */
void Fault_Assert(BatteryPack_t *p_pack, FaultCode_t code)
{
    if (p_pack == NULL) { return; }
    Fault_SetFault(p_pack, code);
    Fault_ExecuteResponse(p_pack, code, Fault_GetLevel(code));
}

/**
 * @brief  Clear a non-latching fault (or force-clear via service command)
 * @param[in,out] p_pack       Battery pack
 * @param[in]     code         Fault code to clear
 * @param[in]     force_clear  True to clear even latching faults (service tool only)
 * @return true if fault was cleared, false if latching and force_clear == false
 */
bool Fault_Clear(BatteryPack_t *p_pack, FaultCode_t code, bool force_clear)
{
    if (p_pack == NULL) { return false; }

    bool is_latching = Fault_IsLatching(code);

    if (is_latching && (!force_clear))
    {
        return false;  /* Cannot clear latching fault without service command */
    }

    Fault_ClearFault(p_pack, code);
    return true;
}

/**
 * @brief  Check if a specific fault is currently active
 * @return true if fault is active
 */
bool Fault_IsActive(const BatteryPack_t *p_pack, FaultCode_t code)
{
    uint8_t i;
    if (p_pack == NULL) { return false; }

    for (i = 0U; i < p_pack->faults.active_fault_count; i++)
    {
        if (p_pack->faults.active_faults[i] == code) { return true; }
    }
    return false;
}

/* =========================================================================
 * PRIVATE — FAULT CHECK IMPLEMENTATIONS
 * ========================================================================= */

/**
 * @brief  Check all voltage-related fault conditions
 */
static void Fault_CheckVoltages(BatteryPack_t *p_pack)
{
    uint8_t m, c;

    for (m = 0U; m < BMS_NUM_MODULES; m++)
    {
        for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
        {
            uint16_t v = p_pack->modules[m].cells[c].voltage_mv;

            /* Overvoltage — Level 3 */
            if (Fault_Debounce(FAULT_CELL_OVERVOLTAGE,
                                v >= CELL_VOLTAGE_ABSOLUTE_MAX_MV))
            {
                Fault_SetFault(p_pack, FAULT_CELL_OVERVOLTAGE);
                Fault_ExecuteResponse(p_pack, FAULT_CELL_OVERVOLTAGE, FAULT_LEVEL_3);
            }

            /* Undervoltage — Level 3 */
            if (Fault_Debounce(FAULT_CELL_UNDERVOLTAGE,
                                v <= CELL_VOLTAGE_ABSOLUTE_MIN_MV))
            {
                Fault_SetFault(p_pack, FAULT_CELL_UNDERVOLTAGE);
                Fault_ExecuteResponse(p_pack, FAULT_CELL_UNDERVOLTAGE, FAULT_LEVEL_3);
            }
        }

        /* Cell voltage spread — Level 1 (imbalance warning) */
        if (Fault_Debounce(FAULT_CELL_VOLTAGE_MISMATCH,
                            p_pack->modules[m].cell_voltage_delta_mv > 100U))
        {
            Fault_SetFault(p_pack, FAULT_CELL_VOLTAGE_MISMATCH);
            Fault_ExecuteResponse(p_pack, FAULT_CELL_VOLTAGE_MISMATCH, FAULT_LEVEL_1);
        }
    }

    /* Pack-level voltage check */
    if (Fault_Debounce(FAULT_PACK_OVERVOLTAGE,
                        p_pack->pack_voltage_mv >= PACK_VOLTAGE_MAX_MV))
    {
        Fault_SetFault(p_pack, FAULT_PACK_OVERVOLTAGE);
        Fault_ExecuteResponse(p_pack, FAULT_PACK_OVERVOLTAGE, FAULT_LEVEL_3);
    }
}

/**
 * @brief  Check current-related fault conditions
 */
static void Fault_CheckCurrents(BatteryPack_t *p_pack)
{
    float current_a = (float)p_pack->pack_current_ma / 1000.0F;

    /* Overcurrent discharge */
    if (Fault_Debounce(FAULT_OVERCURRENT_DISCHARGE,
                        current_a > PACK_CURRENT_FAULT_DISCHARGE_A))
    {
        Fault_SetFault(p_pack, FAULT_OVERCURRENT_DISCHARGE);
        Fault_ExecuteResponse(p_pack, FAULT_OVERCURRENT_DISCHARGE, FAULT_LEVEL_3);
    }

    /* Overcurrent charge (negative current = charging) */
    if (Fault_Debounce(FAULT_OVERCURRENT_CHARGE,
                        current_a < -(PACK_CURRENT_FAULT_CHARGE_A)))
    {
        Fault_SetFault(p_pack, FAULT_OVERCURRENT_CHARGE);
        Fault_ExecuteResponse(p_pack, FAULT_OVERCURRENT_CHARGE, FAULT_LEVEL_3);
    }
}

/**
 * @brief  Check temperature-related fault conditions
 */
static void Fault_CheckTemperatures(BatteryPack_t *p_pack)
{
    float max_temp_c = DECI_C_TO_FLOAT(p_pack->thermal.max_temperature_dc);
    float min_temp_c = DECI_C_TO_FLOAT(p_pack->thermal.min_temperature_dc);

    /* Overtemperature — Level 3 */
    if (Fault_Debounce(FAULT_OVERTEMP_CELL, max_temp_c >= TEMP_FAULT_HIGH_C))
    {
        Fault_SetFault(p_pack, FAULT_OVERTEMP_CELL);
        Fault_ExecuteResponse(p_pack, FAULT_OVERTEMP_CELL, FAULT_LEVEL_3);
    }

    /* Undertemperature — Level 2 (derating) */
    if (Fault_Debounce(FAULT_UNDERTEMP_CELL, min_temp_c <= TEMP_FAULT_LOW_C))
    {
        Fault_SetFault(p_pack, FAULT_UNDERTEMP_CELL);
        Fault_ExecuteResponse(p_pack, FAULT_UNDERTEMP_CELL, FAULT_LEVEL_2);
    }

    /* Check for failed temperature sensors (values out of physical range) */
    uint8_t s;
    for (s = 0U; s < BMS_NUM_TEMP_SENSORS; s++)
    {
        int16_t raw_dc = p_pack->thermal.temperatures_dc[s];
        bool sensor_fault = ((raw_dc < FLOAT_TO_DECI_C(-40.0F)) ||
                              (raw_dc > FLOAT_TO_DECI_C(120.0F)));
        if (sensor_fault)
        {
            p_pack->thermal.faulty_sensor_mask |= (uint8_t)(1U << (s & 7U));
        }
    }
    if (p_pack->thermal.faulty_sensor_mask != 0U)
    {
        Fault_SetFault(p_pack, FAULT_TEMP_SENSOR_OPEN);
        Fault_ExecuteResponse(p_pack, FAULT_TEMP_SENSOR_OPEN, FAULT_LEVEL_1);
    }
}

/**
 * @brief  Check CAN and AFE communication health
 */
static void Fault_CheckCommunications(BatteryPack_t *p_pack)
{
    /* VCU heartbeat timeout check */
    uint32_t now = HAL_GetTickMs();
    uint32_t vcu_silence_ms = now - p_pack->comms.last_vcu_msg_tick;

    if (Fault_Debounce(FAULT_VCU_COMM_TIMEOUT, vcu_silence_ms > 500U))
    {
        Fault_SetFault(p_pack, FAULT_VCU_COMM_TIMEOUT);
        Fault_ExecuteResponse(p_pack, FAULT_VCU_COMM_TIMEOUT, FAULT_LEVEL_2);
    }

    /* CAN bus-off */
    if (Fault_Debounce(FAULT_CAN_BUS_OFF, p_pack->comms.is_bus_off))
    {
        Fault_SetFault(p_pack, FAULT_CAN_BUS_OFF);
        Fault_ExecuteResponse(p_pack, FAULT_CAN_BUS_OFF, FAULT_LEVEL_2);
    }

    /* AFE module communication check */
    uint8_t m;
    for (m = 0U; m < BMS_NUM_MODULES; m++)
    {
        uint32_t afe_silence_ms = now - p_pack->modules[m].last_update_tick;
        if (Fault_Debounce(FAULT_AFE_COMM_TIMEOUT, afe_silence_ms > 200U))
        {
            Fault_SetFault(p_pack, FAULT_AFE_COMM_TIMEOUT);
            Fault_ExecuteResponse(p_pack, FAULT_AFE_COMM_TIMEOUT, FAULT_LEVEL_3);
        }
    }
}

/**
 * @brief  Check for contactor weld (contactor should be open but bus voltage present)
 */
static void Fault_CheckContactors(BatteryPack_t *p_pack)
{
    bool main_open = (p_pack->power_path.main_positive == CONTACTOR_OPEN) &&
                      (p_pack->power_path.main_negative == CONTACTOR_OPEN);

    /* If contactors commanded open but bus voltage is > 80% of pack voltage, suspect weld */
    if (main_open)
    {
        uint32_t threshold_mv = (p_pack->pack_voltage_mv * 80U) / 100U;
        if (Fault_Debounce(FAULT_CONTACTOR_WELD,
                            p_pack->power_path.voltage_mv_bus > threshold_mv))
        {
            Fault_SetFault(p_pack, FAULT_CONTACTOR_WELD);
            Fault_ExecuteResponse(p_pack, FAULT_CONTACTOR_WELD, FAULT_LEVEL_3);
        }
    }
}

/* =========================================================================
 * PRIVATE — FAULT MANAGEMENT HELPERS
 * ========================================================================= */

static void Fault_SetFault(BatteryPack_t *p_pack, FaultCode_t code)
{
    FaultManager_t *p_fm = &p_pack->faults;

    /* Check if already active */
    uint8_t i;
    for (i = 0U; i < p_fm->active_fault_count; i++)
    {
        if (p_fm->active_faults[i] == code) { return; }  /* Already logged */
    }

    /* Add to active fault list (ring buffer with cap) */
    if (p_fm->active_fault_count < 16U)
    {
        p_fm->active_faults[p_fm->active_fault_count] = code;
        p_fm->active_fault_count++;
    }

    p_fm->total_fault_events++;
    Fault_LogToNvm(p_fm, code);
}

static void Fault_ClearFault(BatteryPack_t *p_pack, FaultCode_t code)
{
    FaultManager_t *p_fm = &p_pack->faults;
    uint8_t i, write_idx = 0U;

    /* Remove from active list by compacting array */
    for (i = 0U; i < p_fm->active_fault_count; i++)
    {
        if (p_fm->active_faults[i] != code)
        {
            p_fm->active_faults[write_idx] = p_fm->active_faults[i];
            write_idx++;
        }
    }
    p_fm->active_fault_count = write_idx;
}

static void Fault_ExecuteResponse(BatteryPack_t *p_pack,
                                   FaultCode_t code,
                                   FaultLevel_t level)
{
    (void)code;  /* Future: encode code into CAN DTC frame */

    switch (level)
    {
        case FAULT_LEVEL_1:
            /* Log only — CAN stack will broadcast DTC */
            break;

        case FAULT_LEVEL_2:
            /* Derate current limits to 50% */
            Fault_ApplyCurrentDerating(p_pack, 0.5F);
            break;

        case FAULT_LEVEL_3:
            /* Immediate: open all contactors */
            HAL_Contactor_OpenAll();
            break;

        case FAULT_LEVEL_NONE:
        default:
            break;
    }
}

/**
 * @brief  Debounce a fault condition
 * @param[in] code           Fault code to debounce
 * @param[in] condition_met  True if fault condition is present this cycle
 * @return true if condition has been met for required consecutive cycles
 */
static bool Fault_Debounce(FaultCode_t code, bool condition_met)
{
    uint8_t i;
    uint8_t required_cycles = 1U;

    /* Find debounce entry */
    for (i = 0U; i < FAULT_CATALOG_SIZE; i++)
    {
        if (k_FaultCatalog[i].code == code)
        {
            required_cycles = k_FaultCatalog[i].debounce_cycles;
            break;
        }
    }

    /* Find/update debounce state */
    for (i = 0U; i < MAX_DEBOUNCE_ENTRIES; i++)
    {
        if (s_DebounceStates[i].code == code)
        {
            if (condition_met)
            {
                if (s_DebounceStates[i].debounce_count < 0xFFU)
                {
                    s_DebounceStates[i].debounce_count++;
                }
            }
            else
            {
                s_DebounceStates[i].debounce_count = 0U;
            }

            return (s_DebounceStates[i].debounce_count >= required_cycles);
        }
    }

    return false;
}

static FaultLevel_t Fault_GetLevel(FaultCode_t code)
{
    uint8_t i;
    for (i = 0U; i < FAULT_CATALOG_SIZE; i++)
    {
        if (k_FaultCatalog[i].code == code) { return k_FaultCatalog[i].level; }
    }
    return FAULT_LEVEL_NONE;
}

static bool Fault_IsLatching(FaultCode_t code)
{
    uint8_t i;
    for (i = 0U; i < FAULT_CATALOG_SIZE; i++)
    {
        if (k_FaultCatalog[i].code == code) { return k_FaultCatalog[i].is_latching; }
    }
    return false;
}

static void Fault_LogToNvm(FaultManager_t *p_fm, FaultCode_t code)
{
    /* Write to ring buffer — EEPROM flush handled by eeprom_manager task */
    uint8_t head = p_fm->fault_log_head;
    p_fm->fault_log[head]            = code;
    p_fm->fault_log_timestamps[head] = HAL_GetTickMs();
    p_fm->fault_log_head             = (uint8_t)((head + 1U) % 32U);
}

static void Fault_ApplyCurrentDerating(BatteryPack_t *p_pack, float factor)
{
    p_pack->max_charge_current_a = PACK_CURRENT_CHARGE_MAX_A * factor;
    p_pack->max_discharge_current_a = PACK_CURRENT_DISCHARGE_MAX_A * factor;
}
