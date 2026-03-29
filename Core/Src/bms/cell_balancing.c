/**
 * @file    cell_balancing.c
 * @brief   Passive Cell Balancing Logic
 * @version 1.0.0
 *
 * @details Implements passive (dissipative) cell balancing triggered when
 *          cell voltage spread exceeds BALANCE_VOLTAGE_DIFF_MV (10mV).
 *
 *          Algorithm:
 *            1. Find minimum cell voltage in pack (V_min)
 *            2. For each cell where V_cell - V_min > threshold: enable bypass FET
 *            3. Balancing is only permitted in CHARGING or BALANCING states
 *            4. Balancing FETs are duty-cycled to limit thermal dissipation
 *            5. Session timeout prevents indefinite balancing
 *
 *          Hardware interface: AFE BQ76952 provides per-cell balance FET control
 *          via I2C register writes (abstracted through afe_bq76952.c driver).
 */

#include "cell_balancing.h"
#include "battery_structs.h"
#include "bms_config.h"
#include <string.h>

/* =========================================================================
 * PRIVATE TYPES
 * ========================================================================= */

/**
 * @brief Per-cell balancing state
 */
typedef struct
{
    bool        is_balancing;          /**< Balancing FET currently active             */
    uint32_t    balance_time_ms;       /**< Cumulative balancing time this session     */
    uint32_t    last_toggle_tick;      /**< Tick of last FET toggle (for duty cycle)   */
    bool        duty_cycle_phase;      /**< True = ON phase, False = OFF phase         */
} CellBalanceState_t;

/* =========================================================================
 * MODULE-LEVEL STATIC STATE
 * ========================================================================= */
static CellBalanceState_t s_BalanceState[BMS_NUM_MODULES][BMS_CELLS_PER_MODULE];
static uint32_t           s_SessionStartTick    = 0U;
static bool               s_SessionActive       = false;
static uint32_t           s_TotalBalancedCells  = 0U;

/* =========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ========================================================================= */
static uint16_t Balance_FindPackMinVoltage(const BatteryPack_t *p_pack);
static void     Balance_EnableFet(uint8_t module_idx, uint8_t cell_idx, bool enable);
static void     Balance_DisableAll(void);
static bool     Balance_IsDutyCycleOnPhase(uint8_t module_idx, uint8_t cell_idx,
                                            uint32_t tick_ms);

/* External AFE driver */
extern void     AFE_SetBalancingFet(uint8_t module, uint8_t cell, bool enable);
extern uint32_t HAL_GetTickMs(void);

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Initialize cell balancing module
 */
void Balance_Init(void)
{
    (void)memset(s_BalanceState, 0, sizeof(s_BalanceState));
    s_SessionActive      = false;
    s_SessionStartTick   = 0U;
    s_TotalBalancedCells = 0U;
}

/**
 * @brief  Run cell balancing algorithm — call from balancing RTOS task (1Hz)
 * @param[in,out] p_pack  Global battery pack structure
 *
 * @details Balancing is only permitted when:
 *   - BMS state is CHARGING or BALANCING
 *   - All cells above minimum voltage threshold
 *   - No active Level 3 fault
 *   - Session duration < BALANCE_TIMEOUT_S
 */
void Balance_Run(BatteryPack_t *p_pack)
{
    if (p_pack == NULL) { return; }

    /* Balancing is only allowed in CHARGING or BALANCING states */
    bool state_allows = ((p_pack->state == BMS_STATE_CHARGING) ||
                          (p_pack->state == BMS_STATE_BALANCING));

    /* Disable all balancing if state is wrong or fault active */
    if ((!state_allows) || (p_pack->faults.highest_severity == FAULT_LEVEL_3))
    {
        Balance_DisableAll();
        s_SessionActive = false;
        return;
    }

    uint32_t now_ms = HAL_GetTickMs();

    /* Check session timeout */
    if (s_SessionActive)
    {
        uint32_t session_duration_s = (now_ms - s_SessionStartTick) / 1000U;
        if (session_duration_s >= BALANCE_TIMEOUT_S)
        {
            Balance_DisableAll();
            s_SessionActive = false;
            return;
        }
    }

    /* Find global minimum cell voltage */
    uint16_t v_min_mv = Balance_FindPackMinVoltage(p_pack);

    /* If minimum cell is too low, do not balance (protect weak cells) */
    if (v_min_mv < BALANCE_MIN_CELL_MV)
    {
        Balance_DisableAll();
        return;
    }

    /* Threshold: balance cells more than BALANCE_VOLTAGE_DIFF_MV above minimum */
    uint16_t balance_threshold_mv = v_min_mv + BALANCE_VOLTAGE_DIFF_MV;

    uint8_t  m, c;
    uint32_t active_cell_count = 0U;

    for (m = 0U; m < BMS_NUM_MODULES; m++)
    {
        if (!p_pack->modules[m].is_online) { continue; }  /* Skip offline modules */

        for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
        {
            uint16_t v_cell = p_pack->modules[m].cells[c].voltage_mv;
            bool should_balance = (v_cell > balance_threshold_mv);

            if (should_balance)
            {
                /* Apply duty cycle control */
                bool in_on_phase = Balance_IsDutyCycleOnPhase(m, c, now_ms);

                if (in_on_phase && !s_BalanceState[m][c].is_balancing)
                {
                    Balance_EnableFet(m, c, true);
                    p_pack->modules[m].cells[c].is_balancing = true;
                    active_cell_count++;

                    if (!s_SessionActive)
                    {
                        s_SessionActive    = true;
                        s_SessionStartTick = now_ms;
                    }
                }
                else if (!in_on_phase && s_BalanceState[m][c].is_balancing)
                {
                    Balance_EnableFet(m, c, false);
                    p_pack->modules[m].cells[c].is_balancing = false;
                }

                /* Accumulate balancing time */
                if (s_BalanceState[m][c].is_balancing)
                {
                    s_BalanceState[m][c].balance_time_ms += TASK_PERIOD_BALANCING_MS;
                }
            }
            else
            {
                /* Cell is within threshold — disable its FET */
                if (s_BalanceState[m][c].is_balancing)
                {
                    Balance_EnableFet(m, c, false);
                    p_pack->modules[m].cells[c].is_balancing = false;
                }
            }
        }

        /* Update module-level balancing flag */
        p_pack->modules[m].is_balancing_active = false;
        for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
        {
            if (s_BalanceState[m][c].is_balancing)
            {
                p_pack->modules[m].is_balancing_active = true;
                break;
            }
        }
    }

    s_TotalBalancedCells = active_cell_count;

    /* If no cells need balancing, end session */
    if (active_cell_count == 0U) { s_SessionActive = false; }
}

/**
 * @brief  Emergency disable — call from fault manager or state machine exit
 */
void Balance_DisableAllCells(BatteryPack_t *p_pack)
{
    uint8_t m, c;

    Balance_DisableAll();
    s_SessionActive = false;

    if (p_pack != NULL)
    {
        for (m = 0U; m < BMS_NUM_MODULES; m++)
        {
            for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
            {
                p_pack->modules[m].cells[c].is_balancing = false;
            }
            p_pack->modules[m].is_balancing_active = false;
        }
    }
}

/**
 * @brief  Get count of currently balancing cells
 * @return Number of cells with active balance FETs
 */
uint32_t Balance_GetActiveCellCount(void)
{
    return s_TotalBalancedCells;
}

/**
 * @brief  Get total balancing time for a specific cell this session
 */
uint32_t Balance_GetCellBalanceTimeMs(uint8_t module_idx, uint8_t cell_idx)
{
    if ((module_idx >= BMS_NUM_MODULES) || (cell_idx >= BMS_CELLS_PER_MODULE))
    {
        return 0U;
    }
    return s_BalanceState[module_idx][cell_idx].balance_time_ms;
}

/* =========================================================================
 * PRIVATE FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Find the minimum cell voltage across all online modules
 */
static uint16_t Balance_FindPackMinVoltage(const BatteryPack_t *p_pack)
{
    uint16_t v_min = 0xFFFFU;
    uint8_t  m, c;

    for (m = 0U; m < BMS_NUM_MODULES; m++)
    {
        if (!p_pack->modules[m].is_online) { continue; }
        for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
        {
            uint16_t v = p_pack->modules[m].cells[c].voltage_mv;
            if (v < v_min) { v_min = v; }
        }
    }

    return (v_min == 0xFFFFU) ? 0U : v_min;
}

/**
 * @brief  Enable or disable a single cell's balance FET via AFE driver
 */
static void Balance_EnableFet(uint8_t module_idx, uint8_t cell_idx, bool enable)
{
    s_BalanceState[module_idx][cell_idx].is_balancing = enable;
    AFE_SetBalancingFet(module_idx, cell_idx, enable);
}

/**
 * @brief  Disable all balance FETs immediately
 */
static void Balance_DisableAll(void)
{
    uint8_t m, c;
    for (m = 0U; m < BMS_NUM_MODULES; m++)
    {
        for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
        {
            if (s_BalanceState[m][c].is_balancing)
            {
                AFE_SetBalancingFet(m, c, false);
                s_BalanceState[m][c].is_balancing = false;
            }
        }
    }
}

/**
 * @brief  Compute duty cycle phase for a cell's balance FET
 * @details  50% duty cycle: ON for half period, OFF for other half.
 *           Period = 2 × TASK_PERIOD_BALANCING_MS (2 seconds for 1Hz task)
 * @return true if FET should be ON this cycle
 */
static bool Balance_IsDutyCycleOnPhase(uint8_t module_idx, uint8_t cell_idx,
                                        uint32_t tick_ms)
{
    /* 2-second duty cycle period, 50% on-time */
    const uint32_t PERIOD_MS = 2U * TASK_PERIOD_BALANCING_MS;
    uint32_t phase = tick_ms % PERIOD_MS;

    /* Offset each cell slightly to spread thermal load */
    uint32_t cell_offset_ms = ((uint32_t)module_idx * BMS_CELLS_PER_MODULE +
                                (uint32_t)cell_idx) * 50U;

    uint32_t adjusted_phase = (phase + cell_offset_ms) % PERIOD_MS;

    (void)module_idx;  /* MISRA: suppress if not used after optimization */
    (void)cell_idx;

    return (adjusted_phase < (PERIOD_MS * BALANCE_DUTY_CYCLE_PCT / 100U));
}
