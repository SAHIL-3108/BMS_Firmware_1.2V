/**
 * @file    can_stack.c
 * @brief   CAN 2.0B / J1939 Communication Stack
 * @version 1.0.0
 *
 * @details Implements J1939-compliant CAN communication between BMS, VCU, and
 *          charger. Handles:
 *            - Periodic BMS status broadcast (100ms)
 *            - Cell voltage streaming (round-robin across modules)
 *            - Fault DTC broadcast on fault events
 *            - VCU heartbeat reception and timeout monitoring
 *            - Charger current/voltage command exchange
 *
 * J1939 Frame Structure (29-bit Extended ID):
 *   ┌─────────────┬──────────────┬──────────────┬──────────────┐
 *   │ Priority    │ PGN[17:8]   │ Destination  │ Source Addr  │
 *   │ [28:26] 3b  │ [25:9] 17b  │ [8:1] 8b     │ [0:0] 8b (SA)│
 *   └─────────────┴──────────────┴──────────────┴──────────────┘
 */

#include "can_stack.h"
#include "battery_structs.h"
#include "bms_config.h"
#include <string.h>

/* =========================================================================
 * J1939 FRAME CONSTRUCTION
 * ========================================================================= */

/** @brief Build 29-bit J1939 CAN ID */
#define J1939_BUILD_ID(priority, pgn, dest, src) \
    ( (((uint32_t)(priority) & 0x7U) << 26U)   | \
      (((uint32_t)(pgn)      & 0x1FFFFU) << 9U) | \
      (((uint32_t)(dest)     & 0xFFU) << 1U)    | \
      ((uint32_t)(src)       & 0x1U)             )

#define J1939_PRIORITY_HIGH     (3U)
#define J1939_PRIORITY_NORMAL   (6U)
#define J1939_PRIORITY_LOW      (7U)

/* =========================================================================
 * CAN MESSAGE STRUCTURES
 * ========================================================================= */

/**
 * @brief Raw CAN frame representation
 */
typedef struct
{
    uint32_t    id;             /**< 29-bit extended CAN ID                          */
    uint8_t     data[8];        /**< CAN data payload (up to 8 bytes)                */
    uint8_t     dlc;            /**< Data Length Code (0-8)                          */
    bool        is_extended;    /**< True for 29-bit extended ID (J1939)             */
} CanFrame_t;

/* =========================================================================
 * PRIVATE STATE
 * ========================================================================= */
static uint8_t  s_ModuleBroadcastIndex = 0U;   /**< Round-robin module index for Vx frames */
static uint32_t s_TxSequenceNumber     = 0U;

/* =========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ========================================================================= */
static void     Can_BuildStatusFrame(const BatteryPack_t *p_pack,
                                      CanFrame_t *p_frame);
static void     Can_BuildSocSohFrame(const BatteryPack_t *p_pack,
                                      CanFrame_t *p_frame);
static void     Can_BuildFaultFrame(const BatteryPack_t *p_pack,
                                     CanFrame_t *p_frame);
static void     Can_BuildCellVoltageFrame(const BatteryPack_t *p_pack,
                                           uint8_t module_idx,
                                           CanFrame_t *p_frame);
static void     Can_BuildThermalFrame(const BatteryPack_t *p_pack,
                                       CanFrame_t *p_frame);
static void     Can_BuildChargeRequest(const BatteryPack_t *p_pack,
                                        CanFrame_t *p_frame);
static void     Can_ProcessRxFrame(BatteryPack_t *p_pack,
                                    const CanFrame_t *p_frame);
static void     Can_ProcessVcuFrame(BatteryPack_t *p_pack,
                                     const CanFrame_t *p_frame);
static void     Can_ProcessChargerFrame(BatteryPack_t *p_pack,
                                         const CanFrame_t *p_frame);
static bool     Can_TransmitFrame(const CanFrame_t *p_frame);
static void     Can_PackUint16Be(uint8_t *p_buf, uint16_t val);
static void     Can_PackInt16Be(uint8_t *p_buf, int16_t val);
static uint16_t Can_UnpackUint16Be(const uint8_t *p_buf);

/* External hardware driver */
extern bool     HAL_CAN_Transmit(uint32_t id, const uint8_t *p_data, uint8_t dlc,
                                  bool extended);
extern bool     HAL_CAN_Receive(uint32_t *p_id, uint8_t *p_data, uint8_t *p_dlc);
extern uint32_t HAL_GetTickMs(void);

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Initialize CAN stack
 * @param[in,out] p_pack  Battery pack (to init comms sub-structure)
 */
void Can_Init(BatteryPack_t *p_pack)
{
    if (p_pack == NULL) { return; }

    (void)memset(&p_pack->comms, 0, sizeof(CanCommsData_t));
    s_ModuleBroadcastIndex = 0U;
    s_TxSequenceNumber     = 0U;

    p_pack->comms.last_vcu_msg_tick     = HAL_GetTickMs();  /* Grace period on boot */
    p_pack->comms.last_charger_msg_tick = HAL_GetTickMs();
}

/**
 * @brief  Transmit periodic BMS telemetry — call at 100ms interval
 * @param[in] p_pack  Battery pack data (read-only)
 *
 * @details Broadcasts in round-robin sequence each call:
 *          Call 1: Status + SOC frame
 *          Call 2: Thermal frame
 *          Call 3: Cell voltages (one module per call)
 *          Call 4: Fault status frame
 *          Call 5: Charge request (if charging)
 */
void Can_TransmitTelemetry(const BatteryPack_t *p_pack)
{
    if (p_pack == NULL) { return; }

    CanFrame_t frame;
    (void)memset(&frame, 0, sizeof(CanFrame_t));

    /* --- Frame 1: Pack Status (every call) --- */
    Can_BuildStatusFrame(p_pack, &frame);
    (void)Can_TransmitFrame(&frame);

    /* --- Frame 2: SOC/SOH --- */
    Can_BuildSocSohFrame(p_pack, &frame);
    (void)Can_TransmitFrame(&frame);

    /* --- Frame 3: Thermal data --- */
    Can_BuildThermalFrame(p_pack, &frame);
    (void)Can_TransmitFrame(&frame);

    /* --- Frame 4: Cell voltages (one module per call, round-robin) --- */
    Can_BuildCellVoltageFrame(p_pack, s_ModuleBroadcastIndex, &frame);
    (void)Can_TransmitFrame(&frame);
    s_ModuleBroadcastIndex = (uint8_t)((s_ModuleBroadcastIndex + 1U) %
                                        BMS_NUM_MODULES);

    /* --- Frame 5: Fault status (if any active faults) --- */
    if (p_pack->faults.active_fault_count > 0U)
    {
        Can_BuildFaultFrame(p_pack, &frame);
        (void)Can_TransmitFrame(&frame);
    }

    /* --- Frame 6: Charge request (if in charging state) --- */
    if ((p_pack->state == BMS_STATE_CHARGING) &&
        (p_pack->comms.charger_heartbeat_ok))
    {
        Can_BuildChargeRequest(p_pack, &frame);
        (void)Can_TransmitFrame(&frame);
    }

    s_TxSequenceNumber++;
}

/**
 * @brief  Process all pending received CAN frames — call at 100ms interval
 * @param[in,out] p_pack  Battery pack (updated with received data)
 */
void Can_ProcessRxQueue(BatteryPack_t *p_pack)
{
    if (p_pack == NULL) { return; }

    CanFrame_t rx_frame;
    uint8_t  max_frames = 16U;   /* Process up to 16 frames per call (bounded) */

    while (max_frames > 0U)
    {
        max_frames--;

        (void)memset(&rx_frame, 0, sizeof(CanFrame_t));
        bool received = HAL_CAN_Receive(&rx_frame.id, rx_frame.data, &rx_frame.dlc);

        if (!received) { break; }

        Can_ProcessRxFrame(p_pack, &rx_frame);
        p_pack->comms.messages_rx_total++;
    }

    /* Update heartbeat status */
    uint32_t now = HAL_GetTickMs();
    p_pack->comms.vcu_heartbeat_ok =
        ((now - p_pack->comms.last_vcu_msg_tick) < 500U);
    p_pack->comms.charger_heartbeat_ok =
        ((now - p_pack->comms.last_charger_msg_tick) < 1000U);
}

/* =========================================================================
 * PRIVATE — TX FRAME BUILDERS
 * ========================================================================= */

/**
 * @brief  Build J1939 BMS Status frame
 * @details PGN: J1939_PGN_BMS_STATUS
 *   Byte 0:   BMS State (enum)
 *   Byte 1:   Fault level (0-3)
 *   Byte 2-3: Pack voltage (mV / 10, uint16 BE) → 0.01V resolution
 *   Byte 4-5: Pack current (mA / 10, int16 BE)  → signed, +discharge
 *   Byte 6:   SOC (0-200 → 0%-100%, 0.5% res)
 *   Byte 7:   Contactor status bitmask
 */
static void Can_BuildStatusFrame(const BatteryPack_t *p_pack, CanFrame_t *p_frame)
{
    p_frame->id = J1939_BUILD_ID(J1939_PRIORITY_HIGH,
                                  J1939_PGN_BMS_STATUS,
                                  0xFFU,  /* Broadcast */
                                  CAN_BMS_SOURCE_ADDRESS);
    p_frame->dlc         = 8U;
    p_frame->is_extended = true;

    p_frame->data[0] = (uint8_t)p_pack->state;
    p_frame->data[1] = (uint8_t)p_pack->faults.highest_severity;

    uint16_t pack_v_scaled = (uint16_t)(p_pack->pack_voltage_mv / 10U);
    Can_PackUint16Be(&p_frame->data[2], pack_v_scaled);

    int16_t pack_i_scaled = (int16_t)(p_pack->pack_current_ma / 10);
    Can_PackInt16Be(&p_frame->data[4], pack_i_scaled);

    p_frame->data[6] = (uint8_t)(p_pack->soc_soh.soc_percent * 2.0F);  /* 0.5% res */

    /* Contactor status bitmask */
    uint8_t contactor_mask = 0U;
    contactor_mask |= (uint8_t)((uint8_t)(p_pack->power_path.main_positive == CONTACTOR_CLOSED) << 0U);
    contactor_mask |= (uint8_t)((uint8_t)(p_pack->power_path.main_negative == CONTACTOR_CLOSED) << 1U);
    contactor_mask |= (uint8_t)((uint8_t)(p_pack->power_path.precharge == CONTACTOR_CLOSED) << 2U);
    p_frame->data[7] = contactor_mask;
}

/**
 * @brief  Build SOC/SOH telemetry frame
 *   Byte 0-1: SOC (uint16, 0-10000 = 0.00%-100.00%)
 *   Byte 2-3: SOH (uint16, 0-10000)
 *   Byte 4-5: State of Energy Wh (uint16, Wh)
 *   Byte 6-7: Cycle Count (uint16)
 */
static void Can_BuildSocSohFrame(const BatteryPack_t *p_pack, CanFrame_t *p_frame)
{
    p_frame->id = J1939_BUILD_ID(J1939_PRIORITY_NORMAL,
                                  J1939_PGN_SOC_SOH,
                                  0xFFU,
                                  CAN_BMS_SOURCE_ADDRESS);
    p_frame->dlc         = 8U;
    p_frame->is_extended = true;

    uint16_t soc_scaled = (uint16_t)(p_pack->soc_soh.soc_percent * 100.0F);
    uint16_t soh_scaled = (uint16_t)(p_pack->soc_soh.soh_percent * 100.0F);
    uint16_t soe_wh     = (uint16_t)p_pack->soc_soh.soe_wh;
    uint16_t cycles     = (uint16_t)(p_pack->soc_soh.cycle_count & 0xFFFFU);

    Can_PackUint16Be(&p_frame->data[0], soc_scaled);
    Can_PackUint16Be(&p_frame->data[2], soh_scaled);
    Can_PackUint16Be(&p_frame->data[4], soe_wh);
    Can_PackUint16Be(&p_frame->data[6], cycles);
}

/**
 * @brief  Build fault status frame
 *   Byte 0-1: Active fault code (most severe)
 *   Byte 2:   Active fault count
 *   Byte 3:   Fault level
 *   Byte 4-7: Fault entry timestamp (ms, uint32 BE)
 */
static void Can_BuildFaultFrame(const BatteryPack_t *p_pack, CanFrame_t *p_frame)
{
    p_frame->id = J1939_BUILD_ID(J1939_PRIORITY_HIGH,
                                  J1939_PGN_FAULT_STATUS,
                                  0xFFU,
                                  CAN_BMS_SOURCE_ADDRESS);
    p_frame->dlc         = 8U;
    p_frame->is_extended = true;

    FaultCode_t top_fault = FAULT_NONE;
    if (p_pack->faults.active_fault_count > 0U)
    {
        top_fault = p_pack->faults.active_faults[0];
    }

    Can_PackUint16Be(&p_frame->data[0], (uint16_t)top_fault);
    p_frame->data[2] = p_pack->faults.active_fault_count;
    p_frame->data[3] = (uint8_t)p_pack->faults.highest_severity;

    uint32_t ts = p_pack->faults.fault_entry_tick;
    p_frame->data[4] = (uint8_t)((ts >> 24U) & 0xFFU);
    p_frame->data[5] = (uint8_t)((ts >> 16U) & 0xFFU);
    p_frame->data[6] = (uint8_t)((ts >>  8U) & 0xFFU);
    p_frame->data[7] = (uint8_t)((ts >>  0U) & 0xFFU);
}

/**
 * @brief  Build cell voltage frame for one module
 *   Byte 0: Module index
 *   Byte 1: Max/Min cell index (nibbles)
 *   Bytes 2-3: Max cell voltage (mV, uint16 BE)
 *   Bytes 4-5: Min cell voltage (mV, uint16 BE)
 *   Bytes 6-7: Cell delta voltage (mV, uint16 BE)
 */
static void Can_BuildCellVoltageFrame(const BatteryPack_t *p_pack,
                                       uint8_t module_idx, CanFrame_t *p_frame)
{
    p_frame->id = J1939_BUILD_ID(J1939_PRIORITY_LOW,
                                  J1939_PGN_CELL_VOLTAGES,
                                  0xFFU,
                                  CAN_BMS_SOURCE_ADDRESS);
    p_frame->dlc         = 8U;
    p_frame->is_extended = true;

    const CellModule_t *p_mod = &p_pack->modules[module_idx];

    p_frame->data[0] = module_idx;
    p_frame->data[1] = (uint8_t)(p_mod->is_online ? 0x01U : 0x00U);
    Can_PackUint16Be(&p_frame->data[2], p_mod->max_cell_voltage_mv);
    Can_PackUint16Be(&p_frame->data[4], p_mod->min_cell_voltage_mv);
    Can_PackUint16Be(&p_frame->data[6], p_mod->cell_voltage_delta_mv);
}

/**
 * @brief  Build thermal telemetry frame
 *   Bytes 0-1: Max temperature (deci-°C, int16 BE)
 *   Bytes 2-3: Min temperature (deci-°C, int16 BE)
 *   Bytes 4-5: Average temperature (deci-°C, int16 BE)
 *   Byte 6:    Thermal mode (enum)
 *   Byte 7:    PID output (0-200 → 0-100%)
 */
static void Can_BuildThermalFrame(const BatteryPack_t *p_pack, CanFrame_t *p_frame)
{
    p_frame->id = J1939_BUILD_ID(J1939_PRIORITY_NORMAL,
                                  J1939_PGN_TEMPERATURES,
                                  0xFFU,
                                  CAN_BMS_SOURCE_ADDRESS);
    p_frame->dlc         = 8U;
    p_frame->is_extended = true;

    Can_PackInt16Be(&p_frame->data[0], p_pack->thermal.max_temperature_dc);
    Can_PackInt16Be(&p_frame->data[2], p_pack->thermal.min_temperature_dc);
    Can_PackInt16Be(&p_frame->data[4], p_pack->thermal.avg_temperature_dc);
    p_frame->data[6] = (uint8_t)p_pack->thermal.mode;
    p_frame->data[7] = (uint8_t)(p_pack->thermal.pid_output * 200.0F);
}

/**
 * @brief  Build charge control request frame to charger
 *   Bytes 0-1: Requested voltage (mV / 10, uint16 BE)
 *   Bytes 2-3: Requested current (mA / 100, uint16 BE)
 *   Byte 4:    Charge enable (0x01 = enable)
 *   Byte 5:    BMS SOC (0-200)
 *   Byte 6-7:  Reserved
 */
static void Can_BuildChargeRequest(const BatteryPack_t *p_pack, CanFrame_t *p_frame)
{
    p_frame->id = J1939_BUILD_ID(J1939_PRIORITY_HIGH,
                                  J1939_PGN_CHARGE_REQUEST,
                                  CAN_CHARGER_ADDRESS,
                                  CAN_BMS_SOURCE_ADDRESS);
    p_frame->dlc         = 8U;
    p_frame->is_extended = true;

    float target_voltage_v  = (float)CELL_VOLTAGE_OVERVOLTAGE_MV * 0.001F *
                               (float)BMS_NUM_CELLS_SERIES;
    float max_charge_a      = p_pack->max_charge_current_a;

    uint16_t v_req = (uint16_t)((target_voltage_v * 1000.0F) / 10.0F);
    uint16_t i_req = (uint16_t)((max_charge_a * 1000.0F) / 100.0F);

    Can_PackUint16Be(&p_frame->data[0], v_req);
    Can_PackUint16Be(&p_frame->data[2], i_req);
    p_frame->data[4] = 0x01U;  /* Charge enable */
    p_frame->data[5] = (uint8_t)(p_pack->soc_soh.soc_percent * 2.0F);
    p_frame->data[6] = 0x00U;
    p_frame->data[7] = 0x00U;
}

/* =========================================================================
 * PRIVATE — RX FRAME PROCESSING
 * ========================================================================= */

/**
 * @brief  Route received frame to appropriate handler based on source address
 */
static void Can_ProcessRxFrame(BatteryPack_t *p_pack, const CanFrame_t *p_frame)
{
    /* Extract source address from J1939 29-bit ID (bits 7:0 of lower byte) */
    uint8_t src_addr = (uint8_t)(p_frame->id & 0xFFU);

    if (src_addr == CAN_VCU_ADDRESS)
    {
        Can_ProcessVcuFrame(p_pack, p_frame);
    }
    else if (src_addr == CAN_CHARGER_ADDRESS)
    {
        Can_ProcessChargerFrame(p_pack, p_frame);
    }
    /* Unknown sources: silently drop */
}

/**
 * @brief  Process VCU heartbeat and commands
 */
static void Can_ProcessVcuFrame(BatteryPack_t *p_pack, const CanFrame_t *p_frame)
{
    p_pack->comms.last_vcu_msg_tick = HAL_GetTickMs();

    if (p_frame->dlc < 1U) { return; }

    /* Byte 0: VCU command code */
    uint8_t cmd = p_frame->data[0];

    switch (cmd)
    {
        case 0x01U:  /* Contactor close request — VCU acknowledges precharge */
            /* FSM handles this via event queue — don't act directly here */
            break;

        case 0x02U:  /* Fault clear request */
            /* Handled by fault manager via service command */
            break;

        case 0xFFU:  /* Heartbeat only */
        default:
            break;
    }
}

/**
 * @brief  Process charger status and response
 */
static void Can_ProcessChargerFrame(BatteryPack_t *p_pack,
                                     const CanFrame_t *p_frame)
{
    p_pack->comms.last_charger_msg_tick = HAL_GetTickMs();

    if (p_frame->dlc < 6U) { return; }

    /* Parse charger status frame */
    uint16_t output_v_raw = Can_UnpackUint16Be(&p_frame->data[0]);
    uint16_t output_i_raw = Can_UnpackUint16Be(&p_frame->data[2]);

    p_pack->comms.charger_requested_voltage_v = (float)output_v_raw * 0.1F;
    p_pack->comms.charger_requested_current_a = (float)output_i_raw * 0.1F;

    uint8_t charger_status_byte = p_frame->data[4];
    if ((charger_status_byte & 0x01U) != 0U)
    {
        p_pack->comms.charger_status = CHARGER_CONNECTED_AC;
    }
    else if ((charger_status_byte & 0x02U) != 0U)
    {
        p_pack->comms.charger_status = CHARGER_CONNECTED_DC;
    }
}

/* =========================================================================
 * PRIVATE — UTILITY FUNCTIONS
 * ========================================================================= */

static bool Can_TransmitFrame(const CanFrame_t *p_frame)
{
    bool success = HAL_CAN_Transmit(p_frame->id, p_frame->data,
                                     p_frame->dlc, p_frame->is_extended);
    return success;
}

/** @brief Pack uint16 into 2 bytes, big-endian */
static void Can_PackUint16Be(uint8_t *p_buf, uint16_t val)
{
    p_buf[0] = (uint8_t)((val >> 8U) & 0xFFU);
    p_buf[1] = (uint8_t)((val >> 0U) & 0xFFU);
}

/** @brief Pack int16 into 2 bytes, big-endian */
static void Can_PackInt16Be(uint8_t *p_buf, int16_t val)
{
    Can_PackUint16Be(p_buf, (uint16_t)val);
}

/** @brief Unpack uint16 from 2 bytes, big-endian */
static uint16_t Can_UnpackUint16Be(const uint8_t *p_buf)
{
    return (uint16_t)(((uint16_t)p_buf[0] << 8U) | (uint16_t)p_buf[1]);
}
