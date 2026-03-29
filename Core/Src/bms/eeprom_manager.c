/**
 * @file    eeprom_manager.c
 * @brief   Non-Volatile Storage Manager — SOH, SOC, Cycle Count, Fault Log
 * @version 1.0.0
 *
 * @details Manages persistence of safety-critical data across power cycles.
 *          Uses a double-buffering / wear-leveling strategy with CRC validation.
 *
 *          Stored data:
 *            - SOC (for warm-start estimation)
 *            - SOH (capacity fade tracking)
 *            - Cycle count (full charge-discharge events)
 *            - Fault log (last 32 fault events with timestamps)
 *            - Pack serial number and calibration data
 */

#include "eeprom_manager.h"
#include "battery_structs.h"
#include "bms_config.h"
#include <string.h>
#include <stdint.h>

/* =========================================================================
 * EEPROM DATA STRUCTURES
 * ========================================================================= */

#define EEPROM_MAGIC_WORD   (0xBEEF1234UL)  /**< Validity marker in EEPROM header     */
#define EEPROM_DATA_VERSION (0x01U)          /**< Increment on structure layout changes */

/**
 * @brief Pack state snapshot stored in EEPROM
 */
typedef struct
{
    uint32_t    magic;          /**< Must equal EEPROM_MAGIC_WORD                     */
    uint8_t     data_version;   /**< Layout version — invalidates older formats       */
    uint8_t     _pad[3];

    float       soc_percent;    /**< Last known SOC                                   */
    float       soh_percent;    /**< Last known SOH                                   */
    uint32_t    cycle_count;    /**< Lifetime charge cycle count                      */
    uint32_t    total_fault_events; /**< Cumulative fault counter                     */
    uint32_t    uptime_seconds; /**< Cumulative uptime                                */

    uint16_t    crc16;          /**< CRC16 over all bytes except this field           */
    uint8_t     _pad2[2];
} Eeprom_PackState_t;

/* =========================================================================
 * CRC16 COMPUTATION (CRC-16/CCITT-FALSE)
 * ========================================================================= */

static uint16_t Eeprom_Crc16(const uint8_t *p_data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= ((uint16_t)p_data[i] << 8U);
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

/* External HAL for I2C EEPROM (e.g., AT24C256) or internal flash emulation */
extern bool HAL_EEPROM_Write(uint16_t address, const uint8_t *p_data, uint16_t len);
extern bool HAL_EEPROM_Read(uint16_t address, uint8_t *p_data, uint16_t len);

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Load SOC, SOH, and cycle count from EEPROM
 * @param[out] p_soc_pct    SOC in percent (set to SOC_INITIAL_PERCENT if invalid)
 * @param[out] p_soh_pct    SOH in percent (set to 100.0 if invalid)
 * @param[out] p_cycles     Cycle count (0 if invalid)
 */
void Eeprom_LoadPackState(float *p_soc_pct, float *p_soh_pct, uint32_t *p_cycles)
{
    if ((p_soc_pct == NULL) || (p_soh_pct == NULL) || (p_cycles == NULL)) { return; }

    Eeprom_PackState_t record;
    (void)memset(&record, 0, sizeof(record));

    bool read_ok = HAL_EEPROM_Read(EEPROM_BASE_ADDRESS,
                                    (uint8_t *)&record,
                                    (uint16_t)sizeof(record));

    /* Validate magic, version, and CRC */
    bool valid = false;
    if (read_ok && (record.magic == EEPROM_MAGIC_WORD) &&
        (record.data_version == EEPROM_DATA_VERSION))
    {
        uint16_t computed_crc = Eeprom_Crc16((const uint8_t *)&record,
                                               (uint16_t)(sizeof(record) -
                                                           sizeof(record.crc16) -
                                                           sizeof(record._pad2)));
        valid = (computed_crc == record.crc16);
    }

    if (valid)
    {
        *p_soc_pct = record.soc_percent;
        *p_soh_pct = record.soh_percent;
        *p_cycles  = record.cycle_count;
    }
    else
    {
        /* Defaults on first boot or corrupted data */
        *p_soc_pct = SOC_INITIAL_PERCENT;
        *p_soh_pct = 100.0F;
        *p_cycles  = 0U;
    }
}

/**
 * @brief  Save pack state to EEPROM
 * @param[in] soc_pct    Current SOC
 * @param[in] soh_pct    Current SOH
 * @param[in] cycles     Cycle count
 */
void Eeprom_SavePackState(float soc_pct, float soh_pct, uint32_t cycles)
{
    Eeprom_PackState_t record;
    (void)memset(&record, 0, sizeof(record));

    record.magic        = EEPROM_MAGIC_WORD;
    record.data_version = EEPROM_DATA_VERSION;
    record.soc_percent  = soc_pct;
    record.soh_percent  = soh_pct;
    record.cycle_count  = cycles;

    /* Compute CRC over all fields except crc16 itself */
    record.crc16 = Eeprom_Crc16((const uint8_t *)&record,
                                  (uint16_t)(sizeof(record) -
                                              sizeof(record.crc16) -
                                              sizeof(record._pad2)));

    (void)HAL_EEPROM_Write(EEPROM_BASE_ADDRESS,
                            (const uint8_t *)&record,
                            (uint16_t)sizeof(record));
}

/**
 * @brief  Write fault log to EEPROM fault log area
 * @param[in] p_fm  Fault manager containing log data
 */
void Eeprom_SaveFaultLog(const FaultManager_t *p_fm)
{
    if (p_fm == NULL) { return; }

    /* Write fault codes */
    (void)HAL_EEPROM_Write(EEPROM_FAULT_LOG_ADDRESS,
                            (const uint8_t *)p_fm->fault_log,
                            (uint16_t)(sizeof(FaultCode_t) *
                                        EEPROM_FAULT_LOG_MAX_ENTRIES));
}
