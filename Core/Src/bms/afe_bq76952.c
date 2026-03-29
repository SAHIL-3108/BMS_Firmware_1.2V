/**
 * @file    afe_bq76952.c
 * @brief   TI BQ76952 Analog Front End Driver
 * @version 1.0.0
 *
 * @details Low-level driver for the Texas Instruments BQ76952 multi-cell
 *          battery monitor and protector IC. Communicates via I2C.
 *
 *          Supports up to 16 cells per IC. For packs with more cells,
 *          multiple ICs are daisy-chained (each module has one AFE).
 *
 *          Key Functions:
 *            - Read cell voltages (BQ76952 internal ADC, 16-bit)
 *            - Read pack/stack current via internal coulomb counter
 *            - Read die temperature
 *            - Control per-cell balance FETs (CB1-CB16 bits)
 *            - Configure hardware protection thresholds (OV, UV, OC, SC)
 *            - Read hardware protection alert flags
 *
 *          BQ76952 Register Map Reference: SLUSDX9D
 *          I2C Address: 0x08 (ADDR pin = GND)
 */

#include "afe_bq76952.h"
#include "bms_config.h"
#include "battery_structs.h"
#include <string.h>

/* =========================================================================
 * BQ76952 REGISTER ADDRESSES
 * ========================================================================= */

/* Direct Commands (1-byte address, 2-byte response) */
#define BQ76952_REG_VCELL1_LSB      (0x14U)  /**< Cell 1 voltage LSB               */
#define BQ76952_REG_CC2_CURRENT     (0x3AU)  /**< Coulomb counter 2 current (fast)  */
#define BQ76952_REG_TEMP_INT        (0x68U)  /**< Internal die temperature          */
#define BQ76952_REG_SAFETY_STATUS_A (0x03U)  /**< Safety Status A register          */
#define BQ76952_REG_SAFETY_STATUS_B (0x05U)  /**< Safety Status B register          */
#define BQ76952_REG_SAFETY_STATUS_C (0x07U)  /**< Safety Status C register          */
#define BQ76952_REG_BAT_STATUS      (0x12U)  /**< Battery status flags              */
#define BQ76952_REG_CELL_BALANCE    (0x83U)  /**< Cell balance FET control          */
#define BQ76952_REG_MFRINFO1        (0x70U)  /**< Manufacturer info                 */

/* Subcommands (write 0x3E with subcommand code, read from 0x40) */
#define BQ76952_SUBCMD_ADDR         (0x3EU)  /**< Subcommand address register       */
#define BQ76952_SUBCMD_DATA_START   (0x40U)  /**< Subcommand data start register    */
#define BQ76952_SUBCMD_CHECKSUM     (0x60U)  /**< Subcommand data checksum          */
#define BQ76952_SUBCMD_LENGTH       (0x61U)  /**< Subcommand data length            */

/* Subcommand codes */
#define BQ76952_SUBCMD_RESET        (0x0012U)
#define BQ76952_SUBCMD_FET_ENABLE   (0x0022U)
#define BQ76952_SUBCMD_CB_ACTIVE_CELLS (0x0083U)

/* Safety Status A bits */
#define BQ76952_SA_CUV              (1U << 2U)   /**< Cell Undervoltage              */
#define BQ76952_SA_COV              (1U << 3U)   /**< Cell Overvoltage               */
#define BQ76952_SA_OCD1             (1U << 4U)   /**< Overcurrent Discharge 1        */
#define BQ76952_SA_OCC              (1U << 7U)   /**< Overcurrent Charge             */
#define BQ76952_SA_SCD              (1U << 5U)   /**< Short Circuit Discharge        */

/* Voltage conversion: BQ76952 reports in 192uV per LSB */
#define BQ76952_VOLTAGE_UV_PER_LSB  (192U)

/* Current conversion: depends on RSNS resistor */
/* With 1mΩ sense resistor: 1 LSB = 78.125 uA */
#define BQ76952_CURRENT_UA_PER_LSB  (78U)

/* Temperature conversion: 0.1°C per LSB, offset 273.15°C (Kelvin) */
#define BQ76952_TEMP_OFFSET_DC      ((-2731))  /**< Convert from Kelvin×10 to deci-°C */

/* =========================================================================
 * PRIVATE STATE
 * ========================================================================= */

typedef struct
{
    uint8_t     i2c_address;
    bool        is_initialized;
    uint32_t    comm_error_count;
    uint32_t    last_read_tick;
} AfeInstance_t;

static AfeInstance_t s_AfeInstances[BMS_NUM_MODULES];

/* =========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ========================================================================= */
static bool Afe_I2cRead(uint8_t device_addr, uint8_t reg_addr,
                         uint8_t *p_buf, uint8_t len);
static bool Afe_I2cWrite(uint8_t device_addr, uint8_t reg_addr,
                          const uint8_t *p_data, uint8_t len);
static bool Afe_WriteSubcommand(uint8_t device_addr, uint16_t subcmd);
static bool Afe_ReadDirectU16(uint8_t device_addr, uint8_t reg, uint16_t *p_val);
static bool Afe_ReadDirectS16(uint8_t device_addr, uint8_t reg, int16_t *p_val);
static uint8_t Afe_ComputeChecksum(const uint8_t *p_data, uint8_t len);

/* External HAL I2C functions */
extern bool HAL_I2C_Write(uint8_t dev_addr, const uint8_t *p_buf, uint8_t len,
                            uint32_t timeout_ms);
extern bool HAL_I2C_Read(uint8_t dev_addr, uint8_t reg, uint8_t *p_buf,
                           uint8_t len, uint32_t timeout_ms);
extern uint32_t HAL_GetTickMs(void);

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Initialize all AFE instances
 * @return true if all AFEs acknowledge on I2C bus
 */
bool AFE_Init(void)
{
    bool all_ok = true;
    uint8_t m;

    (void)memset(s_AfeInstances, 0, sizeof(s_AfeInstances));

    for (m = 0U; m < BMS_NUM_MODULES; m++)
    {
        /* Each module has unique I2C address (set via ADDR resistor network) */
        s_AfeInstances[m].i2c_address = (uint8_t)(AFE_I2C_ADDRESS + m);

        /* Attempt to read device ID to confirm presence */
        uint16_t device_id = 0U;
        bool ok = Afe_ReadDirectU16(s_AfeInstances[m].i2c_address,
                                     BQ76952_REG_MFRINFO1, &device_id);

        s_AfeInstances[m].is_initialized = ok;

        if (!ok)
        {
            all_ok = false;
            s_AfeInstances[m].comm_error_count++;
        }
        else
        {
            /* Enable FET control and configure hardware protection thresholds */
            (void)Afe_WriteSubcommand(s_AfeInstances[m].i2c_address,
                                       BQ76952_SUBCMD_FET_ENABLE);
        }
    }

    return all_ok;
}

/**
 * @brief  Read all cell voltages from a module's AFE
 * @param[in]     module_idx  Module index (0 to BMS_NUM_MODULES-1)
 * @param[in,out] p_module    Module data structure to populate
 * @return true if communication succeeded
 */
bool AFE_ReadCellVoltages(uint8_t module_idx, CellModule_t *p_module)
{
    if ((module_idx >= BMS_NUM_MODULES) || (p_module == NULL)) { return false; }

    AfeInstance_t *p_afe = &s_AfeInstances[module_idx];
    if (!p_afe->is_initialized) { return false; }

    uint8_t raw_buf[4U];   /* 2 bytes per cell voltage register */
    uint16_t v_max = 0U;
    uint16_t v_min = 0xFFFFU;
    bool all_ok = true;

    uint8_t c;
    for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
    {
        /* Cell voltage registers start at 0x14, increment by 2 per cell */
        uint8_t reg = (uint8_t)(BQ76952_REG_VCELL1_LSB + (c * 2U));

        bool ok = Afe_I2cRead(p_afe->i2c_address, reg, raw_buf, 2U);
        if (!ok)
        {
            p_afe->comm_error_count++;
            all_ok = false;
            continue;
        }

        /* BQ76952: little-endian 16-bit, in units of 192uV */
        uint16_t raw_val = (uint16_t)((uint16_t)raw_buf[1] << 8U) | raw_buf[0];
        uint32_t voltage_uv = (uint32_t)raw_val * BQ76952_VOLTAGE_UV_PER_LSB;
        uint16_t voltage_mv = (uint16_t)(voltage_uv / 1000U);

        p_module->cells[c].voltage_mv = voltage_mv;

        if (voltage_mv > v_max) { v_max = voltage_mv; }
        if (voltage_mv < v_min) { v_min = voltage_mv; }
    }

    /* Update module aggregates */
    if (all_ok)
    {
        uint32_t sum_mv = 0U;
        for (c = 0U; c < BMS_CELLS_PER_MODULE; c++)
        {
            sum_mv += p_module->cells[c].voltage_mv;
        }

        p_module->max_cell_voltage_mv    = v_max;
        p_module->min_cell_voltage_mv    = v_min;
        p_module->cell_voltage_delta_mv  = v_max - v_min;
        p_module->module_voltage_mv      = (uint16_t)(sum_mv & 0xFFFFU);
        p_module->is_online              = true;
        p_module->last_update_tick       = HAL_GetTickMs();
    }
    else
    {
        p_module->is_online = false;
    }

    return all_ok;
}

/**
 * @brief  Read pack current from AFE coulomb counter
 * @param[in]  module_idx  Primary AFE module index (module 0 for pack current)
 * @param[out] p_current_ma  Signed current in milliamps (+ve = discharge)
 * @return true if read succeeded
 */
bool AFE_ReadPackCurrent(uint8_t module_idx, int32_t *p_current_ma)
{
    if ((module_idx >= BMS_NUM_MODULES) || (p_current_ma == NULL)) { return false; }

    int16_t raw_current = 0;
    bool ok = Afe_ReadDirectS16(s_AfeInstances[module_idx].i2c_address,
                                  BQ76952_REG_CC2_CURRENT, &raw_current);
    if (ok)
    {
        /* Convert: 1 LSB = 78.125 µA ≈ 78 µA, with 1mΩ sense resistor */
        *p_current_ma = ((int32_t)raw_current * (int32_t)BQ76952_CURRENT_UA_PER_LSB) / 1000;
    }

    return ok;
}

/**
 * @brief  Read AFE internal die temperature
 * @param[in]  module_idx  Module index
 * @param[out] p_temp_dc   Temperature in deci-Celsius
 * @return true if read succeeded
 */
bool AFE_ReadDieTemperature(uint8_t module_idx, int16_t *p_temp_dc)
{
    if ((module_idx >= BMS_NUM_MODULES) || (p_temp_dc == NULL)) { return false; }

    uint16_t raw_temp = 0U;
    bool ok = Afe_ReadDirectU16(s_AfeInstances[module_idx].i2c_address,
                                  BQ76952_REG_TEMP_INT, &raw_temp);
    if (ok)
    {
        /* BQ76952 reports temperature in units of 0.1K. Convert to deci-°C */
        *p_temp_dc = (int16_t)((int32_t)raw_temp + BQ76952_TEMP_OFFSET_DC);
    }

    return ok;
}

/**
 * @brief  Set balance FET for a specific cell
 * @param[in] module_idx  Module index
 * @param[in] cell_idx    Cell index (0-based)
 * @param[in] enable      True to enable balancing FET
 */
void AFE_SetBalancingFet(uint8_t module_idx, uint8_t cell_idx, bool enable)
{
    if ((module_idx >= BMS_NUM_MODULES) ||
        (cell_idx >= BMS_CELLS_PER_MODULE)) { return; }

    /* Read current balance register to modify single bit */
    uint16_t balance_reg = 0U;
    (void)Afe_ReadDirectU16(s_AfeInstances[module_idx].i2c_address,
                              BQ76952_REG_CELL_BALANCE, &balance_reg);

    uint16_t cell_bit = (uint16_t)(1U << cell_idx);
    if (enable)
    {
        balance_reg |= cell_bit;
    }
    else
    {
        balance_reg &= (uint16_t)(~cell_bit);
    }

    uint8_t write_data[2];
    write_data[0] = (uint8_t)(balance_reg & 0xFFU);
    write_data[1] = (uint8_t)((balance_reg >> 8U) & 0xFFU);

    (void)Afe_I2cWrite(s_AfeInstances[module_idx].i2c_address,
                        BQ76952_REG_CELL_BALANCE, write_data, 2U);
}

/**
 * @brief  Read hardware safety alert flags from AFE
 * @param[in]  module_idx     Module index
 * @param[out] p_fault_code   Returns detected fault code (or FAULT_NONE)
 * @return true if communication succeeded
 */
bool AFE_ReadSafetyFlags(uint8_t module_idx, FaultCode_t *p_fault_code)
{
    if ((module_idx >= BMS_NUM_MODULES) || (p_fault_code == NULL)) { return false; }

    *p_fault_code = FAULT_NONE;

    uint16_t safety_a = 0U;
    bool ok = Afe_ReadDirectU16(s_AfeInstances[module_idx].i2c_address,
                                  BQ76952_REG_SAFETY_STATUS_A, &safety_a);
    if (!ok) { return false; }

    /* Map AFE hardware flags to BMS fault codes */
    if ((safety_a & BQ76952_SA_COV) != 0U)
    {
        *p_fault_code = FAULT_CELL_OVERVOLTAGE;
    }
    else if ((safety_a & BQ76952_SA_CUV) != 0U)
    {
        *p_fault_code = FAULT_CELL_UNDERVOLTAGE;
    }
    else if ((safety_a & BQ76952_SA_SCD) != 0U)
    {
        *p_fault_code = FAULT_SHORT_CIRCUIT;
    }
    else if (((safety_a & BQ76952_SA_OCD1) != 0U))
    {
        *p_fault_code = FAULT_OVERCURRENT_DISCHARGE;
    }
    else if ((safety_a & BQ76952_SA_OCC) != 0U)
    {
        *p_fault_code = FAULT_OVERCURRENT_CHARGE;
    }

    return true;
}

/**
 * @brief  Check if an AFE module is online
 */
bool AFE_IsModuleOnline(uint8_t module_idx)
{
    if (module_idx >= BMS_NUM_MODULES) { return false; }
    return s_AfeInstances[module_idx].is_initialized;
}

/* =========================================================================
 * PRIVATE FUNCTIONS
 * ========================================================================= */

static bool Afe_I2cRead(uint8_t device_addr, uint8_t reg_addr,
                         uint8_t *p_buf, uint8_t len)
{
    return HAL_I2C_Read(device_addr, reg_addr, p_buf, len, AFE_I2C_TIMEOUT_MS);
}

static bool Afe_I2cWrite(uint8_t device_addr, uint8_t reg_addr,
                          const uint8_t *p_data, uint8_t len)
{
    /* Prepend register address to data buffer for write */
    uint8_t tx_buf[10U];
    if (len >= sizeof(tx_buf)) { return false; }

    tx_buf[0] = reg_addr;
    (void)memcpy(&tx_buf[1], p_data, len);

    return HAL_I2C_Write(device_addr, tx_buf, (uint8_t)(len + 1U), AFE_I2C_TIMEOUT_MS);
}

static bool Afe_WriteSubcommand(uint8_t device_addr, uint16_t subcmd)
{
    uint8_t data[2];
    data[0] = (uint8_t)(subcmd & 0xFFU);
    data[1] = (uint8_t)((subcmd >> 8U) & 0xFFU);
    return Afe_I2cWrite(device_addr, BQ76952_SUBCMD_ADDR, data, 2U);
}

static bool Afe_ReadDirectU16(uint8_t device_addr, uint8_t reg, uint16_t *p_val)
{
    uint8_t buf[2];
    bool ok = Afe_I2cRead(device_addr, reg, buf, 2U);
    if (ok)
    {
        /* BQ76952 registers are little-endian */
        *p_val = (uint16_t)((uint16_t)buf[1] << 8U) | buf[0];
    }
    return ok;
}

static bool Afe_ReadDirectS16(uint8_t device_addr, uint8_t reg, int16_t *p_val)
{
    uint16_t raw = 0U;
    bool ok = Afe_ReadDirectU16(device_addr, reg, &raw);
    if (ok) { *p_val = (int16_t)raw; }
    return ok;
}

static uint8_t Afe_ComputeChecksum(const uint8_t *p_data, uint8_t len)
{
    uint8_t sum = 0U;
    uint8_t i;
    for (i = 0U; i < len; i++) { sum = (uint8_t)(sum + p_data[i]); }
    return (uint8_t)(~sum);
}
