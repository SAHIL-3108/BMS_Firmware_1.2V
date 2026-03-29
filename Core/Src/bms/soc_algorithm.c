/**
 * @file    soc_algorithm.c
 * @brief   SOC/SOH Estimation — Extended Kalman Filter + Coulomb Counting
 * @version 1.0.0
 *
 * @details Implements a 1-state Extended Kalman Filter (EKF) for robust
 *          State of Charge estimation, fused with Coulomb Counting for
 *          open-loop integration between observation updates.
 *
 *          The battery model used:
 *            V_terminal = OCV(SOC) - I * R_internal
 *
 *          EKF State:      x = SOC  [0.0 to 1.0]
 *          Observation:    z = V_terminal (measured)
 *          Input:          u = pack current I (A)
 *
 *          SOH is tracked via capacity fade: comparing accumulated Ah
 *          throughput over full charge cycles against nominal capacity.
 *
 * MISRA C:2012 Rules applied:
 *   - Rule 15.5: Single exit point per function where feasible
 *   - Rule 14.4: Boolean expressions used only in boolean context
 *   - Rule 10.4: Operands in arithmetic use same type
 */

#include "soc_algorithm.h"
#include "battery_structs.h"
#include "bms_config.h"
#include <math.h>   /* fabsf, sqrtf */
#include <string.h> /* memset */

/* =========================================================================
 * OCV - SOC LOOKUP TABLE (NMC Chemistry, measured at 25°C)
 * =========================================================================
 * Format: {SOC_percent [0-100], OCV_millivolts}
 * Resolution: 5% SOC steps — interpolation used between entries.
 */
typedef struct
{
    float   soc_pct;
    float   ocv_mv;
} OcvSocPoint_t;

static const OcvSocPoint_t k_OcvTable[] =
{
    { 0.0F,   3200.0F },
    { 5.0F,   3350.0F },
    { 10.0F,  3450.0F },
    { 15.0F,  3520.0F },
    { 20.0F,  3570.0F },
    { 25.0F,  3610.0F },
    { 30.0F,  3640.0F },
    { 35.0F,  3660.0F },
    { 40.0F,  3680.0F },
    { 45.0F,  3700.0F },
    { 50.0F,  3720.0F },
    { 55.0F,  3740.0F },
    { 60.0F,  3760.0F },
    { 65.0F,  3790.0F },
    { 70.0F,  3830.0F },
    { 75.0F,  3880.0F },
    { 80.0F,  3950.0F },
    { 85.0F,  4020.0F },
    { 90.0F,  4080.0F },
    { 95.0F,  4140.0F },
    { 100.0F, 4200.0F },
};

#define OCV_TABLE_SIZE  ((uint8_t)(sizeof(k_OcvTable) / sizeof(k_OcvTable[0])))

/* Internal resistance estimate (ohms) — can be made SOC/temp dependent */
#define R_INTERNAL_OHM      (0.020F)  /* 20 mOhm typical for NMC 100Ah pack */

/* Seconds-to-hours conversion */
#define SEC_TO_HOURS        (1.0F / 3600.0F)

/* =========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ========================================================================= */
static float    Soc_OcvLookup(float soc_pct);
static float    Soc_InverseOcvLookup(float ocv_mv);
static float    Soc_LinearInterpolate(float x, float x0, float x1,
                                       float y0, float y1);
static void     Soc_EkfPredict(EkfState_t *p_ekf, float current_a,
                                float capacity_ah);
static float    Soc_EkfUpdate(EkfState_t *p_ekf, float terminal_voltage_mv,
                               float current_a);
static float    Soc_ComputeDOcvDSoc(float soc_pct);
static void     Soc_UpdateSoh(SocSohData_t *p_soc_soh, float charged_ah);

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * @brief Initialize the SOC estimation module
 * @param[in,out] p_soc_soh  Pointer to SOC/SOH data structure
 * @param[in]     initial_soc_pct  Starting SOC estimate (e.g., from EEPROM)
 * @param[in]     soh_pct    Known State of Health (from EEPROM)
 */
void Soc_Init(SocSohData_t *p_soc_soh, float initial_soc_pct, float soh_pct)
{
    if (p_soc_soh == NULL) { return; }

    (void)memset(p_soc_soh, 0, sizeof(SocSohData_t));

    /* Clamp initial SOC to valid range */
    float init_soc = CLAMP(initial_soc_pct, 0.0F, 100.0F);

    p_soc_soh->soc_percent          = init_soc;
    p_soc_soh->soh_percent          = CLAMP(soh_pct, 0.0F, 100.0F);
    p_soc_soh->is_estimate_valid    = false;

    /* Initialize EKF */
    EkfState_t *p_ekf = &p_soc_soh->ekf;
    p_ekf->soc_estimate     = init_soc / 100.0F;  /* Normalize to [0,1] */
    p_ekf->P                = EKF_INITIAL_COVARIANCE_P;
    p_ekf->Q                = EKF_PROCESS_NOISE_Q;
    p_ekf->R                = EKF_MEASUREMENT_NOISE_R;
    p_ekf->dt_s             = EKF_DT_SECONDS;
    p_ekf->last_current_a   = 0.0F;
    p_ekf->is_initialized   = true;
}

/**
 * @brief  Run one EKF + Coulomb Counting estimation cycle
 * @param[in,out] p_soc_soh         SOC/SOH state structure
 * @param[in]     terminal_voltage_mv  Measured terminal voltage
 * @param[in]     current_a            Pack current (positive = discharge)
 * @param[in]     dt_s                 Time delta since last call (seconds)
 * @return        Estimated SOC in percent [0.0 - 100.0]
 *
 * @details This function:
 *   1. EKF Predict: propagates SOC using Coulomb Counting model
 *   2. EKF Update:  corrects estimate using OCV-SOC model + measured voltage
 *   3. Updates accumulated Ah for SOH tracking
 */
float Soc_Update(SocSohData_t *p_soc_soh,
                  float terminal_voltage_mv,
                  float current_a,
                  float dt_s)
{
    float result_soc = 0.0F;

    if ((p_soc_soh == NULL) || (!p_soc_soh->ekf.is_initialized))
    {
        return SOC_MIN_PERCENT;
    }

    /* Use effective capacity (nominal × SOH) */
    float effective_capacity_ah = BMS_NOMINAL_CAPACITY_AH *
                                   (p_soc_soh->soh_percent / 100.0F);

    if (effective_capacity_ah <= 0.0F) { effective_capacity_ah = 1.0F; }  /* Guard */

    /* Update time step */
    p_soc_soh->ekf.dt_s = dt_s;

    /* --- STEP 1: EKF PREDICT (Coulomb Counting) --- */
    Soc_EkfPredict(&p_soc_soh->ekf, current_a, effective_capacity_ah);

    /* --- STEP 2: EKF UPDATE (Voltage observation) --- */
    /* Only update when current is low (OCV model more accurate near rest) */
    float abs_current = fabsf(current_a);
    float updated_soc;

    if (abs_current < 5.0F)  /* Near-rest condition: trust voltage measurement */
    {
        updated_soc = Soc_EkfUpdate(&p_soc_soh->ekf,
                                     terminal_voltage_mv,
                                     current_a);
    }
    else
    {
        /* High current: rely on prediction (Coulomb Counting dominates) */
        updated_soc = p_soc_soh->ekf.soc_estimate;
    }

    /* --- STEP 3: Convert to percent and clamp --- */
    result_soc = CLAMP(updated_soc * 100.0F, SOC_MIN_PERCENT, SOC_MAX_PERCENT);

    /* --- STEP 4: Accumulate Ah for SOH tracking --- */
    float delta_ah = fabsf(current_a) * (dt_s * SEC_TO_HOURS);
    p_soc_soh->coulombs_accumulated += delta_ah;

    /* --- STEP 5: Update State of Energy --- */
    float ocv_mv = Soc_OcvLookup(result_soc);
    p_soc_soh->soe_wh = (result_soc / 100.0F) * effective_capacity_ah *
                         (ocv_mv / 1000.0F);

    p_soc_soh->soc_percent      = result_soc;
    p_soc_soh->is_estimate_valid = true;

    return result_soc;
}

/**
 * @brief  Initialize SOC from OCV measurement (at rest, contactors open)
 * @param[in,out] p_soc_soh         SOC/SOH state structure
 * @param[in]     ocv_mv            Open circuit voltage in millivolts
 * @details Call this on boot if pack has been at rest for >30 minutes.
 *          Provides accurate initial SOC seed for EKF.
 */
void Soc_InitFromOcv(SocSohData_t *p_soc_soh, float ocv_mv)
{
    if (p_soc_soh == NULL) { return; }

    float soc_estimate = Soc_InverseOcvLookup(ocv_mv);
    p_soc_soh->soc_percent          = soc_estimate;
    p_soc_soh->ekf.soc_estimate     = soc_estimate / 100.0F;
    p_soc_soh->ekf.P                = EKF_INITIAL_COVARIANCE_P * 0.1F;  /* High confidence */
    p_soc_soh->is_estimate_valid    = true;
}

/**
 * @brief  Trigger SOH update at end of a full charge cycle
 * @param[in,out] p_soc_soh     SOC/SOH data structure
 * @param[in]     charged_ah    Total Ah delivered during this charge cycle
 */
void Soc_OnFullChargeCycleComplete(SocSohData_t *p_soc_soh, float charged_ah)
{
    if (p_soc_soh == NULL) { return; }

    p_soc_soh->cycle_count++;
    Soc_UpdateSoh(p_soc_soh, charged_ah);
    p_soc_soh->coulombs_accumulated = 0.0F;  /* Reset for next cycle */
}

/**
 * @brief  Get current SOC estimate
 * @return SOC in percent [0.0 - 100.0]
 */
float Soc_GetPercent(const SocSohData_t *p_soc_soh)
{
    if (p_soc_soh == NULL) { return 0.0F; }
    return p_soc_soh->soc_percent;
}

/**
 * @brief  Get current SOH estimate
 * @return SOH in percent [0.0 - 100.0]
 */
float Soc_GetSohPercent(const SocSohData_t *p_soc_soh)
{
    if (p_soc_soh == NULL) { return 0.0F; }
    return p_soc_soh->soh_percent;
}

/* =========================================================================
 * PRIVATE FUNCTIONS
 * ========================================================================= */

/**
 * @brief  EKF Prediction step (Coulomb Counting)
 * @details State equation:  SOC(k+1) = SOC(k) - (I * dt) / (3600 * Capacity)
 *          Jacobian F = 1 (linear state transition)
 *          P(k+1) = F * P(k) * F' + Q  →  P = P + Q
 */
static void Soc_EkfPredict(EkfState_t *p_ekf, float current_a,
                             float capacity_ah)
{
    /* State prediction: Coulomb Counting */
    float delta_soc = -(current_a * p_ekf->dt_s) / (3600.0F * capacity_ah);
    p_ekf->soc_estimate += delta_soc;
    p_ekf->soc_estimate  = CLAMP(p_ekf->soc_estimate, 0.0F, 1.0F);

    /* Covariance prediction: P = P + Q */
    p_ekf->P += p_ekf->Q;

    p_ekf->last_current_a = current_a;
}

/**
 * @brief  EKF Update step (Voltage measurement correction)
 * @details Observation equation:  z = OCV(SOC) - I * R_int
 *          H = dOCV/dSOC (Jacobian of observation model)
 *          K = P * H / (H * P * H + R)   (Kalman gain)
 *          x = x + K * (z - h(x))        (state update)
 *          P = (1 - K * H) * P            (covariance update)
 * @return  Updated SOC estimate [0.0, 1.0]
 */
static float Soc_EkfUpdate(EkfState_t *p_ekf, float terminal_voltage_mv,
                             float current_a)
{
    float soc_pct = p_ekf->soc_estimate * 100.0F;

    /* Predicted terminal voltage from model */
    float ocv_predicted_mv  = Soc_OcvLookup(soc_pct);
    float v_predicted_mv    = ocv_predicted_mv - (current_a * R_INTERNAL_OHM * 1000.0F);

    /* Innovation (measurement residual) */
    float innovation = terminal_voltage_mv - v_predicted_mv;

    /* Observation Jacobian: H = dOCV/dSOC (in mV per unit SOC) */
    float H = Soc_ComputeDOcvDSoc(soc_pct);  /* mV / (SOC fraction) */

    /* Innovation covariance: S = H * P * H + R */
    float S = (H * p_ekf->P * H) + p_ekf->R;

    if (fabsf(S) < 1.0E-9F) { return p_ekf->soc_estimate; }  /* Guard div-by-zero */

    /* Kalman gain */
    float K = (p_ekf->P * H) / S;

    /* State update */
    p_ekf->soc_estimate += K * innovation;
    p_ekf->soc_estimate  = CLAMP(p_ekf->soc_estimate, 0.0F, 1.0F);

    /* Covariance update: P = (1 - K*H) * P */
    p_ekf->P = (1.0F - (K * H)) * p_ekf->P;

    /* Clamp covariance to prevent negative/explosive values */
    p_ekf->P = CLAMP(p_ekf->P, 1.0E-9F, 10.0F);

    return p_ekf->soc_estimate;
}

/**
 * @brief  Look up OCV from SOC using linear interpolation on OCV table
 * @param[in] soc_pct  SOC in percent [0.0 - 100.0]
 * @return OCV in millivolts
 */
static float Soc_OcvLookup(float soc_pct)
{
    float soc_clamped = CLAMP(soc_pct, 0.0F, 100.0F);
    uint8_t i;

    /* Scan table for bracketing entry */
    for (i = 1U; i < OCV_TABLE_SIZE; i++)
    {
        if (soc_clamped <= k_OcvTable[i].soc_pct)
        {
            return Soc_LinearInterpolate(
                soc_clamped,
                k_OcvTable[i - 1U].soc_pct, k_OcvTable[i].soc_pct,
                k_OcvTable[i - 1U].ocv_mv,  k_OcvTable[i].ocv_mv);
        }
    }

    return k_OcvTable[OCV_TABLE_SIZE - 1U].ocv_mv;  /* Return max OCV if at 100% */
}

/**
 * @brief  Inverse OCV lookup: estimate SOC from a measured OCV
 * @param[in] ocv_mv  Measured open-circuit voltage in millivolts
 * @return Estimated SOC in percent [0.0 - 100.0]
 */
static float Soc_InverseOcvLookup(float ocv_mv)
{
    uint8_t i;

    if (ocv_mv <= k_OcvTable[0].ocv_mv)     { return 0.0F;   }
    if (ocv_mv >= k_OcvTable[OCV_TABLE_SIZE - 1U].ocv_mv) { return 100.0F; }

    for (i = 1U; i < OCV_TABLE_SIZE; i++)
    {
        if (ocv_mv <= k_OcvTable[i].ocv_mv)
        {
            return Soc_LinearInterpolate(
                ocv_mv,
                k_OcvTable[i - 1U].ocv_mv, k_OcvTable[i].ocv_mv,
                k_OcvTable[i - 1U].soc_pct, k_OcvTable[i].soc_pct);
        }
    }

    return 100.0F;
}

/**
 * @brief  Linear interpolation helper
 */
static float Soc_LinearInterpolate(float x, float x0, float x1,
                                    float y0, float y1)
{
    float dx = x1 - x0;
    if (fabsf(dx) < 1.0E-9F) { return y0; }  /* Guard against division by zero */

    return y0 + ((y1 - y0) * (x - x0)) / dx;
}

/**
 * @brief  Numerically compute dOCV/dSOC at a given SOC for EKF Jacobian
 * @param[in] soc_pct  SOC in percent
 * @return dOCV/dSOC in millivolts per unit SOC fraction
 */
static float Soc_ComputeDOcvDSoc(float soc_pct)
{
    /* Finite difference with small delta: H ≈ ΔOCV / ΔSOC */
    const float DELTA = 1.0F;  /* 1% SOC delta */
    float ocv_high = Soc_OcvLookup(CLAMP(soc_pct + DELTA, 0.0F, 100.0F));
    float ocv_low  = Soc_OcvLookup(CLAMP(soc_pct - DELTA, 0.0F, 100.0F));

    /* Convert to per-unit SOC fraction: dOCV_mv / d(SOC_fraction) */
    return (ocv_high - ocv_low) / (2.0F * DELTA / 100.0F);
}

/**
 * @brief  Update State of Health based on charge throughput
 * @details Capacity fade model:  SOH = f(cycle_count, Ah_throughput)
 *          Simple linear model here; replace with empirical cell data.
 */
static void Soc_UpdateSoh(SocSohData_t *p_soc_soh, float charged_ah)
{
    /* Simple capacity fade: 0.003% per cycle (100%-to-70% over ~10000 cycles) */
    const float FADE_PER_CYCLE = 0.003F;

    if (p_soc_soh->soh_percent > 0.0F)
    {
        /* Fade based on how close this charge was to nominal capacity */
        float charge_efficiency = charged_ah / BMS_NOMINAL_CAPACITY_AH;
        p_soc_soh->soh_percent -= FADE_PER_CYCLE * charge_efficiency;
        p_soc_soh->soh_percent  = CLAMP(p_soc_soh->soh_percent, 0.0F, 100.0F);
    }
}
