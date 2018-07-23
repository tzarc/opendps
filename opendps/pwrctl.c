/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Johan Kanflo (github.com/kanflo)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "pwrctl.h"
#include "tick.h"
#include "hw.h"
#include "dbg_printf.h"
#include "dps-model.h"
#include "pastunits.h"
#include <gpio.h>
#include <dac.h>

/**********************/

#define DEFAULT_CV_KP 0.65
#define DEFAULT_CV_KI 0.058
#define DEFAULT_CV_KD 0.26
#define DEFAULT_CV_KB 1

typedef struct pidctrl_t
{
    uint64_t last_ticks;
    float last_err;
    float integral;
    float Kp, Ki, Kd, Kb; // proportional, integral, derivative, bias

    float control_limit_lo;
    float control_limit_hi;
} pidctrl_t;

void reset_pidctrl(pidctrl_t *pidctrl, float Kp, float Ki, float Kd, float Kb, float control_limit_lo, float control_limit_hi);
uint32_t update_pidctrl(pidctrl_t *pidctrl, uint64_t ticks, uint32_t out_intended, uint32_t out_measured);

void reset_pidctrl(pidctrl_t *pidctrl, float Kp, float Ki, float Kd, float Kb, float control_limit_lo, float control_limit_hi)
{
    pidctrl->last_ticks = get_ticks();
    pidctrl->last_err = 0;
    pidctrl->integral = 0;
    pidctrl->Kp = Kp;
    pidctrl->Ki = Ki;
    pidctrl->Kd = Kd;
    pidctrl->Kb = Kb;
    pidctrl->control_limit_lo = control_limit_lo;
    pidctrl->control_limit_hi = control_limit_hi;
}

#ifdef CONFIG_COMMANDLINE
static int counter = 0;
#endif // CONFIG_COMMANDLINE

uint32_t update_pidctrl(pidctrl_t *pidctrl, uint64_t ticks, uint32_t out_intended, uint32_t out_measured)
{
    // Work out how long between calls
    float timeDelta = ticks - pidctrl->last_ticks;
    pidctrl->last_ticks = ticks;

    // Get the last error, so we can run a derivative
    float last_err = pidctrl->last_err;

    // Work out the coefficients
    float err = (float)out_intended - (float)out_measured;
    float integral = pidctrl->integral + (err * timeDelta);
    float derivative = (err - last_err) / timeDelta;

    // Save the last error
    pidctrl->last_err = err;

    // Perform the PID calculation
    float out_pid = (pidctrl->Kp * err) + (pidctrl->Ki * integral) + (pidctrl->Kd * derivative) + pidctrl->Kb;

    // Work out if we need to limit the control output
    bool wasLimited = false;
    if(out_pid < pidctrl->control_limit_lo)
    {
        wasLimited = true;
        out_pid = pidctrl->control_limit_lo;
    }
    if(out_pid > pidctrl->control_limit_hi)
    {
        wasLimited = true;
        out_pid = pidctrl->control_limit_hi;
    }

    // Only if we're in the allowed range do we try to update the integral, otherwise we could get wild variation (avoiding integral windup)
    if(!wasLimited)
        pidctrl->integral = integral;

#ifdef CONFIG_COMMANDLINE
    ++counter;
    if(counter == 1000) {
        dbg_printf("\n");
        dbg_printf("Vp = %d, Vi = %d, Vd = %d, Vb = %d\n", (int)(pidctrl->Kp * err), (int)(pidctrl->Ki * integral), (int)(pidctrl->Kd * derivative), (int)pidctrl->Kb);
        dbg_printf("dt = %d, err = %d, integral = %d, derivative = %d\n", (int)timeDelta, (int)err, (int)integral, (int)derivative);
        dbg_printf("PID output: %d, intended: %d, measured: %d, PID control: %d\n", (int)(out_intended + out_pid), out_intended, out_measured, (int)out_pid);
        dbg_printf("Kp = %d, Ki = %d, Kd = %d, Kb = %d, last_err = %d, integral = %d\n", (int)(pidctrl->Kp*1000), (int)(pidctrl->Ki*1000), (int)(pidctrl->Kd*1000), (int)pidctrl->Kb, (int)pidctrl->last_err, (int)pidctrl->integral);
        counter = 0;
    }
#endif // CONFIG_COMMANDLINE

    // Output the control value
    return (uint32_t)out_pid;
}

/**********************/

/** This module handles voltage and current calculations
  * Calculations based on measurements found at
  * https://docs.google.com/spreadsheets/d/1AhGsU_gvZjqZyr2ZYrnkz6BeUqMquzh9UNYoTqy_Zp4/edit?usp=sharing
  */

static uint32_t i_out, v_out, i_limit;
static bool v_out_enabled;
static pidctrl_t pidctrl_cv;
static int tick_counter = 0;

float a_adc_k_coef = A_ADC_K;
float a_adc_c_coef = A_ADC_C;
float a_dac_k_coef = A_DAC_K;
float a_dac_c_coef = A_DAC_C;
float v_adc_k_coef = V_ADC_K;
float v_adc_c_coef = V_ADC_C;
float v_dac_k_coef = V_DAC_K;
float v_dac_c_coef = V_DAC_C;
float vin_adc_k_coef = VIN_ADC_K;
float vin_adc_c_coef = VIN_ADC_C;

/** not static as it is referred to from hw.c for performance reasons */
uint32_t pwrctl_i_limit_raw;

/**
  * @brief Timer callback for handling the output voltage PID controller
  * @retval none
  */
static void pwrctl_systick_callback(uint64_t tick_ms)
{
    ++tick_counter;
    if(tick_counter < 10) // Only update once every 10ms.
        return;
    tick_counter = 0;

    if(v_out_enabled) {
        uint16_t i_out_raw, v_in_raw, v_out_raw;
        hw_get_adc_values(&i_out_raw, &v_in_raw, &v_out_raw);
        uint32_t v_measured = pwrctl_calc_vout(v_out_raw);
        uint32_t pid = update_pidctrl(&pidctrl_cv, tick_ms, v_out, v_measured);
        DAC_DHR12R1 = pwrctl_calc_vout_dac(v_out + pid);
    }
}

/**
  * @brief Initialize the power control module
  * @retval none
  */
void pwrctl_init(past_t *past)
{
    uint32_t length;
    float *p;

    /** Load default calibration constants */
    a_adc_k_coef = A_ADC_K;
    a_adc_c_coef = A_ADC_C;
    a_dac_k_coef = A_DAC_K;
    a_dac_c_coef = A_DAC_C;
    v_adc_k_coef = V_ADC_K;
    v_adc_c_coef = V_ADC_C;
    v_dac_k_coef = V_DAC_K;
    v_dac_c_coef = V_DAC_C;
    vin_adc_k_coef = VIN_ADC_K;
    vin_adc_c_coef = VIN_ADC_C;

    /** Load any calibration constants that maybe stored in non-volatile memory (past) */
    if (past_read_unit(past, past_A_ADC_K, (const void**) &p, &length))
        a_adc_k_coef = *p;
    if (past_read_unit(past, past_A_ADC_C, (const void**) &p, &length))
        a_adc_c_coef = *p;
    if (past_read_unit(past, past_A_DAC_K, (const void**) &p, &length))
        a_dac_k_coef = *p;
    if (past_read_unit(past, past_A_DAC_C, (const void**) &p, &length))
        a_dac_c_coef = *p;
    if (past_read_unit(past, past_V_ADC_K, (const void**) &p, &length))
        v_adc_k_coef = *p;
    if (past_read_unit(past, past_V_ADC_C, (const void**) &p, &length))
        v_adc_c_coef = *p;
    if (past_read_unit(past, past_V_DAC_K, (const void**) &p, &length))
        v_dac_k_coef = *p;
    if (past_read_unit(past, past_V_DAC_C, (const void**) &p, &length))
        v_dac_c_coef = *p;
    if (past_read_unit(past, past_VIN_ADC_K, (const void**) &p, &length))
        vin_adc_k_coef = *p;
    if (past_read_unit(past, past_VIN_ADC_C, (const void**) &p, &length))
        vin_adc_c_coef = *p;

    pwrctl_enable_vout(false);
    set_systick_callback(pwrctl_systick_callback);
}

/**
  * @brief Set voltage output
  * @param value_mv voltage in milli volt
  * @retval true requested voltage was within specs
  */
bool pwrctl_set_vout(uint32_t value_mv)
{
    /** @todo Check with max Vout, currently filtered by ui.c */
    v_out = value_mv;
    if (v_out_enabled) {
        reset_pidctrl(&pidctrl_cv, DEFAULT_CV_KP, DEFAULT_CV_KI, DEFAULT_CV_KD, DEFAULT_CV_KB, -(float)(value_mv/10), +(float)(value_mv/10));
        /** Needed for the DPS5005 "communications version" (the one with BT/USB) */
        DAC_DHR12R1 = pwrctl_calc_vout_dac(v_out);
    } else {
        reset_pidctrl(&pidctrl_cv, DEFAULT_CV_KP, DEFAULT_CV_KI, DEFAULT_CV_KD, DEFAULT_CV_KB, 0, 0);
        DAC_DHR12R1 = 0;
    }
    return true;
}

/**
  * @brief Set current output
  * @param current_ma current in milli ampere
  * @retval true requested current was within specs
  */
bool pwrctl_set_iout(uint32_t value_ma)
{
    i_out = value_ma;
    if (v_out_enabled) {
        DAC_DHR12R2 = pwrctl_calc_iout_dac(value_ma);
    } else {
        DAC_DHR12R2 = 0;
    }
    return true;
}

/**
  * @brief Get current output setting
  * @retval current setting in milli amps
  */
uint32_t pwrctl_get_iout(void)
{
    return i_out;
}

/**
  * @brief Get voltage output setting
  * @retval current setting in milli volt
  */
uint32_t pwrctl_get_vout(void)
{
    return v_out;
}

/**
  * @brief Set current limit
  * @param value_ma limit in milliampere
  * @retval true requested current was within specs
  */
bool pwrctl_set_ilimit(uint32_t value_ma)
{
    /** @todo Check with I_limit, currently filtered by ui.c */
    i_limit = value_ma;
    pwrctl_i_limit_raw = pwrctl_calc_ilimit_adc(i_limit);
    return true;
}

/**
  * @brief Get current limit setting
  * @retval current setting in milliampere
  */
uint32_t pwrctl_get_ilimit(void)
{
    return i_limit;
}

/**
  * @brief Enable or disable power output
  * @param enable true for enable, false for disable
  * @retval none
  */
void pwrctl_enable_vout(bool enable)
{
    v_out_enabled = enable;
    if (v_out_enabled) {
      (void) pwrctl_set_vout(v_out);
      (void) pwrctl_set_iout(i_out);
#ifdef DPS5015
        //gpio_clear(GPIOA, GPIO9); // this is power control on '5015
        gpio_set(GPIOB, GPIO11);    // B11 is fan control on '5015
        gpio_clear(GPIOC, GPIO13);  // C13 is power control on '5015
#else
        gpio_clear(GPIOB, GPIO11);  // B11 is power control on '5005
#endif
    } else {
#ifdef DPS5015
        //gpio_set(GPIOA, GPIO9);    // gpio_set(GPIOB, GPIO11);
        gpio_clear(GPIOB, GPIO11); // B11 is fan control on '5015
        gpio_set(GPIOC, GPIO13);   // C13 is power control on '5015
#else
        gpio_set(GPIOB, GPIO11);  // B11 is power control on '5005
#endif
      (void) pwrctl_set_vout(v_out);
      (void) pwrctl_set_iout(i_out);
    }
}

/**
  * @brief Return power output status
  * @retval true if power output is wnabled
  */
bool pwrctl_vout_enabled(void)
{
    return v_out_enabled;
}

/**
  * @brief Calculate V_in based on raw ADC measurement
  * @param raw value from ADC
  * @retval corresponding voltage in milli volt
  */
uint32_t pwrctl_calc_vin(uint16_t raw)
{
    float value = vin_adc_k_coef * raw + vin_adc_c_coef;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; /** Add 0.5f to value so it is correctly rounded when it is truncated */
}

/**
  * @brief Calculate V_out based on raw ADC measurement
  * @param raw value from ADC
  * @retval corresponding voltage in milli volt
  */
uint32_t pwrctl_calc_vout(uint16_t raw)
{
    float value = v_adc_k_coef * raw + v_adc_c_coef;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; /** Add 0.5f to value so it is correctly rounded when it is truncated */
}

/**
  * @brief Calculate DAC setting for requested V_out
  * @param v_out_mv requested output voltage
  * @retval corresponding 12 bit DAC value
  */
uint16_t pwrctl_calc_vout_dac(uint32_t v_out_mv)
{
    float value = v_dac_k_coef * v_out_mv + v_dac_c_coef;
    if (value <= 0)
        return 0;
    else if (value >= 0xfff)
        return 0xfff; /** 12 bits */
    else
        return value + 0.5f; /** Add 0.5f to value so correct rounding is done when truncated */
}

/**
  * @brief Calculate I_out based on raw ADC measurement
  * @param raw value from ADC
  * @retval corresponding current in milliampere
  */
uint32_t pwrctl_calc_iout(uint16_t raw)
{
    float value = a_adc_k_coef * raw + a_adc_c_coef;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; /** Add 0.5f to value so correct rounding is done when truncated */
}

/**
  * @brief Calculate expected raw ADC value based on selected I_limit
  * @param i_limit_ma selected I_limit
  * @retval expected raw ADC value
  */
uint16_t pwrctl_calc_ilimit_adc(uint16_t i_limit_ma)
{
    float value = (i_limit_ma - a_adc_c_coef) / a_adc_k_coef + 1;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; // Add 0.5 so it is correctly rounded when it is truncated
}

/**
  * @brief Calculate DAC setting for constant current mode
  * @param i_out_ma requested constant current
  * @retval corresponding 12 bit DAC value
  * @note this formula is valid for the DPS5005 and would probably need changes
  *       for DPS:es capable of higher current output.
  */
uint16_t pwrctl_calc_iout_dac(uint32_t i_out_ma)
{
    float value = a_dac_k_coef * i_out_ma + a_dac_c_coef;
    if (value <= 0)
        return 0;
    else if (value >= 0xfff)
        return 0xfff; /** 12 bits */
    else
        return value + 0.5f; /** Add 0.5f to value so correct rounding is done when truncated */
}
