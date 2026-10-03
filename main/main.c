/*
 * ================================================================
 * ESP32 + TWO PT100 RTDs + L298N
 *
 * Differential-temperature PID heater controller
 *
 * ================================================================
 *
 * SYSTEM
 * ================================================================
 *
 *      RTD2                 HEATER                 RTD1
 *   (UPSTREAM)                                      (DOWNSTREAM)
 *
 *       │                      │                       │
 *       │                      │                       │
 *       ▼                      ▼                       ▼
 *    PT100 #1              Heating                 PT100 #2
 *       │                   Element                    │
 *       ▼                      │                       ▼
 *    GPIO34                   │                    GPIO35
 *       │                      │                       │
 *       └──────────────┐       │       ┌───────────────┘
 *                      ▼       ▼       ▼
 *                    ESP32 TEMPERATURE
 *                         PROCESSING
 *                             │
 *                             ▼
 *                     DELTA TEMPERATURE
 *
 *                 Delta T = T2 - T1
 *
 *                             │
 *                             ▼
 *                           PID
 *                             │
 *                             ▼
 *                           PWM
 *                             │
 *                             ▼
 *                          GPIO27
 *                             │
 *                             ▼
 *                           ENA
 *                         ┌───────┐
 * GPIO32 ─────────────────► IN1   │
 * GPIO25 ─────────────────► IN2   │
 *                         │ L298N │
 *                         └───┬───┘
 *                             │
 *                             ▼
 *                           HEATER
 *
 *
 * ================================================================
 * PIN ASSIGNMENT
 * ================================================================
 *
 * L298N:
 *
 * GPIO27 -> ENA (PWM)
 * GPIO32 -> IN1
 * GPIO25 -> IN2
 *
 * RTD1:
 *
 * GPIO34 -> ADC1_CHANNEL_6
 *
 * RTD2:
 *
 * GPIO35 -> ADC1_CHANNEL_7
 *
 * ================================================================
 *
 * IMPORTANT:
 *
 * Remove the ENA jumper on the L298N.
 *
 * IN1 = HIGH
 * IN2 = LOW
 *
 * ENA receives PWM from GPIO27.
 *
 * ================================================================
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"


/* ================================================================
 * USER CONFIGURATION
 * ================================================================ */


/* ================================================================
 * L298N GPIO
 * ================================================================ */

#define GPIO_L298_PWM       27
#define GPIO_L298_IN1       32
#define GPIO_L298_IN2       25


/* ================================================================
 * RTD GPIO / ADC
 * ================================================================ */


/*
 * RTD1 = UPSTREAM
 *
 * GPIO34 = ADC1_CHANNEL_6
 */

#define GPIO_RTD1_ADC       34
#define RTD1_ADC_CHANNEL    ADC1_CHANNEL_6


/*
 * RTD2 = DOWNSTREAM
 *
 * GPIO35 = ADC1_CHANNEL_7
 */

#define GPIO_RTD2_ADC       35
#define RTD2_ADC_CHANNEL    ADC1_CHANNEL_7


/* ================================================================
 * PT100 CONFIGURATION
 * ================================================================ */


/*
 * Nominal divider supply.
 *
 * Used only for diagnostic raw resistance.
 */

#define DIVIDER_SUPPLY_V        3.300f


/*
 * RREF for each PT100.
 *
 * Current hardware:
 *
 * 330 ohm
 */

#define RTD1_RREF               330.0f
#define RTD2_RREF               330.0f


/*
 * PT100 resistance at 0°C.
 */

#define PT100_R0                100.0f


/*
 * Callendar-Van Dusen coefficients.
 */

#define PT100_A                 3.9083e-3f
#define PT100_B                 -5.775e-7f
#define PT100_C                 -4.183e-12f


/* ================================================================
 * STARTUP CALIBRATION
 * ================================================================ */


/*
 * Both RTDs must be at this known temperature when ESP32 boots.
 *
 * Since you have been using approximately 25°C as the reference:
 */

#define CALIBRATION_TEMPERATURE_C       25.0f


/*
 * Number of calibration measurements.
 */

#define CALIBRATION_SAMPLES             1000


/*
 * Delay between calibration measurements.
 */

#define CALIBRATION_DELAY_MS             10


/*
 * Initial settling time.
 */

#define CALIBRATION_SETTLE_MS           2000


/*
 * Automatically calibrate on every boot.
 */

#define AUTO_CALIBRATE_ON_BOOT           1


/* ================================================================
 * ADC CONFIGURATION
 * ================================================================ */

#define ADC_SAMPLES                     32

#define ADC_ATTENUATION                 ADC_ATTEN_DB_2_5

#define ADC_DEFAULT_VREF_MV             1100


/* ================================================================
 * TEMPERATURE FILTER
 * ================================================================ */


/*
 * How often the complete control loop executes.
 *
 * Thermal systems are slow, so 500 ms is a reasonable starting
 * point and is less sensitive to measurement noise than 100 ms.
 */

#define CONTROL_PERIOD_MS               500


/*
 * 3-point median filter.
 */

#define MEDIAN_FILTER_SIZE              3


/*
 * EMA smoothing.
 *
 * 1.0 = no smoothing
 * 0.5 = moderate smoothing
 * 0.2 = strong smoothing
 */

#define EMA_ALPHA                       0.50f


/* ================================================================
 * DIFFERENTIAL TEMPERATURE SETPOINT
 * ================================================================ */


/*
 * Desired temperature rise across the heating element:
 *
 * Delta T = T2 - T1
 *
 * Example:
 *
 * RTD1 = 25°C
 * RTD2 = 35°C
 *
 * Delta T = 10°C
 */

#define DELTA_T_SETPOINT_C              10.0f


/* ================================================================
 * PID PARAMETERS
 * ================================================================ */


/*
 * INITIAL VALUES ONLY.
 *
 * These will need tuning using your actual heater/system.
 *
 * Output is normalized from 0.0 to 1.0.
 */

#define PID_KP                          0.10f
#define PID_KI                          0.015f
#define PID_KD                          0.05f


/*
 * Integral anti-windup.
 */

#define PID_INTEGRAL_LIMIT              50.0f


/* ================================================================
 * HEATER SAFETY
 * ================================================================ */


/*
 * The heater resistance is currently unknown.
 *
 * Start conservatively.
 *
 * 0.50 = maximum 50% PWM.
 */

#define MAX_HEATER_OUTPUT               0.50f


/*
 * Absolute over-temperature cutoff.
 *
 * If either RTD reaches this temperature,
 * heater is immediately switched off.
 */

#define ABSOLUTE_TEMP_CUTOFF            120.0f


/*
 * Maximum physically allowed differential temperature.
 *
 * This is another independent safety check.
 */

#define MAX_DELTA_T_C                   80.0f


/*
 * Sensor sanity limits.
 */

#define TEMP_MIN_VALID_C                -50.0f
#define TEMP_MAX_VALID_C                 500.0f


/* ================================================================
 * PWM CONFIGURATION
 * ================================================================ */


/*
 * L298N does not need a very high PWM frequency for a heater.
 *
 * The thermal system averages the power naturally.
 */

#define PWM_FREQUENCY_HZ                1000


#define PWM_RESOLUTION                  LEDC_TIMER_10_BIT

#define PWM_MAX_DUTY                   1023


/* ================================================================
 * LOGGING
 * ================================================================ */

static const char *TAG = "TEMP_CONTROL";


/* ================================================================
 * ADC CALIBRATION
 * ================================================================ */

static esp_adc_cal_characteristics_t adc_chars;


/* ================================================================
 * STARTUP VOLTAGE REFERENCES
 * ================================================================ */

static float rtd1_reference_voltage = NAN;

static float rtd2_reference_voltage = NAN;


/* ================================================================
 * MEASUREMENT STRUCTURE
 * ================================================================ */

typedef struct
{
    float voltage;

    float resistance_raw;

    float resistance_calibrated;

    float temperature_raw;

    float temperature_filtered;

} rtd_measurement_t;


/* ================================================================
 * FILTER STRUCTURE
 * ================================================================ */

typedef struct
{
    float samples[MEDIAN_FILTER_SIZE];

    int count;

    int index;

    bool initialized;

    float ema_output;

} temperature_filter_t;


/* ================================================================
 * PID STRUCTURE
 * ================================================================ */

typedef struct
{
    float kp;

    float ki;

    float kd;

    float integral;

    float previous_error;

    bool first_run;

} pid_controller_t;


/* ================================================================
 * FILTER INITIALIZATION
 * ================================================================ */

static void filter_init(
    temperature_filter_t *filter
)
{
    filter->count = 0;

    filter->index = 0;

    filter->initialized = false;

    filter->ema_output = 0.0f;


    for (
        int i = 0;
        i < MEDIAN_FILTER_SIZE;
        i++
    )
    {
        filter->samples[i] = 0.0f;
    }
}


/* ================================================================
 * SORT THREE VALUES
 * ================================================================ */

static void sort_three(
    float *a,
    float *b,
    float *c
)
{
    float temp;


    if (*a > *b)
    {
        temp = *a;
        *a = *b;
        *b = temp;
    }


    if (*b > *c)
    {
        temp = *b;
        *b = *c;
        *c = temp;
    }


    if (*a > *b)
    {
        temp = *a;
        *a = *b;
        *b = temp;
    }
}


/* ================================================================
 * MEDIAN OF THREE
 * ================================================================ */

static float median_of_three(
    float a,
    float b,
    float c
)
{
    sort_three(
        &a,
        &b,
        &c
    );

    return b;
}


/* ================================================================
 * TEMPERATURE FILTER
 * ================================================================ */

static float filter_update(
    temperature_filter_t *filter,
    float temperature
)
{
    if (!isfinite(temperature))
    {
        return NAN;
    }


    /*
     * Add sample.
     */

    filter->samples[
        filter->index
    ] = temperature;


    filter->index++;


    if (
        filter->index >=
        MEDIAN_FILTER_SIZE
    )
    {
        filter->index = 0;
    }


    if (
        filter->count <
        MEDIAN_FILTER_SIZE
    )
    {
        filter->count++;
    }


    /*
     * Median filter.
     */

    float median_temperature;


    if (
        filter->count <
        MEDIAN_FILTER_SIZE
    )
    {
        median_temperature =
            temperature;
    }
    else
    {
        median_temperature =

            median_of_three(
                filter->samples[0],
                filter->samples[1],
                filter->samples[2]
            );
    }


    /*
     * Initialize EMA.
     */

    if (!filter->initialized)
    {
        filter->ema_output =
            median_temperature;

        filter->initialized = true;

        return filter->ema_output;
    }


    /*
     * EMA.
     */

    filter->ema_output =

        EMA_ALPHA *
        median_temperature

        +

        (1.0f - EMA_ALPHA) *
        filter->ema_output;


    return filter->ema_output;
}


/* ================================================================
 * PID INITIALIZATION
 * ================================================================ */

static void pid_init(
    pid_controller_t *pid
)
{
    pid->kp =
        PID_KP;

    pid->ki =
        PID_KI;

    pid->kd =
        PID_KD;

    pid->integral =
        0.0f;

    pid->previous_error =
        0.0f;

    pid->first_run =
        true;
}


/* ================================================================
 * PID RESET
 * ================================================================ */

static void pid_reset(
    pid_controller_t *pid
)
{
    pid->integral =
        0.0f;

    pid->previous_error =
        0.0f;

    pid->first_run =
        true;
}


/* ================================================================
 * PID COMPUTATION
 * ================================================================ *
 *
 * Input:
 *
 *     measured_delta_T
 *
 * Setpoint:
 *
 *     DELTA_T_SETPOINT_C
 *
 *
 * Error:
 *
 *     setpoint - measured
 *
 *
 * Output:
 *
 *     0.0 = heater OFF
 *     1.0 = maximum allowed heater output
 *
 * ================================================================
 */

static float pid_compute(
    pid_controller_t *pid,
    float setpoint,
    float measured_delta_t,
    float dt
)
{
    /*
     * Calculate control error.
     */

    float error =

        setpoint -
        measured_delta_t;


    /*
     * Derivative of error.
     */

    float derivative = 0.0f;


    if (!pid->first_run)
    {
        derivative =

            (
                error -
                pid->previous_error
            )
            /
            dt;
    }


    /*
     * Candidate integral.
     */

    float candidate_integral =

        pid->integral +

        error *
        dt;


    /*
     * Clamp integral.
     */

    if (
        candidate_integral >
        PID_INTEGRAL_LIMIT
    )
    {
        candidate_integral =
            PID_INTEGRAL_LIMIT;
    }


    if (
        candidate_integral <
        -PID_INTEGRAL_LIMIT
    )
    {
        candidate_integral =
            -PID_INTEGRAL_LIMIT;
    }


    /*
     * Calculate unrestricted PID output.
     */

    float output =

        pid->kp *
        error

        +

        pid->ki *
        candidate_integral

        +

        pid->kd *
        derivative;


    /*
     * Heating only.
     *
     * Negative output means there is already
     * enough temperature difference.
     */

    if (output < 0.0f)
    {
        output = 0.0f;
    }


    /*
     * Maximum heater output.
     */

    if (
        output >
        MAX_HEATER_OUTPUT
    )
    {
        output =
            MAX_HEATER_OUTPUT;
    }


    /*
     * Anti-windup.
     *
     * Only update integral when it is not being
     * driven further into saturation.
     */

    bool output_saturated_high =

        (
            output >=
            MAX_HEATER_OUTPUT
        );


    bool output_saturated_low =

        (
            output <= 0.0f
        );


    /*
     * If saturated high but error is negative,
     * allow the integrator to unwind.
     *
     * If saturated low but error is positive,
     * allow it to build.
     */

    if (
        !output_saturated_high
        &&
        !output_saturated_low
    )
    {
        pid->integral =
            candidate_integral;
    }
    else if (
        output_saturated_high
        &&
        error < 0.0f
    )
    {
        pid->integral =
            candidate_integral;
    }
    else if (
        output_saturated_low
        &&
        error > 0.0f
    )
    {
        pid->integral =
            candidate_integral;
    }


    /*
     * Save state.
     */

    pid->previous_error =
        error;

    pid->first_run =
        false;


    return output;
}


/* ================================================================
 * ADC INITIALIZATION
 * ================================================================ */

static void adc_init(void)
{
    /*
     * 12-bit ADC.
     */

    ESP_ERROR_CHECK(
        adc1_config_width(
            ADC_WIDTH_BIT_12
        )
    );


    /*
     * RTD1:
     *
     * GPIO34 -> ADC1_CH6
     */

    ESP_ERROR_CHECK(
        adc1_config_channel_atten(
            RTD1_ADC_CHANNEL,
            ADC_ATTENUATION
        )
    );


    /*
     * RTD2:
     *
     * GPIO35 -> ADC1_CH7
     */

    ESP_ERROR_CHECK(
        adc1_config_channel_atten(
            RTD2_ADC_CHANNEL,
            ADC_ATTENUATION
        )
    );


    /*
     * ADC characterization.
     */

    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTENUATION,
        ADC_WIDTH_BIT_12,
        ADC_DEFAULT_VREF_MV,
        &adc_chars
    );


    ESP_LOGI(
        TAG,
        "ADC initialized"
    );


    ESP_LOGI(
        TAG,
        "RTD1 GPIO%d -> ADC1_CH6",
        GPIO_RTD1_ADC
    );


    ESP_LOGI(
        TAG,
        "RTD2 GPIO%d -> ADC1_CH7",
        GPIO_RTD2_ADC
    );
}


/* ================================================================
 * READ ADC VOLTAGE
 * ================================================================ */

static float read_adc_voltage(
    adc1_channel_t channel
)
{
    uint32_t adc_sum = 0;


    for (
        int i = 0;
        i < ADC_SAMPLES;
        i++
    )
    {
        adc_sum +=
            adc1_get_raw(
                channel
            );
    }


    uint32_t adc_average =

        adc_sum /
        ADC_SAMPLES;


    uint32_t voltage_mv =

        esp_adc_cal_raw_to_voltage(
            adc_average,
            &adc_chars
        );


    return

        (float)voltage_mv /
        1000.0f;
}


/* ================================================================
 * PT100 TEMPERATURE -> RESISTANCE
 * ================================================================ */

static float pt100_temperature_to_resistance(
    float temperature
)
{
    if (
        temperature >=
        0.0f
    )
    {
        return

            PT100_R0 *

            (
                1.0f

                +

                PT100_A *
                temperature

                +

                PT100_B *
                temperature *
                temperature
            );
    }


    return

        PT100_R0 *

        (
            1.0f

            +

            PT100_A *
            temperature

            +

            PT100_B *
            temperature *
            temperature

            +

            PT100_C *
            (temperature - 100.0f) *
            temperature *
            temperature *
            temperature
        );
}


/* ================================================================
 * RAW RESISTANCE
 * ================================================================ */

static float voltage_to_raw_resistance(
    float voltage,
    float rref
)
{
    if (
        voltage <= 0.001f
        ||
        voltage >= DIVIDER_SUPPLY_V
    )
    {
        return NAN;
    }


    return

        rref *
        voltage /

        (
            DIVIDER_SUPPLY_V -
            voltage
        );
}


/* ================================================================
 * CALIBRATED RESISTANCE
 * ================================================================ */

static float voltage_to_calibrated_resistance(
    float voltage,
    float reference_voltage,
    float rref
)
{
    if (
        !isfinite(voltage)
        ||
        !isfinite(reference_voltage)
    )
    {
        return NAN;
    }


    if (
        voltage <= 0.001f
        ||
        reference_voltage <= 0.001f
    )
    {
        return NAN;
    }


    /*
     * Current voltage / calibration voltage.
     */

    float ratio =

        voltage /
        reference_voltage;


    /*
     * PT100 resistance at calibration temperature.
     */

    float r_cal =

        pt100_temperature_to_resistance(
            CALIBRATION_TEMPERATURE_C
        );


    /*
     * Solve voltage-divider equation using
     * voltage ratio.
     */

    float denominator =

        (
            rref +
            r_cal
        )

        -

        (
            ratio *
            r_cal
        );


    if (
        denominator <= 0.001f
    )
    {
        return NAN;
    }


    return

        ratio *
        r_cal *
        rref /
        denominator;
}


/* ================================================================
 * PT100 RESISTANCE -> TEMPERATURE
 * ================================================================ */

static float pt100_resistance_to_temperature(
    float resistance
)
{
    if (
        !isfinite(resistance)
        ||
        resistance <= 0.0f
    )
    {
        return NAN;
    }


    /*
     * T >= 0°C.
     */

    if (
        resistance >=
        PT100_R0
    )
    {
        float c =

            1.0f -
            resistance /
            PT100_R0;


        float discriminant =

            PT100_A *
            PT100_A

            -

            4.0f *
            PT100_B *
            c;


        if (
            discriminant < 0.0f
        )
        {
            return NAN;
        }


        return

            (
                -PT100_A

                +

                sqrtf(
                    discriminant
                )
            )

            /

            (
                2.0f *
                PT100_B
            );
    }


    /*
     * T < 0°C.
     */

    float temperature =
        -10.0f;


    for (
        int iteration = 0;
        iteration < 20;
        iteration++
    )
    {
        float t =
            temperature;


        float f =

            PT100_R0 *

            (
                1.0f

                +

                PT100_A *
                t

                +

                PT100_B *
                t *
                t

                +

                PT100_C *
                (t - 100.0f) *
                t *
                t *
                t
            )

            -

            resistance;


        float derivative =

            PT100_R0 *

            (
                PT100_A

                +

                2.0f *
                PT100_B *
                t

                +

                PT100_C *

                (
                    4.0f *
                    t *
                    t *
                    t

                    -

                    300.0f *
                    t *
                    t
                )
            );


        if (
            fabsf(derivative)
            < 1e-9f
        )
        {
            break;
        }


        float correction =

            f /
            derivative;


        temperature -=
            correction;


        if (
            fabsf(correction)
            < 0.0001f
        )
        {
            break;
        }
    }


    return temperature;
}


/* ================================================================
 * STARTUP CALIBRATION
 * ================================================================ */

static bool perform_startup_calibration(void)
{
    ESP_LOGW(
        TAG,
        "=========================================="
    );


    ESP_LOGW(
        TAG,
        "STARTUP CALIBRATION"
    );


    ESP_LOGW(
        TAG,
        "Reference temperature = %.2f C",
        CALIBRATION_TEMPERATURE_C
    );


    ESP_LOGW(
        TAG,
        "Both RTDs must be at the reference temperature."
    );


    ESP_LOGW(
        TAG,
        "HEATER MUST BE OFF."
    );


    /*
     * Allow sensors and ADC to settle.
     */

    vTaskDelay(
        pdMS_TO_TICKS(
            CALIBRATION_SETTLE_MS
        )
    );


    float rtd1_sum =
        0.0f;


    float rtd2_sum =
        0.0f;


    /*
     * Read both sensors together.
     */

    for (
        int i = 0;
        i < CALIBRATION_SAMPLES;
        i++
    )
    {
        float v1 =

            read_adc_voltage(
                RTD1_ADC_CHANNEL
            );


        float v2 =

            read_adc_voltage(
                RTD2_ADC_CHANNEL
            );


        rtd1_sum +=
            v1;


        rtd2_sum +=
            v2;


        vTaskDelay(
            pdMS_TO_TICKS(
                CALIBRATION_DELAY_MS
            )
        );
    }


    /*
     * Average.
     */

    rtd1_reference_voltage =

        rtd1_sum /
        CALIBRATION_SAMPLES;


    rtd2_reference_voltage =

        rtd2_sum /
        CALIBRATION_SAMPLES;


    /*
     * Validate.
     */

    if (
        rtd1_reference_voltage <=
        0.001f
        ||
        rtd2_reference_voltage <=
        0.001f
    )
    {
        ESP_LOGE(
            TAG,
            "Calibration failed."
        );

        return false;
    }


    float calibration_resistance =

        pt100_temperature_to_resistance(
            CALIBRATION_TEMPERATURE_C
        );


    ESP_LOGI(
        TAG,
        "Calibration PT100 resistance = %.3f ohm",
        calibration_resistance
    );


    ESP_LOGI(
        TAG,
        "RTD1 reference voltage = %.4f V",
        rtd1_reference_voltage
    );


    ESP_LOGI(
        TAG,
        "RTD2 reference voltage = %.4f V",
        rtd2_reference_voltage
    );


    ESP_LOGI(
        TAG,
        "=========================================="
    );


    return true;
}


/* ================================================================
 * L298N INITIALIZATION
 * ================================================================ */

static void l298n_init(void)
{
    gpio_config_t config =
    {
        .pin_bit_mask =

            (1ULL << GPIO_L298_IN1)
            |
            (1ULL << GPIO_L298_IN2),

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &config
        )
    );


    /*
     * One-direction heater operation.
     */

    gpio_set_level(
        GPIO_L298_IN1,
        1
    );


    gpio_set_level(
        GPIO_L298_IN2,
        0
    );


    ESP_LOGI(
        TAG,
        "L298N direction configured: IN1=HIGH IN2=LOW"
    );
}


/* ================================================================
 * PWM INITIALIZATION
 * ================================================================ */

static void pwm_init(void)
{
    /*
     * LEDC timer.
     */

    ledc_timer_config_t timer_config =
    {
        .speed_mode =
            LEDC_LOW_SPEED_MODE,

        .timer_num =
            LEDC_TIMER_0,

        .duty_resolution =
            PWM_RESOLUTION,

        .freq_hz =
            PWM_FREQUENCY_HZ,

        .clk_cfg =
            LEDC_AUTO_CLK
    };


    ESP_ERROR_CHECK(
        ledc_timer_config(
            &timer_config
        )
    );


    /*
     * GPIO27 -> ENA.
     */

    ledc_channel_config_t channel_config =
    {
        .gpio_num =
            GPIO_L298_PWM,

        .speed_mode =
            LEDC_LOW_SPEED_MODE,

        .channel =
            LEDC_CHANNEL_0,

        .intr_type =
            LEDC_INTR_DISABLE,

        .timer_sel =
            LEDC_TIMER_0,

        .duty =
            0,

        .hpoint =
            0
    };


    ESP_ERROR_CHECK(
        ledc_channel_config(
            &channel_config
        )
    );


    /*
     * Explicitly start with heater OFF.
     */

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0,
            0
        )
    );


    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0
        )
    );


    ESP_LOGI(
        TAG,
        "PWM initialized: GPIO%d, %d Hz",
        GPIO_L298_PWM,
        PWM_FREQUENCY_HZ
    );
}


/* ================================================================
 * SET HEATER OUTPUT
 * ================================================================ */

static void heater_set_output(
    float output
)
{
    /*
     * Restrict to valid range.
     */

    if (
        output < 0.0f
    )
    {
        output = 0.0f;
    }


    if (
        output >
        MAX_HEATER_OUTPUT
    )
    {
        output =
            MAX_HEATER_OUTPUT;
    }


    /*
     * Convert normalized output to PWM duty.
     */

    uint32_t duty =

        (uint32_t)
        (
            output *
            PWM_MAX_DUTY
        );


    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0,
            duty
        )
    );


    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0
        )
    );
}


/* ================================================================
 * READ ONE RTD
 * ================================================================ */

static rtd_measurement_t read_rtd(
    adc1_channel_t channel,
    float rref,
    float reference_voltage
)
{
    rtd_measurement_t result;


    /*
     * Voltage.
     */

    result.voltage =

        read_adc_voltage(
            channel
        );


    /*
     * Raw resistance based on nominal 3.3 V.
     */

    result.resistance_raw =

        voltage_to_raw_resistance(
            result.voltage,
            rref
        );


    /*
     * Calibrated resistance.
     */

    result.resistance_calibrated =

        voltage_to_calibrated_resistance(
            result.voltage,
            reference_voltage,
            rref
        );


    /*
     * Temperature.
     */

    result.temperature_raw =

        pt100_resistance_to_temperature(
            result.resistance_calibrated
        );


    result.temperature_filtered =
        result.temperature_raw;


    return result;
}


/* ================================================================
 * MAIN CONTROL TASK
 * ================================================================ */

static void temperature_control_task(
    void *arg
)
{
    /*
     * Separate filters.
     */

    temperature_filter_t rtd1_filter;

    temperature_filter_t rtd2_filter;


    filter_init(
        &rtd1_filter
    );


    filter_init(
        &rtd2_filter
    );


    /*
     * PID.
     */

    pid_controller_t pid;

    pid_init(
        &pid
    );


    const float dt =

        CONTROL_PERIOD_MS /
        1000.0f;


    /*
     * Control loop.
     */

    while (1)
    {
        /*
         * --------------------------------------------------------
         * READ RTD1
         * --------------------------------------------------------
         */

        rtd_measurement_t rtd1 =

            read_rtd(
                RTD1_ADC_CHANNEL,
                RTD1_RREF,
                rtd1_reference_voltage
            );


        /*
         * --------------------------------------------------------
         * READ RTD2
         * --------------------------------------------------------
         */

        rtd_measurement_t rtd2 =

            read_rtd(
                RTD2_ADC_CHANNEL,
                RTD2_RREF,
                rtd2_reference_voltage
            );


        /*
         * Sensor validation.
         */

        if (
            !isfinite(
                rtd1.temperature_raw
            )
            ||
            !isfinite(
                rtd2.temperature_raw
            )
        )
        {
            ESP_LOGE(
                TAG,
                "RTD SENSOR ERROR - HEATER OFF"
            );


            heater_set_output(
                0.0f
            );


            pid_reset(
                &pid
            );


            vTaskDelay(
                pdMS_TO_TICKS(
                    CONTROL_PERIOD_MS
                )
            );


            continue;
        }


        /*
         * --------------------------------------------------------
         * FILTER TEMPERATURES
         * --------------------------------------------------------
         */

        rtd1.temperature_filtered =

            filter_update(
                &rtd1_filter,
                rtd1.temperature_raw
            );


        rtd2.temperature_filtered =

            filter_update(
                &rtd2_filter,
                rtd2.temperature_raw
            );


        /*
         * Absolute temperature protection.
         */

        if (
            rtd1.temperature_filtered >=
            ABSOLUTE_TEMP_CUTOFF

            ||

            rtd2.temperature_filtered >=
            ABSOLUTE_TEMP_CUTOFF
        )
        {
            ESP_LOGE(
                TAG,
                "OVER TEMPERATURE - HEATER OFF"
            );


            heater_set_output(
                0.0f
            );


            pid_reset(
                &pid
            );


            vTaskDelay(
                pdMS_TO_TICKS(
                    CONTROL_PERIOD_MS
                )
            );


            continue;
        }


        /*
         * --------------------------------------------------------
         * CALCULATE DIFFERENTIAL TEMPERATURE
         * --------------------------------------------------------
         *
         * RTD1 = before heater
         * RTD2 = after heater
         *
         * Therefore:
         *
         * Delta T = T2 - T1
         */

        float delta_temperature =

            rtd2.temperature_filtered
            -
            rtd1.temperature_filtered;


        /*
         * Differential temperature safety limit.
         */

        if (
            delta_temperature >
            MAX_DELTA_T_C
        )
        {
            ESP_LOGE(
                TAG,
                "EXCESSIVE DELTA T - HEATER OFF"
            );


            heater_set_output(
                0.0f
            );


            pid_reset(
                &pid
            );


            vTaskDelay(
                pdMS_TO_TICKS(
                    CONTROL_PERIOD_MS
                )
            );


            continue;
        }


        /*
         * Calculate control error.
         */

        float error =

            DELTA_T_SETPOINT_C
            -
            delta_temperature;


        /*
         * --------------------------------------------------------
         * PID
         * --------------------------------------------------------
         */

        float pid_output =

            pid_compute(
                &pid,

                DELTA_T_SETPOINT_C,

                delta_temperature,

                dt
            );


        /*
         * Apply heater PWM.
         */

        heater_set_output(
            pid_output
        );


        /*
         * --------------------------------------------------------
         * TELEMETRY
         * --------------------------------------------------------
         */

        ESP_LOGI(
            TAG,

            "RTD1=%.2f C | "
            "RTD2=%.2f C | "
            "DeltaT=%.2f C | "
            "SP=%.2f C | "
            "Error=%.2f C | "
            "PID=%.1f%% | "
            "PWM=%lu",

            rtd1.temperature_filtered,

            rtd2.temperature_filtered,

            delta_temperature,

            DELTA_T_SETPOINT_C,

            error,

            pid_output *
            100.0f,

            (unsigned long)
            (
                pid_output *
                PWM_MAX_DUTY
            )
        );


        /*
         * CSV output.
         *
         * Columns:
         *
         * RTD1 temperature
         * RTD2 temperature
         * Delta T
         * Delta T setpoint
         * PID output
         */

        printf(
            "DATA,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f,"
            "%.3f\n",

            rtd1.temperature_filtered,

            rtd2.temperature_filtered,

            delta_temperature,

            DELTA_T_SETPOINT_C,

            pid_output * 100.0f
        );


        /*
         * Wait until next control cycle.
         */

        vTaskDelay(
            pdMS_TO_TICKS(
                CONTROL_PERIOD_MS
            )
        );
    }
}


/* ================================================================
 * MAIN
 * ================================================================ */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "=========================================="
    );


    ESP_LOGI(
        TAG,
        "ESP32 DUAL-PT100 DIFFERENTIAL PID"
    );


    ESP_LOGI(
        TAG,
        "L298N HEATER CONTROLLER"
    );


    ESP_LOGI(
        TAG,
        "=========================================="
    );


    /*
     * Initialize L298N GPIO direction.
     */

    l298n_init();


    /*
     * Initialize PWM.
     *
     * Heater remains OFF.
     */

    pwm_init();


    /*
     * Initialize ADC.
     */

    adc_init();


    /*
     * Startup calibration.
     *
     * Heater is still OFF.
     */

    if (
        AUTO_CALIBRATE_ON_BOOT
    )
    {
        if (
            !perform_startup_calibration()
        )
        {
            ESP_LOGE(
                TAG,
                "STARTUP CALIBRATION FAILED"
            );


            /*
             * Keep heater OFF forever.
             */

            heater_set_output(
                0.0f
            );


            while (1)
            {
                vTaskDelay(
                    pdMS_TO_TICKS(
                        1000
                    )
                );
            }
        }
    }


    /*
     * Make certain heater starts OFF.
     */

    heater_set_output(
        0.0f
    );


    ESP_LOGI(
        TAG,
        "=========================================="
    );


    ESP_LOGI(
        TAG,
        "CONTROL STARTING"
    );


    ESP_LOGI(
        TAG,
        "Delta T setpoint = %.2f C",
        DELTA_T_SETPOINT_C
    );


    ESP_LOGI(
        TAG,
        "Maximum heater output = %.1f%%",
        MAX_HEATER_OUTPUT * 100.0f
    );


    ESP_LOGI(
        TAG,
        "PID Kp=%.4f Ki=%.4f Kd=%.4f",
        PID_KP,
        PID_KI,
        PID_KD
    );


    ESP_LOGI(
        TAG,
        "=========================================="
    );


    /*
     * Start control task.
     */

    xTaskCreate(
        temperature_control_task,

        "temperature_control",

        8192,

        NULL,

        5,

        NULL
    );
}