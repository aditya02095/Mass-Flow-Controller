
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// =====================================================
// USER CONFIGURATION
// Change these values to match your actual circuit.
// =====================================================

// Original ESP32: GPIO34 = ADC1_CHANNEL_6
#define ADC_UNIT_USED       ADC_UNIT_1
#define ADC_CHANNEL_USED    ADC_CHANNEL_6
#define ADC_GPIO            34

// ADC settings
#define ADC_ATTEN_USED      ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH       ADC_BITWIDTH_12
#define NUM_SAMPLES         32

// Wheatstone bridge parameters (ohms and volts)
static const double VEX = 3.300;   // Bridge excitation voltage
static const double R1  = 1000.0;  // Top-left resistor
static const double R2  = 1000.0;  // Bottom-left resistor
static const double R3  = 1000.0;  // Top-right reference resistor

// Amplifier parameters
static const double AMP_GAIN_X = 100.0;  // Set actual gain
static const double AMP_OFFSET_V = 0.0;  // Output at zero bridge difference

// Differential polarity:
// +1 means Vout = offset + gain*(VL - VR)
// -1 means Vout = offset - gain*(VL - VR)
static const int AMP_POLARITY = +1;

#define SAMPLE_DELAY_MS  200

// =====================================================

static const char *TAG = "WHEATSTONE";

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static bool calibration_available = false;


// Initialize ADC and calibration
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_USED,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&unit_cfg, &adc_handle)
    );

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BIT_WIDTH,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            ADC_CHANNEL_USED,
            &channel_cfg
        )
    );

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_USED,
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BIT_WIDTH,
        .default_vref = 1100,
    };

    if (adc_cali_create_scheme_line_fitting(
            &cali_cfg, &cali_handle) == ESP_OK) {
        calibration_available = true;
        ESP_LOGI(TAG, "ADC line-fitting calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable; using raw fallback");
    }
#else
    ESP_LOGW(TAG, "Line-fitting calibration unsupported on this target");
#endif
}


// Average multiple ADC samples
static esp_err_t read_average_raw(int *average_raw)
{
    int64_t sum = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        int raw = 0;

        esp_err_t err = adc_oneshot_read(
            adc_handle, ADC_CHANNEL_USED, &raw
        );

        if (err != ESP_OK) {
            return err;
        }

        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    *average_raw = (int)(sum / NUM_SAMPLES);
    return ESP_OK;
}


// Convert ADC reading to voltage
static esp_err_t read_output_voltage(double *voltage_v, int *raw_out)
{
    int raw = 0;
    ESP_RETURN_ON_ERROR(
        read_average_raw(&raw), TAG, "ADC read failed"
    );

    *raw_out = raw;

    if (calibration_available) {
        int voltage_mv = 0;

        ESP_RETURN_ON_ERROR(
            adc_cali_raw_to_voltage(
                cali_handle, raw, &voltage_mv
            ),
            TAG,
            "ADC voltage conversion failed"
        );

        *voltage_v = voltage_mv / 1000.0;
    } else {
        // Rough fallback only; not precision calibration.
        *voltage_v = (raw / 4095.0) * 3.3;
    }

    return ESP_OK;
}


// Calculate unknown bridge resistance
static bool calculate_resistance(
    double amplifier_output_v,
    double *bridge_diff_v,
    double *unknown_resistance
)
{
    if (AMP_GAIN_X <= 0.0 || VEX <= 0.0 ||
        R1 <= 0.0 || R2 <= 0.0 || R3 <= 0.0) {
        return false;
    }

    // Known left midpoint voltage
    const double VL = VEX * R2 / (R1 + R2);

    // Recover bridge differential voltage from amplifier
    *bridge_diff_v =
        AMP_POLARITY *
        (amplifier_output_v - AMP_OFFSET_V) / AMP_GAIN_X;

    // Assumed polarity convention: bridge_diff = VL - VR
    const double VR = VL - *bridge_diff_v;

    // Divider equation: VR = VEX*Rx/(R3+Rx)
    const double denominator = VEX - VR;

    // Reject impossible or numerically unstable results.
    if (VR <= 0.0 || VR >= VEX ||
        denominator <= 1e-9) {
        return false;
    }

    *unknown_resistance = R3 * VR / denominator;

    return isfinite(*unknown_resistance) &&
           *unknown_resistance > 0.0;
}


void app_main(void)
{
    adc_init();

    ESP_LOGI(TAG, "Wheatstone bridge resistance reader started");
    ESP_LOGI(TAG, "ADC: GPIO%d | Gain: %.3f",
             ADC_GPIO, AMP_GAIN_X);

    while (1) {
        double vout = 0.0;
        int raw = 0;

        esp_err_t err = read_output_voltage(&vout, &raw);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC read error: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
            continue;
        }

        double bridge_diff = 0.0;
        double rx = 0.0;

        bool valid = calculate_resistance(
            vout, &bridge_diff, &rx
        );

        if (valid) {
            ESP_LOGI(TAG,
                "RAW=%d | Vout=%.4f V | "
                "Vbridge_diff=%.7f V | "
                "Rx=%.3f ohm (%.6f kohm)",
                raw, vout, bridge_diff,
                rx, rx / 1000.0);
        } else {
            ESP_LOGW(TAG,
                "RAW=%d | Vout=%.4f V | "
                "Resistance invalid. Check gain, offset, "
                "polarity, bridge values and wiring.",
                raw, vout);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}
