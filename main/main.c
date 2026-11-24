#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

/*
 * Waveshare GP2Y1010AU0F dust sensor on Waveshare ESP32-S3-ETH.
 *
 * Wiring used by this example:
 *   Sensor VCC  -> board 5V
 *   Sensor GND  -> board GND
 *   Sensor AOUT -> GPIO1 (ADC1_CH0)
 *   Sensor ILED -> GPIO17
 *
 * GPIO1 is ADC1_CH0. If you change the analog GPIO, also change the ADC
 * channel to the matching ADC1 channel. Do not feed 5V into an ESP32 GPIO.
 */
#define DUST_AOUT_GPIO              GPIO_NUM_1
#define DUST_ILED_GPIO              GPIO_NUM_17

#define DUST_ADC_UNIT               ADC_UNIT_1
#define DUST_ADC_CHANNEL            ADC_CHANNEL_0
#define DUST_ADC_ATTEN              ADC_ATTEN_DB_0
#define DUST_ADC_BITWIDTH            ADC_BITWIDTH_12

/* Waveshare / Sharp pulse timing. */
#define LED_SAMPLE_DELAY_US         280U
#define LED_PULSE_WIDTH_US          320U
#define SAMPLE_PERIOD_MS            10U

/* The Waveshare board divides AOUT by 11; recover sensor-side voltage. */
#define AOUT_DIVIDER_RATIO          11.0f

/*
 * PROVISIONAL graph-based starting point, NOT a per-unit calibration:
 * The Waveshare manual's output graph appears to start at about 0.6 V
 * sensor-side voltage at zero dust. Sharp's datasheet gives 0.9 V typical
 * no-dust output, but specifies 0 to 1.5 V, so actual units vary.
 *
 * For better results, determine this sensor's baseline in controlled
 * HEPA-filtered air and/or calibrate against a reference particle monitor.
 * Do not automatically learn the baseline from normal room air.
 */
#define BASELINE_SENSOR_MV          600.0f
#define SENSITIVITY_MV_PER_UG       5.0f
#define DUST_MAX_UG_M3              500.0f

/* Smooth individual readings, then average the filtered signal for reporting. */
#define EMA_ALPHA                   0.20f
#define REPORT_INTERVAL_MS          1000

static const char *TAG = "dust_sensor";

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;
static bool s_adc_calibration_enabled;

static void init_iled_gpio(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << DUST_ILED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_set_level(DUST_ILED_GPIO, 0));
}

static void init_adc(void)
{
    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = DUST_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &s_adc_handle));

    const adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = DUST_ADC_BITWIDTH,
        .atten = DUST_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        s_adc_handle, DUST_ADC_CHANNEL, &channel_config));

    adc_cali_scheme_ver_t scheme_mask = 0;
    esp_err_t err = adc_cali_check_scheme(&scheme_mask);
    if (err == ESP_OK &&
        (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING)) {
        const adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = DUST_ADC_UNIT,
            .chan = DUST_ADC_CHANNEL,
            .atten = DUST_ADC_ATTEN,
            .bitwidth = DUST_ADC_BITWIDTH,
        };

        err = adc_cali_create_scheme_curve_fitting(
            &cali_config, &s_adc_cali_handle);
        if (err == ESP_OK) {
            s_adc_calibration_enabled = true;
            ESP_LOGI(TAG, "ADC curve-fitting calibration enabled");
        } else {
            ESP_LOGW(TAG, "ADC calibration init failed: %s; using approximate conversion",
                     esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "ADC curve-fitting calibration scheme unavailable; using approximate conversion");
    }

    ESP_LOGI(TAG, "AOUT GPIO=%d -> ADC1 channel=%d; ILED GPIO=%d",
             DUST_AOUT_GPIO, DUST_ADC_CHANNEL, DUST_ILED_GPIO);
}

static esp_err_t adc_raw_to_millivolts(int raw, int *millivolts)
{
    if (millivolts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_adc_calibration_enabled) {
        return adc_cali_raw_to_voltage(s_adc_cali_handle, raw, millivolts);
    }

    /* Fallback only. Prefer the ESP-IDF calibration scheme. */
    *millivolts = (raw * 1100 + 2047) / 4095;
    return ESP_OK;
}

/* One timed LED pulse and one ADC sample. */
static esp_err_t read_aout(int *raw_out, int *aout_mv_out)
{
    if (raw_out == NULL || aout_mv_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int raw = 0;
    int aout_mv = 0;
    const int64_t pulse_start_us = esp_timer_get_time();

    esp_err_t err = gpio_set_level(DUST_ILED_GPIO, 1);
    if (err != ESP_OK) {
        return err;
    }

    /* Sample at approximately 280 us after the LED turns on. */
    esp_rom_delay_us(LED_SAMPLE_DELAY_US);
    const esp_err_t adc_err = adc_oneshot_read(
        s_adc_handle, DUST_ADC_CHANNEL, &raw);

    /* Finish the nominal 320 us pulse even if the ADC read fails. */
    const int64_t elapsed_us = esp_timer_get_time() - pulse_start_us;
    if (elapsed_us < LED_PULSE_WIDTH_US) {
        esp_rom_delay_us((uint32_t)(LED_PULSE_WIDTH_US - elapsed_us));
    }

    const esp_err_t led_off_err = gpio_set_level(DUST_ILED_GPIO, 0);
    if (led_off_err != ESP_OK) {
        return led_off_err;
    }
    if (adc_err != ESP_OK) {
        return adc_err;
    }

    err = adc_raw_to_millivolts(raw, &aout_mv);
    if (err != ESP_OK) {
        return err;
    }

    *raw_out = raw;
    *aout_mv_out = aout_mv;
    return ESP_OK;
}

/*
 * Nominal dust-density estimate from the sensor-side output voltage.
 * This is not a certified PM2.5 measurement: the sensor is not size-selective
 * and the baseline/sensitivity need unit-specific validation.
 */
static float calculate_dust_estimate(float sensor_output_mv)
{
    float dust = (sensor_output_mv - BASELINE_SENSOR_MV) /
                 SENSITIVITY_MV_PER_UG;

    if (dust < 0.0f) {
        dust = 0.0f;
    } else if (dust > DUST_MAX_UG_M3) {
        dust = DUST_MAX_UG_M3;
    }

    return dust;
}

void app_main(void)
{
    init_iled_gpio();
    init_adc();

    ESP_LOGI(TAG, "Waveshare GP2Y1010AU0F measurement started");
    ESP_LOGI(TAG, "LED pulse=%u us; sample delay=%u us; sample period=%u ms",
             LED_PULSE_WIDTH_US, LED_SAMPLE_DELAY_US, SAMPLE_PERIOD_MS);
    ESP_LOGW(TAG, "Baseline %.0f mV is provisional (Waveshare graph), not per-unit calibrated",
             BASELINE_SENSOR_MV);
    ESP_LOGW(TAG, "Reported value is a dust estimate, not reference-grade PM2.5");

    TickType_t last_wake_tick = xTaskGetTickCount();
    int64_t last_report_us = esp_timer_get_time();

    bool ema_initialized = false;
    float filtered_aout_mv = 0.0f;
    float sum_filtered_aout_mv = 0.0f;
    uint64_t sum_raw = 0;
    uint32_t sample_count = 0;
    uint32_t error_count = 0;

    while (true) {
        int raw = 0;
        int aout_mv = 0;
        const esp_err_t err = read_aout(&raw, &aout_mv);

        if (err == ESP_OK) {
            if (!ema_initialized) {
                filtered_aout_mv = (float)aout_mv;
                ema_initialized = true;
            } else {
                filtered_aout_mv = (EMA_ALPHA * (float)aout_mv) +
                    ((1.0f - EMA_ALPHA) * filtered_aout_mv);
            }

            sum_filtered_aout_mv += filtered_aout_mv;
            sum_raw += (uint32_t)raw;
            sample_count++;
        } else {
            error_count++;
        }

        const int64_t now_us = esp_timer_get_time();
        if ((now_us - last_report_us) >= (REPORT_INTERVAL_MS * 1000LL)) {
            if (sample_count > 0) {
                const float avg_aout_mv = sum_filtered_aout_mv / (float)sample_count;
                const float avg_sensor_mv = avg_aout_mv * AOUT_DIVIDER_RATIO;
                const float delta_mv = avg_sensor_mv - BASELINE_SENSOR_MV;
                const float dust_estimate = calculate_dust_estimate(avg_sensor_mv);
                const float avg_raw = (float)sum_raw / (float)sample_count;

                ESP_LOGI(TAG,
                    "raw_avg=%.1f | AOUT=%.1f mV | sensorVo=%.0f mV | "
                    "delta_from_baseline=%+.0f mV | dust_est=%.1f ug/m3 | samples=%" PRIu32 " | errors=%" PRIu32,
                    avg_raw, avg_aout_mv, avg_sensor_mv, delta_mv,
                    dust_estimate, sample_count, error_count);
            } else {
                ESP_LOGW(TAG, "No valid ADC samples in reporting interval; errors=%" PRIu32,
                         error_count);
            }

            sum_filtered_aout_mv = 0.0f;
            sum_raw = 0;
            sample_count = 0;
            error_count = 0;
            last_report_us = now_us;
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
