# Waveshare Dust Sensor + ESP32-S3-ETH (ESP-IDF 5.5.4)

This is a minimal ESP-IDF C project for the Waveshare Dust Sensor breakout built around the Sharp GP2Y1010AU0F.

## Wiring used by the example

| Waveshare Dust Sensor | ESP32-S3-ETH | Notes |
|---|---|---|
| VCC | 5V | 3.3V is also within the Waveshare module input range |
| GND | GND | Common ground is required |
| AOUT | GPIO1 | ADC1_CH0 |
| ILED | GPIO17 | Digital output; HIGH turns the IR LED on |

The firmware configures ADC1 channel 0 in oneshot mode on GPIO1. GPIO17 is used as the sensor LED control output. Check your ESP32-S3-ETH board revision and other peripherals for pin conflicts before wiring; change `DUST_AOUT_GPIO`, `DUST_ADC_CHANNEL`, and `DUST_ILED_GPIO` together as appropriate. Keep AOUT on an ADC1 pin if Wi-Fi is active.

## Build and flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

On Linux, replace `COMx` with the serial device, for example `/dev/ttyACM0` or `/dev/ttyUSB0`.

## Important calibration note

The breakout divides AOUT by 11. The code multiplies the ADC pin voltage by 11 before estimating dust density. It starts with the Sharp sensor's typical clean-air baseline of 900 mV and nominal sensitivity of 5 mV per ug/m3. Set `BASELINE_SENSOR_MV` after checking your own sensor in low-particle air. This is an approximate optical dust estimate, not a calibrated/official PM2.5 monitor.
