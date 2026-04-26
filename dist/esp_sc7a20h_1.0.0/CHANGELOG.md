# Changelog

All notable changes to **esp_sc7a20h** will be documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-04-26

### Added

- Initial public release.
- I2C driver using ESP-IDF v5.x `i2c_master` API.
- 12-bit raw + scaled `mg` acceleration readout.
- Configurable range (±2 / 4 / 8 / 16 g) and ODR (1 Hz – 400 Hz).
- Motion-detect interrupt with XYZ OR-trigger, configurable threshold/duration, latched on INT1.
- One-line `ConfigDeepSleepWakeup(gpio)` — sets up EXT1 wakeup + RTC GPIO pull-up.
- One-line `InitWithMotionDetection(cfg)` — init + IRQ + 500 ms debounce.
- `esp_err_t` error propagation throughout.
- RAII destructor: powers down sensor and unregisters I2C device.

### Notes

- LIS2DH12 / LIS3DH register-compatible — most LIS2DH12 reference code applies.
- Tested on ESP-IDF 5.3 / 5.4 / 5.5 with ESP32-S3.
