#ifndef SC7A20H_H
#define SC7A20H_H

#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <functional>
#include <cstdint>

/**
 * @brief SC7A20H 3-axis accelerometer driver (Silan Microelectronics).
 *
 * Register-compatible with ST LIS2DH12 / LIS3DH.
 * I2C address: 0x18 (SA0 = GND) or 0x19 (SA0 = VCC, default).
 *
 * Capabilities: acceleration readout, motion-detect interrupt,
 * deep-sleep wakeup, low-power mode.
 */

/// Raw 3-axis sample (12-bit, left-aligned in source registers; this struct
/// already holds the right-shifted, sign-extended value).
struct Sc7a20hRawAcce {
    int16_t x;
    int16_t y;
    int16_t z;
};

/// Scaled 3-axis acceleration in milli-g (mg).
struct Sc7a20hAcce {
    float x;
    float y;
    float z;
};

/// Full-scale range selection.
enum class Sc7a20hRange : uint8_t {
    kRange2G  = 0x00,  ///< +/- 2 g (default on power-up)
    kRange4G  = 0x10,  ///< +/- 4 g
    kRange8G  = 0x20,  ///< +/- 8 g
    kRange16G = 0x30,  ///< +/- 16 g
};

/// Output data rate.
enum class Sc7a20hOdr : uint8_t {
    kPowerDown = 0x00,  ///< Power-down
    kOdr1Hz    = 0x10,  ///< 1   Hz (ultra-low power)
    kOdr10Hz   = 0x20,  ///< 10  Hz (low power)
    kOdr25Hz   = 0x30,  ///< 25  Hz
    kOdr50Hz   = 0x40,  ///< 50  Hz
    kOdr100Hz  = 0x50,  ///< 100 Hz (driver default)
    kOdr200Hz  = 0x60,  ///< 200 Hz
    kOdr400Hz  = 0x70,  ///< 400 Hz
};

/// Motion-detection configuration.
struct Sc7a20hMotionConfig {
    uint8_t threshold = 0x08;  ///< Threshold (step = full-scale/128, default ~250 mg @ +/-4 g)
    uint8_t duration  = 0x02;  ///< Duration  (step = 1/ODR samples)
    bool    enable_x  = true;  ///< Detect motion on X axis
    bool    enable_y  = true;  ///< Detect motion on Y axis
    bool    enable_z  = true;  ///< Detect motion on Z axis
};

class Sc7a20h {
public:
    using WakeupCallback = std::function<void()>;

    /**
     * @brief Construct and register the I2C device.
     * @param i2c_bus I2C master bus handle.
     * @param addr    7-bit I2C address (default 0x19).
     */
    Sc7a20h(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x19);
    ~Sc7a20h();

    Sc7a20h(const Sc7a20h&) = delete;
    Sc7a20h& operator=(const Sc7a20h&) = delete;

    // ===================== Device management =====================

    /**
     * @brief Verify WHO_AM_I and apply default configuration.
     * @param range Full-scale range (default +/- 4 g).
     * @param odr   Output data rate (default 100 Hz).
     * @return ESP_OK on success;
     *         ESP_ERR_NOT_FOUND if WHO_AM_I mismatch;
     *         ESP_ERR_INVALID_STATE if I2C device registration failed.
     */
    esp_err_t Initialize(Sc7a20hRange range = Sc7a20hRange::kRange4G,
                         Sc7a20hOdr odr = Sc7a20hOdr::kOdr100Hz);

    /// Whether @ref Initialize() has succeeded.
    bool IsInitialized() const { return initialized_; }

    // ===================== Data readout ==========================

    /**
     * @brief Read raw acceleration sample.
     * @param[out] raw Raw value (12-bit signed, already right-shifted).
     */
    esp_err_t GetRawAcce(Sc7a20hRawAcce& raw);

    /**
     * @brief Read acceleration scaled to milli-g.
     * @param[out] acce Scaled value.
     */
    esp_err_t GetAcce(Sc7a20hAcce& acce);

    // ===================== Motion detection ======================

    /**
     * @brief Enable or disable the motion-detect interrupt on INT1.
     * @param enable true to enable, false to disable.
     * @param config Detection parameters; pass nullptr to use defaults.
     */
    esp_err_t SetMotionDetection(bool enable, const Sc7a20hMotionConfig* config = nullptr);

    /// Install a user-space callback fired when motion is detected.
    /// The callback runs in caller context, not from an ISR.
    void SetWakeupCallback(WakeupCallback callback);

    // ===================== Power management ======================

    /// Enter low-power mode (output disabled, < 2 uA).
    esp_err_t EnterPowerDown();

    /// Resume from low-power mode (restores the previously configured ODR).
    esp_err_t ExitPowerDown();

    // ===================== Deep-sleep wakeup =====================

    /**
     * @brief One-call deep-sleep wakeup setup using EXT1.
     *
     * After invoking this, calling @c esp_deep_sleep_start() will let the
     * MCU wake when the SC7A20H INT1 line goes low. This routine configures
     * the RTC GPIO pull-up and registers the EXT1 wakeup source for you.
     *
     * @param int1_gpio GPIO connected to SC7A20H INT1 (must be an RTC-capable GPIO).
     */
    esp_err_t ConfigDeepSleepWakeup(gpio_num_t int1_gpio);

    // ===================== Convenience ===========================

    /**
     * @brief One-call init + motion-detect with built-in 500 ms debounce.
     *
     * Equivalent to: @ref Initialize() + @ref SetMotionDetection(true)
     * with a default debounced log callback. Recommended for board-level
     * "pickup-to-wake" use cases.
     *
     * @param config Motion-detect configuration; pass nullptr to use defaults.
     */
    esp_err_t InitWithMotionDetection(const Sc7a20hMotionConfig* config = nullptr);

    // ===================== Runtime configuration =================

    /// Change the full-scale range at runtime.
    esp_err_t SetRange(Sc7a20hRange range);

    /// Change the output data rate at runtime.
    esp_err_t SetOdr(Sc7a20hOdr odr);

private:
    i2c_master_dev_handle_t i2c_dev_ = nullptr;
    bool initialized_ = false;
    Sc7a20hRange range_ = Sc7a20hRange::kRange4G;
    Sc7a20hOdr odr_ = Sc7a20hOdr::kOdr100Hz;
    WakeupCallback wakeup_callback_;

    // Low-level register access.
    esp_err_t WriteReg(uint8_t reg, uint8_t value);
    esp_err_t ReadReg(uint8_t reg, uint8_t& value);
    esp_err_t ReadRegs(uint8_t reg, uint8_t* buffer, size_t length);

    // Sensitivity (mg/LSB) for the currently selected range.
    float GetSensitivity() const;
};

#endif // SC7A20H_H
