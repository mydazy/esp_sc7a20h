#ifndef SC7A20H_H
#define SC7A20H_H

#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <functional>
#include <cstdint>

/**
 * @brief SC7A20H 三轴加速度计驱动（士兰微 Silan）
 *
 * 寄存器体系与 ST LIS2DH12/LIS3DH 兼容。
 * I2C 地址: 0x18 (SA0=0) 或 0x19 (SA0=1，MyDazy 默认)。
 *
 * 功能: 加速度读取 / 运动检测中断 / 敲击检测 / 低功耗模式
 */

/// 三轴原始数据（12-bit 左对齐，需右移 4 位）
struct Sc7a20hRawAcce {
    int16_t x;
    int16_t y;
    int16_t z;
};

/// 三轴加速度值（单位: mg, 毫重力加速度）
struct Sc7a20hAcce {
    float x;
    float y;
    float z;
};

/// 量程选择
enum class Sc7a20hRange : uint8_t {
    kRange2G  = 0x00,  // ±2g (默认)
    kRange4G  = 0x10,  // ±4g
    kRange8G  = 0x20,  // ±8g
    kRange16G = 0x30,  // ±16g
};

/// 输出数据率
enum class Sc7a20hOdr : uint8_t {
    kPowerDown = 0x00,  // 关闭
    kOdr1Hz    = 0x10,  // 1 Hz (超低功耗)
    kOdr10Hz   = 0x20,  // 10 Hz (低功耗)
    kOdr25Hz   = 0x30,  // 25 Hz
    kOdr50Hz   = 0x40,  // 50 Hz
    kOdr100Hz  = 0x50,  // 100 Hz (默认)
    kOdr200Hz  = 0x60,  // 200 Hz
    kOdr400Hz  = 0x70,  // 400 Hz
};

/// 运动检测配置
struct Sc7a20hMotionConfig {
    uint8_t threshold  = 0x08;  // 中断阈值 (步进: range/128, 默认 ~250mg@4g)
    uint8_t duration   = 0x02;  // 持续时间 (步进: 1/ODR)
    bool    enable_x   = true;
    bool    enable_y   = true;
    bool    enable_z   = true;
};

class Sc7a20h {
public:
    using WakeupCallback = std::function<void()>;

    /**
     * @brief 构造函数
     * @param i2c_bus I2C 总线句柄
     * @param addr    I2C 地址 (默认 0x19)
     */
    Sc7a20h(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x19);
    ~Sc7a20h();

    // 禁止拷贝
    Sc7a20h(const Sc7a20h&) = delete;
    Sc7a20h& operator=(const Sc7a20h&) = delete;

    // ========== 设备管理 ==========

    /**
     * @brief 初始化传感器（校验 WHO_AM_I + 配置默认参数）
     * @param range 量程 (默认 ±4g)
     * @param odr   数据率 (默认 100Hz)
     * @return ESP_OK 成功, ESP_ERR_NOT_FOUND 设备未找到
     */
    esp_err_t Initialize(Sc7a20hRange range = Sc7a20hRange::kRange4G,
                         Sc7a20hOdr odr = Sc7a20hOdr::kOdr100Hz);

    /// 检查传感器是否已初始化
    bool IsInitialized() const { return initialized_; }

    // ========== 数据读取 ==========

    /**
     * @brief 读取原始加速度数据
     * @param[out] raw 原始值（12-bit 有符号，已右移）
     * @return ESP_OK 成功
     */
    esp_err_t GetRawAcce(Sc7a20hRawAcce& raw);

    /**
     * @brief 读取加速度值（mg 单位）
     * @param[out] acce 加速度值
     * @return ESP_OK 成功
     */
    esp_err_t GetAcce(Sc7a20hAcce& acce);

    // ========== 运动检测 ==========

    /**
     * @brief 启用/禁用运动检测中断
     * @param enable 是否启用
     * @param config 检测配置 (nullptr 使用默认)
     * @return ESP_OK 成功
     */
    esp_err_t SetMotionDetection(bool enable, const Sc7a20hMotionConfig* config = nullptr);

    /// 设置唤醒回调（运动检测触发时调用）
    void SetWakeupCallback(WakeupCallback callback);

    // ========== 电源管理 ==========

    /// 进入低功耗模式（关闭输出）
    esp_err_t EnterPowerDown();

    /// 退出低功耗模式（恢复之前的 ODR）
    esp_err_t ExitPowerDown();

    // ========== 深睡唤醒 ==========

    /**
     * @brief 一键配置深度睡眠唤醒（EXT1 方式）
     *
     * 调用此方法后进入 deep sleep，SC7A20H 的 INT1 引脚电平变化将唤醒主 CPU。
     * 内部自动配置 RTC GPIO 上拉和 EXT1 唤醒源。
     *
     * @param int1_gpio SC7A20H INT1 中断引脚（必须是 RTC GPIO）
     * @return ESP_OK 成功
     */
    esp_err_t ConfigDeepSleepWakeup(gpio_num_t int1_gpio);

    // ========== 便捷初始化 ==========

    /**
     * @brief 一键初始化 + 启用运动检测（board 层推荐用法）
     *
     * 等效于: Initialize() + SetMotionDetection(true) + 500ms 防抖回调
     *
     * @param config 运动检测配置 (nullptr 使用默认)
     * @return ESP_OK 成功
     */
    esp_err_t InitWithMotionDetection(const Sc7a20hMotionConfig* config = nullptr);

    // ========== 配置 ==========

    /// 设置量程
    esp_err_t SetRange(Sc7a20hRange range);

    /// 设置输出数据率
    esp_err_t SetOdr(Sc7a20hOdr odr);

private:
    i2c_master_dev_handle_t i2c_dev_ = nullptr;
    bool initialized_ = false;
    Sc7a20hRange range_ = Sc7a20hRange::kRange4G;
    Sc7a20hOdr odr_ = Sc7a20hOdr::kOdr100Hz;
    WakeupCallback wakeup_callback_;

    // I2C 寄存器操作
    esp_err_t WriteReg(uint8_t reg, uint8_t value);
    esp_err_t ReadReg(uint8_t reg, uint8_t& value);
    esp_err_t ReadRegs(uint8_t reg, uint8_t* buffer, size_t length);

    // 量程对应的灵敏度 (mg/LSB)
    float GetSensitivity() const;
};

#endif // SC7A20H_H
