#include "sc7a20h.h"
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <driver/rtc_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

#define TAG "Sc7a20h"

// SC7A20H 寄存器地址（兼容 LIS2DH12）
#define REG_WHO_AM_I        0x0F
#define REG_CTRL_REG1       0x20  // ODR + 轴使能 + 低功耗
#define REG_CTRL_REG2       0x21  // 高通滤波器
#define REG_CTRL_REG3       0x22  // INT1 中断路由
#define REG_CTRL_REG4       0x23  // 量程 + 分辨率
#define REG_CTRL_REG5       0x24  // FIFO / 锁存控制
#define REG_CTRL_REG6       0x25  // INT2 / 极性
#define REG_OUT_X_L         0x28
#define REG_OUT_X_H         0x29
#define REG_OUT_Y_L         0x2A
#define REG_OUT_Y_H         0x2B
#define REG_OUT_Z_L         0x2C
#define REG_OUT_Z_H         0x2D
#define REG_INT1_CFG        0x30
#define REG_INT1_THS        0x32
#define REG_INT1_DURATION   0x33

// 期望的设备 ID
#define DEVICE_ID           0x11

// CTRL_REG1: XYZ 轴使能位
#define AXES_ENABLE         0x07  // X + Y + Z

// CTRL_REG3: INT1 使能位
#define INT1_AOI1           0x40  // INT1 路由到 AOI on INT1

// I2C 超时
#define I2C_TIMEOUT_MS      100

// ==================== 构造/析构 ====================

Sc7a20h::Sc7a20h(i2c_master_bus_handle_t i2c_bus, uint8_t addr) {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 400000;

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 设备添加失败: 0x%02X, err=%d", addr, ret);
        i2c_dev_ = nullptr;
    }
}

Sc7a20h::~Sc7a20h() {
    if (initialized_) {
        EnterPowerDown();
    }
    if (i2c_dev_) {
        i2c_master_bus_rm_device(i2c_dev_);
    }
}

// ==================== I2C 操作 ====================

esp_err_t Sc7a20h::WriteReg(uint8_t reg, uint8_t value) {
    if (!i2c_dev_) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(i2c_dev_, buf, 2, I2C_TIMEOUT_MS);
}

esp_err_t Sc7a20h::ReadReg(uint8_t reg, uint8_t& value) {
    if (!i2c_dev_) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(i2c_dev_, &reg, 1, &value, 1, I2C_TIMEOUT_MS);
}

esp_err_t Sc7a20h::ReadRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    if (!i2c_dev_) return ESP_ERR_INVALID_STATE;
    // 连续读取: 设置 MSB（自动地址递增）
    uint8_t reg_addr = reg | 0x80;
    return i2c_master_transmit_receive(i2c_dev_, &reg_addr, 1, buffer, length, I2C_TIMEOUT_MS);
}

// ==================== 初始化 ====================

esp_err_t Sc7a20h::Initialize(Sc7a20hRange range, Sc7a20hOdr odr) {
    if (!i2c_dev_) return ESP_ERR_INVALID_STATE;

    // 校验 WHO_AM_I
    uint8_t id = 0;
    esp_err_t ret = ReadReg(REG_WHO_AM_I, id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取 WHO_AM_I 失败: %d", ret);
        return ret;
    }
    if (id != DEVICE_ID) {
        ESP_LOGE(TAG, "设备 ID 不匹配: 0x%02X, 期望 0x%02X", id, DEVICE_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "设备 ID 确认: 0x%02X", id);

    // 先关闭输出，安全配置
    ret = WriteReg(REG_CTRL_REG1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    // CTRL_REG4: 量程 + BDU(块数据更新) + 高分辨率
    range_ = range;
    ret = WriteReg(REG_CTRL_REG4, static_cast<uint8_t>(range) | 0x80);  // 0x80 = BDU
    if (ret != ESP_OK) return ret;

    // CTRL_REG2: 高通滤波器 — 使能用于 INT1
    ret = WriteReg(REG_CTRL_REG2, 0x01);
    if (ret != ESP_OK) return ret;

    // CTRL_REG5: 锁存中断请求
    ret = WriteReg(REG_CTRL_REG5, 0x08);  // LIR_INT1
    if (ret != ESP_OK) return ret;

    // CTRL_REG3: INT1 路由到运动检测 (AOI1)
    ret = WriteReg(REG_CTRL_REG3, INT1_AOI1);
    if (ret != ESP_OK) return ret;

    // CTRL_REG6: INT1 极性 — 高电平有效
    ret = WriteReg(REG_CTRL_REG6, 0x02);
    if (ret != ESP_OK) return ret;

    // CTRL_REG1: 启用 XYZ + 设置 ODR
    odr_ = odr;
    ret = WriteReg(REG_CTRL_REG1, static_cast<uint8_t>(odr) | AXES_ENABLE);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    initialized_ = true;
    ESP_LOGI(TAG, "初始化完成 (量程=%d, ODR=0x%02X)", static_cast<int>(range), static_cast<uint8_t>(odr));
    return ESP_OK;
}

// ==================== 数据读取 ====================

esp_err_t Sc7a20h::GetRawAcce(Sc7a20hRawAcce& raw) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    uint8_t buf[6] = {};
    esp_err_t ret = ReadRegs(REG_OUT_X_L, buf, 6);
    if (ret != ESP_OK) return ret;

    // 12-bit 左对齐，右移 4 位得到有符号值
    raw.x = static_cast<int16_t>((buf[1] << 8) | buf[0]) >> 4;
    raw.y = static_cast<int16_t>((buf[3] << 8) | buf[2]) >> 4;
    raw.z = static_cast<int16_t>((buf[5] << 8) | buf[4]) >> 4;
    return ESP_OK;
}

float Sc7a20h::GetSensitivity() const {
    // mg/LSB (12-bit 模式)
    switch (range_) {
        case Sc7a20hRange::kRange2G:  return 1.0f;
        case Sc7a20hRange::kRange4G:  return 2.0f;
        case Sc7a20hRange::kRange8G:  return 4.0f;
        case Sc7a20hRange::kRange16G: return 12.0f;
        default: return 2.0f;
    }
}

esp_err_t Sc7a20h::GetAcce(Sc7a20hAcce& acce) {
    Sc7a20hRawAcce raw = {};
    esp_err_t ret = GetRawAcce(raw);
    if (ret != ESP_OK) return ret;

    float sens = GetSensitivity();
    acce.x = raw.x * sens;
    acce.y = raw.y * sens;
    acce.z = raw.z * sens;
    return ESP_OK;
}

// ==================== 运动检测 ====================

esp_err_t Sc7a20h::SetMotionDetection(bool enable, const Sc7a20hMotionConfig* config) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (enable) {
        Sc7a20hMotionConfig cfg;
        if (config) cfg = *config;

        esp_err_t ret = WriteReg(REG_INT1_THS, cfg.threshold);
        if (ret != ESP_OK) return ret;

        ret = WriteReg(REG_INT1_DURATION, cfg.duration);
        if (ret != ESP_OK) return ret;

        // INT1_CFG: 设置检测轴 (OR 组合, 高事件)
        uint8_t int_cfg = 0;
        if (cfg.enable_x) int_cfg |= 0x02;  // XHIE
        if (cfg.enable_y) int_cfg |= 0x08;  // YHIE
        if (cfg.enable_z) int_cfg |= 0x20;  // ZHIE
        ret = WriteReg(REG_INT1_CFG, int_cfg);
        if (ret != ESP_OK) return ret;

        ESP_LOGI(TAG, "运动检测已启用 (阈值=0x%02X, 持续=0x%02X)", cfg.threshold, cfg.duration);
    } else {
        esp_err_t ret = WriteReg(REG_INT1_CFG, 0x00);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "运动检测已禁用");
    }
    return ESP_OK;
}

void Sc7a20h::SetWakeupCallback(WakeupCallback callback) {
    wakeup_callback_ = callback;
}

// ==================== 电源管理 ====================

esp_err_t Sc7a20h::EnterPowerDown() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = WriteReg(REG_CTRL_REG1, 0x00);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "已进入低功耗模式");
    }
    return ret;
}

esp_err_t Sc7a20h::ExitPowerDown() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = WriteReg(REG_CTRL_REG1, static_cast<uint8_t>(odr_) | AXES_ENABLE);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "已退出低功耗模式");
    }
    return ret;
}

// ==================== 配置 ====================

esp_err_t Sc7a20h::SetRange(Sc7a20hRange range) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = WriteReg(REG_CTRL_REG4, static_cast<uint8_t>(range) | 0x80);
    if (ret == ESP_OK) {
        range_ = range;
    }
    return ret;
}

esp_err_t Sc7a20h::SetOdr(Sc7a20hOdr odr) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = WriteReg(REG_CTRL_REG1, static_cast<uint8_t>(odr) | AXES_ENABLE);
    if (ret == ESP_OK) {
        odr_ = odr;
    }
    return ret;
}

// ==================== 深睡唤醒 ====================

esp_err_t Sc7a20h::ConfigDeepSleepWakeup(gpio_num_t int1_gpio) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // 配置 RTC GPIO：上拉，检测低电平唤醒
    esp_err_t ret = esp_sleep_enable_ext1_wakeup_io(
        (1ULL << int1_gpio), ESP_EXT1_WAKEUP_ANY_LOW);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EXT1 唤醒源配置失败: %d", ret);
        return ret;
    }

    ret = rtc_gpio_pullup_en(int1_gpio);
    if (ret != ESP_OK) return ret;

    ret = rtc_gpio_pulldown_dis(int1_gpio);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "深睡唤醒已配置, GPIO%d", int1_gpio);
    return ESP_OK;
}

// ==================== 便捷初始化 ====================

esp_err_t Sc7a20h::InitWithMotionDetection(const Sc7a20hMotionConfig* config) {
    esp_err_t ret = Initialize();
    if (ret != ESP_OK) return ret;

    // 内置 500ms 防抖回调
    SetWakeupCallback([]() {
        static uint64_t last_time = 0;
        uint64_t now = esp_timer_get_time();
        if (now - last_time > 500000) {  // 500ms 防抖
            last_time = now;
            ESP_LOGI(TAG, "运动检测触发");
        }
    });

    ret = SetMotionDetection(true, config);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "一键初始化完成: 运动检测+防抖回调");
    return ESP_OK;
}
