#include "pca9685.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>

static const char *TAG = "PCA9685";

// PCA9685 Registers
#define PCA9685_REG_MODE1      0x00
#define PCA9685_REG_MODE2      0x01
#define PCA9685_REG_LED0_ON_L  0x06
#define PCA9685_REG_ALL_LED_ON_L 0xFA
#define PCA9685_REG_PRE_SCALE  0xFE

// MODE1 bits
#define PCA9685_MODE1_RESTART  (1 << 7)
#define PCA9685_MODE1_EXTCLK   (1 << 6)
#define PCA9685_MODE1_AI       (1 << 5)
#define PCA9685_MODE1_SLEEP    (1 << 4)
#define PCA9685_MODE1_ALLCALL  (1 << 0)

static esp_err_t pca9685_write_reg(pca9685_dev_t *dev, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t pca9685_read_reg(pca9685_dev_t *dev, uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t pca9685_init(pca9685_dev_t *dev, i2c_port_t port, uint8_t addr, 
                      int sda_io_num, int scl_io_num, uint32_t i2c_freq_hz) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Initializing PCA9685 at address 0x%02X", addr);
    dev->i2c_port = port;
    dev->i2c_addr = addr;
    dev->i2c_freq_hz = i2c_freq_hz;
    dev->pwm_freq_hz = 0;

    // Configure I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io_num,
        .scl_io_num = scl_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_freq_hz,
    };
    esp_err_t ret = i2c_param_config(port, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Reset PCA9685
    ret = pca9685_write_reg(dev, PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for oscillator to stabilize
    ret = pca9685_write_reg(dev, PCA9685_REG_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_RESTART);
    if (ret != ESP_OK) return ret;

    // Set all channels off
    ret = pca9685_write_reg(dev, PCA9685_REG_ALL_LED_ON_L, 0);
    if (ret != ESP_OK) return ret;
    ret = pca9685_write_reg(dev, PCA9685_REG_ALL_LED_ON_L + 1, 0);
    if (ret != ESP_OK) return ret;
    ret = pca9685_write_reg(dev, PCA9685_REG_ALL_LED_ON_L + 2, 0);
    if (ret != ESP_OK) return ret;
    ret = pca9685_write_reg(dev, PCA9685_REG_ALL_LED_ON_L + 3, 0);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "PCA9685 initialized");
    return ESP_OK;
}

esp_err_t pca9685_set_frequency(pca9685_dev_t *dev, float freq_hz) {
    if (!dev || freq_hz < 24 || freq_hz > 1526) {
        ESP_LOGE(TAG, "Invalid frequency: %.2f Hz", freq_hz);
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate prescale: round(25MHz / (4096 * freq_hz)) - 1
    uint8_t prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * freq_hz)) - 1);
    if (prescale < 3) prescale = 3; // Minimum prescale

    // Enter sleep mode
    esp_err_t ret = pca9685_write_reg(dev, PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP);
    if (ret != ESP_OK) return ret;

    // Set prescale
    ret = pca9685_write_reg(dev, PCA9685_REG_PRE_SCALE, prescale);
    if (ret != ESP_OK) return ret;

    // Restart with auto-increment
    ret = pca9685_write_reg(dev, PCA9685_REG_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_RESTART);
    if (ret != ESP_OK) return ret;

    dev->pwm_freq_hz = 25000000.0f / ((prescale + 1) * 4096.0f);
    ESP_LOGI(TAG, "Set PWM frequency to %.2f Hz (prescale=%d)", dev->pwm_freq_hz, prescale);
    return ESP_OK;
}

esp_err_t pca9685_set_duty(pca9685_dev_t *dev, pca9685_channel_t channel, uint16_t duty) {
    if (!dev || channel >= PCA9685_CHANNEL_COUNT || duty > 4095) {
        ESP_LOGE(TAG, "Invalid channel %d or duty %u", channel, duty);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = PCA9685_REG_LED0_ON_L + (channel * 4);
    uint8_t data[4] = { 0, 0, duty & 0xFF, (duty >> 8) & 0x0F }; // ON=0, OFF=duty

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, 4, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set duty for channel %d: %s", channel, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Set channel %d duty to %u", channel, duty);
    }
    return ret;
}

esp_err_t pca9685_set_servo_pulse(pca9685_dev_t *dev, pca9685_channel_t channel, uint16_t pulse_us) {
    if (!dev || channel >= PCA9685_CHANNEL_COUNT || pulse_us < 500 || pulse_us > 2500) {
        ESP_LOGE(TAG, "Invalid channel %d or pulse %u us", channel, pulse_us);
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->pwm_freq_hz == 0) {
        ESP_LOGE(TAG, "PWM frequency not set");
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate duty: (pulse_us * 4096 * freq) / 1e6
    uint32_t duty = (uint32_t)((pulse_us * 4096.0f * dev->pwm_freq_hz) / 1000000.0f);
    if (duty > 4095) duty = 4095;

    return pca9685_set_duty(dev, channel, (uint16_t)duty);
}