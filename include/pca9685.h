#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#define PCA9685_DEFAULT_ADDRESS 0x4F  // Default I2C address (A5-A0 = 0)

typedef enum {
    PCA9685_CHANNEL_0 = 0,
    PCA9685_CHANNEL_1,
    PCA9685_CHANNEL_2,
    PCA9685_CHANNEL_3,
    PCA9685_CHANNEL_4,
    PCA9685_CHANNEL_5,
    PCA9685_CHANNEL_6,
    PCA9685_CHANNEL_7,
    PCA9685_CHANNEL_8,
    PCA9685_CHANNEL_9,
    PCA9685_CHANNEL_10,
    PCA9685_CHANNEL_11,
    PCA9685_CHANNEL_12,
    PCA9685_CHANNEL_13,
    PCA9685_CHANNEL_14,
    PCA9685_CHANNEL_15,
    PCA9685_CHANNEL_COUNT
} pca9685_channel_t;

typedef struct {
    i2c_port_t i2c_port;        // I2C port number
    uint8_t i2c_addr;           // I2C address
    uint32_t i2c_freq_hz;       // I2C clock frequency
    int sda_io_num;             // SDA GPIO
    int scl_io_num;             // SCL GPIO
    float pwm_freq_hz;          // Current PWM frequency
} pca9685_dev_t;

/**
 * @brief Initialize PCA9685 device
 * @param dev Device handle
 * @param port I2C port number
 * @param addr I2C address
 * @param sda_io_num SDA GPIO pin
 * @param scl_io_num SCL GPIO pin
 * @param i2c_freq_hz I2C clock frequency (Hz)
 * @return ESP_OK on success
 */
esp_err_t pca9685_init(pca9685_dev_t *dev, i2c_port_t port, uint8_t addr, 
                      int sda_io_num, int scl_io_num, uint32_t i2c_freq_hz);

/**
 * @brief Set PWM frequency
 * @param dev Device handle
 * @param freq_hz PWM frequency (Hz, typically 50 for servos)
 * @return ESP_OK on success
 */
esp_err_t pca9685_set_frequency(pca9685_dev_t *dev, float freq_hz);

/**
 * @brief Set PWM duty cycle for a channel
 * @param dev Device handle
 * @param channel Channel number (0-15)
 * @param duty Duty cycle (0-4095, 12-bit resolution)
 * @return ESP_OK on success
 */
esp_err_t pca9685_set_duty(pca9685_dev_t *dev, pca9685_channel_t channel, uint16_t duty);

/**
 * @brief Set servo pulse width for a channel
 * @param dev Device handle
 * @param channel Channel number (0-15)
 * @param pulse_us Pulse width (µs, typically 500-2500 for servos)
 * @return ESP_OK on success
 */
esp_err_t pca9685_set_servo_pulse(pca9685_dev_t *dev, pca9685_channel_t channel, uint16_t pulse_us);

#endif // PCA9685_H