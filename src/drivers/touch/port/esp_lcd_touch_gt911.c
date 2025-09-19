/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "../esp_panel_touch_conf_internal.h"
#if ESP_PANEL_DRIVERS_TOUCH_ENABLE_GT911

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"

#include "utils/esp_panel_utils_log.h"
#include "esp_utils_helpers.h"

static const char *TAG = "GT911";

/* GT911 registers */
#define ESP_LCD_TOUCH_GT911_READ_KEY_REG    (0x8093)
#define ESP_LCD_TOUCH_GT911_READ_XY_REG     (0x814E)
#define ESP_LCD_TOUCH_GT911_CONFIG_REG      (0x8047)
#define ESP_LCD_TOUCH_GT911_PRODUCT_ID_REG  (0x8140)
#define ESP_LCD_TOUCH_GT911_ENTER_SLEEP     (0x8040)

/* GT911 support key num */
#define ESP_GT911_TOUCH_MAX_BUTTONS         (4)

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_gt911_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
static esp_err_t esp_lcd_touch_gt911_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state);
#endif
static esp_err_t esp_lcd_touch_gt911_del(esp_lcd_touch_handle_t tp);

/* I2C read/write */
static esp_err_t touch_gt911_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_gt911_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data);

/* GT911 reset */
static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp);
/* Read status and config register */
static esp_err_t touch_gt911_read_cfg(esp_lcd_touch_handle_t tp);

/* GT911 enter/exit sleep mode */
static esp_err_t esp_lcd_touch_gt911_enter_sleep(esp_lcd_touch_handle_t tp);
static esp_err_t esp_lcd_touch_gt911_exit_sleep(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_i2c_gt911(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", ESP_LCD_TOUCH_GT911_VER_MAJOR, ESP_LCD_TOUCH_GT911_VER_MINOR,
             ESP_LCD_TOUCH_GT911_VER_PATCH);
    ESP_LOGI(TAG, "Starting GT911 initialization...");
    esp_err_t ret = ESP_OK;

    assert(io != NULL);
    assert(config != NULL);
    assert(out_touch != NULL);

    /* Prepare main structure */
    // Use `calloc` instead of `heap_caps_calloc` for MicroPython compatibility
    esp_lcd_touch_handle_t esp_lcd_touch_gt911 = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(esp_lcd_touch_gt911, ESP_ERR_NO_MEM, err, TAG, "no mem for GT911 controller");

    /* Communication interface */
    esp_lcd_touch_gt911->io = io;

    /* Only supported callbacks are set */
    esp_lcd_touch_gt911->read_data = esp_lcd_touch_gt911_read_data;
    esp_lcd_touch_gt911->get_xy = esp_lcd_touch_gt911_get_xy;
#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
    esp_lcd_touch_gt911->get_button_state = esp_lcd_touch_gt911_get_button_state;
#endif
    esp_lcd_touch_gt911->del = esp_lcd_touch_gt911_del;
    esp_lcd_touch_gt911->enter_sleep = esp_lcd_touch_gt911_enter_sleep;
    esp_lcd_touch_gt911->exit_sleep = esp_lcd_touch_gt911_exit_sleep;

    /* Mutex */
    esp_lcd_touch_gt911->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_gt911->config, config, sizeof(esp_lcd_touch_config_t));
    esp_lcd_touch_io_gt911_config_t *gt911_config = (esp_lcd_touch_io_gt911_config_t *)esp_lcd_touch_gt911->config.driver_data;
    
    ESP_LOGI(TAG, "GT911 config - RST pin: %d, INT pin: %d", esp_lcd_touch_gt911->config.rst_gpio_num, esp_lcd_touch_gt911->config.int_gpio_num);
    if (gt911_config) {
        ESP_LOGI(TAG, "GT911 I2C address configured: 0x%02X", gt911_config->dev_addr);
    } else {
        ESP_LOGW(TAG, "GT911 config driver_data is NULL");
    }

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_gt911->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    if (gt911_config && esp_lcd_touch_gt911->config.rst_gpio_num != GPIO_NUM_NC && esp_lcd_touch_gt911->config.int_gpio_num != GPIO_NUM_NC) {
        /* Prepare pin for touch controller int */
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pull_down_en = 0,
            .pull_up_en = 1,
            .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.int_gpio_num),
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");

        ESP_LOGI(TAG, "Setting RST pin %d to level %d", esp_lcd_touch_gt911->config.rst_gpio_num, esp_lcd_touch_gt911->config.levels.reset);
        ESP_RETURN_ON_ERROR(gpio_set_level(esp_lcd_touch_gt911->config.rst_gpio_num, esp_lcd_touch_gt911->config.levels.reset), TAG, "GPIO set level error!");
        ESP_LOGI(TAG, "Setting INT pin %d to LOW (0) for address configuration", esp_lcd_touch_gt911->config.int_gpio_num);
        ESP_RETURN_ON_ERROR(gpio_set_level(esp_lcd_touch_gt911->config.int_gpio_num, 0), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Select I2C addr, set output high or low */
        uint32_t gpio_level;
        ESP_LOGI(TAG, "Configuring I2C address - requested: 0x%02X, backup: 0x%02X, default: 0x%02X", 
                 gt911_config->dev_addr, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS);
        if (ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP == gt911_config->dev_addr) {
            gpio_level = 1;
            ESP_LOGI(TAG, "Using BACKUP address 0x%02X -> INT pin will be set HIGH", ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
        } else if (ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS == gt911_config->dev_addr) {
            gpio_level = 0;
            ESP_LOGI(TAG, "Using DEFAULT address 0x%02X -> INT pin will be set LOW", ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS);
        } else {
            gpio_level = 0;
            ESP_LOGE(TAG, "Invalid address 0x%02X! Using default 0x%02X -> INT pin will be set LOW", gt911_config->dev_addr, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS);
        }
        ESP_LOGI(TAG, "Setting INT pin %d to level %d for address selection", esp_lcd_touch_gt911->config.int_gpio_num, gpio_level);
        ESP_RETURN_ON_ERROR(gpio_set_level(esp_lcd_touch_gt911->config.int_gpio_num, gpio_level), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(1));

        ESP_LOGI(TAG, "Releasing RST pin %d (setting to %d) to complete address configuration", esp_lcd_touch_gt911->config.rst_gpio_num, !esp_lcd_touch_gt911->config.levels.reset);
        ESP_RETURN_ON_ERROR(gpio_set_level(esp_lcd_touch_gt911->config.rst_gpio_num, !esp_lcd_touch_gt911->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));

        ESP_LOGI(TAG, "Waiting 50ms for GT911 to stabilize with configured I2C address...");
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGI(TAG, "GT911 address configuration sequence completed");
    } else {
        ESP_LOGW(TAG, "Unable to initialize the I2C address - missing config or pins not configured");
        ESP_LOGW(TAG, "GT911 config present: %s, RST pin: %d, INT pin: %d", 
                 gt911_config ? "YES" : "NO", 
                 esp_lcd_touch_gt911->config.rst_gpio_num, 
                 esp_lcd_touch_gt911->config.int_gpio_num);
        /* Reset controller */
        ESP_LOGI(TAG, "Performing simple reset without address configuration");
        ret = touch_gt911_reset(esp_lcd_touch_gt911);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GT911 reset failed");
        ESP_LOGI(TAG, "Simple reset completed");
    }

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_gt911->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = (esp_lcd_touch_gt911->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
            .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");

        /* Register interrupt callback */
        if (esp_lcd_touch_gt911->config.interrupt_callback) {
            esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_gt911, esp_lcd_touch_gt911->config.interrupt_callback);
        }
    }

    /* Read status and config info */
    ESP_LOGI(TAG, "Attempting to read GT911 configuration and verify communication...");
    ret = touch_gt911_read_cfg(esp_lcd_touch_gt911);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GT911 communication successful - initialization completed");
    }
    ESP_GOTO_ON_ERROR(ret, err, TAG, "GT911 init failed");

    *out_touch = esp_lcd_touch_gt911;

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller GT911 initialization failed!", ret);
        if (esp_lcd_touch_gt911) {
            esp_lcd_touch_gt911_del(esp_lcd_touch_gt911);
        }
    }

    return ret;
}

static esp_err_t esp_lcd_touch_gt911_enter_sleep(esp_lcd_touch_handle_t tp)
{
    esp_err_t err = touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_ENTER_SLEEP, 0x05);
    ESP_RETURN_ON_ERROR(err, TAG, "Enter Sleep failed!");

    return ESP_OK;
}

static esp_err_t esp_lcd_touch_gt911_exit_sleep(esp_lcd_touch_handle_t tp)
{
    esp_err_t ret;
    esp_lcd_touch_handle_t esp_lcd_touch_gt911 = tp;

    if (esp_lcd_touch_gt911->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config_high = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config_high);
        ESP_RETURN_ON_ERROR(ret, TAG, "High GPIO config failed");
        gpio_set_level(esp_lcd_touch_gt911->config.int_gpio_num, 1);

        vTaskDelay(pdMS_TO_TICKS(5));

        const gpio_config_t int_gpio_config_float = {
            .mode = GPIO_MODE_OUTPUT_OD,
            .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config_float);
        ESP_RETURN_ON_ERROR(ret, TAG, "Float GPIO config failed");
    }

    return ESP_OK;
}

static esp_err_t esp_lcd_touch_gt911_read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t buf[41];
    uint8_t touch_cnt = 0;
    uint8_t clear = 0;
    size_t i = 0;

    assert(tp != NULL);

    err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, buf, 1);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    /* Any touch data? */
    if ((buf[0] & 0x80) == 0x00) {
        touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
    } else if ((buf[0] & 0x10) == 0x10) {
        /* Read all keys */
        uint8_t key_max = ((ESP_GT911_TOUCH_MAX_BUTTONS < CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS) ? \
                           (ESP_GT911_TOUCH_MAX_BUTTONS) : (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS));
        err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_KEY_REG, &buf[0], key_max);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

        /* Clear all */
        touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C write error!");

        portENTER_CRITICAL(&tp->data.lock);

        /* Buttons count */
        tp->data.buttons = key_max;
        for (i = 0; i < key_max; i++) {
            tp->data.button[i].status = buf[0] ? 1 : 0;
        }

        portEXIT_CRITICAL(&tp->data.lock);
#endif
    } else if ((buf[0] & 0x80) == 0x80) {
#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
        portENTER_CRITICAL(&tp->data.lock);
        for (i = 0; i < CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS; i++) {
            tp->data.button[i].status = 0;
        }
        portEXIT_CRITICAL(&tp->data.lock);
#endif
        /* Count of touched points */
        touch_cnt = buf[0] & 0x0f;
        if (touch_cnt > 5 || touch_cnt == 0) {
            touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
            return ESP_OK;
        }

        /* Read all points */
        err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG + 1, &buf[1], touch_cnt * 8);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

        /* Clear all */
        err = touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

        portENTER_CRITICAL(&tp->data.lock);

        /* Number of touched points */
        touch_cnt = (touch_cnt > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : touch_cnt);
        tp->data.points = touch_cnt;

        /* Fill all coordinates */
        for (i = 0; i < touch_cnt; i++) {
            tp->data.coords[i].x = ((uint16_t)buf[(i * 8) + 3] << 8) + buf[(i * 8) + 2];
            tp->data.coords[i].y = (((uint16_t)buf[(i * 8) + 5] << 8) + buf[(i * 8) + 4]);
            tp->data.coords[i].strength = (((uint16_t)buf[(i * 8) + 7] << 8) + buf[(i * 8) + 6]);
        }

        portEXIT_CRITICAL(&tp->data.lock);
    }

    return ESP_OK;
}

static bool esp_lcd_touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
static esp_err_t esp_lcd_touch_gt911_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state)
{
    esp_err_t err = ESP_OK;
    assert(tp != NULL);
    assert(state != NULL);

    *state = 0;

    portENTER_CRITICAL(&tp->data.lock);

    if (n > tp->data.buttons) {
        err = ESP_ERR_INVALID_ARG;
    } else {
        *state = tp->data.button[n].status;
    }

    portEXIT_CRITICAL(&tp->data.lock);

    return err;
}
#endif

static esp_err_t esp_lcd_touch_gt911_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
        if (tp->config.interrupt_callback) {
            gpio_isr_handler_remove(tp->config.int_gpio_num);
        }
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

/*******************************************************************************
* Private API function
*******************************************************************************/

/* Reset controller */
static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "Performing GT911 reset using RST pin %d", tp->config.rst_gpio_num);
        ESP_LOGI(TAG, "Setting RST to %d (reset state)", tp->config.levels.reset);
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "Setting RST to %d (normal state)", !tp->config.levels.reset);
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "GT911 reset completed");
    } else {
        ESP_LOGW(TAG, "GT911 reset requested but RST pin is not configured (GPIO_NUM_NC)");
    }

    return ESP_OK;
}

static esp_err_t touch_gt911_read_cfg(esp_lcd_touch_handle_t tp)
{
    uint8_t buf[4];
    esp_err_t ret;

    assert(tp != NULL);

    ESP_LOGI(TAG, "Reading GT911 Product ID from register 0x%04X...", ESP_LCD_TOUCH_GT911_PRODUCT_ID_REG);
    ret = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_PRODUCT_ID_REG, (uint8_t *)&buf[0], 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Product ID: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "Reading GT911 Config Version from register 0x%04X...", ESP_LCD_TOUCH_GT911_CONFIG_REG);
    ret = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_CONFIG_REG, (uint8_t *)&buf[3], 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Config Version: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }

    ESP_LOGI(TAG, "GT911 Product ID: 0x%02x,0x%02x,0x%02x (should be ASCII '911')", buf[0], buf[1], buf[2]);
    ESP_LOGI(TAG, "GT911 Config Version: %d", buf[3]);
    
    // Verificar si los datos son válidos
    if (buf[0] == '9' && buf[1] == '1' && buf[2] == '1') {
        ESP_LOGI(TAG, "GT911 Product ID verification: SUCCESS");
    } else {
        ESP_LOGW(TAG, "GT911 Product ID verification: FAILED - expected '911', got '%c%c%c'", buf[0], buf[1], buf[2]);
    }

    return ESP_OK;
}

static esp_err_t touch_gt911_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    esp_err_t ret;
    assert(tp != NULL);
    assert(data != NULL);

    ESP_LOGD(TAG, "I2C READ: register=0x%04X, length=%d", reg, len);
    /* Read data */
    ret = esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C READ FAILED: register=0x%04X, error=%s (0x%x)", reg, esp_err_to_name(ret), ret);
    } else {
        ESP_LOGD(TAG, "I2C READ SUCCESS: register=0x%04X, length=%d", reg, len);
    }
    return ret;
}

static esp_err_t touch_gt911_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data)
{
    esp_err_t ret;
    assert(tp != NULL);

    ESP_LOGD(TAG, "I2C WRITE: register=0x%04X, data=0x%02X", reg, data);
    // *INDENT-OFF*
    /* Write data */
    ret = esp_lcd_panel_io_tx_param(tp->io, reg, (uint8_t[]){data}, 1);
    // *INDENT-ON*
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C WRITE FAILED: register=0x%04X, data=0x%02X, error=%s (0x%x)", reg, data, esp_err_to_name(ret), ret);
    } else {
        ESP_LOGD(TAG, "I2C WRITE SUCCESS: register=0x%04X, data=0x%02X", reg, data);
    }
    return ret;
}

#endif // ESP_PANEL_DRIVERS_TOUCH_ENABLE_GT911
