/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file   BOARD_ELECROW_CROWPANEL_5_0.h
 * @brief  Configuration file for Elecrow CrowPanel Advance 5.0 HMI ESP32-S3
 * @author Copilot
 * @link   https://www.elecrow.com/crowpanel-advance-5-0-hmi-esp32-ai-display-800x480-ips-artificial-intelligent-touch-screen.html
 */

#pragma once

// *INDENT-OFF*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// Please update the following macros to configure general parameters ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Board name (format: "Manufacturer:Model")
 */
#define ESP_PANEL_BOARD_NAME                "Elecrow:CROWPANEL_5_0"

/**
 * @brief Panel resolution configuration in pixels
 */
#define ESP_PANEL_BOARD_WIDTH               (800)   // Panel width (horizontal, in pixels)
#define ESP_PANEL_BOARD_HEIGHT              (480)   // Panel height (vertical, in pixels)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Please update the following macros to configure the LCD panel /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief LCD panel configuration flag (0/1)
 *
 * Set to `1` to enable LCD panel support, `0` to disable
 */
#define ESP_PANEL_BOARD_USE_LCD             (1)

#if ESP_PANEL_BOARD_USE_LCD
/**
 * @brief LCD controller selection
 */
#define ESP_PANEL_BOARD_LCD_CONTROLLER      ST7262

/**
 * @brief LCD bus type selection
 */
#define ESP_PANEL_BOARD_LCD_BUS_TYPE        (ESP_PANEL_BUS_TYPE_RGB)

/**
 * @brief LCD bus parameters configuration
 *
 * Configure parameters based on the selected bus type. Parameters for other bus types will be ignored.
 * For detailed parameter explanations, see:
 * https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/api-reference/peripherals/lcd/index.html
 * https://docs.espressif.com/projects/esp-iot-solution/en/latest/display/lcd/index.html
 */
#if ESP_PANEL_BOARD_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
    /**
     * @brief RGB bus
     */
    /**
     * Set to 0 if using simple "RGB" interface which does not contain "3-wire SPI" interface.
     */
    #define ESP_PANEL_BOARD_LCD_RGB_USE_CONTROL_PANEL       (0) // 0/1. Typically set to 1

    /* For refresh panel (RGB) */
    #define ESP_PANEL_BOARD_LCD_RGB_CLK_HZ          (16 * 1000 * 1000)
                                                            // To increase the upper limit of the PCLK, see: https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/peripherals/lcd.html#how-can-i-increase-the-upper-limit-of-pclk-settings-on-esp32-s3-while-ensuring-normal-rgb-screen-display
    #define ESP_PANEL_BOARD_LCD_RGB_HPW             (4)
    #define ESP_PANEL_BOARD_LCD_RGB_HBP             (8)
    #define ESP_PANEL_BOARD_LCD_RGB_HFP             (8)
    #define ESP_PANEL_BOARD_LCD_RGB_VPW             (4)
    #define ESP_PANEL_BOARD_LCD_RGB_VBP             (8)
    #define ESP_PANEL_BOARD_LCD_RGB_VFP             (8)
    #define ESP_PANEL_BOARD_LCD_RGB_PCLK_ACTIVE_NEG (1)     // 0: rising edge, 1: falling edge. Typically set to 0
                                                                                        // The following sheet shows the valid combinations of
                                                                                        // data width and pixel bits:
                                                                                        // ┏---------------------------------┳- -------------------------------┓
    #define ESP_PANEL_BOARD_LCD_RGB_DATA_WIDTH      (16)                                // |                16               |               8                 |
    #define ESP_PANEL_BOARD_LCD_RGB_PIXEL_BITS      (ESP_PANEL_LCD_COLOR_BITS_RGB565)   // | ESP_PANEL_LCD_COLOR_BITS_RGB565 | ESP_PANEL_LCD_COLOR_BITS_RGB888 |
                                                                                        // ┗---------------------------------┻---------------------------------┛
                                                            // To understand color format of RGB LCD, see: https://docs.espressif.com/projects/esp-iot-solution/en/latest/display/lcd/rgb_lcd.html#color-formats
    #define ESP_PANEL_BOARD_LCD_RGB_BOUNCE_BUF_SIZE (ESP_PANEL_BOARD_WIDTH * 10)
                                                            // Bounce buffer size in bytes. It is used to avoid screen drift
                                                            // for ESP32-S3. Typically set to `ESP_PANEL_BOARD_WIDTH * 10`
                                                            // The size should satisfy `size * N = LCD_width * LCD_height`,
                                                            // where N is an even number.
                                                            // For more details, see: https://github.com/esp-arduino-libs/ESP32_Display_Panel/blob/master/docs/FAQ.md#how-to-fix-screen-drift-issue-when-driving-rgb-lcd-with-esp32-s3

    // CrowPanel Advance 5.0 specific GPIO assignments
    #define ESP_PANEL_BOARD_LCD_RGB_IO_HSYNC        (40)    // Your hardware: GPIO_NUM_40
    #define ESP_PANEL_BOARD_LCD_RGB_IO_VSYNC        (41)    // Your hardware: GPIO_NUM_41
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DE           (42)    // Your hardware: GPIO_NUM_42 (Data Enable)
    #define ESP_PANEL_BOARD_LCD_RGB_IO_PCLK         (39)    // Your hardware: GPIO_NUM_39
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DISP         (-1)    // -1 if not used. Typically set to -1

                                                            // The following sheet shows the mapping of ESP GPIOs to
                                                            // LCD data pins with different data width and color format:
                                                            // ┏------┳- ------------┳--------------------------┓
                                                            // | ESP: | 8-bit RGB888 |      16-bit RGB565       |
                                                            // |------|--------------|--------------------------|
                                                            // | LCD: |    RGB888    | RGB565 | RGB666 | RGB888 |
                                                            // ┗------|--------------|--------|--------|--------|
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA0        (21)    //        |      D0      |   B0   |  B0-1  |   B0-3 |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA1        (47)    //        |      D1      |   B1   |  B2    |   B4   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA2        (48)    //        |      D2      |   B2   |  B3    |   B5   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA3        (45)    //        |      D3      |   B3   |  B4    |   B6   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA4        (38)    //        |      D4      |   B4   |  B5    |   B7   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA5        (9)     //        |      D5      |   G0   |  G0    |   G0-2 |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA6        (10)    //        |      D6      |   G1   |  G1    |   G3   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA7        (11)    //        |      D7      |   G2   |  G2    |   G4   |
#if ESP_PANEL_BOARD_LCD_RGB_DATA_WIDTH > 8                  //        ┗--------------┫--------|--------|--------|
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA8        (12)    //                       |   G3   |  G3    |   G5   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA9        (13)    //                       |   G4   |  G4    |   G6   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA10       (14)    //                       |   G5   |  G5    |   G7   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA11       (7)     //                       |   R0   |  R0-1  |   R0-3 |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA12       (17)    //                       |   R1   |  R2    |   R4   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA13       (18)    //                       |   R2   |  R3    |   R5   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA14       (3)     //                       |   R3   |  R4    |   R6   |
    #define ESP_PANEL_BOARD_LCD_RGB_IO_DATA15       (46)    //                       |   R4   |  R5    |   R7   |
                                                            //                       ┗--------┻--------┻--------┛
#endif // ESP_PANEL_BOARD_LCD_RGB_DATA_WIDTH

#endif // ESP_PANEL_BOARD_LCD_BUS_TYPE

/**
 * @brief LCD vendor initialization commands
 *
 * Vendor specific initialization can be different between manufacturers, should consult the LCD supplier for
 * initialization sequence code. Please uncomment and change the following macro definitions. Otherwise, the LCD driver
 * will use the default initialization sequence code.
 *
 * The initialization sequence can be specified in two formats:
 * 1. Raw format:
 *    {command, (uint8_t []){data0, data1, ...}, data_size, delay_ms}
 * 2. Helper macros:
 *    - ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(delay_ms, command, {data0, data1, ...})
 *    - ESP_PANEL_LCD_CMD_WITH_NONE_PARAM(delay_ms, command)
 */
/*
#define ESP_PANEL_BOARD_LCD_VENDOR_INIT_CMD()                       \
    {                                                               \
        {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},   \
        {0xC0, (uint8_t []){0x3B, 0x00}, 2, 0},                     \
        {0xC1, (uint8_t []){0x0D, 0x02}, 2, 0},                     \
        {0x29, (uint8_t []){0x00}, 0, 120},                         \
        or
        ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}), \
        ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xC0, {0x3B, 0x00}),                   \
        ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xC1, {0x0D, 0x02}),                   \
        ESP_PANEL_LCD_CMD_WITH_NONE_PARAM(120, 0x29),                               \
    }
*/

/**
 * @brief LCD color configuration
 */
#define ESP_PANEL_BOARD_LCD_COLOR_BITS          (ESP_PANEL_LCD_COLOR_BITS_RGB565)
                                                        // ESP_PANEL_LCD_COLOR_BITS_RGB565/RGB666/RGB888
#define ESP_PANEL_BOARD_LCD_COLOR_BGR_ORDER     (0)     // 0: RGB, 1: BGR
#define ESP_PANEL_BOARD_LCD_COLOR_INEVRT_BIT    (1)     // 0/1

/**
 * @brief LCD transformation configuration
 */
#define ESP_PANEL_BOARD_LCD_SWAP_XY             (0)     // 0/1
#define ESP_PANEL_BOARD_LCD_MIRROR_X            (0)     // 0/1
#define ESP_PANEL_BOARD_LCD_MIRROR_Y            (0)     // 0/1
#define ESP_PANEL_BOARD_LCD_GAP_X               (0)     // [0, ESP_PANEL_BOARD_WIDTH]
#define ESP_PANEL_BOARD_LCD_GAP_Y               (0)     // [0, ESP_PANEL_BOARD_HEIGHT]

/**
 * @brief LCD reset pin configuration
 */
#define ESP_PANEL_BOARD_LCD_RST_IO              (-1)    // Reset pin, -1 if not used
#define ESP_PANEL_BOARD_LCD_RST_LEVEL           (0)     // Reset active level, 0: low, 1: high

#endif // ESP_PANEL_BOARD_USE_LCD

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Please update the following macros to configure the touch panel ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Touch panel configuration flag (0/1)
 *
 * Set to `1` to enable touch panel support, `0` to disable
 */
#define ESP_PANEL_BOARD_USE_TOUCH               (1)

#if ESP_PANEL_BOARD_USE_TOUCH
/**
 * @brief Touch controller selection
 */
#define ESP_PANEL_BOARD_TOUCH_CONTROLLER        GT911

/**
 * @brief Touch bus type selection
 */
#define ESP_PANEL_BOARD_TOUCH_BUS_TYPE          (ESP_PANEL_BUS_TYPE_I2C)

#if (ESP_PANEL_BOARD_TOUCH_BUS_TYPE == ESP_PANEL_BUS_TYPE_I2C) || \
    (ESP_PANEL_BOARD_TOUCH_BUS_TYPE == ESP_PANEL_BUS_TYPE_SPI)
/**
 * If set to 1, the bus will skip to initialize the corresponding host. Users need to initialize the host in advance.
 *
 * For drivers which created by this library, even if they use the same host, the host will be initialized only once.
 * So it is not necessary to set the macro to `1`. For other drivers (like `Wire`), please set the macro to `1`
 * ensure that the host is initialized only once.
 */
#define ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST        (0)     // 0/1. Typically set to 0
#endif

/**
 * @brief Touch bus parameters configuration
 */
#if ESP_PANEL_BOARD_TOUCH_BUS_TYPE == ESP_PANEL_BUS_TYPE_I2C

    /**
     * @brief I2C bus
     */
    /* For general */
    #define ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID           (0)     // Typically set to 0
#if !ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST
    /* For host */
    #define ESP_PANEL_BOARD_TOUCH_I2C_CLK_HZ            (400 * 1000)
                                                                // Typically set to 400K
    #define ESP_PANEL_BOARD_TOUCH_I2C_SCL_PULLUP        (0)     // 0/1. Typically set to 1
    #define ESP_PANEL_BOARD_TOUCH_I2C_SDA_PULLUP        (0)     // 0/1. Typically set to 1
    // CrowPanel Advance 5.0 specific I2C pins
    #define ESP_PANEL_BOARD_TOUCH_I2C_IO_SCL            (16)    // Your hardware: GPIO_NUM_16
    #define ESP_PANEL_BOARD_TOUCH_I2C_IO_SDA            (15)    // Your hardware: GPIO_NUM_15
#endif
    /* For panel */
    #define ESP_PANEL_BOARD_TOUCH_I2C_ADDRESS           (0x5D)  // CrowPanel 5.0 GT911 specific address
                                                                // - For touchs with only one address, set to 0
                                                                // - For touchs with multiple addresses, set to 0 or
                                                                //   the address. Like GT911, there are two addresses:
                                                                //   0x5D(default) and 0x14

#endif // ESP_PANEL_BOARD_TOUCH_BUS_TYPE

/**
 * @brief Touch panel transformation flags
 */
#define ESP_PANEL_BOARD_TOUCH_SWAP_XY           (0)     // 0/1
#define ESP_PANEL_BOARD_TOUCH_MIRROR_X          (0)     // 0/1
#define ESP_PANEL_BOARD_TOUCH_MIRROR_Y          (0)     // 0/1

/**
 * @brief Touch panel control pins
 */
#define ESP_PANEL_BOARD_TOUCH_RST_IO            (-1)    // Reset pin, -1 if not used
#define ESP_PANEL_BOARD_TOUCH_RST_LEVEL         (0)     // Reset active level, 0: low, 1: high
#define ESP_PANEL_BOARD_TOUCH_INT_IO            (-1)    // Interrupt pin, -1 if not used
#define ESP_PANEL_BOARD_TOUCH_INT_LEVEL         (0)     // Interrupt active level, 0: low, 1: high

#endif // ESP_PANEL_BOARD_USE_TOUCH

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// Please update the following macros to configure the backlight ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Backlight configuration flag (0/1)
 *
 * Set to `1` to enable backlight support, `0` to disable
 */
#define ESP_PANEL_BOARD_USE_BACKLIGHT           (1)

#if ESP_PANEL_BOARD_USE_BACKLIGHT
/**
 * @brief Backlight control type selection
 * 
 * Using CUSTOM type for STC8H1K28 microcontroller control via I2C (address 0x30)
 * CrowPanel Advance 5.0 V1.1+ uses STC8H1K28 for backlight control
 */
#define ESP_PANEL_BOARD_BACKLIGHT_TYPE          (ESP_PANEL_BACKLIGHT_TYPE_CUSTOM)

/**
 * @brief Custom backlight control function for STC8H1K28
 * 
 * @param percent Brightness percentage (0-100)
 * @param user_data User data pointer (unused)
 * @return true on success, false on failure
 */
#define ESP_PANEL_BOARD_BACKLIGHT_CUSTOM_FUNCTION(percent, user_data) \
    { \
        ESP_LOGI("CrowPanel", "Setting STC8H1K28 backlight to %d%%", percent); \
        /* Map percentage (0-100) to STC8H1K28 values (0x05-0x10) */ \
        uint8_t brightness_cmd; \
        if (percent == 0) { \
            brightness_cmd = 0x05; /* Turn off */ \
        } else { \
            /* Map 1-100% to 0x06-0x10 (6 levels) */ \
            brightness_cmd = 0x06 + ((percent - 1) * 10 / 100); \
            if (brightness_cmd > 0x10) brightness_cmd = 0x10; \
        } \
        \
        /* Send I2C command to STC8H1K28 */ \
        i2c_cmd_handle_t cmd = i2c_cmd_link_create(); \
        if (cmd == NULL) { \
            ESP_LOGE("CrowPanel", "Failed to create I2C command link"); \
            return false; \
        } \
        \
        i2c_master_start(cmd); \
        i2c_master_write_byte(cmd, (0x30 << 1) | I2C_MASTER_WRITE, true); \
        i2c_master_write_byte(cmd, brightness_cmd, true); \
        i2c_master_stop(cmd); \
        \
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); \
        i2c_cmd_link_delete(cmd); \
        \
        if (ret == ESP_OK) { \
            ESP_LOGI("CrowPanel", "STC8H1K28 backlight set to 0x%02X (%d%%)", brightness_cmd, percent); \
        } else { \
            ESP_LOGW("CrowPanel", "Failed to set STC8H1K28 backlight: %s", esp_err_to_name(ret)); \
        } \
        return (ret == ESP_OK); \
    }

/**
 * @brief Post-begin function for backlight initialization
 * Sets initial backlight brightness to maximum
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
#define ESP_PANEL_BOARD_BACKLIGHT_POST_BEGIN_FUNCTION(p) \
    { \
        ESP_LOGI("CrowPanel", "Initializing STC8H1K28 backlight controller..."); \
        /* Wait for I2C bus to be ready (shared with GT911 touch) */ \
        vTaskDelay(pdMS_TO_TICKS(100)); \
        \
        /* Set initial brightness to maximum (100%) */ \
        i2c_cmd_handle_t cmd = i2c_cmd_link_create(); \
        if (cmd != NULL) { \
            i2c_master_start(cmd); \
            i2c_master_write_byte(cmd, (0x30 << 1) | I2C_MASTER_WRITE, true); \
            i2c_master_write_byte(cmd, 0x10, true); /* Maximum brightness */ \
            i2c_master_stop(cmd); \
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); \
            i2c_cmd_link_delete(cmd); \
            if (ret == ESP_OK) { \
                ESP_LOGI("CrowPanel", "STC8H1K28 backlight initialized successfully"); \
            } \
        } \
        return true; \
    }
#endif // ESP_PANEL_BOARD_USE_BACKLIGHT

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// Please update the following macros to configure the IO expander //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief IO expander configuration flag (0/1)
 *
 * Set to `1` to enable IO expander support, `0` to disable
 * 
 * CrowPanel Advance 5.0 does not use an IO expander like the 7.0" version
 */
#define ESP_PANEL_BOARD_USE_EXPANDER            (0)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Please utilize the following macros to execute any additional code if required /////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Pre-begin function for board initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_PRE_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Post-begin function for board initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_POST_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Pre-delete function for board initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_PRE_DEL_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Post-delete function for board initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_POST_DEL_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Pre-begin function for IO expander initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_EXPANDER_PRE_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Post-begin function for IO expander initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_EXPANDER_POST_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Pre-begin function for LCD initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_LCD_PRE_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Post-begin function for LCD initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_LCD_POST_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Pre-begin function for touch panel initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_TOUCH_PRE_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Post-begin function for touch panel initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_TOUCH_POST_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Pre-begin function for backlight initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
/*
#define ESP_PANEL_BOARD_BACKLIGHT_PRE_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        return true;    \
    }
*/

/**
 * @brief Post-begin function for backlight initialization
 * Sets initial backlight brightness to maximum
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
#define ESP_PANEL_BOARD_BACKLIGHT_POST_BEGIN_FUNCTION(p) \
    { \
        ESP_LOGI("CrowPanel", "Initializing STC8H1K28 backlight controller..."); \
        /* Wait for I2C bus to be ready (shared with GT911 touch) */ \
        vTaskDelay(pdMS_TO_TICKS(100)); \
        \
        /* Set initial brightness to maximum (100%) */ \
        i2c_cmd_handle_t cmd = i2c_cmd_link_create(); \
        if (cmd != NULL) { \
            i2c_master_start(cmd); \
            i2c_master_write_byte(cmd, (0x30 << 1) | I2C_MASTER_WRITE, true); \
            i2c_master_write_byte(cmd, 0x10, true); /* Maximum brightness */ \
            i2c_master_stop(cmd); \
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); \
            i2c_cmd_link_delete(cmd); \
            if (ret == ESP_OK) { \
                ESP_LOGI("CrowPanel", "STC8H1K28 backlight initialized successfully"); \
            } \
        } \
        return true; \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// File Version ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Do not change the following versions. These version numbers are used to check compatibility between this
 * configuration file and the library. Rules for version numbers:
 * 1. Major version mismatch: Configurations are incompatible, must use library version
 * 2. Minor version mismatch: May be missing new configurations, recommended to update
 * 3. Patch version mismatch: No impact on functionality
 */
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MAJOR 1
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MINOR 1
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_PATCH 0

// *INDENT-ON*
