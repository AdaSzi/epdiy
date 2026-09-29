#pragma once

#include "sdkconfig.h"

/**
 * Rendering Method / Hardware to use.
 */
enum EpdRenderMethod {
    /// Use the I2S peripheral on ESP32 chips.
    RENDER_METHOD_I2S = 1,
    /// Use the CAM/LCD peripheral in ESP32-S3 chips.
    RENDER_METHOD_LCD = 2,
};

extern const enum EpdRenderMethod EPD_CURRENT_RENDER_METHOD;

#ifdef CONFIG_IDF_TARGET_ESP32
#define RENDER_METHOD_I2S 1
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#ifdef CONFIG_EPD_ESP32S3_I80_ROW_OUTPUT
// Row-by-row render method (latch through the board's set_ctrl()), with the pixel data of each
// row sent over the LCD_CAM peripheral in i80 mode instead of the (ESP32-only) I2S peripheral.
// For boards that drive LEH/STV/OE/MODE through a shift register, see Kconfig.
#define RENDER_METHOD_I2S 1
#define EPD_I2S_ON_I80 1
#else
#define RENDER_METHOD_LCD 1
#endif
#else
#error "unknown chip, cannot choose render method!"
#endif

/**
 * The ESP32 I2S FIFO transmits the bytes of each 32 bit word in the order 2, 3, 0, 1, so the
 * I2S render path swaps the 16 bit halves of every word before output. The i80 bus sends bytes in
 * memory order and needs no such compensation.
 */
#ifdef EPD_I2S_ON_I80
#define EPD_ROW_OUTPUT_NEEDS_REORDER 0
#else
#define EPD_ROW_OUTPUT_NEEDS_REORDER 1
#endif

#ifdef __clang__
#define IRAM_ATTR
// define this if we're using clangd to make it accept the GCC builtin
void __assert_func(const char* file, int line, const char* func, const char* failedexpr);
#endif