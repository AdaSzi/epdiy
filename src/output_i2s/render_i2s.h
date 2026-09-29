#include "epdiy.h"

#include <driver/gpio.h>
#include <esp_assert.h>
#include <esp_system.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

#include "../output_common/render_context.h"
#include "../output_common/render_method.h"
#include "sdkconfig.h"

/**
 * Lighten / darken picels using the I2S driving method.
 */
void epd_push_pixels_i2s(RenderContext_t* ctx, EpdRect area, short time, int color);

/**
 * Do a full update cycle with a configured context.
 */
void i2s_do_update(RenderContext_t* ctx);

/**
 * Worker to fetch framebuffer data and write into a queue for processing.
 */
void i2s_fetch_frame_data(RenderContext_t* ctx, int thread_id);

/**
 * Worker to output frame data to the display.
 */
void i2s_output_frame(RenderContext_t* ctx, int thread_id);

/**
 * Deinitialize the I2S peripheral for low power consumption.
 */
void i2s_deinit();

/*
 * Write bits directly using the registers in the ESP32.
 * Won't work for some pins (>= 32).
 */
static inline void fast_gpio_set_hi(gpio_num_t gpio_num) {
#ifdef CONFIG_IDF_TARGET_ESP32
    gpio_dev_t* device = GPIO_LL_GET_HW(GPIO_PORT_0);
    device->out_w1ts = (1 << gpio_num);
#elif defined(EPD_I2S_ON_I80)
    // handles the pins of the second GPIO bank (32 and up) as well
    gpio_ll_set_level(GPIO_LL_GET_HW(GPIO_PORT_0), gpio_num, 1);
#else
    // not supportd on non ESP32 chips
    assert(false);
#endif
}

static inline void fast_gpio_set_lo(gpio_num_t gpio_num) {
#ifdef CONFIG_IDF_TARGET_ESP32
    gpio_dev_t* device = GPIO_LL_GET_HW(GPIO_PORT_0);
    device->out_w1tc = (1 << gpio_num);
#elif defined(EPD_I2S_ON_I80)
    gpio_ll_set_level(GPIO_LL_GET_HW(GPIO_PORT_0), gpio_num, 0);
#else
    // not supportd on non ESP32 chips
    assert(false);
#endif
}
