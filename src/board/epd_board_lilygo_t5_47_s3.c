/**
 * Board definition for the LILYGO T5-4.7 inch e-paper board based on the ESP32-S3
 * (product name "T5 4.7 Inch E-paper V2.3 ESP32-S3", PCB marking "Screen-4.7-S3 V2.3/V2.4").
 *
 * Not to be confused with
 *  - epd_board_lilygo_t5_47: the original ESP32 (WROVER) T5-4.7, and
 *  - lilygo_board_s3: the T5-4.7 S3 *Pro*, which has a PCA9555 expander and a TPS65185.
 *
 * Like the original T5-4.7, this board drives latch enable, STV, output enable, mode and the
 * display power rails through a 74HCT4094 shift register instead of GPIOs. It therefore needs
 * the row-by-row render method. On the ESP32-S3 that has to be enabled with
 * CONFIG_EPD_ESP32S3_I80_ROW_OUTPUT (see Kconfig).
 *
 * Pin map (from LILYGO's LilyGo-EPD47 esp32s3 branch and verified on hardware):
 *   data D0..D7 = GPIO 8,1,2,3,4,5,6,7
 *   CKV = 38, STH = 40, CKH = 41
 *   shift register: data = 13, clock = 12, strobe = 0
 */
#include "epd_board.h"

#include <sdkconfig.h>

#ifdef CONFIG_EPD_ESP32S3_I80_ROW_OUTPUT

#include "../output_i2s/i2s_data_bus.h"
#include "../output_i2s/render_i2s.h"
#include "../output_i2s/rmt_pulse.h"

#define CFG_DATA GPIO_NUM_13
#define CFG_CLK GPIO_NUM_12
#define CFG_STR GPIO_NUM_0

#define D7 GPIO_NUM_7
#define D6 GPIO_NUM_6
#define D5 GPIO_NUM_5
#define D4 GPIO_NUM_4
#define D3 GPIO_NUM_3
#define D2 GPIO_NUM_2
#define D1 GPIO_NUM_1
#define D0 GPIO_NUM_8

/* Control Lines */
#define CKV GPIO_NUM_38
#define STH GPIO_NUM_40

/* Edges */
#define CKH GPIO_NUM_41

typedef struct {
    bool power_disable : 1;
    bool pos_power_enable : 1;
    bool neg_power_enable : 1;
    /// Repurposed as power enable of the touch controller / display supply. Kept set, as in
    /// LILYGO's driver.
    bool ep_scan_direction : 1;
} epd_config_register_t;

static i2s_bus_config i2s_config = {
    .clock = CKH,
    .start_pulse = STH,
    .data_0 = D0,
    .data_1 = D1,
    .data_2 = D2,
    .data_3 = D3,
    .data_4 = D4,
    .data_5 = D5,
    .data_6 = D6,
    .data_7 = D7,
};

static epd_config_register_t config_reg;

static void IRAM_ATTR push_cfg_bit(bool bit) {
    fast_gpio_set_lo(CFG_CLK);
    if (bit) {
        fast_gpio_set_hi(CFG_DATA);
    } else {
        fast_gpio_set_lo(CFG_DATA);
    }
    fast_gpio_set_hi(CFG_CLK);
}

static void epd_board_init(uint32_t epd_row_width, const EpdInitConfig* init_config) {
    (void)init_config;
    /* Power Control Output/Off */
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CFG_DATA], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CFG_CLK], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CFG_STR], PIN_FUNC_GPIO);
    gpio_set_direction(CFG_DATA, GPIO_MODE_OUTPUT);
    gpio_set_direction(CFG_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(CFG_STR, GPIO_MODE_OUTPUT);
    fast_gpio_set_lo(CFG_STR);

    config_reg.power_disable = true;
    config_reg.pos_power_enable = false;
    config_reg.neg_power_enable = false;
    config_reg.ep_scan_direction = true;

    // Set up the data bus.
    // Add an offset of dummy bytes to allow for enough timing headroom.
    i2s_bus_init(&i2s_config, epd_row_width + 32);

    rmt_pulse_init(CKV);
}

static void IRAM_ATTR
epd_board_set_ctrl(epd_ctrl_state_t* state, const epd_ctrl_state_t* const mask) {
    // STH is driven by the data bus itself (i80 D/C line) while rows are transmitted.
    if (state->ep_sth) {
        fast_gpio_set_hi(STH);
    } else {
        fast_gpio_set_lo(STH);
    }

    if (mask->ep_output_enable || mask->ep_mode || mask->ep_stv || mask->ep_latch_enable) {
        fast_gpio_set_lo(CFG_STR);

        // push config bits in reverse order
        push_cfg_bit(state->ep_output_enable);
        push_cfg_bit(state->ep_mode);
        push_cfg_bit(config_reg.ep_scan_direction);
        push_cfg_bit(state->ep_stv);

        push_cfg_bit(config_reg.neg_power_enable);
        push_cfg_bit(config_reg.pos_power_enable);
        push_cfg_bit(config_reg.power_disable);
        push_cfg_bit(state->ep_latch_enable);

        fast_gpio_set_hi(CFG_STR);
    }
}

static void epd_board_poweron(epd_ctrl_state_t* state) {
    i2s_gpio_attach(&i2s_config);

    config_reg.ep_scan_direction = true;

    // POWERON
    epd_ctrl_state_t mask = {
        // Trigger output to shift register
        .ep_stv = true,
    };
    config_reg.power_disable = false;
    epd_board_set_ctrl(state, &mask);
    epd_busy_delay(100 * 240);
    config_reg.neg_power_enable = true;
    epd_board_set_ctrl(state, &mask);
    epd_busy_delay(500 * 240);
    config_reg.pos_power_enable = true;
    epd_board_set_ctrl(state, &mask);
    epd_busy_delay(100 * 240);
    state->ep_stv = true;
    state->ep_sth = true;
    mask.ep_sth = true;
    epd_board_set_ctrl(state, &mask);
    // END POWERON
}

static void epd_board_poweroff(epd_ctrl_state_t* state) {
    // POWEROFF
    epd_ctrl_state_t mask = {
        // Trigger output to shift register
        .ep_stv = true,
    };
    config_reg.pos_power_enable = false;
    epd_board_set_ctrl(state, &mask);
    epd_busy_delay(10 * 240);

    config_reg.neg_power_enable = false;
    config_reg.pos_power_enable = false;
    epd_board_set_ctrl(state, &mask);
    epd_busy_delay(100 * 240);

    state->ep_stv = false;
    mask.ep_stv = true;
    state->ep_output_enable = false;
    mask.ep_output_enable = true;
    state->ep_mode = false;
    mask.ep_mode = true;
    config_reg.power_disable = true;
    // ep_scan_direction stays set (touch controller supply), as in LILYGO's driver.
    epd_board_set_ctrl(state, &mask);

    i2s_gpio_detach(&i2s_config);
    // END POWEROFF
}

const EpdBoardDefinition epd_board_lilygo_t5_47_s3 = {
    .init = epd_board_init,
    .deinit = NULL,
    .set_ctrl = epd_board_set_ctrl,
    .poweron = epd_board_poweron,
    .poweroff = epd_board_poweroff,
    .get_temperature = NULL,
    .set_vcom = NULL,
};

#endif  // CONFIG_EPD_ESP32S3_I80_ROW_OUTPUT
