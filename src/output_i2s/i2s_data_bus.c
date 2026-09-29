#include "i2s_data_bus.h"

#include "esp_intr_alloc.h"
#include "sdkconfig.h"

// the I2S driver is based on ESP32 registers and won't compile on the S3
#ifdef CONFIG_IDF_TARGET_ESP32

#include <esp_idf_version.h>

#include "driver/rtc_io.h"
#include "esp_system.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#include "esp_rom_gpio.h"
#include "rom/lldesc.h"
#else
#include "esp32/rom/gpio.h"
#include "esp32/rom/lldesc.h"
#endif
#include "esp_heap_caps.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/rtc.h"

#include "esp_private/periph_ctrl.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#include "hal/i2s_ll.h"
#endif

/// DMA descriptors for front and back line buffer.
/// We use two buffers, so one can be filled while the other
/// is transmitted.
typedef struct {
    volatile lldesc_t* dma_desc_a;
    volatile lldesc_t* dma_desc_b;

    /// Front and back line buffer.
    uint8_t* buf_a;
    uint8_t* buf_b;
} i2s_parallel_state_t;

/// Indicates which line buffer is currently back / front.
static int current_buffer = 0;

/// The I2S state instance.
static i2s_parallel_state_t i2s_state;

static intr_handle_t gI2S_intr_handle = NULL;

/// Indicates the device has finished its transmission and is ready again.
static volatile bool output_done = true;
/// The start pulse pin extracted from the configuration for use in the "done"
/// interrupt.
static gpio_num_t start_pulse_pin;

/// Initializes a DMA descriptor.
static void fill_dma_desc(
    volatile lldesc_t* dmadesc, uint8_t* buf, uint32_t buf_size, uint32_t buf_length
) {
    assert(buf_length % 4 == 0);  // Must be word aligned
    dmadesc->size = buf_size;
    dmadesc->length = buf_length;
    dmadesc->buf = buf;
    dmadesc->eof = 1;
    dmadesc->sosf = 1;
    dmadesc->owner = 1;
    dmadesc->qe.stqe_next = 0;
    dmadesc->offset = 0;
}

/// Address of the currently front DMA descriptor,
/// which uses only the lower 20bits (according to TRM)
uint32_t dma_desc_addr() {
    return (uint32_t)(current_buffer ? i2s_state.dma_desc_a : i2s_state.dma_desc_b) & 0x000FFFFF;
}

// Helper function to help align values
static size_t align_up(size_t x, size_t a) {
    return (size_t)(x + ((size_t)a - 1)) & ~(size_t)(a - 1);
}

/// Set up a GPIO as output and route it to a signal.
static void gpio_setup_out(int gpio, int sig, bool invert) {
    if (gpio == -1)
        return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_DEF_OUTPUT);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_rom_gpio_connect_out_signal(gpio, sig, invert, false);
#else
    gpio_matrix_out(gpio, sig, invert, false);
#endif
}

/// Resets "Start Pulse" signal when the current row output is done.
static void IRAM_ATTR i2s_int_hdl(void* arg) {
    i2s_dev_t* dev = &I2S1;
    bool done = dev->int_st.out_done;

    // Clear the interrupt before publishing completion so a new transfer cannot race the ACK.
    dev->int_clr.val = dev->int_raw.val;

    if (done) {
        // gpio_set_level(start_pulse_pin, 1);
        // gpio_set_level(GPIO_NUM_26, 0);
        output_done = true;
    }
}

uint8_t* IRAM_ATTR i2s_get_current_buffer() {
    return (uint8_t*)(current_buffer ? i2s_state.dma_desc_a->buf : i2s_state.dma_desc_b->buf);
}

bool IRAM_ATTR i2s_is_busy() {
    // DMA and FIFO must be done
    return !output_done || !I2S1.state.tx_idle;
}

void IRAM_ATTR i2s_switch_buffer() {
    // either device is done transmitting or the switch must be away from the
    // buffer currently used by the DMA engine.
    while (i2s_is_busy() && dma_desc_addr() != I2S1.out_link.addr) {
    };
    current_buffer = !current_buffer;
}

void IRAM_ATTR i2s_start_line_output() {
    output_done = false;

    i2s_dev_t* dev = &I2S1;
    dev->conf.tx_start = 0;
    dev->conf.tx_reset = 1;
    dev->conf.tx_fifo_reset = 1;
    dev->conf.rx_fifo_reset = 1;
    dev->conf.tx_reset = 0;
    dev->conf.tx_fifo_reset = 0;
    dev->conf.rx_fifo_reset = 0;
    dev->out_link.addr = dma_desc_addr();
    dev->out_link.start = 1;

    // sth is pulled up through peripheral interrupt
    // This is timing-critical!
    gpio_set_level(start_pulse_pin, 0);
    dev->conf.tx_start = 1;
}

void i2s_gpio_attach(i2s_bus_config* cfg) {
    gpio_num_t I2S_GPIO_BUS[] = { cfg->data_6, cfg->data_7, cfg->data_4, cfg->data_5,
                                  cfg->data_2, cfg->data_3, cfg->data_0, cfg->data_1 };

    gpio_set_direction(cfg->start_pulse, GPIO_MODE_OUTPUT);
    gpio_set_level(cfg->start_pulse, 1);
    // Use I2S1 with no signal offset (for some reason the offset seems to be
    // needed in 16-bit mode, but not in 8 bit mode.
    int signal_base = I2S1O_DATA_OUT0_IDX;

    // Setup and route GPIOS
    for (int x = 0; x < 8; x++) {
        gpio_setup_out(I2S_GPIO_BUS[x], signal_base + x, false);
    }

    // Free CKH after wakeup
    gpio_hold_dis(cfg->clock);
    // Invert word select signal
    gpio_setup_out(cfg->clock, I2S1O_WS_OUT_IDX, true);
}

void i2s_gpio_detach(i2s_bus_config* cfg) {
    gpio_set_direction(cfg->data_0, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_1, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_2, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_3, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_4, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_5, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_6, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_7, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->start_pulse, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->clock, GPIO_MODE_INPUT);

    gpio_reset_pin(cfg->clock);
    if (cfg->clock != 5) {
        rtc_gpio_isolate(cfg->clock);
    }
}

void i2s_bus_init(i2s_bus_config* cfg, uint32_t epd_row_width) {
    i2s_gpio_attach(cfg);

    // store pin in global variable for use in interrupt.
    start_pulse_pin = cfg->start_pulse;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    PERIPH_RCC_ATOMIC() {
        i2s_ll_enable_bus_clock(1, true);
        i2s_ll_reset_register(1);
    }
#else
    periph_module_enable(PERIPH_I2S1_MODULE);
#endif

    i2s_dev_t* dev = &I2S1;

    // Initialize device
    dev->conf.tx_reset = 1;
    dev->conf.tx_reset = 0;

    // Reset DMA
    dev->lc_conf.in_rst = 1;
    dev->lc_conf.in_rst = 0;
    dev->lc_conf.out_rst = 1;
    dev->lc_conf.out_rst = 0;

    // Setup I2S config. See section 12 of Technical Reference Manual
    // Enable LCD mode
    dev->conf2.val = 0;
    dev->conf2.lcd_en = 1;

    // Enable FRAME1-Mode (See technical reference manual)
    dev->conf2.lcd_tx_wrx2_en = 1;
    dev->conf2.lcd_tx_sdx2_en = 0;

    // Set to 8 bit parallel output
    dev->sample_rate_conf.val = 0;
    dev->sample_rate_conf.tx_bits_mod = 8;

    // Half speed of bit clock in LCD mode.
    // (Smallest possible divider according to the spec).
    dev->sample_rate_conf.tx_bck_div_num = 2;

    // Initialize Audio Clock (APLL)
    rtc_clk_apll_enable(true);
    rtc_clk_apll_coeff_set(0, 0, 0, 8);

    // Set Audio Clock Dividers
    dev->clkm_conf.val = 0;
    dev->clkm_conf.clka_en = 1;
    dev->clkm_conf.clkm_div_a = 1;
    dev->clkm_conf.clkm_div_b = 0;
    // 2 is the smallest possible divider according to the spec.
    dev->clkm_conf.clkm_div_num = 2;

    // Set up FIFO
    dev->fifo_conf.val = 0;
    dev->fifo_conf.tx_fifo_mod_force_en = 1;
    dev->fifo_conf.tx_fifo_mod = 1;
    dev->fifo_conf.tx_data_num = 32;
    dev->fifo_conf.dscr_en = 1;

    // Stop after transmission complete
    dev->conf1.val = 0;
    dev->conf1.tx_stop_en = 0;
    dev->conf1.tx_pcm_bypass = 1;

    // Configure TX channel
    dev->conf_chan.val = 0;
    dev->conf_chan.tx_chan_mod = 1;
    dev->conf.tx_right_first = 1;

    dev->timing.val = 0;

    // Allocate DMA descriptors
    const size_t buf_size = align_up(epd_row_width / 4, 4);  // Buf size must be word aligned
    i2s_state.buf_a = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    i2s_state.buf_b = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    i2s_state.dma_desc_a = heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
    i2s_state.dma_desc_b = heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);

    // and fill them
    fill_dma_desc(i2s_state.dma_desc_a, i2s_state.buf_a, epd_row_width / 4, buf_size);
    fill_dma_desc(i2s_state.dma_desc_b, i2s_state.buf_b, epd_row_width / 4, buf_size);

    // enable "done" interrupt
    SET_PERI_REG_BITS(I2S_INT_ENA_REG(1), I2S_OUT_DONE_INT_ENA_V, 1, I2S_OUT_DONE_INT_ENA_S);
    // register interrupt
    esp_intr_alloc(ETS_I2S1_INTR_SOURCE, 0, i2s_int_hdl, 0, &gI2S_intr_handle);

    // Reset FIFO/DMA
    dev->lc_conf.in_rst = 1;
    dev->lc_conf.out_rst = 1;
    dev->lc_conf.ahbm_rst = 1;
    dev->lc_conf.ahbm_fifo_rst = 1;
    dev->lc_conf.in_rst = 0;
    dev->lc_conf.out_rst = 0;
    dev->lc_conf.ahbm_rst = 0;
    dev->lc_conf.ahbm_fifo_rst = 0;
    dev->conf.tx_reset = 1;
    dev->conf.tx_fifo_reset = 1;
    dev->conf.rx_fifo_reset = 1;
    dev->conf.tx_reset = 0;
    dev->conf.tx_fifo_reset = 0;
    dev->conf.rx_fifo_reset = 0;

    // Start dma on front buffer
    dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
    dev->out_link.addr = ((uint32_t)(i2s_state.dma_desc_a));
    dev->out_link.start = 1;

    dev->int_clr.val = dev->int_raw.val;

    dev->int_ena.val = 0;
    dev->int_ena.out_done = 1;

    dev->conf.tx_start = 0;
}

void i2s_bus_deinit() {
    I2S1.int_ena.val = 0;
    I2S1.int_clr.val = I2S1.int_raw.val;

    if (gI2S_intr_handle != NULL) {
        esp_intr_disable(gI2S_intr_handle);
        esp_intr_free(gI2S_intr_handle);
        gI2S_intr_handle = NULL;
    }

    free(i2s_state.buf_a);
    free(i2s_state.buf_b);
    free((void*)i2s_state.dma_desc_a);
    free((void*)i2s_state.dma_desc_b);

    rtc_clk_apll_coeff_set(0, 0, 0, 8);
    rtc_clk_apll_enable(true);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    PERIPH_RCC_ATOMIC() {
        i2s_ll_enable_bus_clock(1, false);
    }
#else
    periph_module_disable(PERIPH_I2S1_MODULE);
#endif
}

/*
 * ESP32-S3 replacement for the I2S data bus, built on the LCD_CAM peripheral in i80 mode.
 *
 * It exposes the same interface as the ESP32 I2S implementation above, so the row-by-row render
 * method (render_i2s.c) and the shift-register board definitions work unchanged. One call to
 * i2s_start_line_output() sends one row of pixel data. Row latching, CKV pulses and all other
 * control lines are handled by the board and the RMT pulse code, exactly as on the ESP32.
 *
 * Wiring of the i80 signals:
 *   WR  -> CKH (pixel clock),   D/C -> STH (start pulse, active low),   D0..D7 -> data lines
 *
 * The start pulse comes from the D/C line: it idles low, goes high for a short dummy command
 * phase in front of every row and returns low for the data phase. This is the arrangement of
 * LILYGO's own ESP32-S3 driver. On a LILYGO T5-4.7 S3 the panel stayed blank with the plain
 * arrangement (D/C idle high, low only during the data phase) and works with this one.
 *
 * The data pins are attached to the bus in the same pairwise swapped order the ESP32 I2S code
 * uses: { d6, d7, d4, d5, d2, d3, d0, d1 }.
 */
#elif defined(CONFIG_IDF_TARGET_ESP32S3) && defined(CONFIG_EPD_ESP32S3_I80_ROW_OUTPUT)

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"

static const char* TAG = "epdiy-i80";

/// Pixel clock of the i80 bus, in Hz (one byte, i.e. four pixels, per clock).
#define I80_PCLK_HZ (10 * 1000 * 1000)

typedef struct {
    esp_lcd_i80_bus_handle_t bus;
    esp_lcd_panel_io_handle_t io;
    /// Front and back row buffer. One is filled while the other is being transmitted.
    uint8_t* buf[2];
    size_t row_bytes;
    /// Index of the buffer that is currently writable.
    int current;
} i80_state_t;

static i80_state_t i80;

/// Set by the transfer-done interrupt; cleared when a new row transmission is started.
static volatile bool output_done = true;

static bool IRAM_ATTR on_trans_done(
    esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx
) {
    (void)panel_io;
    (void)edata;
    (void)user_ctx;
    output_done = true;
    return false;
}

uint8_t* IRAM_ATTR i2s_get_current_buffer() {
    return i80.buf[i80.current];
}

bool IRAM_ATTR i2s_is_busy() {
    return !output_done;
}

void IRAM_ATTR i2s_switch_buffer() {
    // The render code only starts a new transmission after the previous one has finished, and
    // starts it on the buffer that is written to. After switching, the other buffer is therefore
    // never in use by the hardware.
    i80.current = !i80.current;
}

void IRAM_ATTR i2s_start_line_output() {
    output_done = false;
    // cmd 0: the (dummy) command phase generates the start pulse, see above
    esp_err_t err = esp_lcd_panel_io_tx_color(i80.io, 0, i80.buf[i80.current], i80.row_bytes);
    if (err != ESP_OK) {
        // never leave the renderer waiting for a transfer that did not start
        output_done = true;
        ESP_DRAM_LOGE(TAG, "row transmission failed");
    }
}

void i2s_gpio_attach(i2s_bus_config* cfg) {
    // The bus keeps ownership of its pins between refreshes. Unlike on the ESP32, the pins are
    // not released when the panel is powered off.
    (void)cfg;
}

void i2s_gpio_detach(i2s_bus_config* cfg) {
    (void)cfg;
}

void i2s_bus_init(i2s_bus_config* cfg, uint32_t epd_row_width) {
    // Bytes per row: 4 pixels per byte, plus the dummy pixels the board asks for as timing
    // headroom.
    i80.row_bytes = (epd_row_width / 4 + 3) & ~3U;
    i80.current = 0;
    output_done = true;

    for (int i = 0; i < 2; i++) {
        // 16 byte alignment for the vector-extension lookup routines, internal RAM for the DMA.
        i80.buf[i] = heap_caps_aligned_calloc(
            16, 1, i80.row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        );
        assert(i80.buf[i] != NULL);
    }

    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = cfg->start_pulse,
        .wr_gpio_num = cfg->clock,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = {
            cfg->data_6,
            cfg->data_7,
            cfg->data_4,
            cfg->data_5,
            cfg->data_2,
            cfg->data_3,
            cfg->data_0,
            cfg->data_1,
        },
        .bus_width = 8,
        .max_transfer_bytes = i80.row_bytes,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80.bus));

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = -1,
        .pclk_hz = I80_PCLK_HZ,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_trans_done,
        .user_ctx = NULL,
        .lcd_cmd_bits = 10,
        .lcd_param_bits = 0,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 1,
            .dc_dummy_level = 0,
            .dc_data_level = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80.bus, &io_config, &i80.io));

    ESP_LOGI(TAG, "i80 row output ready, %u bytes per row", (unsigned)i80.row_bytes);
}

void i2s_bus_deinit() {
    if (i80.io != NULL) {
        esp_lcd_panel_io_del(i80.io);
        i80.io = NULL;
    }
    if (i80.bus != NULL) {
        esp_lcd_del_i80_bus(i80.bus);
        i80.bus = NULL;
    }
    for (int i = 0; i < 2; i++) {
        heap_caps_free(i80.buf[i]);
        i80.buf[i] = NULL;
    }
}

#endif
