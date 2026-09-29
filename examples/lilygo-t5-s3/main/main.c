// First-light test for the LILYGO T5-4.7" S3 (V2.3/V2.4, shift-register board)
// using the epdiy row-output engine on the ESP32-S3.
#include <stdio.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "epdiy.h"

static const char* TAG = "t5s3";
static EpdiyHighlevelState hl;

static void check(enum EpdDrawError e) {
    if (e != EPD_DRAW_SUCCESS)
        ESP_LOGE(TAG, "draw error 0x%X", e);
}

static void draw_pattern(uint8_t* fb) {
    int w = epd_rotated_display_width(), h = epd_rotated_display_height();
    // fb is 4bpp, colour = high nibble (0x00 black .. 0xF0 white)
    epd_fill_rect((EpdRect){ 0, 0, w, h }, 0xFF, fb);
    epd_draw_rect((EpdRect){ 0, 0, w, h }, 0x00, fb);
    epd_draw_rect((EpdRect){ 6, 6, w - 12, h - 12 }, 0x00, fb);
    // corner markers: 1,2,3,4 squares so orientation is obvious
    int cx[4] = { 20, w - 20 - 40, 20, w - 20 - 40 };
    int cy[4] = { 20, 20, h - 20 - 40, h - 20 - 40 };
    for (int c = 0; c < 4; c++)
        for (int k = 0; k <= c; k++)
            epd_fill_rect((EpdRect){ cx[c] + k * 10, cy[c], 8, 40 }, 0x00, fb);
    // 16 step grey ramp
    int rw = (w - 120) / 16;
    for (int i = 0; i < 16; i++)
        epd_fill_rect((EpdRect){ 60 + i * rw, h / 2 - 60, rw, 120 }, (uint8_t)(i * 16), fb);
    // 1px lines
    for (int x = 60; x < w - 60; x += 4)
        epd_draw_vline(x, h / 2 + 80, 60, 0x00, fb);
    for (int y = h / 2 + 150; y < h - 60; y += 4)
        epd_draw_hline(60, y, w - 120, 0x00, fb);
    epd_fill_circle(w / 2, 110, 50, 0x00, fb);
}

void app_main(void) {
    ESP_LOGI(TAG, "epd_init");
    epd_init(&epd_board_lilygo_t5_47_s3, &ED047TC1, EPD_LUT_64K);
    hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
    epd_set_rotation(EPD_ROT_LANDSCAPE);
    ESP_LOGI(TAG, "display %d x %d", epd_rotated_display_width(), epd_rotated_display_height());
    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);

    uint8_t* fb = epd_hl_get_framebuffer(&hl);
    int temperature = 25;

    ESP_LOGI(TAG, "clear");
    int64_t t0 = esp_timer_get_time();
    epd_poweron();
    epd_clear();
    epd_poweroff();
    ESP_LOGI(TAG, "clear took %lld ms", (esp_timer_get_time() - t0) / 1000);

    draw_pattern(fb);
    ESP_LOGI(TAG, "draw GC16");
    t0 = esp_timer_get_time();
    epd_poweron();
    check(epd_hl_update_screen(&hl, MODE_GC16, temperature));
    epd_poweroff();
    ESP_LOGI(TAG, "GC16 update took %lld ms", (esp_timer_get_time() - t0) / 1000);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // partial update test: black bar, fast DU mode
    epd_fill_rect((EpdRect){ 300, 20, 360, 40 }, 0x00, fb);
    t0 = esp_timer_get_time();
    epd_poweron();
    check(epd_hl_update_area(&hl, MODE_DU, temperature, (EpdRect){ 300, 20, 360, 40 }));
    epd_poweroff();
    ESP_LOGI(TAG, "DU partial took %lld ms", (esp_timer_get_time() - t0) / 1000);

    ESP_LOGI(TAG, "done");
    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
}
