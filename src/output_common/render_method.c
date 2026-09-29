#include "render_method.h"
#include "sdkconfig.h"

#ifdef CONFIG_IDF_TARGET_ESP32
const enum EpdRenderMethod EPD_CURRENT_RENDER_METHOD = RENDER_METHOD_I2S;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#ifdef CONFIG_EPD_ESP32S3_I80_ROW_OUTPUT
const enum EpdRenderMethod EPD_CURRENT_RENDER_METHOD = RENDER_METHOD_I2S;
#else
const enum EpdRenderMethod EPD_CURRENT_RENDER_METHOD = RENDER_METHOD_LCD;
#endif
#else
#error "unknown chip, cannot choose render method!"
#endif