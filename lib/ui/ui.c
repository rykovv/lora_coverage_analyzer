#include "ui.h"
#include <U8x8lib.h>

#define SSD1306_SCL_PIN     15
#define SSD1306_SDA_PIN     4
#define SSD1306_RST_PIN     16

#include "img/img.h"

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(SSD1306_SCL_PIN, SSD1306_SDA_PIN, SSD1306_RST_PIN);

static void _set_ui_display(void);
static void _ui_set_lora_bar(ui_lora_bar_t ui_lora_bar);

void ui_init(void) {
    u8x8.begin();
    u8x8.setFont(u8x8_font_saikyosansbold8_u);

    _set_ui_display();
}

void ui_set_state(ui_state_t ui_state) {
    u8x8.clearDisplay(); // clear state line
    switch (ui_state) {
    case UI_STATE_JOINING:
        u8x8.drawString(0, 10, "JOINING");
        //Heltec.display->drawXbm(72, 14, LORA_RSSI_WIDTH, LORA_RSSI_HEIGHT, no_signal_adj);
        break;
    case UI_STATE_JOINED:
        u8x8.drawString(0, 10, "JOINED");
        //Heltec.display->drawXbm(72, 14, LORA_RSSI_WIDTH, LORA_RSSI_HEIGHT, no_signal_adj);
        break;
    case UI_STATE_TXCOMPLETED:
        /* code */
        break;
    case UI_STATE_ACK_RECEIVED:
        /* code */
        break;
    default:
        break;
    }
}

void ui_set_signal_values(int16_t rssi, int16_t snr) {

}


void ui_set_gps_status(ui_gps_status_t ui_gps_status) {

}

static void _set_ui_display(void) {

}

static void _ui_set_lora_bar(ui_lora_bar_t ui_lora_bar) {
    
}