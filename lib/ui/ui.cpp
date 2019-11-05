#include "ui.hpp"

typedef enum {
    UI_LORA_BAR_NO_SIGNAL = 0,
    UI_LORA_BAR_POOR_SIGNAL,
    UI_LORA_BAR_GOOD_SIGNAL,
    UI_LORA_BAR_PERFECT_SIGNAL
} ui_lora_bar_t;

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(SSD1306_SCL_PIN, SSD1306_SDA_PIN, SSD1306_RST_PIN);

static void _ui_set_lora_bar(ui_lora_bar_t ui_lora_bar);
static uint8_t _takes_chars(int16_t num);
static inline uint8_t _small_fmt_needed(int16_t rssi, int16_t snr);

UI::UI(void) {
    u8x8.begin();
    u8x8.setFont(u8x8_font_saikyosansbold8_u);

    _set_ui_display();
}

void UI::set_state(ui_state_t ui_state) {
    char msg[32] = "";

    u8x8.clearLine(0);

    switch (ui_state) {
        case UI_STATE_JOINING:
            strncpy_P(msg, (PGM_P)F("JOINING"), 32);
            break;
        case UI_STATE_JOINED:
            strncpy_P(msg, (PGM_P)F("JOINED"), 32);
            break;
        case UI_STATE_TXSTART:
            strncpy_P(msg, (PGM_P)F("DATAUP START"), 32);
            break; 
        case UI_STATE_TXCOMPLETED:
            strncpy_P(msg, (PGM_P)F("DATAUP COMPLETED"), 32);
            break;
        case UI_STATE_ACK_RECEIVED:
            strncpy_P(msg, (PGM_P)F("ACK RECEIVED"), 32);
            break;
    }

    u8x8.drawString(0, 0, msg);
}

void UI::set_signal_values(int16_t rssi, int16_t snr) {
    char vals[16];
    memset(vals, ' ', 11);
    vals[11] = '\0';
    u8x8.draw1x2String(2, 4, vals);
    // TODO: adjust character shift
    if (_small_fmt_needed(rssi, snr)) {
        sprintf_P(vals, (PGM_P)F("%d  .  %d"), rssi, snr);
    } else {
        snprintf_P(vals, 11, (PGM_P)F("%d . %d"), rssi, snr);
    }
    u8x8.draw1x2String(2, 4, vals);

    if (rssi > 30) {
        _ui_set_lora_bar(UI_LORA_BAR_PERFECT_SIGNAL);
    } else if (rssi > 20) {
        _ui_set_lora_bar(UI_LORA_BAR_GOOD_SIGNAL);
    } else {
        _ui_set_lora_bar(UI_LORA_BAR_POOR_SIGNAL);
    }
}


void UI::set_gps_status(ui_gps_status_t ui_gps_status) {
    char msg[32] = "";

    u8x8.clearLine(u8x8.getRows()-1);

    switch (ui_gps_status) {
    case UI_GPS_STATUS_NO_SIGNAL :
        strncpy_P(msg, (PGM_P)F("NO GPS DATA"), 32);
        break;
    case UI_GPS_STATUS_TAKING_DATA :
        strncpy_P(msg, (PGM_P)F("TAKING GPS DATA"), 32);
        break;
    case UI_GPS_STATUS_DATA_TAKEN :
        strncpy_P(msg, (PGM_P)F("GPS DATA TAKEN"), 32);
        break;
    case UI_GPS_STATUS_SENT :
        strncpy_P(msg, (PGM_P)F("GPS DATA SENT"), 32);
        break;
    case UI_GPS_STATUS_CONFIRMED :
        strncpy_P(msg, (PGM_P)F("GPS ST CONFIRMED"), 32);
        break;
    case UI_GPS_STATUS_TIMEOUT :
        strncpy_P(msg, (PGM_P)F("GPS READ TIMEOUT"), 32);
        break;
    }
    u8x8.drawString(0, u8x8.getRows()-1, msg);
}

void UI::_set_ui_display(void) {
    u8x8.clearDisplay();

    set_state(UI_STATE_JOINING);

    _ui_set_lora_bar(UI_LORA_BAR_NO_SIGNAL);

    u8x8.drawString(1, 2, "RSSI . SNR");
    u8x8.draw1x2String(2, 4, "--  .  --");

    set_gps_status(UI_GPS_STATUS_NO_SIGNAL);
}

static uint8_t _takes_chars(int16_t num) {
    uint res = num < 0 ? 1 : 0;

    while (num) {
        res++;
        num /= 10;
    }

    return res;
}

static inline uint8_t _small_fmt_needed(int16_t rssi, int16_t snr) {
    return (_takes_chars(rssi) + _takes_chars(snr) < 5);
}

static void _ui_set_lora_bar(ui_lora_bar_t ui_lora_bar) {
    uint8_t filled_tile[16] = {0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE};
    uint8_t empty_tile[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    switch (ui_lora_bar) {
        case UI_LORA_BAR_NO_SIGNAL :
            u8x8.drawTile(12, 2, 2, empty_tile);
            u8x8.drawTile(14, 2, 2, empty_tile);

            u8x8.drawTile(13, 3, 2, empty_tile);
            u8x8.drawTile(15, 3, 1, empty_tile);

            u8x8.drawTile(14, 4, 2, empty_tile);

            u8x8.drawTile(15, 5, 1, filled_tile);
        break;
        case UI_LORA_BAR_POOR_SIGNAL :
            u8x8.drawTile(12, 2, 2, empty_tile);
            u8x8.drawTile(14, 2, 2, empty_tile);

            u8x8.drawTile(13, 3, 2, empty_tile);
            u8x8.drawTile(15, 3, 1, empty_tile);

            u8x8.drawTile(14, 4, 2, filled_tile);

            u8x8.drawTile(15, 5, 1, filled_tile);
        break;
        case UI_LORA_BAR_GOOD_SIGNAL :
            u8x8.drawTile(12, 2, 2, empty_tile);
            u8x8.drawTile(14, 2, 2, empty_tile);

            u8x8.drawTile(13, 3, 2, filled_tile);
            u8x8.drawTile(15, 3, 1, filled_tile);

            u8x8.drawTile(14, 4, 2, filled_tile);

            u8x8.drawTile(15, 5, 1, filled_tile);
        break;
        case UI_LORA_BAR_PERFECT_SIGNAL :
            u8x8.drawTile(12, 2, 2, filled_tile);
            u8x8.drawTile(14, 2, 2, filled_tile);

            u8x8.drawTile(13, 3, 2, filled_tile);
            u8x8.drawTile(15, 3, 1, filled_tile);

            u8x8.drawTile(14, 4, 2, filled_tile);

            u8x8.drawTile(15, 5, 1, filled_tile);
        break;
    }
}