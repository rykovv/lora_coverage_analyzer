#ifndef __UI__H__
#define __UI__H__

#include <stdint.h>

typedef enum {
    UI_STATE_JOINING = 0,
    UI_STATE_JOINED,
    UI_STATE_TXCOMPLETED,
    UI_STATE_ACK_RECEIVED
} ui_state_t;

typedef enum {
    UI_LORA_BAR_NO_SIGNAL = 0,
    UI_LORA_BAR_POOR_SIGNAL,
    UI_LORA_BAR_GOOD_SIGNAL,
    UI_LORA_BAR_PERFECT_SIGNAL
} ui_lora_bar_t;

typedef enum {
    UI_GPS_STATUS_NO_SIGNAL = 0,
    UI_GPS_STATUS_TAKING_DATA,
    UI_GPS_STATUS_SENT,
    UI_GPS_STATUS_CONFIRMED
} ui_gps_status_t;

void ui_init(void);

void ui_set_state(ui_state_t ui_state);

void ui_set_signal_values(int16_t rssi, int16_t snr);

void ui_set_gps_status(ui_gps_status_t ui_gps_status);

#endif /* __UI__H__ */