#ifndef __UI__HPP__
#define __UI__HPP__

#include <stdint.h>
#include <U8x8lib.h>

// OLED display I2C pin definitions
#define SSD1306_SCL_PIN     15
#define SSD1306_SDA_PIN     4
#define SSD1306_RST_PIN     16
typedef enum {
    UI_STATE_JOINING = 0,
    UI_STATE_JOINED,
    UI_STATE_TXCOMPLETED,
    UI_STATE_ACK_RECEIVED
} ui_state_t;

typedef enum {
    UI_GPS_STATUS_NO_SIGNAL = 0,
    UI_GPS_STATUS_TAKING_DATA,
    UI_GPS_STATUS_SENT,
    UI_GPS_STATUS_CONFIRMED
} ui_gps_status_t;

class UI {
    public :
        UI(void);

        void init(void);

        void set_state(ui_state_t ui_state);

        void set_signal_values(int16_t rssi, int16_t snr);

        void set_gps_status(ui_gps_status_t ui_gps_status);

    private :
        void _set_ui_display(void);

};

#endif /* __UI__HPP__ */