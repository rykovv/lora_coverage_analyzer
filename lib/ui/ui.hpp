/**
 * Universidad Jaume I. Departamento de Ingenieria y Ciencia de Computadores.
 * High-Performance Computing and Architectures (HPCA) Research Group.
 * 
 * File: ui.hpp
 * Purpose: User interface library header
 * 
 * @author Vladislav Rykov
 * @version 1.0 11/11/19 
**/
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
    UI_STATE_ACK_RECEIVED,
    UI_STATE_TXSTART,
    /*
    UI_STATE_SCAN_TIMEOUT,
    UI_STATE_JOIN_FAILED,
    UI_STATE_REJOIN_FAILED,
    UI_STATE_RXCOMPLETE,
    UI_STATE_RESET,
    UI_STATE_LINK_DEAD,
    UI_STATE_LINK_ALIVE,
    UI_STATE_SCAN_FOUND,
    UI_STATE_TXCANCELED,
    UI_STATE_RXSTART,
    */
} ui_state_t;

typedef enum {
    UI_GPS_STATUS_NO_SIGNAL = 0,
    UI_GPS_STATUS_TAKING_DATA,
    UI_GPS_STATUS_DATA_TAKEN,
    UI_GPS_STATUS_SENT,
    UI_GPS_STATUS_CONFIRMED,
    UI_GPS_STATUS_TIMEOUT,
} ui_gps_status_t;

class UI {
    public :
        UI(void);

        /**
         * @desc sets state on user interface OLED display
         * @param ui_state_t ui_state - [IN] - state
        */
        void set_state(ui_state_t ui_state);

        /**
         * @desc sets network signal values on user interface OLED display
         * @param int16_t rssi - [IN] - rssi value
         * @param int16_t snr -  [IN] - snr (signal to noise ratio) value
        */
        void set_signal_values(int16_t rssi, int16_t snr);

        /**
         * @desc sets GPS status on user interface OLED display
         * @param ui_gps_status_t ui_gps_status - [IN] - GPS status
        */
        void set_gps_status(ui_gps_status_t ui_gps_status);

    private :
        void _set_ui_display(void);

};

#endif /* __UI__HPP__ */