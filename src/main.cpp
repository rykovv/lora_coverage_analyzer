/**
 * Universidad Jaume I. Departamento de Ingenieria y Ciencia de Computadores.
 * High-Performance Computing and Architectures (HPCA) Research Group.
 *
 * File: main.cpp
 * Purpose: Firmware which implements main behaviour and functionality of the device.
 *
 * @author Vladislav Rykov
 * @version 1.0 11/11/19
**/
#include <Arduino.h>
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <WiFi.h>

#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

#include "ui.hpp"

#include "TinyGPS++.h"

#define GPS_RX_PIN              21
#define GPS_TX_PIN              22
#define GPS_UART_READ_TIMEOUT   10000 // must be less than TX period in secs
#define GPS_TIMEOUT_CNT_MAX     5
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
#define TX_INTERVAL             30

HardwareSerial gps_serial(2);
TinyGPSPlus gps;

// TTN API EUI
static const u1_t PROGMEM APPEUI[8]={ 0x71, 0x4A, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DEV EUI taken from the correctly extended device WiFi MAC address
void get_eui64(uint8_t *eui);
void os_getDevEui (u1_t* buf) { get_eui64((uint8_t *)buf); }

// TTN APP KEY copied as is from the TTN application console
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// Jobs definition
static osjob_t sendjob;
void do_send(osjob_t* j);

static osjob_t gpsjob;
void gps_update(osjob_t* j);

// Pin mapping necessary for the LMiC
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

// User interface declaration
UI ui;

// GPS state/data register
typedef struct {
    float lat = -1.0;
    float lng = -1.0;
    uint8_t updated = 0;
    uint8_t timeout_cnt = 0;
} gps_data_t;
gps_data_t gps_reg;

// device join flag (1 when joined to ttn)
uint8_t dev_joined = 0;

// function to restart completely the mcu hardware
void hard_restart();

// device state
typedef enum {
    DEV_STATE_PERIODIC_SEND = 0,
    DEV_STATE_GPS_DATA_SEND,
} dev_state_t;
dev_state_t dev_state = DEV_STATE_PERIODIC_SEND;

/************************ SETUP ************************/

void setup() {
    Serial.begin(9600);
    ESP_LOGI(TAG, "Starting...");

    gps_serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    // enlarge max clock error for more successful joining
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

/************************ LOOP ************************/

void loop() {
    os_runloop_once();
}

// LMiC onEvent redefinition
void onEvent (ev_t ev) {
    if (CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG) {
        Serial.print(os_getTime());
        Serial.print(F(": "));
    }
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            ESP_LOGD(TAG, "EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            ESP_LOGD(TAG, "EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            ESP_LOGD(TAG, "EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            ESP_LOGD(TAG, "EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            ESP_LOGD(TAG, "EV_JOINING");
            // Schedule display update
            ui.set_state(UI_STATE_JOINING);
            break;
        case EV_JOINED:
            ESP_LOGD(TAG, "EV_JOINED");
            if (CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG) {
                u4_t netid = 0;
                devaddr_t devaddr = 0;
                u1_t nwkKey[16];
                u1_t artKey[16];
                LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                Serial.print(F("netid: "));
                Serial.println(netid, DEC);
                Serial.print(F("devaddr: "));
                Serial.println(devaddr, HEX);
                Serial.print(F("artKey: "));
                for (size_t i=0; i<sizeof(artKey); ++i) {
                    Serial.print(artKey[i], HEX);
                }
                Serial.println("");
                Serial.print("nwkKey: ");
                for (size_t i=0; i<sizeof(nwkKey); ++i) {
                    Serial.print(nwkKey[i], HEX);
                }
                Serial.println("");
            }
            dev_joined = 1;

            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	        // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);

            // Schedule display update
            ui.set_state(UI_STATE_JOINED);
            break;
        case EV_JOIN_FAILED:
            ESP_LOGD(TAG, "EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            ESP_LOGD(TAG, "EV_REJOIN_FAILED");
            break;
        case EV_TXCOMPLETE:
            ESP_LOGD(TAG, "EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK) {
                ESP_LOGD(TAG, "Received ACK");
                ui.set_state(UI_STATE_ACK_RECEIVED);

                if (dev_state == DEV_STATE_GPS_DATA_SEND) {
                    ESP_LOGV(TAG, "GPS_DATA_CONFIRMED");
                    ui.set_gps_status(UI_GPS_STATUS_CONFIRMED);

                    dev_state = DEV_STATE_PERIODIC_SEND;
                    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
                } else if (dev_state == DEV_STATE_PERIODIC_SEND) {
                    ESP_LOGV(TAG, "GPS not sent -> update");
                    // once periodic ack received -> get gps data and schedule send
                    //   or if the gps has not been initialized, schedule the next try
                    //   to at least update the display
                    gps_update(&gpsjob);
                    if (gps_reg.updated) {
                        ESP_LOGV(TAG, "GPS data updated -> schedule send immediately");
                        dev_state = DEV_STATE_GPS_DATA_SEND;
                        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
                    } else {
                        dev_state = DEV_STATE_PERIODIC_SEND;
                        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
                    }
                }
            } else {
                ESP_LOGV(TAG, "schedule next periodic send");
                os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

                ui.set_state(UI_STATE_TXCOMPLETED);
            }

            if (CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG) {
                if (LMIC.dataLen) {
                    Serial.print(F("Received "));
                    Serial.print(LMIC.dataLen);
                    Serial.println(F(" bytes of payload"));
                }
            }

            ui.set_signal_values(LMIC.rssi, LMIC.snr);
            break;
        case EV_LOST_TSYNC:
            ESP_LOGD(TAG, "EV_LOST_TSYNC");
            break;
        case EV_RESET:
            ESP_LOGD(TAG, "EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            ESP_LOGD(TAG, "EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            ESP_LOGD(TAG, "EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            ESP_LOGD(TAG, "EV_LINK_ALIVE");
            break;
        case EV_TXSTART:
            ESP_LOGD(TAG, "EV_TXSTART");
            if (dev_joined) {
                ui.set_state(UI_STATE_TXSTART);
            }
            break;
        case EV_JOIN_TXCOMPLETE:
            ESP_LOGD(TAG, "EV_JOIN_TXCOMPLETE");
            break;
        default:
            ESP_LOGD(TAG, "Unknown event: %u", (unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        ESP_LOGW(TAG, "OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        uint8_t mydata[64] = "-1#-1#-1#-1";
        if (gps_reg.updated && dev_state == DEV_STATE_GPS_DATA_SEND) {
            snprintf_P((char *)mydata, sizeof(mydata), (PGM_P)F("%d#%d#%.6f#%.6f"),
                                                                        LMIC.rssi,
                                                                        LMIC.snr,
                                                                        gps_reg.lat,
                                                                        gps_reg.lng);

            ESP_LOGD(TAG, "GPS DATA : %s", (char *)mydata);
            ui.set_gps_status(UI_GPS_STATUS_SENT);
            gps_reg.updated = 0;
        }
        //              port                                ack required
        LMIC_setTxData2(10, mydata, strlen((char *)mydata), 1);
        ESP_LOGD(TAG, "Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void gps_update(osjob_t* j) {
    ui.set_gps_status(UI_GPS_STATUS_TAKING_DATA);

    uint32_t gps_read_start = millis();
    uint8_t timeout_ok = 1;

    while (!gps.location.isUpdated() && (timeout_ok = (millis() - gps_read_start) < GPS_UART_READ_TIMEOUT)) {
        if (gps_serial.available()) {
            gps.encode(gps_serial.read());
        }
    }

    if (timeout_ok) {
        gps_reg.lat = gps.location.lat();
        gps_reg.lng = gps.location.lng();
        gps_reg.updated = 1;

        gps_reg.timeout_cnt = 0;

        ui.set_gps_status(UI_GPS_STATUS_DATA_TAKEN);
    } else {
        gps_reg.timeout_cnt++;
        if (gps_reg.timeout_cnt == GPS_TIMEOUT_CNT_MAX) {
            hard_restart();
        }

        gps_reg.updated = 0;

        ui.set_gps_status(UI_GPS_STATUS_TIMEOUT);
    }
}

void get_eui64(uint8_t *eui) {
    esp_efuse_mac_get_default(eui);

    memmove(&eui[5], &eui[3], 3);
    eui[3] = 0xFF;
    eui[4] = 0xFE;
}

void hard_restart() {
    esp_task_wdt_init(1, true);
    esp_task_wdt_add(NULL);
    while(true);
}
