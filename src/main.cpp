/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

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

#define GPS_RX_PIN  21
#define GPS_TX_PIN  22
#define GPS_UART_READ_TIMEOUT   10000 // must be less than TX period in secs
#define GPS_TIMEOUT_CNT_MAX     5

HardwareSerial gps_serial(2);
TinyGPSPlus gps;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x71, 0x4A, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

void get_eui64(uint8_t *eui);
void os_getDevEui (u1_t* buf) { get_eui64((uint8_t *)buf); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x79, 0xE9, 0x8F, 0x21, 0xBC, 0x2C, 0x8F, 0xC2, 0xAB, 0xF3, 0x6E, 0x65, 0x43, 0x5D, 0x53, 0x9D };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;
void do_send(osjob_t* j);

static osjob_t gpsjob;
void gps_update(osjob_t* j);

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

UI ui;

typedef struct {
    float lat = -1.0;
    float lng = -1.0;
    uint8_t updated = 0;
    uint8_t sent = 0;
} gps_data_t;
gps_data_t gps_reg;

uint8_t joined = 0;

uint8_t gps_timeout_cnt = 0;

void hard_restart();

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            // Schedule display update
            ui.set_state(UI_STATE_JOINING);
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
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
            joined = 1;
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);

            // Schedule display update
            ui.set_state(UI_STATE_JOINED);
            // Schedule GPS update
            // os_setTimedCallback(&gpsjob, os_getTime()+sec2osticks(1), gps_update);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("Received ack"));
                ui.set_state(UI_STATE_ACK_RECEIVED);

                if (gps_reg.sent) {
                    gps_reg.sent = 0;
                    ui.set_gps_status(UI_GPS_STATUS_CONFIRMED);
                    
                    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
                } else {
                    // once periodic ack received -> get gps data and schedule send
                    gps_update(&gpsjob);
                    if (gps_reg.updated) {
                        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
                    }
                }
            } else {
                ui.set_state(UI_STATE_TXCOMPLETED);
                
                if (gps_reg.updated) {
                    gps_reg.sent = 1;
                    gps_reg.updated = 0;

                    ui.set_gps_status(UI_GPS_STATUS_SENT);
                }
            }

            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }

            ui.set_signal_values(LMIC.rssi, LMIC.snr);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            if (joined) {
                ui.set_state(UI_STATE_TXSTART);
            }
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        uint8_t mydata[64] = "-1#-1#-1#-1";
        if (gps_reg.updated) {
            snprintf_P((char *)mydata, sizeof(mydata), (PGM_P)F("%d#%d#%.6f#%.6f"), 
                                                                        LMIC.rssi, 
                                                                        LMIC.snr, 
                                                                        gps_reg.lat, 
                                                                        gps_reg.lng);

            Serial.println((char *)mydata);
        }
        //              port                                ack required
        LMIC_setTxData2(10, mydata, strlen((char *)mydata), 1);
        Serial.println(F("Packet queued"));
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

        gps_timeout_cnt = 0;

        ui.set_gps_status(UI_GPS_STATUS_DATA_TAKEN);
    } else {
        gps_timeout_cnt++;
        if (gps_timeout_cnt == GPS_TIMEOUT_CNT_MAX) {
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

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));

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

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}