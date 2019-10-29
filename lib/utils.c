#include "utils.h"

void get_eui64(uint8_t *eui) {
    esp_efuse_mac_get_default(eui);
    
    memmove(&eui[5], &eui[3], 3);
    eui[3] = 0xFF;
    eui[4] = 0xFE;
}