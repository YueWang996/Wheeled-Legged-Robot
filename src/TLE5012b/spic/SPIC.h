//
// Created by Justin on 18/11/2021.
//

#ifndef TLE5012B_TEST_SPIC_H
#define TLE5012B_TEST_SPIC_H

#include <SPI.h>
#include <Arduino.h>
#include "../tle5012b_util.h"

#define SPEED 1000000UL

class SPIC {
public:
    SPIC(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, SPIClass *spiClass);

    Error_t init();
    Error_t deinit();
    Error_t sendReceiveSpi(uint16_t* sent_data, uint16_t size_of_sent_data, uint16_t* received_data, uint16_t size_of_received_data);
    Error_t triggerUpdate();

protected:
    SPIClass  * vspi = nullptr;
    int8_t sck;
    int8_t mosi;
    int8_t miso;
    int8_t cs;
};


#endif //TLE5012B_TEST_SPIC_H
