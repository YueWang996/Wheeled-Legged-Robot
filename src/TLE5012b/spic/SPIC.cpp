//
// Created by Justin on 18/11/2021.
//

#include "SPIC.h"

SPIC::SPIC(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, SPIClass *spiClass) {
    //this->vspi = new SPIClass(VSPI);
    this->vspi = spiClass;
    this->sck = sck;
    this->miso = miso;
    this->mosi = mosi;
    this->cs = cs;
}

Error_t SPIC::init() {
    //this->vspi->begin(this->sck, this->miso, this->mosi);
    pinMode(this->cs, OUTPUT); //VSPI SS
    digitalWrite(this->cs, HIGH);
    return OKK;
}

Error_t SPIC::sendReceiveSpi(uint16_t *sent_data, uint16_t size_of_sent_data, uint16_t *received_data,
                          uint16_t size_of_received_data) {
    uint32_t data_index = 0;
    //send via TX
    digitalWrite(this->cs, LOW);
    this->vspi->beginTransaction(SPISettings(SPEED,MSBFIRST,SPI_MODE1));

    for(data_index = 0; data_index < size_of_sent_data; data_index++)
    {
        received_data[0] = this->vspi->transfer16(sent_data[data_index]);
    }

    // receive via RX
    delayMicroseconds(5);

    for(data_index = 0; data_index < size_of_received_data; data_index++)
    {
        received_data[data_index] = this->vspi->transfer16(0xFFFF);
    }
    this->vspi->endTransaction();
    digitalWrite(this->cs, HIGH);

    return OKK;
}

Error_t SPIC::deinit() {
    this->vspi->endTransaction();
    this->vspi->end();
    return OKK;
}

Error_t SPIC::triggerUpdate() {
    digitalWrite(this->sck, LOW);
    digitalWrite(this->mosi, HIGH);
    digitalWrite(this->cs, LOW);
    //grace period for register snapshot
    delayMicroseconds(5);
    digitalWrite(this->cs, HIGH);
    return OKK;
}


