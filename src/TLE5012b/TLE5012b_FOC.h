//
// Created by Justin on 19/11/2021.
//

#ifndef TLE5012B_TEST_TLE5012B_FOC_H
#define TLE5012B_TEST_TLE5012B_FOC_H

#include "../SimpleFOC/SimpleFOC.h"
#include "TLE5012b.h"

class TLE5012b_FOC: public Sensor{
public:
    TLE5012b_FOC();
    void init(uint8_t cs, SPIClass *spiClass);
    float getSensorAngle() override;

private:
    TLE5012b * tle5012B;
    errorTypes checkError;
    double angleValue{};
    double angleValueConverted{};
};


#endif //TLE5012B_TEST_TLE5012B_FOC_H
