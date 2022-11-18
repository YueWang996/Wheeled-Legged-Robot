//
// Created by Justin on 19/11/2021.
//

#include "TLE5012b_FOC.h"

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

TLE5012b_FOC::TLE5012b_FOC() {
    tle5012B = new TLE5012b();
    checkError = NO_ERROR;
}

void TLE5012b_FOC::init(uint8_t cs, SPIClass *spiClass) {
    tle5012B->begin(cs, spiClass);
}

float TLE5012b_FOC::getSensorAngle() {
    tle5012B->getAngleValue(angleValue);
    angleValueConverted = map_double(angleValue, -180.0, 180.0, 0.0, 6.28);
    return angleValueConverted;
}