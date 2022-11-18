//
// Created by Justin on 18/11/2021.
//

#ifndef TLE5012B_TEST_TLE5012B_H
#define TLE5012B_TEST_TLE5012B_H

#include "tle5012b_util.h"
#include "spic/SPIC.h"
#include "tle5012b_reg.h"

//#define PIN_SPI_SS      5
#define PIN_SPI_MISO    19
#define PIN_SPI_MOSI    23
#define PIN_SPI_SCK     18

class TLE5012b{
public:
    /*!
    * Offset for the slave number register to identify the
    * right selected slave. Max 4 slaves with separated CSQ
    * lines are possible. If more than one sensor is used on the SPI
    * interface, than the SNR register must we written with the correct slave number
    */
    enum slaveNum {
        TLE5012B_S0 = 0x0000,    //!< \brief TLE5012B_S0 default setting for only one sensor on the SPI
        TLE5012B_S1 = 0x2000,    //!< \brief TLE5012B_S1 second sensor needs also a second CSQ
        TLE5012B_S2 = 0x4000,    //!< \brief TLE5012B_S2 third sensor and ditto
        TLE5012B_S3 = 0x6000     //!< \brief TLE5012B_S3 fourth sensor and ditto
    };

    struct safetyWord {  //!< \brief Safety word bit setting
        bool STAT_RES;           //!< \brief bits 15:15 Indication of chip reset or watchdog overflow
        bool STAT_ERR;           //!< \brief bits 14:14 System error
        bool STAT_ACC;           //!< \brief bits 13:13 Interface access error
        bool STAT_ANG;           //!< \brief bits 12:12 Invalid angle value
        uint8_t RESP;            //!< \brief bits 11:8 Sensor number response indicator
        uint8_t CRC;             //!< \brief bits 7:0 Status ADC Test

        /*!
        * \brief Returns the safety word slave number to identify the sensor
        * @return slaveNum setting in safety word
        */
        slaveNum responseSlave(){
            return (RESP == 0x7 ? TLE5012B_S3
                                : (RESP == 0xB ? TLE5012B_S2
                                               : (RESP == 0xD ? TLE5012B_S1
                                                              : TLE5012B_S0)));
        }

        /*!
        * \brief Function separates safety word bits
        * @param [in,out] reg actual safety or last fetched as default
        * @return safety word
        */
        uint16_t fetch_Safety(uint16_t reg)
        {
            CRC      = (reg & 0x7F);
            RESP     = (reg & 0xF00) >> 8;
            STAT_ANG = (reg & 0x1000) >> 12;
            STAT_ACC = (reg & 0x2000) >> 13;
            STAT_ERR = (reg & 0x4000) >> 14;
            STAT_RES = (reg & 0x8000) >> 15;
            return (reg);
        }
    } safetyWord_t;

    //!< \brief constructor for the Sensor
    TLE5012b();

    //!< \brief destructor stops the Sensor
    ~TLE5012b(){};

    //!< \brief default begin with standard pin setting
    errorTypes begin(int8_t pinCS, SPIClass *spiClass);

    /*! \brief begin method with default assignments for the SPI bus
     * and the slave setting. Only the csPin is needed.
     *
     * \param csPin    pin number of the CS pin
     * \param slave    slave offset setting for the SNR register, default is TLE5012B_S0
     */
    //errorTypes begin(uint8_t csPin, slaveNum slave=TLE5012B_S0);

    /*!
		* Function enables Sensor by switch on EN pin which is only possible
		* on Sensor2go shields, but also sets chipselect high.
		* So it is called always.
		*/
    void enableSensor(){};

    /*!
    * Functions disables Sensor by switch off EN pin (only possible on Sensor2go shield)
    */
    void disableSensor(){};

    errorTypes readBlockCRC();
    errorTypes readFromSensor(uint16_t command, uint16_t &data, updTypes upd=UPD_low, safetyTypes safe=SAFE_high);
    errorTypes readMoreRegisters(uint16_t command, uint16_t data[], updTypes upd, safetyTypes safe);
    errorTypes readStatus(uint16_t &data, updTypes upd=UPD_low, safetyTypes safe=SAFE_high);
    errorTypes readActivationStatus(uint16_t &data, updTypes upd=UPD_low, safetyTypes safe=SAFE_high);

    /*!
		* The next functions are used primarily for storing the parameters and
		* control of how the sensor works. The values stored in them are used to calculate
		* the CRC, and their values are stored in the private component of the class, _registers.
		* @param [out] data where the data received from the _registers will be stored
		* @return CRC error type
		*/
    errorTypes readActiveStatus(uint16_t &data);    //!< \brief read register offset 0x01
    errorTypes readIntMode1(uint16_t &data);        //!< \brief read register offset 0x06
    errorTypes readSIL(uint16_t &data);             //!< \brief read register offset 0x07
    errorTypes readIntMode2(uint16_t &data);        //!< \brief read register offset 0x08
    errorTypes readIntMode3(uint16_t &data);        //!< \brief read register offset 0x09
    errorTypes readOffsetX(uint16_t &data);         //!< \brief read register offset 0x0A
    errorTypes readOffsetY(uint16_t &data);         //!< \brief read register offset 0x0B
    errorTypes readSynch(uint16_t &data);           //!< \brief read register offset 0x0C
    errorTypes readIFAB(uint16_t &data);            //!< \brief read register offset 0x0D
    errorTypes readIntMode4(uint16_t &data);        //!< \brief read register offset 0x0E
    errorTypes readTempCoeff(uint16_t &data);       //!< \brief read register offset 0x0F
    errorTypes readTempDMag(uint16_t &data);        //!< \brief read register offset 0x14
    errorTypes readTempRaw(uint16_t &data);         //!< \brief read register offset 0x15
    errorTypes readTempIIFCnt(uint16_t &data);      //!< \brief read register offset 0x20
    errorTypes readTempT25(uint16_t &data);         //!< \brief read register offset 0x30

    /*!
		* The rawX value is signed 16 bit value
		* @param data pointer to 16bit word
		* @return CRC error type
		*/
    errorTypes readRawX(int16_t &data);

    /*!
    * The rawY value is signed 16 bit value
    * @param data pointer to 16bit word
    * @return CRC error type
    */
    errorTypes readRawY(int16_t &data);

    // Get functions start
    /*!
    * Returns the Angle Range
    * Angle Range is stored in bytes 14 - 4 of MOD_2.
    * @param angleRange pointer to 16bit double value
    * @return CRC error type
    */
    errorTypes getAngleRange(double &angleRange);

    /*!
	* Returns the angleValue calculated on the base of a 15 bit signed integer.
	* However, the register returns 16 bits, so we need to do some bit arithmetic.
	* @param [in,out] angleValue pointer to 16bit double angle value
	* @return CRC error type
    */
    errorTypes getAngleValue(double &angleValue);
    /*!
	* Same function as before but also returns a pointer to the raw data
	* @param [in,out] angleValue pointer to 16bit double angle value
	* @param [in,out] rawAnglevalue point to an int16_t raw data value
	* @param [in] upd read from update (UPD_high) register or directly (default, UPD_low)
	* @param [in] safe generate safety word (default, SAFE_high) or no (SAFE_low)
	* @return CRC error type
	*/
    errorTypes getAngleValue(double &angleValue, int16_t &rawAnglevalue, updTypes upd=UPD_low, safetyTypes safe=SAFE_high);

    /*!
	* Returns the number of revolutions done from the angle value which is a 9 bit signed integer.
	* However, the register returns 16 bits, so we need to do some bit arithmetic.
	* Therefore the resulting revolution can b only between -256 < numRev < 256 and
	* it will switch from positive to negative and vice versa values at the borders.
	* @param [in,out] numRev pointer to 16bit word for the number of revolutions
	* @param [in] upd read from update (UPD_high) register or directly (default, UPD_low)
	* @param [in] safe generate safety word (default, SAFE_high) or no (SAFE_low)
	* @return CRC error type
	*/
    errorTypes getNumRevolutions(int16_t &numRev, updTypes upd=UPD_low, safetyTypes safe=SAFE_high);

    /*!
    * Return the temperature.
    * The temperature value is a 9 bit signed integer.
    * However, the register returns 16 bits, so we need to do some bit arithmetic.
    * @param [in,out] temp pointer to 16bit double value of the temperature
    * @return CRC error type
    */
    errorTypes getTemperature(double &temp);
    /*!
    * Same as above but also returns a pointer to the raw data
    * @param [in,out] temp pointer to 16bit double value of the temperature
    * @param [in,out] rawTemp pointer to int16_t raw value data
    * @param [in] upd read from update (UPD_high) register or directly (default, UPD_low)
    * @param [in] safe generate safety word (default, SAFE_high) or no (SAFE_low)
    * @return CRC error type
    */
    errorTypes getTemperature(double &temp, int16_t &rawTemp, updTypes upd=UPD_low, safetyTypes safe=SAFE_high);

    /*!
    * Returns the calculated angle speed.
    * The angle speed is a 15 bit signed integer,
    * however, the register returns 16 bits, so we need to do some bit arithmetic.
    * @param [in,out] angleSpeed pointer to 16bit double value
    * @return CRC error type
    */
    errorTypes getAngleSpeed(double &angleSpeed);
    /*!
    * Same as above but also returns a pointer to the raw data
    * @param [in,out] angleSpeed angleSpeed pointer to 16bit double value
    * @param [in,out] rawSpeed pointer to int16_t raw value data
    * @param [in] upd read from update (UPD_high) register or directly (default, UPD_low)
    * @param [in] safe generate safety word (default, SAFE_high) or no (SAFE_low)
    * @return CRC error type
    */
    errorTypes getAngleSpeed(double &angleSpeed,int16_t &rawSpeed, updTypes upd=UPD_low, safetyTypes safe=SAFE_high);
    // end get functions

    //-----------------------------------------------------------------------------
    // begin write functions
    /*!
	* Function sets the SNR register with the correct slave number
	* @param [in] dataToWrite the new data that will be written to the register
	* @return CRC error type
	*/
    errorTypes writeSlaveNumber(uint16_t dataToWrite);

    /*!
    * General write function for writing registers to the Tle5012b. The safety flag will be
    * set always and only some of all registers are writable. See documentation for further information.
    * @param [in] command the command to execute the write
    * @param [in] dataToWrite the new data that will be written to the register
    * @param [in] changeCRC the registerIndex helps figure out in which register the value changed,
    *        so that we don't need to read all the register again to calculate the CRC
    * @return CRC error type
    */
    errorTypes writeToSensor(uint16_t command, uint16_t dataToWrite, bool changeCRC);

    /*!
    * This function is used in order to update the CRC in the register 0F(second byte)
    * @param [in] dataToWrite the new data that will be written to the register
    * @return CRC error type
    */
    errorTypes writeTempCoeffUpdate(uint16_t dataToWrite);

    /*!
    * Standard function used for updating the CRC
    * @param [in] dataToWrite the new data that will be written to the register
    * @return CRC error type
    */
    errorTypes writeActivationStatus(uint16_t dataToWrite);    //!< \brief write register offset 0x01
    errorTypes writeIntMode1(uint16_t dataToWrite);            //!< \brief write register offset 0x06
    errorTypes writeSIL(uint16_t dataToWrite);                 //!< \brief write register offset 0x07
    errorTypes writeIntMode2(uint16_t dataToWrite);            //!< \brief write register offset 0x08
    errorTypes writeIntMode3(uint16_t dataToWrite);            //!< \brief write register offset 0x09
    errorTypes writeOffsetX(uint16_t dataToWrite);             //!< \brief write register offset 0x0A
    errorTypes writeOffsetY(uint16_t dataToWrite);             //!< \brief write register offset 0x0B
    errorTypes writeSynch(uint16_t dataToWrite);               //!< \brief write register offset 0x0C
    errorTypes writeIFAB(uint16_t dataToWrite);                //!< \brief write register offset 0x0D
    errorTypes writeIntMode4(uint16_t dataToWrite);            //!< \brief write register offset 0x0E
    errorTypes writeTempCoeff(uint16_t dataToWrite);           //!< \brief write register offset 0x0F
    // end write functions

    safetyWord safetyStatus;
    uint16_t safetyWord;                                       //!< the last fetched safety word

    /*!
	* Function reset the Sensor to fuse defaults
	* @return CRC error type
	*/
    errorTypes resetFirmware();

    /*!
    * Function reads all readable sensor registers
    * and separates the information fields. This function
    * is needed for finding the selected interface type.
    * @return CRC error type
    */
    errorTypes readRegMap();

    /*!
    * Functions switches between all possible interface types.
    * ATTENTION: The different interfaces support not always all
    * values, see documentation for the ability of each interface.
    * If you want to be save, than choose the default SSC interface
    * which always supports all possible parameter.
    * @param iface type of interface to switch to
    * @return CRC error type
    */
    errorTypes writeInterfaceType(Reg::interfaceType_t iface);

    /*!
    * Function set the sensors calibration mode. Keep in mind,
    * not all Sensor interface setups have the autocalibration
    * switched on, so maybe you have to set it explicitly.
    * @param [in] calMode the auto calibration mode to set
    * @return CRC error type
    */
    errorTypes setCalibration(Reg::calibrationMode_t calMode);

    Reg reg;                //!< \brief Reg map
    SPIC * spic;
    slaveNum mSlave;             //!< \brief actual set slave number
private:

    uint16_t _command[2];                      //!< \brief  command write data [0] = command [1] = data to write
    uint16_t _received[MAX_REGISTER_MEM];      //!< \brief fetched data from sensor with last word = safety word
    uint16_t _registers[CRC_NUM_REGISTERS+1];  //!< \brief keeps track of the values stored in the 8 _registers, for which the CRC is calculated

    errorTypes regularCrcUpdate();
    errorTypes checkSafety(uint16_t safety, uint16_t command, uint16_t* readreg, uint16_t length);
    void resetSafety();
};


#endif //TLE5012B_TEST_TLE5012B_H
