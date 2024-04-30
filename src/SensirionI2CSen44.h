/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * I2C-Generator: 0.2.0
 * Yaml Version: 0.1.0
 * Template Version: 0.7.0
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SENSIRIONI2CSEN44_H
#define SENSIRIONI2CSEN44_H

#include <Wire.h>

#include <SensirionCore.h>

class SensirionI2CSen44 {

  public:
    SensirionI2CSen44();
    /**
     * begin() - Initializes the SensirionI2CSen44 class.
     *
     * @param i2cBus Arduino stream object to use for communication.
     *
     */
    void begin(TwoWire& i2cBus);

    /**
     * startMeasurement() - Starts a continuous measurement.
     *
     * @note This command is only available in idle mode.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t startMeasurement(void);

    /**
     * stopMeasurement() - Stops the measurement and returns to idle mode.
     *
     * @note This command is only available in measurement mode.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t stopMeasurement(void);

    /**
     * readDataReady() - This command can be used to check if new measurement
     * results are ready to read. The data ready flag is automatically reset
     * after reading the measurement values with the 0x03.. \"Read Measured
     * Values\" commands.
     *
     * @param padding Padding byte, always 0x00
     *
     * @param dataReady True (0x01) if data is ready, False (0x00) if not.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readDataReady(bool& dataReady);

    /**
     * readMeasuredPmValues() - Returns the measured particulate matter values.
     * The command 0x0202 \"Read Data Ready\" can be used to check if new data
     * is available since the last read operation. If no new data is available,
     * the previous values will be returned again. If no data is available at
     * all (no measurement running or immediately after starting the
     * measurement), all values will be zero.
     *
     * @note This command is only available in measure mode.
     *
     * @param massConcentrationPm1p0 Mass concentration PM1.0 [µg/m³].
     *
     * @param massConcentrationPm2p5 Mass concentration PM2.5 [µg/m³].
     *
     * @param massConcentrationPm4p0 Mass concentration PM4.0 [µg/m³].
     *
     * @param massConcentrationPm10p0 Mass concentration PM10.0 [µg/m³].
     *
     * @param numberConcentrationPm0p5 Number concentration PM0.5 [#/cm³].
     *
     * @param numberConcentrationPm1p0 Number concentration PM1.0 [#/cm³].
     *
     * @param numberConcentrationPm2p5 Number concentration PM2.5 [#/cm³].
     *
     * @param numberConcentrationPm4p0 Number concentration PM4.0 [#/cm³].
     *
     * @param numberConcentrationPm10p0 Number concentration PM10.0 [#/cm³].
     *
     * @param typicalParticleSize Typical particle size [µm] with scale factor
     * 1000.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readMeasuredPmValues(
        uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
        uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
        uint16_t& numberConcentrationPm0p5, uint16_t& numberConcentrationPm1p0,
        uint16_t& numberConcentrationPm2p5, uint16_t& numberConcentrationPm4p0,
        uint16_t& numberConcentrationPm10p0, uint16_t& typicalParticleSize);

    /**
     * readMeasuredMassConcentrationAndAmbientValuesTicks() - Returns the
     * measured mass concentration, VOC index and ambient temperature and
     * humidity. The command 0x0202 \"Read Data Ready\" can be used to check if
     * new data is available since the last read operation. If no new data is
     * available, the previous values will be returned again. If no data is
     * available at all (no measurement running or immediately after starting
     * the measurement), all values will be zero.
     *
     * @note This command is only available in measure mode.
     *
     * @param massConcentrationPm1p0 Mass concentration PM1.0 [µg/m³].
     *
     * @param massConcentrationPm2p5 Mass concentration PM2.5 [µg/m³].
     *
     * @param massConcentrationPm4p0 Mass concentration PM4.0 [µg/m³].
     *
     * @param massConcentrationPm10p0 Mass concentration PM10.0 [µg/m³].
     *
     * @param vocIndex VOC index with scale factor 10.
     *
     * @param ambientHumidity Compensated ambient relative humidity [%RH] with
     * scale factor 100.
     *
     * @param ambientTemperature Compensated ambient temperature [°C] with scale
     * factor 200.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readMeasuredMassConcentrationAndAmbientValuesTicks(
        uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
        uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
        int16_t& vocIndex, int16_t& ambientHumidity,
        int16_t& ambientTemperature);

    /**
     * readMeasuredMassConcentrationAndAmbientValues() - Returns the measured
     * mass concentration, VOC index and ambient temperature and humidity. The
     * command 0x0202 \"Read Data Ready\" can be used to check if new data is
     * available since the last read operation. If no new data is available, the
     * previous values will be returned again. If no data is available at all
     * (no measurement running or immediately after starting the measurement),
     * all values will be zero.
     *
     * @note This command is only available in measure mode.
     *
     * @param massConcentrationPm1p0 Mass concentration PM1.0 [µg/m³].
     *
     * @param massConcentrationPm2p5 Mass concentration PM2.5 [µg/m³].
     *
     * @param massConcentrationPm4p0 Mass concentration PM4.0 [µg/m³].
     *
     * @param massConcentrationPm10p0 Mass concentration PM10.0 [µg/m³].
     *
     * @param vocIndex VOC index.
     *
     * @param ambientHumidity Compensated ambient relative humidity [%RH].
     *
     * @param ambientTemperature Compensated ambient temperature [°C].
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readMeasuredMassConcentrationAndAmbientValues(
        uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
        uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
        float& vocIndex, float& ambientHumidity, float& ambientTemperature);

    /**
     * readMeasuredAmbientValuesTicks() - Returns the measured VOC index and
     * ambient temperature and humidity. The command 0x0202 \"Read Data Ready\"
     * can be used to check if new data is available since the last read
     * operation. If no new data is available, the previous values will be
     * returned again. If no data is available at all (no measurement running or
     * immediately after starting the  measurement), all values will be zero.
     *
     * @note This command is only available in measure mode.
     *
     * @param vocIndex VOC index with scale factor 10.
     *
     * @param ambientHumidity Compensated ambient relative humidity [%RH] with
     * scale factor 100.
     *
     * @param ambientTemperature Compensated ambient temperature [°C] with scale
     * factor 200.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readMeasuredAmbientValuesTicks(int16_t& vocIndex,
                                            int16_t& ambientHumidity,
                                            int16_t& ambientTemperature);

    /**
     * readMeasuredAmbientValues() - Returns the measured VOC index and ambient
     * temperature and humidity. The command 0x0202 \"Read Data Ready\" can be
     * used to check if new data is available since the last read operation. If
     * no new data is available, the previous values will be returned again. If
     * no data is available at all (no measurement running or immediately after
     * starting the  measurement), all values will be zero.
     *
     * @note This command is only available in measure mode.
     *
     * @param vocIndex VOC index.
     *
     * @param ambientHumidity Compensated ambient relative humidity [%RH].
     *
     * @param ambientTemperature Compensated ambient temperature [°C].
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readMeasuredAmbientValues(float& vocIndex, float& ambientHumidity,
                                       float& ambientTemperature);

    /**
     * startFanCleaning() - Starts the fan cleaning manually.
     *
     * @note This command is only available in measure mode.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t startFanCleaning(void);

    /**
     * setAutoCleaningInterval() - Sets the auto cleaning interval for the
     * device.
     *
     * @param interval Auto cleaning interval [s]. Set to zero to disable auto
     * cleaning.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t setAutoCleaningInterval(uint32_t interval);

    /**
     * getAutoCleaningInterval() - Gets the auto cleaning interval from the
     * device.
     *
     * @param interval Auto cleaning interval [s]. Zero means auto cleaning is
     * disabled.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t getAutoCleaningInterval(uint32_t& interval);

    /**
     * getArticleCode() - Gets the article code from the device.
     *
     * @param articleCode String containing the article code.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t getArticleCode(unsigned char articleCode[],
                            uint8_t articleCodeSize);

    /**
     * getSerialNumber() - Gets the serial number from the device.
     *
     * @param serialNumber String containing the serial number.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t getSerialNumber(unsigned char serialNumber[],
                             uint8_t serialNumberSize);

    /**
     * getVersion() - Gets the version information for the hardware, firmware.
     *
     * @param firmwareMajor Firmware major version number.
     *
     * @param firmwareMinor Firmware minor version number.
     *
     * @param firmwareDebug Firmware debug state. If the debug state is set, the
     * firmware is in development.
     *
     * @param hardwareMajor Hardware major version number.
     *
     * @param hardwareMinor Hardware minor version number.
     *
     * @param padding Padding byte, ignore this.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t getVersion(uint8_t& firmwareMajor, uint8_t& firmwareMinor,
                        bool& firmwareDebug, uint8_t& hardwareMajor,
                        uint8_t& hardwareMinor);

    /**
     * readDeviceStatus() - Reads the current device status register.
     *
     * @param deviceStatus Device status (32 flags as integer).
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readDeviceStatus(uint32_t& deviceStatus);

    /**
     * clearDeviceStatus() - Clears all flags in device status register.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t clearDeviceStatus(void);

    /**
     * deviceReset() - Executes a reset on the device. This has the same effect
     * as a power cycle.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t deviceReset(void);

    /**
     * setTemperatureOffset() - Sets the temperature offset for this device.
     *
     * @param temperature_offset temperature offset [°C]. Set to zero to disable
     * temp offset.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t setTemperatureOffset(float temperature_offset);

    /**
     * getTemperatureOffset() - Get the temperature offset for this device.
     *
     * @param temperature_offset Temperature offset [°C]. Zero means temperature
     * offset is disabled.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t getTemperatureOffset(float& temperature_offset);

    /**
     * writeTemperatureOffsetToPersistentMemory() - Write Temperature Offset to
     * persistent Memory.
     *
     * @note This command is only available in idle mode.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t writeTemperatureOffsetToPersistentMemory(void);

  private:
    TwoWire* _i2cBus = nullptr;
};

#endif /* SENSIRIONI2CSEN44_H */
