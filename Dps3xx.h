/**
 * Arduino library to control Dps3xx
 *
 * "Dps3xx" represents Infineon's high-sensitive pressure and temperature sensor.
 * It measures in ranges of 300 - 1200 hPa and -40 and 85 °C.
 * The sensor can be connected via SPI or I2C.
 * It is able to perform single measurements
 * or to perform continuous measurements of temperature and pressure at the same time,
 * and stores the results in a FIFO to reduce bus communication.
 *
 * Have a look at the datasheet for more information.
 */
#ifndef DPS3xx_H_
#define DPS3xx_H_

#include <stdint.h>
#include "util/dps_config.h"
#include "util/dps3xx_config.h"
#include "util/DpsRegister.h"

extern struct Dps3xxDev dps3xx_dev;
extern const struct spi_dt_spec dps3_spi;
/*!
 * @brief Dps3xx device structure
 */
struct Dps3xxDev
{
    // scaling factor table
    const int32_t scaling_facts[DPS__NUM_OF_SCAL_FACTS];

    enum Mode m_opMode;

    // flags
    uint8_t m_initFail;

    // the Product ID of the connected Dps3xx sensor
    uint8_t m_productID;
    // the Revision ID of the connected Dps3xx sensor
    uint8_t m_revisionID;

    // settings
    uint8_t m_tempMr;
    uint8_t m_tempOsr;
    uint8_t m_prsMr;
    uint8_t m_prsOsr;

    // compensation coefficients for both dps3xx and dps422
    int32_t m_c00;
    int32_t m_c10;
    int32_t m_c01;
    int32_t m_c11;
    int32_t m_c20;
    int32_t m_c21;
    int32_t m_c30;

    // last measured scaled temperature (necessary for pressure compensation)
    float m_lastTempScal;

    // used for SPI
    struct spi_dt_spec *m_spibus;
    uint8_t m_threeWire;

    uint8_t m_tempSensor;

    // compensation coefficients
    int32_t m_c0Half;
    int32_t m_c1;
};

/**
 * Standard SPI begin function
 *
 * @param dps3xx_dev:      reference to a Dps3xxDev structure
 * @param threeWire:        1 if Dps3xx is connected with 3-wire SPI
 *                          0 if Dps3xx is connected with 4-wire SPI (standard)
 */
void dps3xx_begin(struct Dps3xxDev *dps3xx_dev, uint8_t threeWire);

/**
 * End function for Dps3xx
 * Sets the sensor to idle mode
 */
void dps3xx_end(struct Dps3xxDev *dps3xx_dev);

/**
 * Sets the Dps3xx to standby mode
 *
 * @return  status code
 */
int16_t dps3xx_standby(struct Dps3xxDev *dps3xx_dev);

/**
 * performs one temperature measurement
 *
 * @param result:      pointer to a float value where the result will be written
 * @return 	status code
 */
int16_t dps3xx_measure_temp_once(struct Dps3xxDev *dev, float *result);

/**
 * performs one temperature measurement with specified oversamplingRate
 *
 * @param result:              reference to a float where the result will be written
 * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
 *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128,
 *                              which are defined as integers 0 - 7
 *                              The number of measurements equals to 2^n, if the value written to
 *                              the register field is n. 2^n internal measurements are combined to
 *                              return a more exact measurement
 * @return   status code
 */
int16_t dps3xx_measure_temp_once_with_osr(struct Dps3xxDev *dps3xx_dev, float *result, uint8_t oversamplingRate);

/**
 * starts a single temperature measurement
 *
 * @return 	status code
 */
int16_t dps3xx_start_measure_temp_once(struct Dps3xxDev *dps3xx_dev);

/**
 * starts a single temperature measurement with specified oversamplingRate
 *
 * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
 *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128, which are defined as integers 0 - 7
 * @return  status code
 */
int16_t dps3xx_start_measure_temp_once_with_osr(struct Dps3xxDev *dps3xx_dev, uint8_t oversamplingRate);

/**
 * performs one pressure measurement
 *
 * @param result:              reference to a float value where the result will be written
 * @return 	status code
 */
int16_t dps3xx_measure_pressure_once(struct Dps3xxDev *dps3xx_dev, float *result);

/**
 * performs one pressure measurement with specified oversamplingRate
 *
 * @param result:              reference to a float where the result will be written
 * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
 *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 * @return  status code
 */
int16_t dps3xx_measure_pressure_once_with_osr(struct Dps3xxDev *dps3xx_dev, float *result, uint8_t oversamplingRate);

/**
 * starts a single pressure measurement
 *
 * @return 	status code
 */
int16_t dps3xx_start_measure_pressure_once(struct Dps3xxDev *dps3xx_dev);

/**
 * starts a single pressure measurement with specified oversamplingRate
 *
 * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
 *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 * @return  status code
 */
int16_t dps3xx_start_measure_pressure_once_with_osr(struct Dps3xxDev *dps3xx_dev, uint8_t oversamplingRate);

/**
 * gets the result a single temperature or pressure measurement in °C or Pa
 *
 * @param result:              reference to a float value where the result will be written
 * @return 	status code
 */
int16_t dps3xx_get_single_result(struct Dps3xxDev *dps3xx_dev, float *result);

/**
 * starts a continuous temperature measurement with specified measurement rate and oversampling rate
 * If measure rate is n and oversampling rate is m, the DPS3xx performs 2^(n+m) internal measurements per second.
 * The DPS3xx cannot operate with high precision and high speed at the same time. Consult the datasheet for more information.
 *
 * @param measureRate:          DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
 * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 *
 * @return  status code
 *
 */
int16_t dps3xx_start_measure_temp_cont(struct Dps3xxDev *dps3xx_dev, uint8_t measureRate, uint8_t oversamplingRate);

/**
 * starts a continuous temperature measurement with specified measurement rate and oversampling rate
 *
 * @param measureRate:          DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
 * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 * @return  status code
 */
int16_t dps3xx_start_measure_pressure_cont(struct Dps3xxDev *dps3xx_dev, uint8_t measureRate, uint8_t oversamplingRate);

/**
 * starts a continuous temperature and pressure measurement with specified measurement rate and oversampling rate for temperature and pressure measurement respectively.
 *
 * @param tempMr:               measure rate for temperature
 * @param tempOsr:              oversampling rate for temperature
 * @param prsMr:                measure rate for pressure
 * @param prsOsr:               oversampling rate for pressure
 * @return  status code
 */
int16_t dps3xx_start_measure_both_cont(struct Dps3xxDev *dps3xx_dev, uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr);

/**
 * Gets the interrupt status flag of the FIFO
 *
 * @return 	    1 if the FIFO is full and caused an interrupt
 *              0 if the FIFO is not full or FIFO interrupt is disabled
 *              -1 on fail
 */
int16_t dps3xx_get_int_status_fifo_full(struct Dps3xxDev *dps3xx_dev);

/**
 * Gets the interrupt status flag that indicates a finished temperature measurement
 *
 * @return 	    1 if a finished temperature measurement caused an interrupt;
 *              0 if there is no finished temperature measurement or interrupts are disabled;
 *              -1 on fail.
 */
int16_t dps3xx_get_int_status_temp_ready(struct Dps3xxDev *dps3xx_dev);

/**
 * Gets the interrupt status flag that indicates a finished pressure measurement
 *
 * @return      1 if a finished pressure measurement caused an interrupt;
 *              0 if there is no finished pressure measurement or interrupts are disabled;
 *              -1 on fail.
 */
int16_t dps3xx_get_int_status_prs_ready(struct Dps3xxDev *dps3xx_dev);

/**
 * Function to fix a hardware problem on some devices
 * You have this problem if you measure a temperature which is too high (e.g. 60°C when temperature is around 20°C)
 * Call dps3xx_correct_temp() directly after begin() to fix this issue
 */
int16_t dps3xx_correct_temp(struct Dps3xxDev *dps3xx_dev);

/**
 * @brief Set the source of interrupt (FIFO full, measurement values ready)
 *
 * @param intr_source Interrupt source as defined by Interrupt_source_3xx_e
 * @return status code
 */
int16_t dps3xx_set_interrupt_sources(struct Dps3xxDev *dps3xx_dev, uint8_t intr_source);

/**
 * @brief Set the source of interrupt (FIFO full, measurement values ready) with specified polarity
 *
 * @param intr_source Interrupt source as defined by Interrupt_source_3xx_e
 * @param polarity
 * @return status code
 */
int16_t dps3xx_set_interrupt_sources_with_polarity(struct Dps3xxDev *dps3xx_dev, uint8_t intr_source, uint8_t polarity);

/**
 * Gets the results from continuous measurements and writes them to given arrays
 *
 * @param *tempBuffer:      The start address of the buffer where the temperature results are written
 *                          If this is NULL, no temperature results will be written out
 * @param &tempCount:       The size of the buffer for temperature results.
 *                          When the function ends, it will contain the number of bytes written to the buffer.
 * @param *prsBuffer:       The start address of the buffer where the pressure results are written
 *                          If this is NULL, no pressure results will be written out
 * @param &prsCount:        The size of the buffer for pressure results.
 *                          When the function ends, it will contain the number of bytes written to the buffer.
 * @param reg               The FIFO empty register field; needed since this field is different for each sensor (should be registers[FIFO_EMPTY] removed for combined code)
 * @return  status code
 */
int16_t dps3xx_get_cont_results(struct Dps3xxDev *dps3xx_dev, float *tempBuffer, uint8_t *tempCount, float *prsBuffer, uint8_t *prsCount);

#endif