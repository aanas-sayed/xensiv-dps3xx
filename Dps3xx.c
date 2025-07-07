#include "Dps3xx.h"
#include "util/dps_config.h"
#include "util/dps3xx_config.h"
#include "util/DpsRegister.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "../config.h"
#include "../utils/spi_helper.h"

// Logging
LOG_MODULE_REGISTER(dps3_class, CONFIG_LOCAL_DPS3_CLASS_LOG_LEVEL);

/* Retrieve the SPI device structure */
#define SPIOP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_MODE_CPOL)
const struct spi_dt_spec dps3_spi = SPI_DT_SPEC_GET(DPS3_SPI_NODE, SPIOP, 0);

struct Dps3xxDev dps3xx_dev = {
    .scaling_facts = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960},
    .m_opMode = IDLE,
    .m_initFail = 1U,
    .m_productID = 0U,
    .m_revisionID = 0U,
    .m_tempMr = 0U,
    .m_tempOsr = 0U,
    .m_prsMr = 0U,
    .m_prsOsr = 0U,
    .m_c00 = 0,
    .m_c10 = 0,
    .m_c01 = 0,
    .m_c11 = 0,
    .m_c20 = 0,
    .m_c21 = 0,
    .m_c30 = 0,
    .m_lastTempScal = 0.0f,
    .m_spibus = &dps3_spi,
    .m_threeWire = 0U,
};

// Forward declarations
static void dps3xx_init(struct Dps3xxDev *dev);
static int16_t dps3xx_read_coeffs(struct Dps3xxDev *dev);
static int16_t dps3xx_set_op_mode(struct Dps3xxDev *dev, uint8_t opMode);
static int16_t dps3xx_config_temp(struct Dps3xxDev *dev, uint8_t tempMr, uint8_t tempOsr);
static int16_t dps3xx_config_pressure(struct Dps3xxDev *dev, uint8_t prsMr, uint8_t prsOsr);
static int16_t dps3xx_enable_fifo(struct Dps3xxDev *dev);
static int16_t dps3xx_disable_fifo(struct Dps3xxDev *dev);
static uint16_t dps3xx_calc_busy_time(uint16_t mr, uint16_t osr);
static int16_t dps3xx_get_fifo_value(struct Dps3xxDev *dev, int32_t *value);
static int16_t dps3xx_read_byte(struct Dps3xxDev *dev, uint8_t regAddress);
static int16_t dps3xx_write_byte(struct Dps3xxDev *dev, uint8_t regAddress, uint8_t data, uint8_t check);
static int16_t dps3xx_write_byte_bitfield(struct Dps3xxDev *dev, uint8_t data, RegMask_t regMask, uint8_t check);
static int16_t dps3xx_read_byte_bitfield(struct Dps3xxDev *dev, RegMask_t regMask);
static int16_t dps3xx_read_block(struct Dps3xxDev *dev, RegBlock_t regBlock, uint8_t *buffer);
static void getTwosComplement(int32_t *raw, uint8_t length);
static int16_t dps3xx_get_raw_result(struct Dps3xxDev *dev, int32_t *raw, RegBlock_t reg);
static float dps3xx_calc_temp(struct Dps3xxDev *dev, int32_t raw);
static float dps3xx_calc_pressure(struct Dps3xxDev *dev, int32_t raw);
static int16_t dps3xx_flush_fifo(struct Dps3xxDev *dev);

/**
 * Initializes the sensor.
 * This function has to be called from dps3xx_begin()
 * and requires a valid bus initialization.
 */
static void dps3xx_init(struct Dps3xxDev *dev)
{
    LOG_DBG("Initializing Dps3xx sensor");

    int16_t prodId = dps3xx_read_byte_bitfield(dev, registers[PROD_ID]);
    if (prodId < 0)
    {
        // Connected device is not a Dps3xx
        dev->m_initFail = 1U;
        LOG_ERR("Connected device is not a Dps3xx");
        return;
    }
    LOG_DBG("Product ID: %u", prodId);
    dev->m_productID = prodId;

    int16_t revId = dps3xx_read_byte_bitfield(dev, registers[REV_ID]);
    if (revId < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to read revision id");
        return;
    }
    LOG_DBG("Revision ID: %u", revId);
    dev->m_revisionID = revId;

    // reset
    dps3xx_reset(dev);

    // find out which temperature sensor is calibrated with coefficients...
    int16_t sensor = dps3xx_read_byte_bitfield(dev, registers[TEMP_SENSORREC]);
    LOG_DBG("Temperature sensor: %u", sensor);
    if (sensor < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to read temp sensor id");
        return;
    }

    //...and use this sensor for temperature measurement
    dev->m_tempSensor = sensor;
    if (dps3xx_write_byte_bitfield(dev, (uint8_t)sensor, registers[TEMP_SENSOR], 0U) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to set temp sensor id");
        return;
    }

    // read coefficients
    if (dps3xx_read_coeffs(dev) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to read coefficients");
        return;
    }

    // set to standby for further configuration
    if (dps3xx_standby(dev) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to set standby");
        return;
    }

    // set measurement precision and rate to standard values;
    if (dps3xx_config_temp(dev, DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to set temperature measurement rate");
        return;
    }
    if (dps3xx_config_pressure(dev, DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to set pressure measurement rate");
        return;
    }

    // perform a first temperature measurement
    // the most recent temperature will be saved internally
    // and used for compensation when calculating pressure
    float trash;
    if (dps3xx_measure_temp_once(dev, &trash) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to perform first temperature measurement");
        return;
    }

    // make sure the Dps3xx is in standby after initialization
    if (dps3xx_standby(dev) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to set standby");
        return;
    }

    // Fix IC with a fuse bit problem, which lead to a wrong temperature
    // Should not affect ICs without this problem
    if (dps3xx_correct_temp(dev) < 0)
    {
        dev->m_initFail = 1U;
        LOG_ERR("Failed to correct temperature");
        return;
    }

    LOG_DBG("Dps3xx sensor initialized");
}

void dps3xx_begin(struct Dps3xxDev *dev, uint8_t threeWire)
{
    // this flag will show if the initialization was successful
    dev->m_initFail = 0U;

    // Set SPI bus connection
    dev->m_spibus = &dps3_spi;

    // Initialize SPI bus
    if (!device_is_ready(dps3_spi.bus))
    {
        LOG_ERR("SPI initialization failed");
        dev->m_initFail = 1U;
        return;
    }

    k_msleep(50); // startup time of Dps3xx

    // Switch to 3-wire mode if necessary
    // do not use writeByteBitfield or check option to set SPI mode!
    // Reading is not possible until SPI-mode is valid
    if (threeWire)
    {
        LOG_DBG("Switching to 3-wire mode");
        dev->m_threeWire = 1U;
        if (dps3xx_write_byte(dev, DPS3xx__REG_ADR_SPI3W, DPS3xx__REG_CONTENT_SPI3W, 0U))
        {
            dev->m_initFail = 1U;
            LOG_ERR("Failed to set SPI mode");
            return;
        }
    }

    dps3xx_init(dev);
}

/**
 * resets the sensor
 *
 * @return 	0 on success, -1 on fail
 */
int16_t dps3xx_reset(struct Dps3xxDev *dev)
{
    // switch to reset mode
    if (dps3xx_write_byte_bitfield(dev, 0b1001, registers[SOFT_RESET], 0U) < 0)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    k_msleep(100);
    return 0;
}

/**
 * reads the compensation coefficients from the sensor
 * this is called once from init(), which is called from begin()
 *
 * @return 	0 on success, -1 on fail
 */
static int16_t dps3xx_read_coeffs(struct Dps3xxDev *dev)
{
    // TODO: remove magic number
    uint8_t buffer[18];
    // read COEF registers to buffer
    int16_t ret = dps3xx_read_block(dev, coeffBlock, buffer);
    if (!ret)
        return DPS__FAIL_INIT_FAILED;
    LOG_DBG("Read sensor compensation coefficients: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
            buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7],
            buffer[8], buffer[9], buffer[10], buffer[11], buffer[12], buffer[13], buffer[14], buffer[15], buffer[16], buffer[17]);

    // compose coefficients from buffer content
    dev->m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    getTwosComplement(&dev->m_c0Half, 12);
    // c0 is only used as c0*0.5, so c0_half is calculated immediately
    dev->m_c0Half = dev->m_c0Half / 2U;

    // now do the same thing for all other coefficients
    dev->m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    getTwosComplement(&dev->m_c1, 12);
    dev->m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
    getTwosComplement(&dev->m_c00, 20);
    dev->m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
    getTwosComplement(&dev->m_c10, 20);

    dev->m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    getTwosComplement(&dev->m_c01, 16);

    dev->m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    getTwosComplement(&dev->m_c11, 16);
    dev->m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    getTwosComplement(&dev->m_c20, 16);
    dev->m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    getTwosComplement(&dev->m_c21, 16);
    dev->m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    getTwosComplement(&dev->m_c30, 16);

    return DPS__SUCCEEDED;
}

int16_t dps3xx_get_cont_results(struct Dps3xxDev *dev, float *tempBuffer, uint8_t *tempCount, float *prsBuffer, uint8_t *prsCount)
{
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in background mode
    if (!(dev->m_opMode & 0x04))
    {
        return DPS__FAIL_TOOBUSY;
    }

    if (!tempBuffer || !prsBuffer)
    {
        return DPS__FAIL_UNKNOWN;
    }
    *tempCount = 0U;
    *prsCount = 0U;

    // while FIFO is not empty
    while (dps3xx_read_byte_bitfield(dev, registers[FIFO_EMPTY]) == 0)
    {
        int32_t raw_result;
        float result;
        // read next result from FIFO
        int16_t type = dps3xx_get_fifo_value(dev, &raw_result);
        switch (type)
        {
        case 0: // temperature
            if (*tempCount < DPS__FIFO_SIZE)
            {
                result = dps3xx_calc_temp(dev, raw_result);
                tempBuffer[(*tempCount)++] = result;
            }
            break;
        case 1: // pressure
            if (*prsCount < DPS__FIFO_SIZE)
            {
                result = dps3xx_calc_pressure(dev, raw_result);
                prsBuffer[(*prsCount)++] = result;
            }
            break;
        case -1: // read failed
            break;
        }
    }
    return DPS__SUCCEEDED;
}

/**
 * gets the result a single temperature or pressure measurement in °C or Pa
 *
 * @param result:              reference to a float value where the result will be written
 * @return 	status code
 */
int16_t dps3xx_get_single_result(struct Dps3xxDev *dev, float *result)
{
    // abort if initialization failed
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }

    // read finished bit for current opMode
    int16_t rdy;
    switch (dev->m_opMode)
    {
    case CMD_TEMP: // temperature
        rdy = dps3xx_read_byte_bitfield(dev, config_registers[TEMP_RDY]);
        break;
    case CMD_PRS: // pressure
        rdy = dps3xx_read_byte_bitfield(dev, config_registers[PRS_RDY]);
        break;
    default: // DPS3xx not in command mode
        return DPS__FAIL_TOOBUSY;
    }
    // read new measurement result
    switch (rdy)
    {
    case DPS__FAIL_UNKNOWN: // could not read ready flag
        return DPS__FAIL_UNKNOWN;
    case 0: // ready flag not set, measurement still in progress
        return DPS__FAIL_UNFINISHED;
    case 1: // measurement ready, expected case
        enum Mode oldMode = dev->m_opMode;
        dev->m_opMode = IDLE; // opcode was automatically reset by DPS3xx
        int32_t raw_val;
        int16_t status;

        switch (oldMode)
        {
        case CMD_TEMP: // temperature
            status = dps3xx_get_raw_result(dev, &raw_val, registerBlocks[TEMP]);
            if (status != DPS__SUCCEEDED)
                return status;
            *result = dps3xx_calc_temp(dev, raw_val);
            return DPS__SUCCEEDED; // TODO
        case CMD_PRS:              // pressure
            status = dps3xx_get_raw_result(dev, &raw_val, registerBlocks[PRS]);
            if (status != DPS__SUCCEEDED)
                return status;
            *result = dps3xx_calc_pressure(dev, raw_val);
            return DPS__SUCCEEDED; // TODO
        default:
            return DPS__FAIL_UNKNOWN; // should already be filtered above
        }
    }
    return DPS__FAIL_UNKNOWN;
}

int16_t dps3xx_measure_temp_once(struct Dps3xxDev *dev, float *result)
{
    return dps3xx_measure_temp_once_with_osr(dev, result, dev->m_tempOsr);
}

int16_t dps3xx_measure_temp_once_with_osr(struct Dps3xxDev *dev, float *result, uint8_t oversamplingRate)
{
    // Start measurement
    int16_t ret = dps3xx_start_measure_temp_once_with_osr(dev, oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }
    LOG_DBG("Started temperature measurement with oversampling rate %d", oversamplingRate);

    // wait until measurement is finished
    LOG_DBG("Waiting for measurement to finish... (%d ms)", dps3xx_calc_busy_time(0U, oversamplingRate) / DPS__BUSYTIME_SCALING + DPS3xx__BUSYTIME_FAILSAFE);
    k_msleep(dps3xx_calc_busy_time(0U, oversamplingRate) / DPS__BUSYTIME_SCALING);
    k_msleep(DPS3xx__BUSYTIME_FAILSAFE);
    k_msleep(1000U);

    // get measurement result
    ret = dps3xx_get_single_result(dev, result);
    if (ret != DPS__SUCCEEDED)
    {
        LOG_ERR("Failed to get measurement result: %d", ret);
        dps3xx_standby(dev);
    }
    return ret;
}

int16_t dps3xx_start_measure_temp_once(struct Dps3xxDev *dev)
{
    return dps3xx_start_measure_temp_once_with_osr(dev, dev->m_tempOsr);
}

int16_t dps3xx_start_measure_temp_once_with_osr(struct Dps3xxDev *dev, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }

    if (oversamplingRate != dev->m_tempOsr)
    {
        // configuration of oversampling rate
        if (dps3xx_config_temp(dev, 0U, oversamplingRate) != DPS__SUCCEEDED)
        {
            return DPS__FAIL_UNKNOWN;
        }
    }

    // set device to temperature measuring mode
    return dps3xx_set_op_mode(dev, CMD_TEMP);
}

int16_t dps3xx_measure_pressure_once(struct Dps3xxDev *dev, float *result)
{
    return dps3xx_measure_pressure_once_with_osr(dev, result, dev->m_prsOsr);
}

int16_t dps3xx_measure_pressure_once_with_osr(struct Dps3xxDev *dev, float *result, uint8_t oversamplingRate)
{
    // start the measurement
    int16_t ret = dps3xx_start_measure_pressure_once_with_osr(dev, oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }

    // wait until measurement is finished
    k_msleep(dps3xx_calc_busy_time(0U, dev->m_prsOsr) / DPS__BUSYTIME_SCALING);
    k_msleep(DPS3xx__BUSYTIME_FAILSAFE);

    ret = dps3xx_get_single_result(dev, result);
    if (ret != DPS__SUCCEEDED)
    {
        dps3xx_standby(dev);
    }
    return ret;
}

int16_t dps3xx_start_measure_pressure_once(struct Dps3xxDev *dev)
{
    return dps3xx_start_measure_pressure_once_with_osr(dev, dev->m_prsOsr);
}

int16_t dps3xx_start_measure_pressure_once_with_osr(struct Dps3xxDev *dev, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // configuration of oversampling rate, lowest measure rate to avoid conflicts
    if (oversamplingRate != dev->m_prsOsr)
    {
        if (dps3xx_config_pressure(dev, 0U, oversamplingRate))
        {
            return DPS__FAIL_UNKNOWN;
        }
    }
    // set device to pressure measuring mode
    return dps3xx_set_op_mode(dev, CMD_PRS);
}

int16_t dps3xx_start_measure_temp_cont(struct Dps3xxDev *dev, uint8_t measureRate, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (dps3xx_calc_busy_time(measureRate, oversamplingRate) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (dps3xx_config_temp(dev, measureRate, oversamplingRate))
    {
        return DPS__FAIL_UNKNOWN;
    }

    if (dps3xx_enable_fifo(dev))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (dps3xx_set_op_mode(dev, CONT_TMP))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t dps3xx_start_measure_pressure_cont(struct Dps3xxDev *dev, uint8_t measureRate, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (dps3xx_calc_busy_time(measureRate, oversamplingRate) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (dps3xx_config_pressure(dev, measureRate, oversamplingRate))
        return DPS__FAIL_UNKNOWN;
    // enable result FIFO
    if (dps3xx_enable_fifo(dev))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (dps3xx_set_op_mode(dev, CONT_PRS))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t dps3xx_start_measure_both_cont(struct Dps3xxDev *dev, uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr)
{
    // abort if initialization failed
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (dps3xx_calc_busy_time(tempMr, tempOsr) + dps3xx_calc_busy_time(prsMr, prsOsr) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (dps3xx_config_temp(dev, tempMr, tempOsr))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // update precision and measuring rate
    if (dps3xx_config_pressure(dev, prsMr, prsOsr))
        return DPS__FAIL_UNKNOWN;
    // enable result FIFO
    if (dps3xx_enable_fifo(dev))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (dps3xx_set_op_mode(dev, CONT_BOTH))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t dps3xx_standby(struct Dps3xxDev *dev)
{
    // abort if initialization failed
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // set device to idling mode
    int16_t ret = dps3xx_set_op_mode(dev, IDLE);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }
    ret = dps3xx_disable_fifo(dev);
    return ret;
}

int16_t dps3xx_correct_temp(struct Dps3xxDev *dev)
{
    if (dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    dps3xx_write_byte(dev, 0x0E, 0xA5, 0U);
    dps3xx_write_byte(dev, 0x0F, 0x96, 0U);
    dps3xx_write_byte(dev, 0x62, 0x02, 0U);
    dps3xx_write_byte(dev, 0x0E, 0x00, 0U);
    dps3xx_write_byte(dev, 0x0F, 0x00, 0U);

    // perform a first temperature measurement (again)
    // the most recent temperature will be saved internally
    // and used for compensation when calculating pressure
    float trash;
    dps3xx_measure_temp_once(dev, &trash);

    return DPS__SUCCEEDED;
}

int16_t dps3xx_get_int_status_fifo_full(struct Dps3xxDev *dev)
{
    return dps3xx_read_byte_bitfield(dev, config_registers[INT_FLAG_FIFO]);
}

int16_t dps3xx_get_int_status_temp_ready(struct Dps3xxDev *dev)
{
    return dps3xx_read_byte_bitfield(dev, config_registers[INT_FLAG_TEMP]);
}

int16_t dps3xx_get_int_status_prs_ready(struct Dps3xxDev *dev)
{
    return dps3xx_read_byte_bitfield(dev, config_registers[INT_FLAG_PRS]);
}

int16_t dps3xx_set_interrupt_sources(struct Dps3xxDev *dev, uint8_t intr_source)
{
    return dps3xx_set_interrupt_sources_with_polarity(dev, intr_source, 1U);
}

int16_t dps3xx_set_interrupt_sources_with_polarity(struct Dps3xxDev *dev, uint8_t intr_source, uint8_t polarity)
{
    // Interrupts are not supported with 4 Wire SPI
    if (!dev->m_threeWire)
    {
        return DPS__FAIL_UNKNOWN;
    }
    return dps3xx_write_byte_bitfield(dev, intr_source, registers[INT_SEL], 0U) || dps3xx_write_byte_bitfield(dev, polarity, registers[INT_HL], 0U);
}

/**
 * Sets the Operation Mode of the sensor
 *
 * @param dev:       reference to a Dps3xxDev structure
 * @param opMode:           the new OpMode as defined by Mode; CMD_BOTH should not be used for DPS3xx
 * @return                  0 on success,
 *                          -1 on fail
 */
static int16_t dps3xx_set_op_mode(struct Dps3xxDev *dev, uint8_t opMode)
{
    if (dps3xx_write_byte_bitfield(dev, opMode, config_registers[MSR_CTRL], 0U) == -1)
    {
        return DPS__FAIL_UNKNOWN;
    }
    dev->m_opMode = (enum Mode)opMode;
    return DPS__SUCCEEDED;
}

/**
 * Configures temperature measurement
 *
 * @param tempMr:          DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
 * @param tempOsr:         DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 *
 * @return 	0 normally or -1 on fail
 */
static int16_t dps3xx_config_temp(struct Dps3xxDev *dev, uint8_t tempMr, uint8_t tempOsr)
{
    tempMr &= 0x07;
    tempOsr &= 0x07;
    // two accesses to the same register; for readability
    int16_t ret = dps3xx_write_byte_bitfield(dev, tempMr, config_registers[TEMP_MR], 0U);
    ret = dps3xx_write_byte_bitfield(dev, tempOsr, config_registers[TEMP_OSR], 0U);

    // abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    dev->m_tempMr = tempMr;
    dev->m_tempOsr = tempOsr;

    dps3xx_write_byte_bitfield(dev, dev->m_tempSensor, registers[TEMP_SENSOR], 0U);
    // set TEMP SHIFT ENABLE if oversampling rate higher than eight(2^3)
    if (tempOsr > DPS3xx__OSR_SE)
    {
        ret = dps3xx_write_byte_bitfield(dev, 1U, registers[TEMP_SE], 0U);
    }
    else
    {
        ret = dps3xx_write_byte_bitfield(dev, 0U, registers[TEMP_SE], 0U);
    }
    return ret;
}

/**
 * Configures pressure measurement
 *
 * @param prsMr:           DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
 * @param prsOsr:          DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 * @return 	                0 normally or
 *                          -1 on fail
 */
static int16_t dps3xx_config_pressure(struct Dps3xxDev *dev, uint8_t prsMr, uint8_t prsOsr)
{
    prsMr &= 0x07;
    prsOsr &= 0x07;
    int16_t ret = dps3xx_write_byte_bitfield(dev, prsMr, config_registers[PRS_MR], 0U);
    ret = dps3xx_write_byte_bitfield(dev, prsOsr, config_registers[PRS_OSR], 0U);

    // abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    dev->m_prsMr = prsMr;
    dev->m_prsOsr = prsOsr;

    // set PM SHIFT ENABLE if oversampling rate higher than eight(2^3)
    if (prsOsr > DPS3xx__OSR_SE)
    {
        ret = dps3xx_write_byte_bitfield(dev, 1U, registers[PRS_SE], 0U);
    }
    else
    {
        ret = dps3xx_write_byte_bitfield(dev, 0U, registers[PRS_SE], 0U);
    }
    return ret;
}

static int16_t dps3xx_enable_fifo(struct Dps3xxDev *dev)
{
    return dps3xx_write_byte_bitfield(dev, 1U, config_registers[FIFO_EN], 0U);
}

static int16_t dps3xx_disable_fifo(struct Dps3xxDev *dev)
{
    int16_t ret = dps3xx_flush_fifo(dev);
    ret = dps3xx_write_byte_bitfield(dev, 0U, config_registers[FIFO_EN], 0U);
    return ret;
}

/**
 * calculates the time that the sensor needs for 2^mr measurements with an oversampling rate of 2^osr (see table "pressure measurement time (ms) versus oversampling rate")
 * Note that the total measurement time for temperature and pressure must not be more than 1 second.
 * Timing behavior of pressure and temperature sensors can be considered the same.
 *
 * @param mr:               DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
 * @param osr:              DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 * @return time that the sensor needs for this measurement
 */
static uint16_t dps3xx_calc_busy_time(uint16_t mr, uint16_t osr)
{
    // formula from datasheet (optimized)
    return ((uint32_t)20U << mr) + ((uint32_t)16U << (osr + mr));
}

/**
 * reads the next raw value from the FIFO
 *
 * @param value:  the raw pressure or temperature value read from the pressure register blocks, where the LSB of PRS_B0 marks whether the value is a temperature or a pressure.
 *
 * @return          -1 on fail
 *                  0 if result is a temperature raw value
 *                  1 if result is a pressure raw value
 */
static int16_t dps3xx_get_fifo_value(struct Dps3xxDev *dev, int32_t *value)
{
    uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};

    // abort on invalid argument or failed block reading
    if (value == NULL || dps3xx_read_block(dev, registerBlocks[PRS], buffer) != DPS__RESULT_BLOCK_LENGTH)
        return DPS__FAIL_UNKNOWN;
    *value = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    getTwosComplement(value, 24);
    return buffer[2] & 0x01;
}

/**
 * reads a byte from the sensor via SPI
 *
 * @param regAddress:        Address that has to be read
 * @return  register content or -1 on fail
 */
static int16_t dps3xx_read_byte(struct Dps3xxDev *dev, uint8_t regAddress)
{
    // mask regAddress
    regAddress &= ~DPS3xx__SPI_RW_MASK;
    // send address with read command and receive register content from Dps3xx
    uint8_t ret;
    spi_read_reg(dev->m_spibus, regAddress | DPS3xx__SPI_READ_CMD, &ret, 1);
    // return received data
    return ret;
}

/**
 * writes a byte to a register of the sensor via SPI
 *
 * @param regAddress:       Address of the register that has to be updated
 * @param data:             Byte that will be written to the register
 * @param check:            If this is true, register content will be read after writing
 *                          to check if update was successful
 * @return  0 if byte was written successfully or
 *          -1 on fail
 */
static int16_t dps3xx_write_byte(struct Dps3xxDev *dev, uint8_t regAddress, uint8_t data, uint8_t check)
{
    // mask regAddress
    regAddress &= ~DPS3xx__SPI_RW_MASK;
    // send address with write command
    spi_write_reg(dev->m_spibus, regAddress | DPS3xx__SPI_WRITE_CMD, data);

    // check if necessary
    if (check == 0)
    {
        // no checking necessary
        return DPS__SUCCEEDED;
    }
    // checking necessary
    if (dps3xx_read_byte(dev, regAddress) == data)
    {
        // check passed
        return DPS__SUCCEEDED;
    }
    else
    {
        // check failed
        return DPS__FAIL_UNKNOWN;
    }
}

/**
 * updates a bit field of the sensor
 *
 * regMask:             Mask of the register that has to be updated
 * data:                BitValues that will be written to the register
 * check:               enables/disables check after writing; 0 disables check.
 *                      if check fails, -1 will be returned
 * @return  0 if byte was written successfully or
 *          -1 on fail
 */
static int16_t dps3xx_write_byte_bitfield(struct Dps3xxDev *dev, uint8_t data, RegMask_t regMask, uint8_t check)
{
    int16_t old = dps3xx_read_byte(dev, regMask.regAddress);
    if (old < 0)
    {
        // fail while reading
        return old;
    }
    return dps3xx_write_byte(dev, regMask.regAddress, ((uint8_t)old & ~regMask.mask) | ((data << regMask.shift) & regMask.mask), check);
}

/**
 * reads a bit field from the sensor
 * regMask:             Mask of the register that has to be updated
 * data:                BitValues that will be written to the register
 * @return  read and processed bits or -1 on fail
 */
static int16_t dps3xx_read_byte_bitfield(struct Dps3xxDev *dev, RegMask_t regMask)
{
    int16_t ret = dps3xx_read_byte(dev, regMask.regAddress);
    if (ret < 0)
    {
        return ret;
    }
    return (((uint8_t)ret) & regMask.mask) >> regMask.shift;
}

/**
 * reads a block from the sensor via SPI
 *
 * @param regAddress:       Address that has to be read
 * @param length:           Length of data block
 * @param buffer:           Buffer where data will be stored
 * @return  number of bytes that have been read successfully, which might not always equal to length due to rx-Buffer overflow etc.
 */
static int16_t dps3xx_read_block(struct Dps3xxDev *dev, RegBlock_t regBlock, uint8_t *buffer)
{
    // do not read if there is no buffer
    if (buffer == NULL)
    {
        return 0; // 0 bytes were read successfully
    }
    // mask regAddress
    regBlock.regAddress &= ~DPS3xx__SPI_RW_MASK;
    // read block from Dps3xx
    spi_read_reg(dev->m_spibus, regBlock.regAddress | DPS3xx__SPI_READ_CMD, buffer, regBlock.length);
    // return received data
    return regBlock.length;
}

/**
 * @brief converts non-32-bit negative numbers to 32-bit negative numbers with 2's complement
 *
 * @param raw The raw number of less than 32 bits
 * @param length The bit length
 */
static void getTwosComplement(int32_t *raw, uint8_t length)
{
    if (*raw & ((uint32_t)1 << (length - 1)))
    {
        *raw -= (uint32_t)1 << length;
    }
}

/**
 * @brief Get a raw result from a given register block
 *
 * @param raw The address where the raw value is to be written
 * @param reg The register block to be read from
 * @return status code
 */
static int16_t dps3xx_get_raw_result(struct Dps3xxDev *dev, int32_t *raw, RegBlock_t reg)
{
    uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};
    if (dps3xx_read_block(dev, reg, buffer) != DPS__RESULT_BLOCK_LENGTH)
        return DPS__FAIL_UNKNOWN;

    *raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    getTwosComplement(raw, 24);
    return DPS__SUCCEEDED;
}

static float dps3xx_calc_temp(struct Dps3xxDev *dev, int32_t raw)
{
    float temp = raw;

    // scale temperature according to scaling table and oversampling
    temp /= dev->scaling_facts[dev->m_tempOsr];

    // update last measured temperature
    // it will be used for pressure compensation
    dev->m_lastTempScal = temp;

    // Calculate compensated temperature
    temp = dev->m_c0Half + dev->m_c1 * temp;

    return temp;
}

static float dps3xx_calc_pressure(struct Dps3xxDev *dev, int32_t raw)
{
    float prs = raw;

    // scale pressure according to scaling table and oversampling
    prs /= dev->scaling_facts[dev->m_prsOsr];

    // Calculate compensated pressure
    prs = dev->m_c00 + prs * (dev->m_c10 + prs * (dev->m_c20 + prs * dev->m_c30)) + dev->m_lastTempScal * (dev->m_c01 + prs * (dev->m_c11 + prs * dev->m_c21));

    // return pressure
    return prs;
}

static int16_t dps3xx_flush_fifo(struct Dps3xxDev *dev)
{
    return dps3xx_write_byte_bitfield(dev, 1U, registers[FIFO_FL], 0U);
}
