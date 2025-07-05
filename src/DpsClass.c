#include "DpsClass.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "../../config.h"
#include "../../utils/spi_helper.h"

// Logging
LOG_MODULE_REGISTER(dps3_class, CONFIG_LOCAL_DPS3_CLASS_LOG_LEVEL);

/* Retrieve the SPI API-device structure */
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
struct spi_dt_spec dps3_spi = SPI_DT_SPEC_GET(DPS3_SPI_NODE, SPIOP, 0);

const int32_t DpsClass::scaling_facts[DPS__NUM_OF_SCAL_FACTS] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

Dps3xxDev dps3xx_dev = {
    .scaling_facts = scaling_facts,
    .m_opMode = DPS__IDLE,
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

DpsClass::~DpsClass(void)
{
    end();
}

void dps3xx_begin(struct Dps3xxDev *dps3xx_dev, uint8_t threeWire)
{
    // this flag will show if the initialization was successful
    dps3xx_dev->m_initFail = 0U;

    // Set SPI bus connection
    dps3xx_dev->m_spibus = &dps3_spi;

    // Initialize SPI bus
    if (!device_is_ready(dps3_spi.bus))
    {
        LOG_ERR("SPI initialization failed");
        dps3xx_dev->m_initFail = 1U;
        return;
    }

    k_msleep(50); // startup time of Dps3xx

    // Switch to 3-wire mode if necessary
    // do not use writeByteBitfield or check option to set SPI mode!
    // Reading is not possible until SPI-mode is valid
    if (threeWire)
    {
        dps3xx_dev->m_threeWire = 1U;
        if (dps3xx_write_byte(dps3xx_dev, DPS3xx__REG_ADR_SPI3W, DPS3xx__REG_CONTENT_SPI3W))
        {
            dps3xx_dev->m_initFail = 1U;
            return;
        }
    }

    // TODO:

    init();
}

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
 * @param reg               The FIFO empty register field; needed since this field is different for each sensor
 * @return  status code
 */
static int16_t dps3xx_get_cont_results(struct Dps3xxDev *dps3xx_dev, float *tempBuffer, uint8_t *tempCount, float *prsBuffer, uint8_t *prsCount, RegMask_t reg)
{
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in background mode
    if (!(dps3xx_dev->m_opMode & 0x04))
    {
        return DPS__FAIL_TOOBUSY;
    }

    if (!tempBuffer || !prsBuffer)
    {
        return DPS__FAIL_UNKNOWN;
    }
    tempCount = 0U;
    prsCount = 0U;

    // while FIFO is not empty
    while (dps3xx_read_byte_bitfield(dps3xx_dev, dps3xx_dev->fifo_empty_reg) == 0)
    {
        int32_t raw_result;
        float result;
        // read next result from FIFO
        int16_t type = dps3xx_get_fifo_value(dps3xx_dev, &raw_result);
        switch (type)
        {
        case 0: // temperature
            if (tempCount < DPS__FIFO_SIZE)
            {
                result = calcTemp(raw_result);
                tempBuffer[tempCount++] = result;
            }
            break;
        case 1: // pressure
            if (prsCount < DPS__FIFO_SIZE)
            {
                result = calcPressure(raw_result);
                prsBuffer[prsCount++] = result;
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
int16_t dps3xx_get_single_result(struct Dps3xxDev *dps3xx_dev, float *result)
{
    // abort if initialization failed
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }

    // read finished bit for current opMode
    int16_t rdy;
    switch (dps3xx_dev->m_opMode)
    {
    case CMD_TEMP: // temperature
        rdy = dps3xx_read_byte_bitfield(dps3xx_dev, config_registers[TEMP_RDY]);
        break;
    case CMD_PRS: // pressure
        rdy = dps3xx_read_byte_bitfield(dps3xx_dev, config_registers[PRS_RDY]);
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
        Mode oldMode = dps3xx_dev->m_opMode;
        m_opMode = IDLE; // opcode was automatically reset by DPS3xx
        int32_t raw_val;
        switch (oldMode)
        {
        case CMD_TEMP: // temperature
            dps3xx_get_raw_result(dps3xx_dev, &raw_val, registerBlocks[TEMP]);
            result = calcTemp(raw_val);
            return DPS__SUCCEEDED; // TODO
        case CMD_PRS:              // pressure
            dps3xx_get_raw_result(dps3xx_dev, &raw_val, registerBlocks[PRS]);
            result = calcPressure(raw_val);
            return DPS__SUCCEEDED; // TODO
        default:
            return DPS__FAIL_UNKNOWN; // should already be filtered above
        }
    }
    return DPS__FAIL_UNKNOWN;
}

int16_t dps3xx_measure_temp_once(struct Dps3xxDev *dps3xx_dev, float *results);
{
    return dps3xx_measure_temp_once(dps3xx_dev, result, dps3xx_dev->m_tempOsr);
}

int16_t dps3xx_measure_temp_once(struct Dps3xxDev *dps3xx_dev, float *result, uint8_t oversamplingRate)
{
    // Start measurement
    int16_t ret = dps3xx_start_measure_temp_once(dps3xx_dev, oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }

    // wait until measurement is finished
    k_msleep(dps3xx_calc_busy_time(0U, dps3xx_dev->m_tempOsr) / DPS__BUSYTIME_SCALING);
    k_msleep(DPS3xx__BUSYTIME_FAILSAFE);

    ret = dps3xx_get_single_result(dps3xx_dev, result);
    if (ret != DPS__SUCCEEDED)
    {
        dps3xx_standby(dps3xx_dev);
    }
    return ret;
}

int16_t dps3xx_start_measure_temp_once(struct Dps3xxDev *dps3xx_dev)
{
    return dps3xx_start_measure_temp_once(dps3xx_dev, dps3xx_dev->m_tempOsr);
}

int16_t dps3xx_start_measure_temp_once(struct Dps3xxDev *dps3xx_dev, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dps3xx_dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }

    if (dps3xx_dev->m_oversamplingRate != dps3xx_dev->m_tempOsr)
    {
        // configuration of oversampling rate
        if (dps3xx_config_temp(dps3xx_dev, 0U, oversamplingRate) != DPS__SUCCEEDED)
        {
            return DPS__FAIL_UNKNOWN;
        }
    }

    // set device to temperature measuring mode
    return dps3xx_set_op_mode(dps3xx_dev, CMD_TEMP);
}

int16_t dps3xx_measure_pressure_once(struct Dps3xxDev *dps3xx_dev, float *result)
{
    return dps3xx_measure_pressure_once(dps3xx_dev, result, dps3xx_dev->m_prsOsr);
}

int16_t dps3xx_measure_pressure_once(struct Dps3xxDev *dps3xx_dev, float *result, uint8_t oversamplingRate)
{
    // start the measurement
    int16_t ret = dps3xx_start_measure_pressure_once(oversamplingRate);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }

    // wait until measurement is finished
    k_msleep(dps3xx_calc_busy_time(0U, dps3xx_dev->m_prsOsr) / DPS__BUSYTIME_SCALING);
    k_msleep(DPS3xx__BUSYTIME_FAILSAFE);

    ret = dps3xx_get_single_result(dps3xx_dev, result);
    if (ret != DPS__SUCCEEDED)
    {
        dps3xx_standby(dps3xx_dev);
    }
    return ret;
}

int16_t dps3xx_start_measure_pressure_once(struct Dps3xxDev *dps3xx_dev)
{
    return dps3xx_start_measure_pressure_once(dps3xx_dev, dps3xx_dev->m_prsOsr);
}

int16_t dps3xx_start_measure_pressure_once(struct Dps3xxDev *dps3xx_dev, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dps3xx_dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // configuration of oversampling rate, lowest measure rate to avoid conflicts
    if (oversamplingRate != dps3xx_dev->m_prsOsr)
    {
        if (dps3xx_config_pressure(dps3xx_dev, 0U, oversamplingRate))
        {
            return DPS__FAIL_UNKNOWN;
        }
    }
    // set device to pressure measuring mode
    return dps3xx_set_op_mode(dps3xx_dev, CMD_PRS);
}

int16_t dps3xx_start_measure_temp_cont(struct Dps3xxDev *dps3xx_dev, uint8_t measureRate, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dps3xx_dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (dps3xx_calc_busy_time(measureRate, oversamplingRate) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (dps3xx_config_temp(dps3xx_dev, measureRate, oversamplingRate))
    {
        return DPS__FAIL_UNKNOWN;
    }

    if (dps3xx_enable_fifo(dps3xx_dev))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (dps3xx_set_op_mode(dps3xx_dev, CONT_TMP))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t dps3xx_start_measure_pressure_cont(struct Dps3xxDev *dps3xx_dev, uint8_t measureRate, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dps3xx_dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (dps3xx_calc_busy_time(measureRate, oversamplingRate) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (dps3xx_config_pressure(dps3xx_dev, measureRate, oversamplingRate))
        return DPS__FAIL_UNKNOWN;
    // enable result FIFO
    if (dps3xx_enable_fifo(dps3xx_dev))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (dps3xx_set_op_mode(dps3xx_dev, CONT_PRS))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t dps3xx_start_measure_both_cont(struct Dps3xxDev *dps3xx_dev, uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr)
{
    // abort if initialization failed
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (dps3xx_dev->m_opMode != IDLE)
    {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (dps3xx_calc_busy_time(tempMr, tempOsr) + dps3xx_calc_busy_time(prsMr, prsOsr) >= DPS3xx__MAX_BUSYTIME)
    {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (dps3xx_config_temp(dps3xx_dev, tempMr, tempOsr))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // update precision and measuring rate
    if (dps3xx_config_pressure(dps3xx_dev, prsMr, prsOsr))
        return DPS__FAIL_UNKNOWN;
    // enable result FIFO
    if (dps3xx_enable_fifo(dps3xx_dev))
    {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (dps3xx_set_op_mode(dps3xx_dev, CONT_BOTH))
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t dps3xx_standby(struct Dps3xxDev *dps3xx_dev)
{
    // abort if initialization failed
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    // set device to idling mode
    int16_t ret = dps3xx_set_op_mode(dps3xx_dev, IDLE);
    if (ret != DPS__SUCCEEDED)
    {
        return ret;
    }
    ret = dps3xx_disable_fifo(dps3xx_dev);
    return ret;
}

int16_t dps3xx_correct_temp(struct Dps3xxDev *dps3xx_dev)
{
    if (dps3xx_dev->m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }
    dps3xx_write_byte(dps3xx_dev, 0x0E, 0xA5);
    dps3xx_write_byte(dps3xx_dev, 0x0F, 0x96);
    dps3xx_write_byte(dps3xx_dev, 0x62, 0x02);
    dps3xx_write_byte(dps3xx_dev, 0x0E, 0x00);
    dps3xx_write_byte(dps3xx_dev, 0x0F, 0x00);

    // perform a first temperature measurement (again)
    // the most recent temperature will be saved internally
    // and used for compensation when calculating pressure
    float trash;
    dps3xx_measure_temp_once(trash);

    return DPS__SUCCEEDED;
}

int16_t dps3xx_get_int_status_fifo_full(struct Dps3xxDev *dps3xx_dev)
{
    return dps3xx_read_byte_bitfield(dps3xx_dev, config_registers[INT_FLAG_FIFO]);
}

int16_t dps3xx_get_int_status_temp_ready(struct Dps3xxDev *dps3xx_dev)
{
    return dps3xx_read_byte_bitfield(dps3xx_dev, config_registers[INT_FLAG_TEMP]);
}

int16_t dps3xx_get_int_status_prs_ready(struct Dps3xxDev *dps3xx_dev)
{
    return dps3xx_read_byte_bitfield(dps3xx_dev, config_registers[INT_FLAG_PRS]);
}

/**
 * Sets the Operation Mode of the sensor
 *
 * @param dps3xx_dev:       reference to a Dps3xxDev structure
 * @param opMode:           the new OpMode as defined by Mode; CMD_BOTH should not be used for DPS3xx
 * @return                  0 on success,
 *                          -1 on fail
 */
static int16_t dps3xx_set_op_mode(struct Dps3xxDev *dps3xx_dev, uint8_t opMode)
{
    if (dps3xx_write_byte_bitfield(dps3xx_dev, opMode, config_registers[MSR_CTRL]) == -1)
    {
        return DPS__FAIL_UNKNOWN;
    }
    dps3xx_dev->m_opMode = (Mode)opMode;
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
static int16_t dps3xx_config_temp(struct Dps3xxDev *dps3xx_dev, uint8_t tempMr, uint8_t tempOsr)
{
    tempMr &= 0x07;
    tempOsr &= 0x07;
    // two accesses to the same register; for readability
    int16_t ret = dps3xx_write_byte_bitfield(dps3xx_dev, tempMr, config_registers[TEMP_MR]);
    ret = dps3xx_write_byte_bitfield(dps3xx_dev, tempOsr, config_registers[TEMP_OSR]);

    // abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    dps3xx_dev->m_tempMr = tempMr;
    dps3xx_dev->m_tempOsr = tempOsr;
    return DPS__SUCCEEDED;
}

/**
 * Configures pressure measurement
 *
 * @param prsMr:           DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
 * @param prsOsr:          DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
 * @return 	                0 normally or
 *                          -1 on fail
 */
static int16_t dps3xx_config_pressure(struct Dps3xxDev *dps3xx_dev, uint8_t prsMr, uint8_t prsOsr)
{
    prsMr &= 0x07;
    prsOsr &= 0x07;
    int16_t ret = dps3xx_write_byte_bitfield(dps3xx_dev, prsMr, config_registers[PRS_MR]);
    ret = dps3xx_write_byte_bitfield(dps3xx_dev, prsOsr, config_registers[PRS_OSR]);

    // abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    dps3xx_dev->m_prsMr = prsMr;
    dps3xx_dev->m_prsOsr = prsOsr;
    return DPS__SUCCEEDED;
}

static int16_t dps3xx_enable_fifo(struct Dps3xxDev *dps3xx_dev)
{
    return dps3xx_write_byte_bitfield(dps3xx_dev, 1U, config_registers[FIFO_EN]);
}

static int16_t dps3xx_disable_fifo(struct Dps3xxDev *dps3xx_dev)
{
    int16_t ret = flushFIFO();
    ret = dps3xx_write_byte_bitfield(dps3xx_dev, 0U, config_registers[FIFO_EN]);
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
static int16_t dps3xx_get_fifo_value(struct Dps3xxDev *dps3xx_dev, int32_t *value)
{
    uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};

    // abort on invalid argument or failed block reading
    if (value == NULL || dps3xx_read_block(dps3xx_dev, registerBlocks[PRS], buffer) != DPS__RESULT_BLOCK_LENGTH)
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
static int16_t dps3xx_read_byte(struct Dps3xxDev *dps3xx_dev, uint8_t regAddress)
{
    // mask regAddress
    regAddress &= ~DPS3xx__SPI_RW_MASK;
    // send address with read command and receive register content from Dps3xx
    uint8_t ret;
    spi_read_reg(dps3xx_dev->m_spibus, regAddress | DPS3xx__SPI_READ_CMD, &ret, 1);
    // return received data
    return ret;
}

/**
 * writes a byte to a given register of the sensor without checking
 *
 * @param regAddress:       Address of the register that has to be updated
 * @param data:             Byte that will be written to the register
 * @return  0 if byte was written successfully or
 *          -1 on fail
 */
static int16_t dps3xx_write_byte(struct Dps3xxDev *dps3xx_dev, uint8_t regAddress, uint8_t data)
{
    return dps3xx_write_byte(dps3xx_dev, regAddress, data, 0U);
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
static int16_t dps3xx_write_byte(struct Dps3xxDev *dps3xx_dev, uint8_t regAddress, uint8_t data, uint8_t check)
{
    // mask regAddress
    regAddress &= ~DPS3xx__SPI_RW_MASK;
    // send address with write command
    spi_write_reg(dps3xx_dev->m_spibus, regAddress | DPS3xx__SPI_WRITE_CMD, &data, 1);

    // check if necessary
    if (check == 0)
    {
        // no checking necessary
        return DPS__SUCCEEDED;
    }
    // checking necessary
    if (dps3xx_read_byte(dps3xx_dev, regAddress) == data)
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
 * updates a bit field of the sensor without checking
 *
 * @param regMask:          Mask of the register that has to be updated
 * @param data:             BitValues that will be written to the register
 * @return  0 if byte was written successfully or
 *          -1 on fail
 */
static int16_t dps3xx_write_byte_bitfield(struct Dps3xxDev *dps3xx_dev, uint8_t data, RegMask_t regMask)
{
    return dps3xx_write_byte_bitfield(dps3xx_dev, data, regMask.regAddress, regMask.mask, regMask.shift, 0U);
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
static int16_t dps3xx_write_byte_bitfield(struct Dps3xxDev *dps3xx_dev, uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift, uint8_t check)
{
    int16_t old = dps3xx_read_byte(dps3xx_dev, regAddress);
    if (old < 0)
    {
        // fail while reading
        return old;
    }
    return dps3xx_write_byte(dps3xx_dev, regAddress, ((uint8_t)old & ~mask) | ((data << shift) & mask), check);
}

/**
 * reads a bit field from the sensor
 * regMask:             Mask of the register that has to be updated
 * data:                BitValues that will be written to the register
 * @return  read and processed bits or -1 on fail
 */
static int16_t dps3xx_read_byte_bitfield(struct Dps3xxDev *dps3xx_dev, RegMask_t regMask)
{
    int16_t ret = dps3xx_read_byte(dps3xx_dev, regMask.regAddress);
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
static int16_t dps3xx_read_block(struct Dps3xxDev *dps3xx_dev, RegBlock_t regBlock, uint8_t *buffer)
{
    // do not read if there is no buffer
    if (buffer == NULL)
    {
        return 0; // 0 bytes were read successfully
    }
    // mask regAddress
    regBlock.regAddress &= ~DPS3xx__SPI_RW_MASK;
    // read block from Dps3xx
    spi_read_reg(dps3xx_dev->m_spibus, regBlock.regAddress | DPS3xx__SPI_READ_CMD, buffer, regBlock.length);
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
static int16_t dps3xx_get_raw_result(struct Dps3xxDev *dps3xx_dev, int32_t *raw, RegBlock_t reg)
{
    uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};
    if (dps3xx_read_block(dps3xx_dev, reg, buffer) != DPS__RESULT_BLOCK_LENGTH)
        return DPS__FAIL_UNKNOWN;

    *raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    getTwosComplement(raw, 24);
    return DPS__SUCCEEDED;
}
