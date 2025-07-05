#ifndef DPS3xx_CONFIG_H_
#define DPS3xx_CONFIG_H_
#include "DpsRegister.h"

#define DPS3xx_NUM_OF_REGMASKS 16

enum Interrupt_source_3xx_e
{
    DPS3xx_NO_INTR = 0,
    DPS3xx_PRS_INTR = 1,
    DPS3xx_TEMP_INTR = 2,
    DPS3xx_BOTH_INTR = 3,
    DPS3xx_FIFO_FULL_INTR = 4,
};

enum Registers_e
{
    PROD_ID = 0,
    REV_ID,
    TEMP_SENSOR,    // internal vs external
    TEMP_SENSORREC, // temperature sensor recommendation
    TEMP_SE,        // temperature shift enable (if temp_osr>3)
    PRS_SE,         // pressure shift enable (if prs_osr>3)
    FIFO_FL,        // FIFO flush
    FIFO_EMPTY,     // FIFO empty
    FIFO_FULL,      // FIFO full
    INT_HL,
    INT_SEL, // interrupt select
};

// Externs
extern const RegMask_t registers[DPS3xx_NUM_OF_REGMASKS];
extern const RegBlock_t coeffBlock;

#endif