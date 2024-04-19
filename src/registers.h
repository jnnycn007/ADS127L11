/**
 * @file registers.h
 * @author Wellington Rodrigo Gallo (wrgallo@hotmail.com)
 * @brief ADS127L11 Registers Information retrieved from Datasheet 'ADS127L11 – APRIL 2021 – REVISED SEPTEMBER 2022'
 * @version 0.1
 * @date 2024-Jan-04
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//**********************************************************************************
//
// Device commands
//
//**********************************************************************************

#define ADS127L11_OPCODE_NULL                             ((uint8_t) 0x00)
#define ADS127L11_OPCODE_RREG                             ((uint8_t) 0x40)
#define ADS127L11_OPCODE_WREG                             ((uint8_t) 0x80)

//**********************************************************************************
//
// Constants
//
//**********************************************************************************

#define ADS127L11_NUM_REGISTERS                           ((uint8_t) 16)

//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (DEVID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           DEVID[7:0]                                                          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                              R-00h                                                            |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** DEVID_REG register 
     * Device ID.
     * 00h = ADS127L11
    */
    typedef uint8_t ads127l11_devid_reg_t;
    #define ADS127L11_DEVID_REG_ADDRESS                                                ((uint8_t) 0x00)
    #ifndef ADS127L11_DEVID_REG_DEFAULT
        #define ADS127L11_DEVID_REG_DEFAULT                                            ((uint8_t) 0x00)
    #endif



/* Register 0x01 (REVID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           REVID[7:0]                                                          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           R-xxxxxxxxb                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** REVID_REG register 
     * Die revision ID
     * The die revision ID can change during device production without notice. */
    typedef uint8_t ads127l11_revid_reg_t;
    #define ADS127L11_REVID_REG_ADDRESS                                                ((uint8_t) 0x01)
    #ifndef ADS127L11_REVID_REG_DEFAULT
        #define ADS127L11_REVID_REG_DEFAULT                                            ((uint8_t) 0x00)
    #endif



/* Register 0x02 (STATUS_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    CS_MODE    |    ALV_FLAG   |    POR_FLAG   |    SPI_ERR    |    REG_ERR    |     ADC_ERR   |   MOD_FLAG    |      DRDY     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     R-xb      |     R/W-1b    |     R/W-1b    |    R/W-0b     |    R/W-0b     |     R-xb      |     R-xb      |     R-xb      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
    /** STATUS_REG register */
    typedef union _ads127l11_status_reg_t
    {
        struct
        {
            /** Data-ready bit. 
             * DRDY indicates when new conversion data are ready. 
             * The DRDY bit is the inverse of the DRDY pin. 
             * Poll the bit to determine if conversion data are new or are repeated data from the last read operation. 
             * DRDY = 1 indicates data are new. In one-shot control mode, the bit remains at 1b until a new conversion is started.
             * 0b = Data are not new
             * 1b = Data are new */
            uint8_t drdy      : 1;

            /** Modulator saturation flag.
             * This bit indicates a transient or continuous modulator saturation occurred during the conversion cycle. 
             * The flag is updated at the completion of each conversion.
             * 0b = Modulator not saturated
             * 1b = Modulator saturation detected during the conversion cycle */
            uint8_t mod_flag  : 1;

            /** Internal ADC error.
             * ADC_ERR indicates an internal error. Perform a power cycle or reset the device.
             * 0b = No ADC error
             * 1b = ADC error detected */
            uint8_t adc_err   : 1;

            /** Register map CRC error.
             * REG_ERR indicates if the written register map CRC (0Fh) value 
             * does not match the internal ADC calculated value. 
             * Write 1b to clear the register map CRC error or clear the REG_CRC bit. 
             * Set the REG_CRC bit (CONFIG4 register) to enable the register map error check.
             * 0b = No error
             * 1b = Register map CRC error */
            uint8_t reg_err   : 1;

            /** SPI communication CRC error.
             * This bit indicates an SPI CRC error. 
             * If set, register write operations are blocked, 
             * except for the STATUS register that allows clearing the error (write 1b to clear the error). 
             * Register read operations remain functional. 
             * The SPI CRC error detection is enabled by the SPI_CRC bit (CONFIG4 register).
             * 0b = No error
             * 1b = SPI CRC error */
            uint8_t spi_err   : 1;

            /** Power-on reset (POR) flag.
             * This bit indicates a reset from device power-on, by a brownout of the IOVDD supply, 
             * CAPD bypass output, or by a user-initiated reset. 
             * Write 1b to clear the flag to detect the next reset.
             * 0b = No reset from when the flag last cleared
             * 1b = Device reset occurred */
            uint8_t por_flag  : 1;

            /** Analog supply low-voltage flag.
             * This bit indicates a low-voltage condition occurred on the analog power supplies. 
             * Write 1b to clear the flag to detect the next low-voltage condition.
             * 0b = No analog supply low-voltage condition from when flag last cleared
             * 1b = Analog supply low-voltage condition detected */
            uint8_t alv_flag  : 1;

            /** CS mode.
             * This bit indicates 4-wire or 3-wire SPI mode. The mode is 
             * determined by the state of CS at power up or after reset.
             * 0b = 4-wire SPI operation (CS is active)
             * 1b = 3-wire SPI operation (CS is tied low) */
            uint8_t cs_mode   : 1;
        };
        uint8_t u8; //!< register value
    } ads127l11_status_reg_t;
    #define ADS127L11_STATUS_REG_ADDRESS                                            ((uint8_t) 0x02)
    #ifndef ADS127L11_STATUS_REG_DEFAULT
        #define ADS127L11_STATUS_REG_DEFAULT                                            ((uint8_t) 0x00)
    #endif

    /* CS Mode Field */
    #define ADS127L11_STATUS_REG_CS_MODE_MASK                                       ((uint8_t) 0x80)
    #define ADS127L11_STATUS_REG_CS_MODE_4WIRE_SPI                                  ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_CS_MODE_3WIRE_SPI                                  ((uint8_t) 0x80)

    /* Analog Supply Low Voltage Field */
    #define ADS127L11_STATUS_REG_ALV_FLAG_MASK                                      ((uint8_t) 0x40)
    #define ADS127L11_STATUS_REG_ALV_FLAG_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_ALV_FLAG_1                                         ((uint8_t) 0x40)

    /* POR Flag Field */
    #define ADS127L11_STATUS_REG_POR_FLAG_MASK                                      ((uint8_t) 0x20)
    #define ADS127L11_STATUS_REG_POR_FLAG_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_POR_FLAG_1                                         ((uint8_t) 0x20)

    /* SPI CRC Error Field */
    #define ADS127L11_STATUS_REG_SPI_ERR_MASK                                       ((uint8_t) 0x10)
    #define ADS127L11_STATUS_REG_SPI_ERR_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_SPI_ERR_1                                          ((uint8_t) 0x10)

    /* Register Map CRC Error Field */
    #define ADS127L11_STATUS_REG_REG_ERR_MASK                                       ((uint8_t) 0x08)
    #define ADS127L11_STATUS_REG_REG_ERR_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_REG_ERR_1                                          ((uint8_t) 0x08)

    /* Internal ADC Error Field */
    #define ADS127L11_STATUS_REG_ADC_ERR_MASK                                       ((uint8_t) 0x04)
    #define ADS127L11_STATUS_REG_ADC_ERR_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_ADC_ERR_1                                          ((uint8_t) 0x04)

    /* Modulator Overload Flag */
    #define ADS127L11_STATUS_REG_MOD_FLAG_MASK                                      ((uint8_t) 0x02)
    #define ADS127L11_STATUS_REG_MOD_FLAG_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_MOD_FLAG_1                                         ((uint8_t) 0x02)

    /* Data Ready Bit */
    #define ADS127L11_STATUS_REG_DRDY_MASK                                          ((uint8_t) 0x01)
    #define ADS127L11_STATUS_REG_DRDY_0                                             ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_STATUS_REG_DRDY_1                                             ((uint8_t) 0x01)



/* Register 0x03 (CONTROL_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                           RESET[5:0]                                          |     START     |      STOP     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                            W-000000b                                          |     W-0b      |     W-0b      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** CONTROL_REG register */
    typedef union _ads127l11_control_reg_t
    {
        struct 
        {
            /** Stop conversion.
             * This bit stops conversions after the current conversion completes. 
             * This bit has no effect in synchronized control mode. 
             * Writing 1b to both the START and STOP has no effect. 
             * STOP is self-clearing and always reads 0b.
             * 0b = No operation
             * 1b = Stop conversion after the current conversion completes */
            uint8_t stop  : 1;

            /** Start conversion.
             * Conversions are started or restarted by writing 1b. 
             * Preclear the CONTROL register by writing 00h prior to writing the START bit. 
             * In one-shot control mode, one conversion is started. 
             * In start/stop control mode, conversions are started and continue until stopped by the STOP bit. 
             * Writing 1b to START while a conversion is ongoing restarts the conversion. 
             * This bit has no effect in synchronized control mode. 
             * Writing 1b to both the START and STOP bits has no effect. 
             * START is self-clearing and always reads 0b.
             * 0b = No operation
             * 1b = Start or restart conversion */
            uint8_t start : 1;

            /** Device reset.
             * Write 010110b to reset the ADC. 
             * The adjacent START and STOP bits must be set to 00b in the same write operation to reset the ADC. 
             * These bits always read 000000b. */
            uint8_t reset : 6;
        };
        uint8_t u8; //!< Register value
    } ads127l11_control_reg_t;
    #define ADS127L11_CONTROL_REG_ADDRESS                                            ((uint8_t) 0x03)
    #ifndef ADS127L11_CONTROL_REG_DEFAULT
        #define ADS127L11_CONTROL_REG_DEFAULT                                            ((uint8_t) 0x00)
    #endif

    /* RESET field */
    #define ADS127L11_CONTROL_REG_RESET_MASK                                         ((uint8_t) 0xFC)
    #define ADS127L11_CONTROL_REG_RESET_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONTROL_REG_RESET_1                                            ((uint8_t) 0x04)
    #define ADS127L11_CONTROL_REG_RESET_COMMAND                                      ((uint8_t) 0x58)

    /* START field */
    #define ADS127L11_CONTROL_REG_START_MASK                                         ((uint8_t) 0x02)
    #define ADS127L11_CONTROL_REG_START_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONTROL_REG_START_1                                            ((uint8_t) 0x02)

    /* STOP field */
    #define ADS127L11_CONTROL_REG_STOP_MASK                                          ((uint8_t) 0x01)
    #define ADS127L11_CONTROL_REG_STOP_0                                             ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONTROL_REG_STOP_1                                             ((uint8_t) 0x01)



/* Register 0x04 (MUX_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                           RESERVED                                            |             MUX[1:0]          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                            R-000000b                                          |             R/W-00b           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** MUX_REG register */
    typedef union _ads127l11_mux_reg_t
    {
        struct
        {
            /** Input multiplexer selection.
             * These bits select the polarity of the analog input and selects the test modes. 
             * See the Analog Input section for details.
             * 00b = Normal input polarity
             * 01b = Inverted input polarity
             * 10b = Offset and noise test: AINP and AINN disconnected, ADC inputs internally shorted to (AVDD1 + AVSS) / 2
             * 11b = Common-mode test: ADC inputs internally shorted and connected to AINP */
            uint8_t mux : 2;

            /** Reserved */
            uint8_t     : 6;
        };
        uint8_t u8; //!< Register value
    } ads127l11_mux_reg_t;
    #define ADS127L11_MUX_REG_ADDRESS                                                  ((uint8_t) 0x04)
    #ifndef ADS127L11_MUX_REG_DEFAULT
        #define ADS127L11_MUX_REG_DEFAULT                                                  ((uint8_t) 0x00)
    #endif

    /* MUX field */
    #define ADS127L11_MUX_REG_MUX_MASK                                                 ((uint8_t) 0x03)
    #define ADS127L11_MUX_REG_MUX_NORMAL                                               ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_MUX_REG_MUX_INVERTED                                             ((uint8_t) 0x01)
    #define ADS127L11_MUX_REG_MUX_OFFSET_TEST                                          ((uint8_t) 0x02)
    #define ADS127L11_MUX_REG_MUX_CM_TEST                                              ((uint8_t) 0x03)



/* Register 0x05 (CONFIG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |    REF_RNG    |    INP_RNG    |    VCM        |    REFP_BUF   |    RESERVED   |    AINP_BUF   |    AINN_BUFF  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     R-0b      |     R/W-0b    |     R/W-0b    |    R/W-0b     |    R/W-0b     |     R-0b      |    R/W-0b     |    R/W-0b     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** CONFIG1 register */
    typedef union _ads127l11_config1_reg_t
    {
        struct
        {
            /** Analog input negative buffer enable.
             * This bit enables the AINN analog input precharge buffer.
             * 0b = Disabled
             * 1b = Enabled */
            uint8_t ainn_buff : 1;

            /** Analog input positive buffer enable.
             * This bit enables the AINP analog input precharge buffer.
             * 0b = Disabled
             * 1b = Enabled */
            uint8_t ainp_buff : 1;
            
            /** Reserved */
            uint8_t           : 1;

            /** Reference positive buffer enable.
             * This bit enables the REFP reference input precharge buffer.
             * 0b = Disabled
             * 1b = Enabled */
            uint8_t refp_buf  : 1;

            /** VCM output enable.
             * This bit enables the VCM output voltage pin. 
             * The VCM voltage is (AVDD1 + AVSS) / 2.
             * 0b = Disabled
             * 1b = Enabled */
            uint8_t vcm       : 1;

            /** Input range selection.
             * This bit selects the 1x or 2x input range. 
             * See the Input Range section for more details.
             * 0b = 1x input range
             * 1b = 2x input range */
            uint8_t inp_rng   : 1;

            /** Voltage reference range selection.
             * Program this bit to select the low- or high-reference voltage range to match the applied reference voltage. 
             * See the Recommended Operating Conditions table for the range of reference voltages. 
             * When the high-reference range is selected, the INP_RNG bit is internally overridden to the 1x input range.
             * 0b = Low-reference range
             * 1b = High-reference range */
            uint8_t ref_rng   : 1;

            /** Reserved */
            uint8_t           : 1;
        };
        uint8_t u8; //!< Register value
    } ads127l11_config1_reg_t;
    #define ADS127L11_CONFIG1_REG_ADDRESS                                             ((uint8_t) 0x05)
    #ifndef ADS127L11_CONFIG1_REG_DEFAULT
        #define ADS127L11_CONFIG1_REG_DEFAULT                                             ((uint8_t) 0x00)
    #endif

    /* Voltage Reference Range field */
    #define ADS127L11_CONFIG1_REG_REF_RNG_MASK                                        ((uint8_t) 0x40)
    #define ADS127L11_CONFIG1_REG_REF_RNG_LOW                                         ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG1_REG_REF_RNG_HIGH                                        ((uint8_t) 0x40)

    /* Input Range field */
    #define ADS127L11_CONFIG1_REG_INP_RNG_MASK                                        ((uint8_t) 0x20)
    #define ADS127L11_CONFIG1_REG_INP_RNG_1X                                          ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG1_REG_INP_RNG_2X                                          ((uint8_t) 0x20)

    /* VCM Output field */
    #define ADS127L11_CONFIG1_REG_VCM_MASK                                            ((uint8_t) 0x10)
    #define ADS127L11_CONFIG1_REG_VCM_DISABLED                                        ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG1_REG_VCM_ENABLED                                         ((uint8_t) 0x10)

    /* Reference Positive Buffer field */
    #define ADS127L11_CONFIG1_REG_REFP_BUFF_MASK                                      ((uint8_t) 0x08)
    #define ADS127L11_CONFIG1_REG_REFP_BUFF_DISABLED                                  ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG1_REG_REFP_BUFF_ENABLED                                   ((uint8_t) 0x08)

    /* Analog Input Positive Buffer field */
    #define ADS127L11_CONFIG1_REG_AINP_BUF_MASK                                       ((uint8_t) 0x02)
    #define ADS127L11_CONFIG1_REG_AINP_BUF_DISABLED                                   ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG1_REG_AINP_BUF_ENABLED                                    ((uint8_t) 0x02)

    /* Analog Input Negative Buffer field */
    #define ADS127L11_CONFIG1_REG_AINN_BUF_MASK                                       ((uint8_t) 0x01)
    #define ADS127L11_CONFIG1_REG_AINN_BUF_DISABLED                                   ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG1_REG_AINN_BUF_ENABLED                                    ((uint8_t) 0x01)


/* Register 0x06 (CONFIG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    EXT_RNG    |    RESERVED   |    SDO_MODE   |        START_MODE[1:0]        |   SPEED_MODE  |    STBY_MODE  |     PWDN      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    R/W-0b     |      R-0b     |     R/W-0b    |           R/W-00b             |    R/W-0b     |    R/W-0b     |    R/W-0b     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** CONFIG2 register */
    typedef union _ads127l11_config2_reg_t
    {
        struct
        {
            /** Power-down mode selection.
             * When set, the ADC is powered down. 
             * All functions are powered down except for SPI operation and the digital LDO to retain user register settings.
             * 0b = Normal operation
             * 1b = Power-down mode */
            uint8_t pwdn       : 1;

            /** Standby mode selection.
             * This bit enables the auto engagement of the low-power standby mode after conversions are stopped.
             * 0b = Idle mode; ADC remains fully powered when conversions are stopped.
             * 1b = Standby mode; ADC powers down when conversions are stopped. Standby mode is exited when conversions restart. */
            uint8_t stby_mode  : 1;

            /** Speed mode selection.
             * This bit programs the power-scalable speed mode of the device. 
             * The clock frequency corresponds to the mode.
             * 0b = High-speed mode (fCLK = 25.6 MHz)
             * 1b = Low-speed mode (fCLK = 3.2 MHz) */
            uint8_t speed_mode : 1;

            /** START mode selection.
             * These bits program the mode of the START pin. 
             * See the Synchronization section for more details.
             * 00b = Start/stop control mode
             * 01b = One-shot control mode
             * 10b = Synchronized control mode
             * 11b = Reserved */
            uint8_t start_mode : 2;

            /** SDO/DRDY mode selection.
             * This bit programs the mode of the SDO/DRDY pin to either data-output function only, 
             * or to dual-mode function of data output and data ready. 
             * For daisy-chain connection of ADCs, use the data-output function only mode. 
             * See the SDO/DRDY section for more details.
             * 0b = Data output only mode
             * 1b = Dual mode: data output and data ready */
            uint8_t sdo_mode   : 1;

            /** Reserved */
            uint8_t            : 1;

            /** Extended input range selection.
             * This bit extends the input range by 25%. 
             * See the Input Range section for more details.
             * 0b = Standard input range
             * 1b = 25% extended input range */
            uint8_t ext_rng    : 1;
        };
        uint8_t u8; //!< Register value
    } ads127l11_config2_reg_t;
    #define ADS127L11_CONFIG2_REG_ADDRESS                                             ((uint8_t) 0x06)
    #ifndef ADS127L11_CONFIG2_REG_DEFAULT
        #define ADS127L11_CONFIG2_REG_DEFAULT                                             ((uint8_t) 0x00)
    #endif

    /* Extended Input Range field */
    #define ADS127L11_CONFIG2_REG_EXT_RNG_MASK                                        ((uint8_t) 0x80)
    #define ADS127L11_CONFIG2_REG_EXT_RNG_DISABLED                                    ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG2_REG_EXT_RNG_25_PCT                                      ((uint8_t) 0x80)

    /* SDO/nDRDY Mode field */
    #define ADS127L11_CONFIG2_REG_SDO_MODE_MASK                                       ((uint8_t) 0x20)
    #define ADS127L11_CONFIG2_REG_SDO_MODE_OUT_ONLY                                   ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG2_REG_SDO_MODE_DUAL_MODE                                  ((uint8_t) 0x20)

    /* START Mode field */
    #define ADS127L11_CONFIG2_START_MODE_MASK                                         ((uint8_t) 0x18)
    #define ADS127L11_CONFIG2_START_MODE_START_STOP                                   ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG2_START_MODE_ONESHOT                                      ((uint8_t) 0x08)
    #define ADS127L11_CONFIG2_START_MODE_SYNC                                         ((uint8_t) 0x10)
    #define ADS127L11_CONFIG2_START_MODE_RESERVED                                     ((uint8_t) 0x18)

    /* Speed Mode field */
    #define ADS127L11_CONFIG2_REG_SPEED_MODE_MASK                                     ((uint8_t) 0x04)
    #define ADS127L11_CONFIG2_REG_SPEED_MODE_HIGH                                     ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG2_REG_SPEED_MODE_LOW                                      ((uint8_t) 0x04)

    /* Standby Mode field */
    #define ADS127L11_CONFIG2_REG_STBY_MODE_MASK                                      ((uint8_t) 0x02)
    #define ADS127L11_CONFIG2_REG_STBY_MODE_IDLE                                      ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG2_REG_STBY_MODE_STDBY                                     ((uint8_t) 0x02)

    /* Software Power Down field */
    #define ADS127L11_CONFIG2_REG_PWDN_MASK                                           ((uint8_t) 0x01)
    #define ADS127L11_CONFIG2_REG_PWDN_NORMAL                                         ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG2_REG_PWDN_POWER_DOWN                                     ((uint8_t) 0x01)

/* Register 0x07 (CONFIG3_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  DELAY[2:0]                   |                            FILTER[4:0]                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   R/W-000b                    |                             R/W-00000b                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** CONFIG3 register */
    typedef union _ads127l11_config3_reg_t
    {
        struct
        {
            /** Digital filter mode and oversampling ratio selection.
             * These bits configure the digital filter. The digital filter has five modes: 
             * wideband, sinc4, sinc4 + sinc1, sinc3, and sinc3 + sinc1, each with a range of OSR values. 
             * See Table 7-1 through Table 7-5 for data rate and bandwidth information.
             * 00000 = wideband, OSR = 32
             * 00001 = wideband, OSR = 64
             * 00010 = wideband, OSR = 128
             * 00011 = wideband, OSR = 256
             * 00100 = wideband, OSR = 512
             * 00101 = wideband, OSR = 1024
             * 00110 = wideband, OSR = 2048
             * 00111 = wideband, OSR = 4096
             * 01000 = sinc4, OSR = 12
             * 01001 = sinc4, OSR = 16
             * 01010 = sinc4, OSR = 24
             * 01011 = sinc4, OSR = 32
             * 01100 = sinc4, OSR = 64
             * 01101 = sinc4, OSR = 128
             * 01110 = sinc4, OSR = 256
             * 01111 = sinc4, OSR = 512
             * 10000 = sinc4, OSR = 1024
             * 10001 = sinc4, OSR = 2048
             * 10010 = sinc4, OSR = 4096
             * 10011 = sinc4, OSR = 32 + sinc1, OSR = 2
             * 10100 = sinc4, OSR = 32 + sinc1, OSR = 4
             * 10101 = sinc4, OSR = 32 + sinc1, OSR = 10
             * 10110 = sinc4, OSR = 32 + sinc1, OSR = 20
             * 10111 = sinc4, OSR = 32 + sinc1, OSR = 40
             * 11000 = sinc4, OSR = 32 + sinc1, OSR = 100
             * 11001 = sinc4, OSR = 32 + sinc1, OSR = 200
             * 11010 = sinc4, OSR = 32 + sinc1, OSR = 400
             * 11011 = sinc4, OSR = 32 + sinc1, OSR = 1000
             * 11100 = sinc3, OSR = 26667
             * 11101 = sinc3, OSR = 32000
             * 11110 = sinc3, OSR = 32000 + sinc1, OSR = 3
             * 11111 = sinc3, OSR = 32000 + sinc1, OSR = 5 */
            uint8_t filter : 5;

            /** Conversion-start delay time selection.
             * Programmable delay time before the start of the first conversion when START is applied. 
             * Delay time is given in number of fMOD clock cycles (fMOD = fCLK / 2).
             * 000b = 0
             * 001b = 4
             * 010b = 8
             * 011b = 16
             * 100b = 32
             * 101b = 128
             * 110b = 512
             * 111b = 1024 */
            uint8_t delay  : 3;
        };
        uint8_t u8; //!< Register value
    } ads127l11_config3_reg_t;
    #define ADS127L11_CONFIG3_REG_ADDRESS                                               ((uint8_t) 0x07)
    #ifndef ADS127L11_CONFIG3_REG_DEFAULT
        #define ADS127L11_CONFIG3_REG_DEFAULT                                               ((uint8_t) 0x00)
    #endif

    /* Conversion-Start Delay Time field */
    #define ADS127L11_CONFIG3_REG_DELAY_MASK                                            ((uint8_t) 0xE0)
    #define ADS127L11_CONFIG3_REG_DELAY_0                                               ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG3_REG_DELAY_4                                               ((uint8_t) 0x20)
    #define ADS127L11_CONFIG3_REG_DELAY_8                                               ((uint8_t) 0x40)
    #define ADS127L11_CONFIG3_REG_DELAY_16                                              ((uint8_t) 0x60)
    #define ADS127L11_CONFIG3_REG_DELAY_32                                              ((uint8_t) 0x80)
    #define ADS127L11_CONFIG3_REG_DELAY_128                                             ((uint8_t) 0xA0)
    #define ADS127L11_CONFIG3_REG_DELAY_512                                             ((uint8_t) 0xC0)
    #define ADS127L11_CONFIG3_REG_DELAY_1024                                            ((uint8_t) 0xE0)


    /* Filter field */
    #define ADS127L11_CONFIG3_FILTER_MASK                                               ((uint8_t) 0x1F)
    typedef enum _ads127l11_filter_t
    {                                               //     Data-Rate ~ fCLK / (2 * OSR_F1 * OSR_F2)
                                                    //     fCLK = 25.6MHz               || fCLK = 3.2MHz
                                                    //     Data-Rate @ Cutoff Frequency || Data-Rate @ Cutoff Frequency |
        // Wideband Filter
        ADS127L11_FILTER_WIDEBAND_OSR_32 = 0         ,//!< 400kSPS     @ 165 kHz        || 50kSPS        @ 20.625 kHz   |
        ADS127L11_FILTER_WIDEBAND_OSR_64             ,//!< 200kSPS     @ 82.5 kHz       || 25kSPS        @ 10.312 kHz   |
        ADS127L11_FILTER_WIDEBAND_OSR_128            ,//!< 100kSPS     @ 41.25 kHz      || 12.5kSPS      @  5.156 kHz   |
        ADS127L11_FILTER_WIDEBAND_OSR_256            ,//!<  50kSPS     @ 20.625 kHz     ||  6.25kSPS     @  2.578 kHz   |
        ADS127L11_FILTER_WIDEBAND_OSR_512            ,//!<  25kSPS     @ 10.312 kHz     ||  3.125kSPS    @  1.289 kHz   |
        ADS127L11_FILTER_WIDEBAND_OSR_1024           ,//!<  12.5kSPS   @  5.156 kHz     ||  1.5625kSPS   @  0.645 kHz   |
        ADS127L11_FILTER_WIDEBAND_OSR_2048           ,//!<   6.25kSPS  @  2.578 kHz     ||  0.78125kSPS  @  0.322 kHz   |
        ADS127L11_FILTER_WIDEBAND_OSR_4096           ,//!<   3.125kSPS @  1.289 kHz     ||  0.390625kSPS @  0.161 kHz   |
        // Sinc4 Filter
        ADS127L11_FILTER_SINC4_OSR_12                ,//!< 1066.667kSPS @ 243.667 kHz   || 133.333kSPS   @ 30.333 kHz   |
        ADS127L11_FILTER_SINC4_OSR_16                ,//!< 800kSPS      @ 182 kHz       || 100kSPS       @ 22.750 Hz    |
        ADS127L11_FILTER_SINC4_OSR_24                ,//!< 533.333kSPS  @ 121.33 kHz    || 66.667kSPS    @ 15.166 Hz    |
        ADS127L11_FILTER_SINC4_OSR_32                ,//!< 400kSPS      @  91 kHz       || 50kSPS        @ 11.375 Hz    |
        ADS127L11_FILTER_SINC4_OSR_64                ,//!< 200kSPS      @  45 kHz       || 25kSPS        @  5.687 kHz   |
        ADS127L11_FILTER_SINC4_OSR_128               ,//!< 100kSPS      @  22.75 kHz    || 12.5kSPS      @  2.844 kHz   |
        ADS127L11_FILTER_SINC4_OSR_256               ,//!<  50kSPS      @  11.375 kHz   ||  6.25kSPS     @  1.422 kHz   |
        ADS127L11_FILTER_SINC4_OSR_512               ,//!<  25kSPS      @  5.687 kHz    ||  3.125kSPS    @  0.711 kHz   |
        ADS127L11_FILTER_SINC4_OSR_1024              ,//!<  12.5kSPS    @  2.844 kHz    ||  1.5625kSPS   @  0.355 kHz   |
        ADS127L11_FILTER_SINC4_OSR_2048              ,//!<   6.25kSPS   @  1.422 kHz    ||  0.78125kSPS  @  0.177 kHz   |
        ADS127L11_FILTER_SINC4_OSR_4096              ,//!<   3.125kSPS  @  0.711 kHz    ||  0.390625kSPS @  0.089 kHz   |
        // Sinc4 + Sinc1
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_2    ,//!< 200kSPS   @ 68.35 kHz        || 25kSPS     @  8.544 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_4    ,//!< 100kSPS   @ 40.97 kHz        || 12.5kSPS   @  5.121 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_10   ,//!<  40kSPS   @ 17.47 kHz        ||  5.0kSPS   @  2.184 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_20   ,//!<  20kSPS   @  8.814 kHz       ||  2.5kSPS   @  1.102 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_40   ,//!<  10kSPS   @  4.420 kHz       ||  1.25kSPS  @  0.552 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_100  ,//!<   4kSPS   @  1.770 kHz       ||  0.5kSPS   @  0.221 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_200  ,//!<   2kSPS   @  0.885 kHz       ||  0.25kSPS  @  0.111 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_400  ,//!<   1kSPS   @  0.442 kHz       ||  0.125kSPS @  0.055 kHz      |
        ADS127L11_FILTER_SINC4_OSR_32_SINC1_OSR_1000 ,//!<   0.4kSPS @  0.177 kHz       ||  0.05kSPS  @  0.022 kHz      |
        // Sinc3
        ADS127L11_FILTER_SINC3_OSR_26667             ,//!< 480 SPS @ 126 Hz             || 60 SPS     @ 16 Hz           |
        ADS127L11_FILTER_SINC3_OSR_32000             ,//!< 400 SPS @ 105 Hz             || 50 SPS     @ 13 Hz           |
        // Sinc3 + Sinc1
        ADS127L11_FILTER_SINC3_OSR_32000_SINC1_OSR_3 ,//!< 133.3 SPS @ 54 Hz            || 16.6 SPS   @ 6.7 Hz          |
        ADS127L11_FILTER_SINC3_OSR_32000_SINC1_OSR_5 ,//!< 80 SPS    @ 34 Hz            || 10 SPS     @ 4.3 Hz          |
    } ads127l11_filter_t;


/* Register 0x08 (CONFIG4_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    CLK_SEL    |    CLK_DIV    |    OUT_DRV    |    RESERVED   |     DATA      |    SPI_CRC    |     REG_CRC   |     STATUS    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    R/W-0b     |     R/W-0b    |     R/W-0b    |      R-0b     |    R/W-0b     |    R/W-0b     |    R/W-0b     |    R/W-0b     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** CONFIG4 register */
    typedef union _ads127l11_config4_reg_t
    {
        struct
        {
            /** STATUS byte output enable.
             * Program this bit to prefix the STATUS byte to the conversion data output.
             * 0b = Status byte not prefixed to the conversion data
             * 1b = Status byte prefixed to the conversion data */
            uint8_t status : 1;

            /** Register map CRC enable.
             * This bit enables the register map CRC error check. 
             * Write the register map CRC value to the 0Fh register, calculated over registers 0h to 1h and 4h to Eh. 
             * An internal CRC value is compared to the value written to the register map CRC register. 
             * The REG_ERR bit of the STATUS byte sets if the CRC values do not match.
             * 0b = Register map CRC function disabled
             * 1b = Register map CRC function enabled */
            uint8_t reg_crc : 1;

            /** SPI CRC enable.
             * This bit enables the SPI CRC error check. 
             * When enabled, the device verifies the CRC input byte and appends a CRC output byte to the output data. 
             * The SPI_ERR bit of the STATUS byte sets if an input SPI CRC error is detected. 
             * Write 1b to the SPI_ERR bit to clear the error.
             * 0b = SPI CRC function disabled
             * 1b = SPI CRC function enabled */
            uint8_t spi_crc : 1;

            /** Data resolution selection.
             * This bit selects the output data resolution.
             * 0b = 24-bit resolution
             * 1b = 16-bit resolution */
            uint8_t data    : 1;

            /** Reserved */
            uint8_t         : 1;

            /** Digital output drive selection.
             * Select the drive strength of the digital outputs.
             * 0b = Full-drive strength
             * 1b = Half-drive strength */
            uint8_t out_drv : 1;

            /** External clock divider selection.
             * This bit is used to divide the external clock by 8.
             * 0b = No clock division
             * 1b = Clock division by 8 */
            uint8_t clk_div : 1;

            /** Clock selection.
             * Selects internal or external clock operation.
             * 0b = Internal clock operation
             * 1b = External clock operation */
            uint8_t clk_sel : 1;
        };
        uint8_t u8; //!< Register value
    } ads127l11_config4_reg_t;
    #define ADS127L11_CONFIG4_REG_ADDRESS                                             ((uint8_t) 0x08)
    #ifndef ADS127L11_CONFIG4_REG_DEFAULT
        #define ADS127L11_CONFIG4_REG_DEFAULT                                             ((uint8_t) 0x00)
    #endif

    /* Clock Selection field */
    #define ADS127L11_CONFIG4_REG_CLK_SEL_MASK                                        ((uint8_t) 0x80)
    #define ADS127L11_CONFIG4_REG_CLK_SEL_INTERNAL                                    ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG4_REG_CLK_SEL_EXTERNAL                                    ((uint8_t) 0x80)

    /* External Clock Division Selection field */
    #define ADS127L11_CONFIG4_REG_CLK_DIV_MASK                                        ((uint8_t) 0x40)
    #define ADS127L11_CONFIG4_REG_CLK_DIV_1                                           ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG4_REG_CLK_DIV_8                                           ((uint8_t) 0x40)

    /* Digital OUtput Drive Selection Field */
    #define ADS127L11_CONFIG4_REG_OUT_DRV_MASK                                        ((uint8_t) 0x20)
    #define ADS127L11_CONFIG4_REG_OUT_DRV_FULL                                        ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG4_REG_OUT_DRV_HALF                                        ((uint8_t) 0x20)

    /* Data Resolution Selection field */
    #define ADS127L11_CONFIG4_REG_DATA_MASK                                           ((uint8_t) 0x08)
    #define ADS127L11_CONFIG4_REG_DATA_24_BIT                                         ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG4_REG_DATA_16_BIT                                         ((uint8_t) 0x08)

    /* SPI CRC Enable field */
    #define ADS127L11_CONFIG4_REG_SPI_CRC_MASK                                        ((uint8_t) 0x04)
    #define ADS127L11_CONFIG4_REG_SPI_CRC_DISABLED                                    ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG4_REG_SPI_CRC_ENABLED                                     ((uint8_t) 0x04)

    /* Register Map CRC Enable field */
    #define ADS127L11_CONFIG4_REG_REG_CRC_MASK                                        ((uint8_t) 0x02)
    #define ADS127L11_CONFIG4_REG_REG_CRC_DISABLED                                    ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG4_REG_REG_CRC_ENABLED                                     ((uint8_t) 0x02)

    /* Status Byte Output Enable field */
    #define ADS127L11_CONFIG4_REG_STATUS_MASK                                         ((uint8_t) 0x01)
    #define ADS127L11_CONFIG4_REG_STATUS_DISABLED                                     ((uint8_t) 0x00)    // DEFAULT
    #define ADS127L11_CONFIG4_REG_STATUS_ENABLED                                      ((uint8_t) 0x01)


/* Register 0x09 (OFFSET_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       OFFSET[23:16]                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       R/W-00000000b                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** OFFSET_MSB_REG register 
     * User offset calibration value.
     * Three registers form the 24-bit offset calibration word. 
     * OFFSET[23:0] is in 2's-complement representation and is subtracted from the conversion result. 
     * The offset operation precedes the gain operation. */
    typedef uint8_t ads127l11_offset2_reg_t;
    #define ADS127L11_OFFSET_MSB_REG_ADDRESS                                            ((uint8_t) 0x09)
    #ifndef ADS127L11_OFFSET_MSB_REG_DEFAULT
        #define ADS127L11_OFFSET_MSB_REG_DEFAULT                                            ((uint8_t) 0x00)
    #endif


/* Register 0x0A (OFFSET_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       OFFSET[15:8]                                                            |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       R/W-00000000b                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** OFFSET_MID_REG register 
     * User offset calibration value.
     * Three registers form the 24-bit offset calibration word. 
     * OFFSET[23:0] is in 2's-complement representation and is subtracted from the conversion result. 
     * The offset operation precedes the gain operation. */
    typedef uint8_t ads127l11_offset1_reg_t;
    #define ADS127L11_OFFSET_MID_REG_ADDRESS                                          ((uint8_t) 0x0A)
    #ifndef ADS127L11_OFFSET_MID_REG_DEFAULT
        #define ADS127L11_OFFSET_MID_REG_DEFAULT                                          ((uint8_t) 0x00)
    #endif


/* Register 0x0B (OFFSET_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       OFFSET[7:0]                                                             |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       R/W-00000000b                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** OFFSET_LSB_REG register 
     * User offset calibration value.
     * Three registers form the 24-bit offset calibration word. 
     * OFFSET[23:0] is in 2's-complement representation and is subtracted from the conversion result. 
     * The offset operation precedes the gain operation. */
    typedef uint8_t ads127l11_offset0_reg_t;

    /* User offset calibration value.
     * Three registers form the 24-bit offset calibration word. 
     * OFFSET[23:0] is in 2's-complement representation and is subtracted from the conversion result. 
     * The offset operation precedes the gain operation. */
    typedef struct _ads127l11_offset_reg_t
    {
        uint8_t offset_lsb; //!< OFFSET0 register value
        uint8_t offset_mid; //!< OFFSET1 register value
        uint8_t offset_msb; //!< OFFSET2 register value
    } ads127l11_offset_reg_t;
    #define ADS127L11_CREATE_OFFSET_REG( I32_OFFSET ) \
    {                                                                                             \
        .offset_lsb = (uint8_t)( (I32_OFFSET) & 0xFF ),                                           \
        .offset_mid = (uint8_t)( ( (I32_OFFSET) >> 8) & 0xFF ),                                   \
        .offset_msb = (uint8_t)( ( ((I32_OFFSET) >> 16) & 0x7F) | (((I32_OFFSET) >> 24) & 0x80) ) \
    }

    #define ADS127L11_OFFSET_LSB_REG_ADDRESS                                          ((uint8_t) 0x0B)
    #ifndef ADS127L11_OFFSET_LSB_REG_DEFAULT
        #define ADS127L11_OFFSET_LSB_REG_DEFAULT                                          ((uint8_t) 0x00)
    #endif


/* Register 0x0C (GAIN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GAIN[23:16]                                                             |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       R/W-01000000b                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** GAIN_MSB_REG register 
     * User gain calibration value.
     * Three registers form the 24-bit gain calibration word. 
     * GAIN[23:0] is in straight-binary representation and normalized to 400000h for gain = 1. 
     * The conversion data is multiplied by GAIN[23:0] / 400000h after the offset operation. */
    typedef uint8_t ads127l11_gain2_reg_t;
    #define ADS127L11_GAIN_MSB_REG_ADDRESS                                            ((uint8_t) 0x0C)
    #ifndef ADS127L11_GAIN_MSB_REG_DEFAULT
        #define ADS127L11_GAIN_MSB_REG_DEFAULT                                            ((uint8_t) 0x40)
    #endif


/* Register 0x0D (GAIN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GAIN[15:8]                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       R/W-00000000b                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN_MID_REG register 
     * User gain calibration value.
     * Three registers form the 24-bit gain calibration word. 
     * GAIN[23:0] is in straight-binary representation and normalized to 400000h for gain = 1. 
     * The conversion data is multiplied by GAIN[23:0] / 400000h after the offset operation. */
    typedef uint8_t ads127l11_gain1_reg_t;
    #define ADS127L11_GAIN_MID_REG_ADDRESS                                            ((uint8_t) 0x0D)
    #ifndef ADS127L11_GAIN_MID_REG_DEFAULT
        #define ADS127L11_GAIN_MID_REG_DEFAULT                                            ((uint8_t) 0x00)
    #endif


/* Register 0x0E (GAIN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GAIN[7:0]                                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       R/W-00000000b                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** GAIN_MSB_REG register 
     * User gain calibration value.
     * Three registers form the 24-bit gain calibration word. 
     * GAIN[23:0] is in straight-binary representation and normalized to 400000h for gain = 1. 
     * The conversion data is multiplied by GAIN[23:0] / 400000h after the offset operation. */
    typedef uint8_t ads127l11_gain0_reg_t;

    /* User gain calibration value.
     * Three registers form the 24-bit gain calibration word. 
     * GAIN[23:0] is in straight-binary representation and normalized to 400000h for gain = 1. 
     * The conversion data is multiplied by GAIN[23:0] / 400000h after the offset operation. */
    typedef union _ads127l11_gain_reg_t
    {
        struct 
        {
            uint32_t gain0 : 8; //!< GAIN0 register value
            uint32_t gain1 : 8; //!< GAIN1 register value
            uint32_t gain2 : 8; //!< GAIN2 register value
            uint32_t       : 8; //!< Unused
        };
        uint32_t gain; //!< 24 bit User gain calibration value
    } ads127l11_gain_reg_t;

    #define ADS127L11_GAIN_LSB_REG_ADDRESS                                            ((uint8_t) 0x0E)
    #ifndef ADS127L11_GAIN_LSB_REG_DEFAULT
        #define ADS127L11_GAIN_LSB_REG_DEFAULT                                            ((uint8_t) 0x00)
    #endif


/* Register 0x0F (CRC_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CRC[7:0]                                                                |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     R/W-00000000b                                                             |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /** CRC_REG register 
     * Register map CRC value.
     * The register map CRC is a user-computed value of registers 0h to 1h together with registers 4h through Eh. 
     * The value written to this register is compared to an internal CRC calculation. 
     * If the values do not match, the REG_ERR bit is set in the STATUS header byte and register. 
     * The register CRC check is enabled by the REG_CRC bit. 
     * If the register CRC function is disabled, this register is available for scratchpad purposes. */
    typedef uint8_t ads127l11_crc_reg_t;
    #define ADS127L11_CRC_REG_ADDRESS                                                 ((uint8_t) 0x0C)
    #ifndef ADS127L11_CRC_REG_DEFAULT
        #define ADS127L11_CRC_REG_DEFAULT                                                 ((uint8_t) 0x00)
    #endif

// -----------------------------------------------------------------------------------------------------------------------------------

// This macro may be useful when debugging
#define ADS127L11_GET_REGISTER_NAME( I ) \
(                                        \
    ( I ) == 0x00 ? "DEV_ID"  :          \
    ( I ) == 0x01 ? "REV_ID"  :          \
    ( I ) == 0x02 ? "STATUS"  :          \
    ( I ) == 0x03 ? "CONTROL" :          \
    ( I ) == 0x04 ? "MUX"     :          \
    ( I ) == 0x05 ? "CONFIG1" :          \
    ( I ) == 0x06 ? "CONFIG2" :          \
    ( I ) == 0x07 ? "CONFIG3" :          \
    ( I ) == 0x08 ? "CONFIG4" :          \
    ( I ) == 0x09 ? "OFFSET2" :          \
    ( I ) == 0x0A ? "OFFSET1" :          \
    ( I ) == 0x0B ? "OFFSET0" :          \
    ( I ) == 0x0C ? "GAIN2"   :          \
    ( I ) == 0x0D ? "GAIN1"   :          \
    ( I ) == 0x0E ? "GAIN0"   :          \
    ( I ) == 0x0F ? "CRC"     :          \
    ""                                   \
)

#ifdef __cplusplus
}
#endif