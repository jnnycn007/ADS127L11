#include "ads_core.h"

//#region Private Types
//#endregion Private Types

//#region Private Prototypes

static void ads127l11_init_crc(ads127l11_t* ads);

//#endregion Private Prototypes


//#region Static Private

//*****************************************************************************
//
//! Combines ADC data bytes into a single signed 32-bit word.
//!
//! \fn int32_t combineDataBytes(const uint8_t dataBytes[])
//!
//! \param dataBytes is a pointer to uint8_t[] where the first element is the MSB.
//!
//! \return Returns the signed-extend 32-bit result.
//
//*****************************************************************************
static int32_t signExtend(const bool is_24_bit, const uint8_t dataBytes[])
{
    if ( is_24_bit )
    {
        int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
        int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
        int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

        // Right-shift of signed data maintains signed bit
        return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);
    }

    // 16 bit sample
    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);
    // Right-shift of signed data maintains signed bit
    return (((int32_t) (upperByte | lowerByte)) >> 16);
}

//*****************************************************************************
//
//! Builds SPI TX data arrays according to number of opcodes provided and
//! currently programmed device word length.
//!
//! \fn uint8_t buildSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[])
//!
//! \param opcodeArray[] pointer to an array of 8-bit opcodes to use in the SPI command.
//! \param byteArray[] pointer to an array of 8-bit SPI bytes to send to the device.
//!
//! NOTE: The calling function must ensure it reserves sufficient memory for byteArray[]!
//!
//! \return number of bytes added to byteArray[].
//
//*****************************************************************************
static inline uint8_t buildSPIarray(ads127l11_t* ads, const uint8_t opcodeArray[], uint8_t byteArray[])
{
    #if 0
    // May be faster, but not generic
    switch( ads->data_mode )
    {
        default:
        case ADS127L11_RES_24_CRC_OFF_STATUS_OFF:
        {
            byteArray[1] = opcodeArray[0];
            byteArray[2] = opcodeArray[1];
            return 3;
        }
        case ADS127L11_RES_24_CRC_OFF_STATUS_ON:
        {
            byteArray[2] = opcodeArray[0];
            byteArray[3] = opcodeArray[1];
            return 4;
        }
        case ADS127L11_RES_24_CRC_ON_STATUS_OFF:
        {
            byteArray[1] = opcodeArray[0];
            byteArray[2] = opcodeArray[1];
            byteArray[3] = getCRC(opcodeArray, 2, CRC_INITIAL_SEED);
            return 3;
        }
        case ADS127L11_RES_24_CRC_ON_STATUS_ON:
        {
            byteArray[2] = opcodeArray[0];
            byteArray[3] = opcodeArray[1];
            byteArray[4] = getCRC(opcodeArray, 2, CRC_INITIAL_SEED);
            return 4;
        }

        case ADS127L11_RES_16_CRC_OFF_STATUS_OFF:
        {
            byteArray[0] = opcodeArray[0];
            byteArray[1] = opcodeArray[1];
            return 2;
        }
        case ADS127L11_RES_16_CRC_OFF_STATUS_ON:
        {
            byteArray[1] = opcodeArray[0];
            byteArray[2] = opcodeArray[1];
            return 3;
        }
        case ADS127L11_RES_16_CRC_ON_STATUS_OFF:
        {
            byteArray[0] = opcodeArray[0];
            byteArray[1] = opcodeArray[1];
            byteArray[2] = getCRC(opcodeArray, 2, CRC_INITIAL_SEED);
            return 2;
        }
        case ADS127L11_RES_16_CRC_ON_STATUS_ON:
        {
            byteArray[1] = opcodeArray[0];
            byteArray[2] = opcodeArray[1];
            byteArray[3] = getCRC(opcodeArray, 2, CRC_INITIAL_SEED);
            return 3;
        }
    }
    #else
	//Fixed at 2. 8 bits for the command + address, and 8 bits of data (W) or arbitrary bits (R)
    const uint8_t numberOpcodes  = 2;
	//Determine where the opcode/data bytes will be placed in the array
    const uint8_t frontPadBytes  = (ads->cfg.flags.status_byte_en ? 1 : 0) + (ads->cfg.flags.is_16_bit ? 0 : 1);
	//Determine total number of bytes to clock
    const uint8_t numberOfBytes  = numberOpcodes + frontPadBytes + (ads->cfg.flags.crc_en ? 1 : 0);

    //Place the command byte/data in the correct place in the array
    byteArray[frontPadBytes]     = opcodeArray[0];
    byteArray[frontPadBytes + 1] = opcodeArray[1];

    //Calculate/lookup CRC value
	if( ads->cfg.flags.crc_en )
	{
		byteArray[frontPadBytes + 2] = getCRC(opcodeArray, numberOpcodes, CRC_INITIAL_SEED);
	}

    return numberOfBytes;
    #endif
}

//*****************************************************************************
//
//! Modifies CONFIG4 register data to maintain device operation according to
//! preselected mode(s) (DATA, SPI_CRC, STATUS).
//!
//! \fn uint8_t enforce_selected_device_mode(uint8_t data)
//!
//! \param data uint8_t register data.
//!
//! \return uint8_t modified register data.
//
//*****************************************************************************
static uint8_t enforce_selected_device_modes(ads127l11_t* ads, uint8_t data)
{

    ///////////////////////////////////////////////////////////////////////////
    // Enforce DATA Resolution setting
	if( ads->cfg.flags.is_16_bit )
    {
		// When writing to the CONFIG4 register, ensure DATA bit is ALWAYS set
    	data |= ADS127L11_CONFIG4_REG_DATA_16_BIT;
	}
	else
    {
    	// When writing to the CONFIG4 register, ensure DATA bit is NEVER set
    	data &= ADS127L11_CONFIG4_REG_DATA_24_BIT;
	}
	

    ///////////////////////////////////////////////////////////////////////////
    // Enforce SPI_CRC Setting
	if( ads->cfg.flags.crc_en )
	{
		// When writing to the CONFIG4 register, ensure SPI_CRC bit is ALWAYS set
    	data |= ADS127L11_CONFIG4_REG_SPI_CRC_ENABLED;	
	}
	else
	{
		// When writing to the CONFIG4 register, ensure SPI_CRC bit is NEVER set
    	data &= ADS127L11_CONFIG4_REG_SPI_CRC_DISABLED;
	}


    ///////////////////////////////////////////////////////////////////////////
    // Enforce STATUS Byte Output Setting
	if( ads->cfg.flags.status_byte_en )
	{
		// When writing to the CONFIG4 register, ensure STATUS bit is ALWAYS set
    	data |= ADS127L11_CONFIG4_REG_STATUS_ENABLED;
	}
    else
	{
		// When writing to the CONFIG4 register, ensure STATUS bit is NEVER set
    	data &= ADS127L11_CONFIG4_REG_STATUS_DISABLED;
	}

    // Return modified register data
    return data;
}

static void ads127l11_init_crc(ads127l11_t* ads)
{
	// TODO don't use Texas CRC library
	initCRC();
}

//#endregion Static Private


//#region Auxiliar

void     ads127l11_write_register(ads127l11_t* ads, const uint8_t address, uint8_t data)
{
	/* Check that the register address is in range */
    // assert(address < ADS127L11_NUM_REGISTERS);

    // (OPTIONAL) Check if write operation will result in a reset
    // and restore defaults to ensure register map stays in sync
    if ( (address == ADS127L11_CONTROL_REG_ADDRESS) &&
	     (data == ADS127L11_CONTROL_REG_RESET_COMMAND) )
	{
		ads127l11_restore_default(ads);
	}

    // (OPTIONAL) Enforce certain register field values when
    // writing to the CONFIG4 register to fix the operation mode
    else if (address == ADS127L11_CONFIG4_REG_ADDRESS)
    {
        data = enforce_selected_device_modes(ads, data);
    }

    // Build TX and RX byte array
    uint8_t dataTx[5] = { 0, 0, 0, 0, 0 };
    uint8_t dataRx[5] = { 0, 0, 0, 0, 0 };

    // Build opcode and SPI TX array
    uint8_t   opcode[2]     = {(ADS127L11_OPCODE_WREG | address), data};
    uint8_t numberOfBytes   = buildSPIarray(ads, opcode, dataTx);

    // Send command
	ads->hal.spi_exchange_array( ads->hal.param, dataTx, dataRx, numberOfBytes);

    // Update internal array
	ads->reg[ address ] = data;

    // (RECOMMENDED) Read back register to confirm register write was successful
	ads127l11_read_register(ads, address);

    // NOTE: Enabling the CRC words in the SPI command will NOT prevent an invalid write
}

uint8_t  ads127l11_read_register(ads127l11_t* ads, const uint8_t address)
{
	/* Check that the register address is in range */
	//assert(address < ADS127L11_NUM_REGISTERS);

	// Build TX and RX byte array --NOTE maybe move the data arrays outside function to save memory
    uint8_t dataTx[5] = { 0, 0, 0, 0 ,0 };
    uint8_t dataRx[5] = { 0, 0, 0, 0, 0 };

    // Build opcode and SPI TX array
    const uint8_t       arbitrary = 0x00;
    uint8_t             opcode[2] = {(ADS127L11_OPCODE_RREG | address), arbitrary};
    uint8_t         numberOfBytes = buildSPIarray(ads, opcode, dataTx);

	// [FRAME 1] Send RREG command
	ads->hal.spi_exchange_array( ads->hal.param, dataTx, dataRx, numberOfBytes);

	// [FRAME 2] Send NULL command to retrieve the register data
	uint8_t nullopcode[2] = {ADS127L11_OPCODE_NULL, ADS127L11_OPCODE_NULL};
	ads->reg[address]     = ads127l11_send_cmd(ads, nullopcode);

	return ads->reg[address];
}

void     ads127l11_read_raw_24bit_crc_off_status_off(ads127l11_t* ads, uint8_t* rx)
{
    // Optimized for 24 bit resolution, CRC: Off, Status Byte: Off

    // Sending dummy values
    const uint8_t dataTx[3] = { ADS127L11_OPCODE_NULL, ADS127L11_OPCODE_NULL, ADS127L11_OPCODE_NULL };

    // Send command
    ads->hal.spi_exchange_array( ads->hal.param, dataTx, rx, 3);
}

int32_t  ads127l11_read_data(ads127l11_t* ads, ads127l11_ch_data_t* data)
{
    // Build TX and RX byte array
    uint8_t dataTx[5] = { 0, 0, 0, 0, 0 };
    uint8_t dataRx[5] = { 0, 0, 0, 0, 0 };

    // Build opcode and SPI TX array
    uint8_t opcode[2]     = {ADS127L11_OPCODE_NULL, ADS127L11_OPCODE_NULL};
    uint8_t numberOfBytes = buildSPIarray(ads, opcode, dataTx);

    // Send command
    ads->hal.spi_exchange_array( ads->hal.param, dataTx, dataRx, numberOfBytes);

    // Parse ADC response...
    uint8_t byteIndex = ads->cfg.flags.status_byte_en ? 1 : 0;
    int32_t signedExtendedData = signExtend(ads->cfg.flags.is_16_bit ? 0 : 1,  &dataRx[byteIndex]);
    if ( data ) //Check if dataStruct is a null pointer
    {
        if ( ads->cfg.flags.status_byte_en )
        {
			data->status = dataRx[0];
        }

        if ( ads->cfg.flags.crc_en )
        {
            byteIndex = byteIndex + (ads->cfg.flags.is_16_bit ? 2 : 3);
            data->crc = dataRx[byteIndex];
        }

        data->data = signedExtendedData;
    }

    // Return ADC code
    return signedExtendedData;
}

uint8_t  ads127l11_get_register_val(ads127l11_t* ads, const uint8_t address)
{
    return ads->reg[address];
}

//*****************************************************************************
//
//! Updates the registerMap[] array to its default values.
//!
//! \fn void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADS127L11 register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! hardware pin control or SPI software command).
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result; however, this might be problematic if the
//! device was previously in CRC mode or STATUS output mode, since
//! resetting the device exits these modes. If the MCU is not aware of this
//! mode change, then read register commands will return invalid data due to
//! the expectation of data appearing in a different byte position.
//!
//! \return None.
//
//*****************************************************************************
void ads127l11_restore_default(ads127l11_t* ads)
{
	ads->reg[ADS127L11_DEVID_REG_ADDRESS]          =   ADS127L11_DEVID_REG_DEFAULT;
//  ads->reg[ADS127L11_REVID_REG_ADDRESS]          =   ADS127L11_REVID_REG_DEFAULT;
    ads->reg[ADS127L11_STATUS_REG_ADDRESS]         =   ADS127L11_STATUS_REG_DEFAULT;
    ads->reg[ADS127L11_CONTROL_REG_ADDRESS]        =   ADS127L11_CONTROL_REG_DEFAULT;
    ads->reg[ADS127L11_MUX_REG_ADDRESS]            =   ADS127L11_MUX_REG_DEFAULT;
    ads->reg[ADS127L11_CONFIG1_REG_ADDRESS]        =   ADS127L11_CONFIG1_REG_DEFAULT;
    ads->reg[ADS127L11_CONFIG2_REG_ADDRESS]        =   ADS127L11_CONFIG2_REG_DEFAULT;
    ads->reg[ADS127L11_CONFIG3_REG_ADDRESS]        =   ADS127L11_CONFIG3_REG_DEFAULT;
    ads->reg[ADS127L11_CONFIG4_REG_ADDRESS]        =   ADS127L11_CONFIG4_REG_DEFAULT;
    ads->reg[ADS127L11_OFFSET_MSB_REG_ADDRESS]     =   ADS127L11_OFFSET_MSB_REG_DEFAULT;
    ads->reg[ADS127L11_OFFSET_MID_REG_ADDRESS]     =   ADS127L11_OFFSET_MID_REG_DEFAULT;
    ads->reg[ADS127L11_OFFSET_LSB_REG_ADDRESS]     =   ADS127L11_OFFSET_LSB_REG_DEFAULT;
    ads->reg[ADS127L11_GAIN_MSB_REG_ADDRESS]       =   ADS127L11_GAIN_MSB_REG_DEFAULT;
    ads->reg[ADS127L11_GAIN_MID_REG_ADDRESS]       =   ADS127L11_GAIN_MID_REG_DEFAULT;
    ads->reg[ADS127L11_GAIN_LSB_REG_ADDRESS]       =   ADS127L11_GAIN_LSB_REG_DEFAULT;
//  ads->reg[ADS127L11_CRC_REG_ADDRESS]            =   ADS127L11_CRC_REG_DEFAULT;	
}

//*****************************************************************************
//
//! Resets the device by SPI command to CONTROL register
//!
//! \fn void resetDeviceByCommand(void)
//!
//! NOTE: This function does not capture DOUT data, but it could be modified
//! to do so.
//!
//! \return None.
//
//*****************************************************************************
void     ads127l11_reset_by_command(ads127l11_t* ads)
{
    //Reset by SPI Register Write

    // Write data 0x58 to CONTROL register
    uint8_t address    =   ADS127L11_CONTROL_REG_ADDRESS;
    uint8_t data       =   ADS127L11_CONTROL_REG_RESET_COMMAND;

    // Build TX and RX byte array
    uint8_t dataTx[5] = { 0, 0, 0, 0, 0 };
    uint8_t dataRx[5] = { 0, 0, 0, 0, 0 };

    // Build opcode and SPI TX array
    uint8_t     opcode[2] = {(ADS127L11_OPCODE_WREG | address), data};
    uint8_t numberOfBytes = buildSPIarray(ads, opcode, dataTx);

    // Send command
    ads->hal.spi_exchange_array( ads->hal.param, dataTx, dataRx, numberOfBytes);

    // tSRLRST delay
    ads->hal.delay_ms( ads->hal.param,  1 );

    // Update register setting array to keep software in sync with device
    ads127l11_restore_default( ads );
}

//*****************************************************************************
//
//! Resets the device by SPI input pattern
//!
//! \fn void resetDeviceByPattern(void)
//!
//! NOTE: This function does not capture DOUT data, but it could be modified
//! to do so.
//!
//! \return None.
//
//*****************************************************************************
void     ads127l11_reset_by_pattern(ads127l11_t* ads)
{
    if( ads->cfg.flags.spi_3_wire )
    {
        //Reset by SPI Input Pattern Option 1 (3 wire or 4 wire SPI mode)
        /* Send 1024 ones followed by zero*/
        for (int i = 0; i < 128; i++)
        {
            ads->hal.spi_exchange_byte( ads->hal.param, (uint8_t) 0xFF );
        }
        ads->hal.spi_exchange_byte( ads->hal.param, (uint8_t) 0x00 );
    }
    else
    {
        //Reset by SPI Input Pattern Option 2 (recommended for daisy-chain operation -- 4 wire SPI mode only)

        /* Set the nCS pin LOW */
        ads->hal.write_cs( ads->hal.param,  0 );

        for (int i = 0; i < 128; i++)
        {
            ads->hal.spi_exchange_byte( ads->hal.param, (uint8_t) 0xFF);
        }

        /* Set the nCS pin HIGH */
        ads->hal.write_cs( ads->hal.param,  1 );
    }

    // tSRLRST delay
    ads->hal.delay_ms( ads->hal.param,  1 );

    // Update register setting array to keep software in sync with device
    ads127l11_restore_default( ads );
}

//*****************************************************************************
//
//! Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).
//!
//! \fn uint8_t ads127l11_send_cmd(ads127l11_t* ads, uint8_t opcode[])
//!
//! \param opcode SPI command byte.
//!
//! NOTE: Other commands have their own dedicated functions to support
//! additional functionality.
//!
//! \return ADC response byte (typically the STATUS byte).
//
//*****************************************************************************
uint8_t  ads127l11_send_cmd(ads127l11_t* ads, uint8_t opcode[])
{
    /* Assert if this function is used to send any of the following opcodes */
    //assert(ADS127L11_OPCODE_RREG != opcode[0]);      /* Use "readSingleRegister()"   */
    //assert(ADS127L11_OPCODE_WREG != opcode[0]);      /* Use "writeSingleRegister()"  */

    // Build TX and RX byte array
    uint8_t dataTx[5] = { 0, 0, 0, 0, 0 };
    uint8_t dataRx[5] = { 0, 0, 0, 0, 0 };

    uint8_t numberOfBytes = buildSPIarray(ads, opcode, dataTx);

    /* Set the nCS pin LOW */
	ads->hal.write_cs( ads->hal.param,  0 );

    // Send the opcode
    //ads->hal.spi_exchange_array( ads->hal.param, dataTx, dataRx, numberOfBytes); // Why not use this instead of for loop?
    for (int i = 0; i < numberOfBytes; i++)
    {
       dataRx[i] = ads->hal.spi_exchange_byte( ads->hal.param, dataTx[i] );
    }

    /* Set the nCS pin HIGH */
	ads->hal.write_cs( ads->hal.param,  1 );

    // Return response byte
    uint8_t adcResponse = dataRx[(ads->cfg.flags.status_byte_en ? 1 : 0)];
    return adcResponse;
}

//#endregion Auxiliar


//#region Public

bool ads127l11_setup( ads127l11_t* ads, 
    const ads127l11_config1_reg_t config1_reg,
    const ads127l11_config2_reg_t config2_reg,
    const ads127l11_config3_reg_t config3_reg,
    const ads127l11_config4_reg_t config4_reg )
{
    // The CS_MODE is determined by the state of CS at power up or after reset.
    if( ads->cfg.flags.spi_3_wire )
    {
        // 1b = 3-wire SPI operation (CS is tied low)
        ads->hal.write_cs( ads->hal.param,  0 );
    }
    else
    {
        // 0b = 4-wire SPI operation (CS is active)
        ads->hal.write_cs( ads->hal.param,  1 );
    }

	/* (OPTIONAL) Provide additional delay time for power supply settling */
	ads->hal.delay_ms( ads->hal.param,  50 );

    /* (OPTIONAL) Populate CRC lookup table */
	if( ads->cfg.flags.crc_en )
	{
		ads127l11_init_crc(ads);
	}

	/* (REQUIRED) Set nRESET pin high for ADC operation */
	ads->hal.write_reset( ads->hal.param,  1 );

	/* (REQUIRED) Set START pin high to begin conversions    */
	ads->hal.write_start( ads->hal.param,  1 );
    
	/* (OPTIONAL) Toggle nRESET pin to ensure default register settings. */
	{
		// Minimum /RESET pulse width (tSRLRST) equals 2,048 CLKIN periods (1 ms @ 2.048 MHz)
        ads->hal.write_reset( ads->hal.param,  0 );
		ads->hal.delay_ms( ads->hal.param,  5 );
		ads->hal.write_reset( ads->hal.param,  1 );

		// Delay before communicating with the device again
		ads->hal.delay_ms( ads->hal.param,  5 );
	}

    // Set CS back to IDLE
    if( ads->cfg.flags.spi_3_wire )
    {
        ads->hal.write_cs( ads->hal.param,  1 );
        ads->hal.delay_ms( ads->hal.param,  1 );
    }

	/* (REQUIRED) Initialize internal 'registerMap' array with device default settings */
	ads127l11_restore_default( ads );

    // Fix possible brownout flags due to the power-on reset
    ads127l11_write_register(ads, 
        ADS127L11_STATUS_REG_ADDRESS, 
        ADS127L11_STATUS_REG_ALV_FLAG_1 |
        ADS127L11_STATUS_REG_POR_FLAG_1 |
        ADS127L11_STATUS_REG_SPI_ERR_1  |
        ADS127L11_STATUS_REG_REG_ERR_1
    );

	/* (REQUIRED) Configure CONFIG4 register settings */
    const uint8_t enforced_config4 = enforce_selected_device_modes(ads, config4_reg.u8);
	ads127l11_write_register(ads, ADS127L11_CONFIG4_REG_ADDRESS, enforced_config4 );
	ads127l11_write_register(ads, ADS127L11_CONFIG3_REG_ADDRESS, config3_reg.u8);
	ads127l11_write_register(ads, ADS127L11_CONFIG2_REG_ADDRESS, config2_reg.u8);
	ads127l11_write_register(ads, ADS127L11_CONFIG1_REG_ADDRESS, config1_reg.u8);

    // Refresh local register map
    #if 0
    uint8_t reg_addr = 0;
    while( reg_addr < ADS127L11_NUM_REGISTERS )
    {
        ads127l11_read_register(ads, reg_addr);
        reg_addr++;
    }
    #endif

    const bool success = 
        ( ads127l11_get_register_val(ads, ADS127L11_CONFIG1_REG_ADDRESS) == config1_reg.u8   ) &&
        ( ads127l11_get_register_val(ads, ADS127L11_CONFIG2_REG_ADDRESS) == config2_reg.u8   ) &&
        ( ads127l11_get_register_val(ads, ADS127L11_CONFIG3_REG_ADDRESS) == config3_reg.u8   ) &&
        ( ads127l11_get_register_val(ads, ADS127L11_CONFIG4_REG_ADDRESS) == enforced_config4 );

    return success;
}


//#endregion Public