#include <ads127l11.h>

//#region ads_hal_t functions

#include <Arduino.h>
#include <SPI.h>

#define PIN_ADC_CS        1
#define PIN_ADC_RST      10
#define PIN_ADC_SDI      11
#define PIN_ADC_SCLK     12
#define PIN_ADC_SDO      13
#define PIN_ADC_RDY      14
#define PIN_ADC_EXT_CLK  15
#define PIN_ADC_START    16
#define ADS127L11_SCLK_FREQ_HZ    ( 5000000  )  /* 3.2e6 Hz is enough for 100kSPS with 1 device */
#define ADS127L11_SPI_BIT_ORDER   SPI_MSBFIRST  /* SPI Data Bit Order */
#define ADS127L11_SPI_MODE        SPI_MODE1     /* ADS127L11 is compatible to SPI mode 1 (CPOL = 0 and CPHA = 1). */

/* ADS127L11 Register 'CONFIG1' Value */
#define ADS127L11_CONFIG1_REG                    \
{                                                \
	{                                            \
		.ainn_buff = 1, /* AINN Buffer enable */ \
		.ainp_buff = 1, /* AINP Buffer enable */ \
		.refp_buf  = 1, /* REFP Buffer enable */ \
		.vcm       = 0, /* VCM Output disable */ \
		.inp_rng   = 0, /* 1x Input Range     */ \
		.ref_rng   = 0, /* Low-Ref Range      */ \
	}                                            \
}

/* ADS127L11 Register 'CONFIG2' Value */
#define ADS127L11_CONFIG2_REG                          \
{                                                      \
	{                                                  \
		.pwdn       = 0, /* Normal Operation        */ \
		.stby_mode  = 0, /* Idle Mode               */ \
		.speed_mode = 0, /* High-Speed Mode         */ \
		.start_mode = 0, /* Start-Stop Control Mode */ \
		.sdo_mode   = 0, /* Data Output Only        */ \
		.ext_rng    = 0, /* Standard Input Range    */ \
	}                                                  \
}

/* ADS127L11 Register 'CONFIG3' Value */
#define ADS127L11_CONFIG3_REG_FILTER_VAL ADS127L11_FILTER_SINC3_OSR_32000_SINC1_OSR_5
#define ADS127L11_CONFIG3_REG                        \
{                                                    \
	{                                                \
		.filter = ADS127L11_CONFIG3_REG_FILTER_VAL,  \
		.delay  = 0, /* No delay */                  \
	}                                                \
}

/* ADS127L11 Register 'CONFIG4' Value */
#define ADS127L11_CONFIG4_REG                                        \
{                                                                    \
	{                                                                \
		.status  = 0, /* Status won't be added to the output data */ \
		.reg_crc = 0, /* CRC Disabled        */                      \
		.spi_crc = 0, /* SPI Disabled        */                      \
		.data    = 0, /* 24 bit Resolution   */                      \
		.out_drv = 0, /* Full Drive Strength */                      \
		.clk_div = 0, /* No Clock Divide     */                      \
		.clk_sel = 0, /* Use Internal Clock  */                      \
	}                                                                \
}


void    delay_ms(void* obj, uint32_t millis)
{
	delay(millis);
}

void    write_cs(void* obj, const bool state)
{
	//digitalWrite( static_cast<MyClass*>(obj)->pin_adc_cs, (uint8_t)state ); // the 'obj' param may be helpful in cpp
	digitalWrite( PIN_ADC_CS   , (uint8_t)state );
}

void    write_start(void* obj, const bool state)
{
	digitalWrite( PIN_ADC_START, (uint8_t)state );
}

void    write_reset(void* obj, const bool state)
{
	digitalWrite( PIN_ADC_RST  , (uint8_t)state );
}

void    spi_exchange_array(void* obj, const uint8_t tx[], uint8_t rx[], const uint8_t len)
{
	write_cs(obj, 0);
	SPI.transferBytes(tx, rx, len);
	write_cs(obj, 1);
}

uint8_t spi_exchange_byte(void* obj, const uint8_t tx)
{
	return SPI.transfer(tx);
}

//#endregion ads_hal_t functions

// ADS127L11 instance
ads127l11_t  ads =
{
	.hal = 
	{
		.param              = nullptr             ,
		.delay_ms           = &delay_ms           ,
		.write_cs           = &write_cs           ,
		.write_start        = &write_start        ,
		.write_reset        = &write_reset        ,
		.spi_exchange_array = &spi_exchange_array ,
		.spi_exchange_byte  = &spi_exchange_byte  
	},
	.reg = {0},
	.cfg =
	{
		.flags =
		{
			.status_byte_en = ( (ads127l11_config4_reg_t)ADS127L11_CONFIG4_REG ).status,
			.crc_en         = ( (ads127l11_config4_reg_t)ADS127L11_CONFIG4_REG ).spi_crc,
			.is_16_bit      = ( (ads127l11_config4_reg_t)ADS127L11_CONFIG4_REG ).data,
			.spi_3_wire     = 0,
		}
	}
};

void setup(void) 
{
	// Wait power to be stable
	delay(5000);
	
	// Start UART for debug
	Serial.begin( 115200 );
	
	// ADC127L11
	pinMode( PIN_ADC_CS      , OUTPUT       );
	pinMode( PIN_ADC_RST     , OUTPUT       );
	pinMode( PIN_ADC_RDY     , INPUT_PULLUP );
	pinMode( PIN_ADC_EXT_CLK , OUTPUT       );
	pinMode( PIN_ADC_START   , OUTPUT       );
	
	// SPI Pins
	SPI.begin
	(
		PIN_ADC_SCLK, // MCU_SCLK <-> ADC_SCLK
		PIN_ADC_SDO , // MCU_SDI  <-> ADC_SDO
		PIN_ADC_SDI , // MCU_SDO  <-> ADC_SDI
		PIN_ADC_CS    // ADC_CS
	);
	SPI.setFrequency( ADS127L11_SCLK_FREQ_HZ  );
	SPI.setBitOrder(  ADS127L11_SPI_BIT_ORDER );
	SPI.setDataMode(  ADS127L11_SPI_MODE      );
	
	// Start ADS127L11
	bool setup_res = ads127l11_setup
	(
		&ads,
		(ads127l11_config1_reg_t)ADS127L11_CONFIG1_REG,
		(ads127l11_config2_reg_t)ADS127L11_CONFIG2_REG,
		(ads127l11_config3_reg_t)ADS127L11_CONFIG3_REG,
		(ads127l11_config4_reg_t)ADS127L11_CONFIG4_REG
	);
	
	// Check ADS127L11 initialization
	Serial.printf("ADS Setup  = %s\r\n", setup_res ? "ok" : "failed" );
	for(uint8_t i=0; i < ADS127L11_NUM_REGISTERS; i++)
	{
		Serial.printf(
			"ADS127L11 REG[%8s] = 0x%02X\r\n", 
			ADS127L11_GET_REGISTER_NAME( i ),
			ads127l11_read_register( &this->ads, i )
		);
	}
	
	// Get samples
	uint32_t time_last_print = millis();
	uint32_t time_now        = 0;
	uint32_t time_elapsed    = 0;
	uint32_t ready_counter   = 0;	
	ads127l11_ch_data_t    ads_ch_data;
	while(1)
	{
		// ADS127L11 is ready
		if( !digitalRead( PIN_ADC_RDY ) )
		{
			ads127l11_read_data(&ads, &ads_ch_data);
			ready_counter++;
		}

		// Show how many samples happened in the last second
		time_now = millis();
		time_elapsed = (uint32_t)(time_now - time_last_print);
		if( time_elapsed >= 1000U )
		{
			time_last_print = time_now;
			Serial.printf("SampleRate = %d, Sample = %d\r\n", ready_counter, ads_ch_data.data);
			ready_counter = 0;
		}
	}
}

void loop(void)
{
    //
}