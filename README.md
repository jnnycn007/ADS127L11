<h1 align="center">ADS127L11 ADC Library</h1>
<br>
<p align="center">
<a href="https://www.ti.com/product/ADS127L11" title="Product Details">Product Details</a> |
<a href="https://www.ti.com/lit/ds/symlink/ads127l11.pdf" title="Datasheet">Datasheet</a> |
<a href="https://www.ti.com/tool/download/SBAC296" title="C Example Code">SBAC296 Example Code</a>
</p>
<br>

## :ledger: Overview

**Description**<br>
This library aims to be a port of the <a href="https://www.ti.com/tool/download/SBAC296" title="C Example Code">SBAC296 Example Code</a> provided by Texas Instrument.

**Hardware Abstraction Layer**<br>
Use a `ads_hal_t` constant to set your SPI and GPIO functions.

```C
typedef struct _ads_hal_t
{
    // Callback handlers extra param
    void*                   param;                //!< Extra param for callback handlers

    // System
    cb_delay_ms_t           delay_ms;             //!< Wait milliseconds

    // GPIO
    cb_write_pin_t          write_cs;             //!< Write SPI CS Pin status
    cb_write_pin_t          write_start;          //!< Write START Pin status
    cb_write_pin_t          write_reset;          //!< Write RESET Pin status

    // SPI
    cb_spi_exchange_array_t spi_exchange_array;   //!< Send and receive multiple bytes over SPI
    cb_spi_exchange_byte_t  spi_exchange_byte;    //!< Send and receive a single byte over SPI
} ads_hal_t;
```

Save the `ads_hal_t` and the desired data mode inside the `ads127l11_t` instance and that's it.
```C
/// @brief ADS127L11 instance
typedef struct _ads127l11_t
{
    ads_hal_t            hal;                             //!< HAL Callback Functions
    uint8_t              reg[ ADS127L11_NUM_REGISTERS ];  //!< ADS127L11 Register Map
    ads127l11_mode_cfg_t cfg;                             //!< Constant configs
} ads127l11_t;
```

**Configuration**<br>
Initialize the `ads127l11_t` instance with `ads127l11_setup`
```C
/// @brief Setup the ADS127L11 before usage
///
/// @note Call 'ads127l11_write_register' for every other register configuration after calling this function
///
/// @param[in] ads 'ads127l11_t' instance
/// @param[in] config1_reg CONFIG1_REG register configuration
/// @param[in] config2_reg CONFIG2_REG register configuration
/// @param[in] config3_reg CONFIG3_REG register configuration
/// @param[in] config4_reg CONFIG4_REG register configuration
/// @return True if all config registers were written successfully
bool ads127l11_setup( ads127l11_t* ads,
    const ads127l11_config1_reg_t config1_reg,
    const ads127l11_config2_reg_t config2_reg,
    const ads127l11_config3_reg_t config3_reg,
    const ads127l11_config4_reg_t config4_reg );
```

Use `ads127l11_read_data` to get a sample when the ADC is ready.

```C
/// @brief ADC sample data information
typedef struct _ads127l11_ch_data_t
{
    uint8_t     status;   //!< Status
    uint8_t     crc;      //!< CRC
    int32_t     data;     //!< ADC sample as signed 32-bit word
} ads127l11_ch_data_t;

/// @brief Read sample
/// @param[in] ads 'ads127l11_t' instance
/// @param[out] data Pointer to where data will be saved (can be NULL)
/// @return ADC sample as a signed 32-bit word
int32_t  ads127l11_read_data(ads127l11_t* ads, ads127l11_ch_data_t* data);
```
