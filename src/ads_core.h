#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "./ads_hal.h"
#include "./registers.h"
#include "./crc.h"

/// @brief ADC sample data information
typedef struct _ads127l11_ch_data_t
{
    uint8_t     status;   //!< Status
    uint8_t     crc;      //!< CRC
    int32_t     data;     //!< ADC sample as signed 32-bit word
} ads127l11_ch_data_t;

/// @brief ADS127L11 instance
typedef struct _ads127l11_t
{
    ads_hal_t                     hal;                            //!< HAL Callback Functions
    const ads127l11_config4_reg_t cfg4_reg;                       //!< CONFIG4 Register Value
    const bool                    spi_3_wire;                     //!< Flag SPI 3 Wire is enabled
    uint8_t                       reg[ ADS127L11_NUM_REGISTERS ]; //!< ADS127L11 Register Map
} ads127l11_t;

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

/// @brief Read sample
/// @param[in] ads 'ads127l11_t' instance
/// @param[out] data Pointer to where data will be saved (can be NULL)
/// @return ADC sample as a signed 32-bit word
int32_t  ads127l11_read_data(ads127l11_t* ads, ads127l11_ch_data_t* data);

/// @brief Read sample as raw binary faster
/// @param[in] ads 'ads127l11_t' instance
/// @param[out] rx Pointer to where data will be saved
void     ads127l11_read_raw_24bit_crc_off_status_off(ads127l11_t* ads, uint8_t* rx);

/// @brief Write register
/// @param[in] ads 'ads127l11_t' instance
/// @param[in] address ADS127L11 address
/// @param[in] data New register value
void     ads127l11_write_register(ads127l11_t* ads, const uint8_t address, uint8_t data);

/// @brief Read register using SPI
/// @param[in] ads 'ads127l11_t' instance
/// @param[in] address ADS127L11 address
/// @return register value
uint8_t  ads127l11_read_register(ads127l11_t* ads, const uint8_t address);

/// @brief Get current register value from memory
/// @param[in] ads 'ads127l11_t' instance
/// @param[in] address ADS127L11 address
/// @return register value saved in memory
uint8_t  ads127l11_get_register_val(ads127l11_t* ads, const uint8_t address);

/// @brief Resets the device by SPI command to CONTROL register
/// @param[in] ads 'ads127l11_t' instance
void     ads127l11_reset_by_command(ads127l11_t* ads);

/// @brief Resets the device by SPI input pattern
/// @param[in] ads 'ads127l11_t' instance
void     ads127l11_reset_by_pattern(ads127l11_t* ads);

/// @brief Adjust calibration gain
/// @param[in] ads 'ads127l11_t' instance
/// @param[in] gain calibration gain
void    ads127l11_set_gain(ads127l11_t* ads, ads127l11_gain_reg_t gain);

/// @brief Adjust calibration offset
/// @param[in] ads 'ads127l11_t' instance
/// @param[in] off calibration offset
void    ads127l11_set_offset(ads127l11_t* ads,  ads127l11_offset_reg_t off);

#ifdef __cplusplus
}
#endif