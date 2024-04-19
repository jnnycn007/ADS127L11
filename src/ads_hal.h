#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/// @brief Delay milliseconds
/// @param[in] param Additional argument for handler (ads_hal_t::param)
/// @param[in] time_ms Milliseconds to delay
typedef void (*cb_delay_ms_t)(void* param, const uint32_t time_ms);

/// @brief Change the state of a digital output pin
/// @param[in] param Additional argument for handler (ads_hal_t::param)
/// @param[in] state desired state (true for HIGH, false for LOW)
typedef void (*cb_write_pin_t)(void* param, const bool state);

/// @brief Send and Receive multiple bytes over SPI
/// @param[in] param Additional argument for handler (ads_hal_t::param)
/// @param[in] tx Data to send
/// @param[out] rx Where data received will be stored
/// @param[in] len Length of data being sent and received
typedef void (*cb_spi_exchange_array_t)(void* param, const uint8_t tx[], uint8_t rx[], const uint8_t len);

/// @brief Send and receive one byte over SPI
/// @param[in] param Additional argument for handler (ads_hal_t::param)
/// @param[in] tx Data to send
/// @return Byte received
typedef uint8_t (*cb_spi_exchange_byte_t)(void* param, const uint8_t tx);

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

#ifdef __cplusplus
}
#endif