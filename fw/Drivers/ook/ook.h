/**
 * @file OOK.h
 * @brief OOK Transmitter Driver
 *
 * Listing file author: Silvano Cortesi
 *
 * Listing file notice:
 *   Licensed under LGPL-3.0
 *   File Version: 1.0.1
 */

#ifndef OOK_H
#define OOK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Preprocessor macros to convert byte array to physical symbols. Requires BIT_ONE and BIT_ZERO to be defined as a phy_symbol. Converts MAX 7 bytes (as fh101rf can not handle more)
#ifndef BIT_ONE
    #error "BIT_ONE must be defined as PHYSYMBOL_A, PHYSYMBOL_B, PHYSYMBOL_ONE, or PHYSYMBOL_ZERO"
#endif

#ifndef BIT_ZERO
    #error "BIT_ZERO must be defined as PHYSYMBOL_A, PHYSYMBOL_B, PHYSYMBOL_ONE, or PHYSYMBOL_ZERO"
#endif

#if !(BIT_ONE == PHYSYMBOL_A || BIT_ONE == PHYSYMBOL_B || BIT_ONE == PHYSYMBOL_ONE || BIT_ONE == PHYSYMBOL_ZERO)
    #error "BIT_ONE must be PHYSYMBOL_A, PHYSYMBOL_B, _ONE, or _ZERO"
#endif

#if !(BIT_ZERO == PHYSYMBOL_A || BIT_ZERO == PHYSYMBOL_B || BIT_ZERO == PHYSYMBOL_ONE || BIT_ZERO == PHYSYMBOL_ZERO)
    #error "BIT_ZERO must be PHYSYMBOL_A, PHYSYMBOL_B, PHYSYMBOL_ONE, or PHYSYMBOL_ZERO"
#endif

#define PHYSYMBOL_SIZE 4
#define _PHYSYMBOL_A {0x6A, 0xCD, 0x67, 0x4F}    // use ONLY in ook.h and ook.c
#define _PHYSYMBOL_B {0x6D, 0x38, 0x97, 0x73}    // use ONLY in ook.h and ook.c
#define _PHYSYMBOL_ONE {0xFF, 0xFF, 0xFF, 0xFF}  // use ONLY in ook.h and ook.c
#define _PHYSYMBOL_ZERO {0x00, 0x00, 0x00, 0x00} // use ONLY in ook.h and ook.c

/**
 * @brief Driver error codes
 */
typedef enum {
  E_OOK_SUCCESS = 0,            //!< Success
  E_OOK_NULLPTR_ERR = (1 << 0), //!< Nullpointer error
  E_OOK_COM_ERR = (1 << 1),     //!< Communication error
  E_OOK_CONFIG_ERR = (1 << 2),  //!< Configuration error
  E_OOK_NO_DATA_ERR = (1 << 3), //!< No Data available error
  E_OOK_ERR = (1 << 4),         //!< Other error
} ook_err_t;

enum phy_symbol { PHYSYMBOL_A, PHYSYMBOL_B, PHYSYMBOL_ONE, PHYSYMBOL_ZERO };

enum phy_speed {
  PHYSPEED_256,
  PHYSPEED_512,
  PHYSPEED_1024,
  PHYSPEED_2048,
  PHYSPEED_4096,
  PHYSPEED_8192,
  PHYSPEED_16384,
  PHYSPEED_32768,
};

struct wuc_pattern {
  enum phy_symbol slow_pattern;
  enum phy_speed slow_speed;
  enum phy_symbol *fast_pattern;
  uint8_t fast_pattern_len;
  enum phy_speed fast_speed;
};

/**
 * @brief Converts a byte array into an array of physical symbols.
 *
 * Each byte is converted into 8 physical symbols, where each bit
 * is represented by either BIT_ONE or BIT_ZERO as defined.
 *
 * @param input_data Pointer to the input byte array.
 * @param input_size The number of bytes in the input array.
 * @param output_buffer Pointer to the buffer where the physical symbols
 * will be written. Each element is a enum phy_symbol.
 * @param output_buffer_size The size of the output buffer in number of
 * enum phy_symbols. Should be at least 8 * input_size.
 * @return The number of enum phy_symbols written to the buffer,
 * or 0 if the output buffer is too small, input_data is NULL,
 * or output_buffer is NULL.
 */
size_t ook_bytes_to_physymbols(const uint8_t *input_data, size_t input_size,
                               enum phy_symbol *output_buffer, size_t output_buffer_size);

struct ook_h {
  /**
   * @brief Pointer to the device-specific spi-write function
   * @warning Required!
   *
   * @param data Pointer to the data to write.
   * @param len Length to write
   * @return @ref E_OOK_SUCCESS if write was successful, otherwise @ref
   * E_OOK_COM_ERR.
   */
  ook_err_t (*spi_write)(uint8_t *data, uint16_t len);

  /**
   * @brief Pointer to the device-specific spi-baudrate selection function
   * @warning Required!
   *
   * @param speed phy_speed indicating the needed SPI frequency.
   * @return @ref E_OOK_SUCCESS if write was successful, otherwise @ref
   * E_OOK_COM_ERR.
   */
  ook_err_t (*spi_speed)(enum phy_speed speed);

  /**
   * @brief Pointer to the device-specific spi-enable function
   * @warning Required!
   *
   * @param enable bool indicating whether spi needs to be enabled or disabled.
   * @return @ref E_OOK_SUCCESS if write was successful, otherwise @ref
   * E_OOK_COM_ERR.
   */
  ook_err_t (*spi_enable)(bool enable);

  /**
   * @brief Pointer to a sleep_ms function.
   * Called by the driver to sleep for ms milliseconds.
   *
   * @param ms the time to sleep
   */
  void (*sleep_ms)(uint32_t ms);

  // === Interface function pointers. Optional. ===

  /**
   * @brief Pointer to logging function.
   * Called by the driver to log status and error messages, with an optional
   * integer variable to log. Note that the string does not contain any
   * formatting specifiers, and should be logged as follows (if has_int_arg is
   * true):
   *
   * printf("%s: %s %i", is_err ? "ERR" : "LOG", msg, arg);
   *
   * @param msg the log message
   * @param is_err indicates if this is an error message
   * @param has_int_arg indicates if this message is accompanied by an integer
   * variable to log.
   * @param arg the integer variable to log if has_int_arg is true.
   */
  void (*log)(char *msg, bool is_err, bool has_int_arg, uint32_t arg);
};

ook_err_t ook_init(const struct ook_h *h);
ook_err_t ook_send_wuc(const struct ook_h *h, struct wuc_pattern pattern);

#endif // OOK_H
