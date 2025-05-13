/**
 * @file ook.c
 * @brief OOK Transmitter Driver
 *
 * Listing file author: Silvano Cortesi
 *
 * Listing file notice:
 *   Licensed under LGPL-3.0
 *   File Version: 1.0.0
 */

#include "ook.h"
#include <stdlib.h>
#include <string.h>

// ======== Macro Quick Access
// =================================================================

#define DRVR_LOG(msg)                                                          \
  if (h != 0 && h->log != 0) {                                                 \
    h->log(msg, false, false, 0);                                              \
  }

#define DRVR_LOG_VAR(msg, val)                                                 \
  if (h != 0 && h->log != 0) {                                                 \
    h->log(msg, false, true, val);                                             \
  }

#define DRVR_ERR(msg)                                                          \
  if (h != 0 && h->log != 0) {                                                 \
    h->log(msg, true, false, 0);                                               \
  }

#define DRVR_ERR_VAR(msg, val)                                                 \
  if (h != 0 && h->log != 0) {                                                 \
    h->log(msg, true, true, val);                                              \
  }

// ======== Private Prototypes (not shared)
// =================================================================
static ook_err_t validate_handle(const struct ook_h *h);

// ======== Private Functions
// =================================================================

// Check that the handles are not null pointers
ook_err_t validate_handle(const struct ook_h *h) {
  ook_err_t err = E_OOK_SUCCESS;

  if (h == 0) {
    return E_OOK_NULLPTR_ERR;
  }

  if (h->spi_write == 0) {
    DRVR_ERR("spi_write is nullptr!");
    err |= E_OOK_NULLPTR_ERR;
  }

  if (h->spi_speed == 0) {
    DRVR_ERR("spi_speed is nullptr!");
    err |= E_OOK_NULLPTR_ERR;
  }

  if (h->sleep_ms == 0) {
    DRVR_ERR("sleep_ms is nullptr!");
    err |= E_OOK_NULLPTR_ERR;
  }

  return err;
}

// Public functions
size_t ook_bytes_to_physymbols(const uint8_t *input_data, size_t input_size,
                               enum phy_symbol *output_buffer, size_t output_buffer_size)
{
    if (input_data == NULL || output_buffer == NULL) {
        return 0;
    }

    // Each byte expands to 8 symbols
    if (output_buffer_size < input_size * 8) {
        return 0;
    }

    // If input size is 0, no symbols are written.
    if (input_size == 0) {
        return 0;
    }

    // --- Conversion Logic ---
    size_t output_index = 0; // Index for the output buffer

    // Iterate over each byte in the input data array
    for (size_t i = 0; i < input_size; i++) {
        uint8_t current_byte = input_data[i];

        // Iterate over each bit in the current byte, from MSB (bit 7) to LSB (bit 0)
        for (int j = 7; j >= 0; j--) {
            // Check the value of the j-th bit
            // (current_byte >> j) shifts the bit to the LSB position
            // & 0x01 isolates that bit
            if ((current_byte >> j) & 0x01) {
                // Bit is 1
                output_buffer[output_index] = BIT_ONE;
            } else {
                // Bit is 0
                output_buffer[output_index] = BIT_ZERO;
            }
            output_index++; // Move to the next position in the output buffer
        }
    }

    return output_index;
}

ook_err_t ook_init(const struct ook_h *h) {
  ook_err_t err = E_OOK_SUCCESS;

  // Check if handles are valid
  err |= validate_handle(h);
  if (err)
    return err;

  // Disable spi
  return h->spi_enable(false);
}

ook_err_t ook_send_wuc(const struct ook_h *h, struct wuc_pattern pattern) {
  const uint8_t A_data[PHYSYMBOL_SIZE] = _PHYSYMBOL_A;
  const uint8_t B_data[PHYSYMBOL_SIZE] = _PHYSYMBOL_B;
  const uint8_t One_data[PHYSYMBOL_SIZE] = _PHYSYMBOL_ONE;
  const uint8_t Zero_data[PHYSYMBOL_SIZE] = _PHYSYMBOL_ZERO;

  // Prepare data, copy in newly allocated array
  uint8_t *wuc_slow_data = NULL;
  switch (pattern.slow_pattern) {
  case PHYSYMBOL_A:
    wuc_slow_data = (uint8_t *)&A_data;
    break;
  case PHYSYMBOL_B:
    wuc_slow_data = (uint8_t *)&B_data;
    break;
  case PHYSYMBOL_ONE:
    wuc_slow_data = (uint8_t *)&One_data;
    break;
  case PHYSYMBOL_ZERO:
    wuc_slow_data = (uint8_t *)&Zero_data;
    break;
  }

  uint8_t *wuc_fast_data =
      (pattern.fast_pattern_len > 0)
          ? (uint8_t *)malloc(pattern.fast_pattern_len * PHYSYMBOL_SIZE)
          : NULL;

  if (pattern.fast_pattern_len > 0) {
    for (size_t i = 0; i < pattern.fast_pattern_len; i++) {
      uint8_t *symbol = NULL;
      switch (pattern.fast_pattern[i]) {
      case PHYSYMBOL_A:
        symbol = (uint8_t *)&A_data;
        break;
      case PHYSYMBOL_B:
        symbol = (uint8_t *)&B_data;
        break;
      case PHYSYMBOL_ONE:
        symbol = (uint8_t *)&One_data;
        break;
      case PHYSYMBOL_ZERO:
        symbol = (uint8_t *)&Zero_data;
        break;
      }
      memcpy(&wuc_fast_data[i * PHYSYMBOL_SIZE], symbol, PHYSYMBOL_SIZE);
    }
  }

  ook_err_t err = E_OOK_SUCCESS;

  // Configure SPI
  err |= h->spi_enable(true);
  if (err) {
    DRVR_ERR("Failed to enable OOK spi");
    return err;
  }

  // Transmit wake-up signal for OOK transmitter
  err |= h->spi_speed(PHYSPEED_16384);
  if (err) {
    DRVR_ERR("Failed to set 16384 baudrate on OOK spi for wakeup");
    return err;
  }
  uint8_t enabledata[] = {0xFC};
  err |= h->spi_write(enabledata, sizeof(enabledata));
  if (err) {
    DRVR_ERR("Failed to write ook wakeup pattern to OOK spi");
    return err;
  }

  // Transmit slow pattern
  err |= h->spi_speed(pattern.slow_speed);
  if (err) {
    DRVR_ERR("Failed to set slow baudrate on OOK spi");
    return err;
  }
  err |= h->spi_write(wuc_slow_data, PHYSYMBOL_SIZE);
  if (err) {
    DRVR_ERR("Failed to write slow data to OOK spi");
    return err;
  }

  // Delay 1ms
  h->sleep_ms(1);

  // Transmit fast pattern
  err |= h->spi_speed(pattern.fast_speed);
  if (err) {
    DRVR_ERR("Failed to set fast baudrate on OOK spi");
    return err;
  }
  err |= h->spi_write(wuc_fast_data, pattern.fast_pattern_len * PHYSYMBOL_SIZE);
  if (err) {
    DRVR_ERR("Failed to write fast data to OOK spi");
    return err;
  }

  // Disable SPI
  err |= h->spi_enable(false);
  if (err) {
    DRVR_ERR("Failed to disable OOK spi");
    return err;
  }

  return err;
}
