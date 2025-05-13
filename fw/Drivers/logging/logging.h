/**
 ******************************************************************************
 * @file           : logging.h
 * @brief          : General Logging Configuration.
 ******************************************************************************
 */

#ifndef __LOGGING_H
#define __LOGGING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "ulog.h"

#ifdef ULOG_ENABLED
void rtt_logger_init(uint8_t rtt_buffer_idx, uint32_t (*get_ms_runtime)(void),
                     ulog_level_t log_level);
#endif

#ifdef ULOG_ENABLED
#define RTT_LOGGER_INIT(buffer_idx, get_ms_runtime, log_level)                 \
  rtt_logger_init(buffer_idx, get_ms_runtime, log_level)
#else
// uLog vanishes when disabled at compile time...
#define RTT_LOGGER_INIT(buffer_idx, get_ms_runtime, log_level)                 \
  do {                                                                         \
  } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LOGGING_H */
