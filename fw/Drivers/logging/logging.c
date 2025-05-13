/**
 ******************************************************************************
 * @file           : logging.c
 * @brief          : General Logging Configuration
 ******************************************************************************
 */

#include "logging.h"
#include "SEGGER_RTT.h"
#include "ulog.h"
#include <stdint.h>

#define UNUSED(...) (void)(__VA_ARGS__)

#define PRINTF_BUF_SIZE_BYTE 64

#ifdef ULOG_ENABLED

// Private variables and function prototypes
static uint32_t (*local_get_ms_runtime)(void) = NULL;
static uint8_t local_rtt_buffer_idx = 0;
void local_rtt_logger(int line, const char *file, ulog_level_t severity,
                      const char *msg);

// Functions
void rtt_logger_init(uint8_t rtt_buffer_idx, uint32_t (*get_ms_runtime)(void),
                     ulog_level_t log_level) {
  local_get_ms_runtime = get_ms_runtime;
  local_rtt_buffer_idx = rtt_buffer_idx;

  ULOG_INIT();
  ULOG_SUBSCRIBE((ulog_function_t)local_rtt_logger, log_level);
}

void local_rtt_logger(int line, const char *file, ulog_level_t severity,
                      const char *msg) {
  // CRITICAL_SECTION_ENTER();
  uint64_t ticks = local_get_ms_runtime != NULL ? local_get_ms_runtime() : 0;
  uint32_t ms = ticks % 1000;
  uint32_t s = ticks / 1000;
  uint32_t min = s / 60;
  s = s % 60;
  uint32_t h = min / 60;
  min = min % 60;
#ifdef ULOG_LOG_FILENAME
  SEGGER_RTT_printf(local_rtt_buffer_idx,
                    "%02ld:%02ld:%02ld.%03ld [%s] - %s:%d:\t%s\n", h, min, s,
                    ms, ulog_level_name(severity), file, line, msg);
#else
  UNUSED(line);
  UNUSED(file);
  SEGGER_RTT_printf(local_rtt_buffer_idx, "%02ld:%02ld:%02ld.%03ld [%s]: %s\n",
                    h, min, s, ms, ulog_level_name(severity), msg);
#endif
}
#endif
