/**
MIT License

Copyright (c) 2019 R. Dunbar Poor <rdpoor@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * \file ulog.c
 *
 * \brief uLog: lightweight logging for embedded systems
 *
 * See ulog.h for sparse documentation.
 */

#include "ulog.h"
#include "ulog_port.h"
#ifdef ULOG_ENABLED  // whole file...

#include <stdio.h>
#include <string.h>
#include <stdarg.h>


// =============================================================================
// types and definitions

typedef struct {
  ulog_function_t fn;
  ulog_level_t threshold;
} subscriber_t;

// =============================================================================
// local storage

static subscriber_t s_subscribers[ULOG_MAX_SUBSCRIBERS];
static char s_message[ULOG_MAX_MESSAGE_LENGTH];

// =============================================================================
// user-visible code

void ulog_init() {
  memset(s_subscribers, 0, sizeof(s_subscribers));
}

// search the s_subscribers table to install or update fn
ulog_err_t ulog_subscribe(ulog_function_t fn, ulog_level_t threshold) {
  int available_slot = -1;
  int i;
  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn == fn) {
      // already subscribed: update threshold and return immediately.
      s_subscribers[i].threshold = threshold;
      return ULOG_ERR_NONE;

    } else if (s_subscribers[i].fn == NULL) {
      // found a free slot
      available_slot = i;
    }
  }
  // fn is not yet a subscriber.  assign if possible.
  if (available_slot == -1) {
    return ULOG_ERR_SUBSCRIBERS_EXCEEDED;
  }
  s_subscribers[available_slot].fn = fn;
  s_subscribers[available_slot].threshold = threshold;
  return ULOG_ERR_NONE;
}

// search the s_subscribers table to remove
ulog_err_t ulog_unsubscribe(ulog_function_t fn) {
  int i;
  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn == fn) {
      s_subscribers[i].fn = NULL;    // mark as empty
      return ULOG_ERR_NONE;
    }
  }
  return ULOG_ERR_NOT_SUBSCRIBED;
}

#ifndef ULOG_COLOR
const char *ulog_level_name(ulog_level_t severity) {
  switch(severity) {
   case ULOG_TRA_LEVEL: return "TRA";
   case ULOG_DBG_LEVEL: return "DBG";
   case ULOG_INF_LEVEL: return "INF";
   case ULOG_WRN_LEVEL: return "WRN";
   case ULOG_ERR_LEVEL: return "ERR";
   case ULOG_CRI_LEVEL: return "CRI";
   default: return "???";
  }
}
#else
#define BLK "\033[0;30m"
#define RED "\033[0;31m"
#define GRN "\033[0;32m"
#define YEL "\033[0;33m"
#define BLU "\033[0;34m"
#define MAG "\033[0;35m"
#define CYN "\033[0;36m"
#define WHT "\033[0;37m"
#define RST "\033[0m"

const char *ulog_level_name(ulog_level_t severity) {
  switch(severity) {
   case ULOG_TRA_LEVEL: return BLU "TRA" RST;
   case ULOG_DBG_LEVEL: return GRN "DBG" RST;
   case ULOG_INF_LEVEL: return CYN "INF" RST;
   case ULOG_WRN_LEVEL: return YEL "WRN" RST;
   case ULOG_ERR_LEVEL: return RED "ERR" RST;
   case ULOG_CRI_LEVEL: return MAG "CRI" RST;
   default: return "???";
  }
}
#endif

void ulog_message(int line, const char *file, ulog_level_t severity,
                  const char *fmt, ...) {
  ULOG_CRITICAL_ENTER();
  va_list ap;
  int i;
  va_start(ap, fmt);
  vsnprintf(s_message, ULOG_MAX_MESSAGE_LENGTH, fmt, ap);
  va_end(ap);

  const char *temp_file = strrchr(file, '/');

  if (temp_file == NULL) {
    temp_file = file;
  } else {
    temp_file += 1;
  }

  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn != NULL) {
      if (severity >= s_subscribers[i].threshold) {
        s_subscribers[i].fn(line, temp_file, severity, s_message);
      }
    }
  }
  ULOG_CRITICAL_EXIT();
}

// =============================================================================
// private code

#endif  // #ifdef ULOG_ENABLED
