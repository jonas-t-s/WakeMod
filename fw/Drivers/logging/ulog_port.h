#ifndef ULOG_PORT_H_
#define ULOG_PORT_H_

#include "segger/SEGGER_RTT_Conf.h"

#define ULOG_CRITICAL_ENTER() SEGGER_RTT_LOCK()
#define ULOG_CRITICAL_EXIT() SEGGER_RTT_UNLOCK()

#endif // ULOG_PORT_H_
