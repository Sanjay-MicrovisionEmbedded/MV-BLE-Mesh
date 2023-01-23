/***************************************************************************//**
 * @file
 * @brief Application logging configuration
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_LOG_CONFIG_H
#define APP_LOG_CONFIG_H

#include "sl_iostream.h"

// <<< Use Configuration Wizard in Context Menu >>>

// <e APP_LOG_ENABLE> Application Logging
// <i> Enables Logging.
#define APP_LOG_ENABLE            1

// <h> Filter and trace

// <o APP_LOG_LEVEL> Level
// <APP_LOG_LEVEL_DEBUG=> DEBUG
// <APP_LOG_LEVEL_INFO=> INFO
// <APP_LOG_LEVEL_WARNING=> WARNING
// <APP_LOG_LEVEL_ERROR=> ERROR
// <APP_LOG_LEVEL_CRITICAL=> CRITICAL
// <i> Default: DEBUG
#define APP_LOG_LEVEL              APP_LOG_LEVEL_DEBUG

// <q APP_LOG_TRACE_ENABLE> Trace
// <i> Enables printing file, line and function information.
#define APP_LOG_TRACE_ENABLE      0

// </h>

// <e APP_LOG_OVERRIDE_DEFAULT_STREAM> Override default stream
// <i> Enable overriding the system level default stream to use for logging.
#define APP_LOG_OVERRIDE_DEFAULT_STREAM            0

// <o APP_LOG_STREAM_TYPE> Stream type
// <SL_IOSTREAM_TYPE_SWO=> SWO
// <SL_IOSTREAM_TYPE_RTT=> RTT
// <SL_IOSTREAM_TYPE_UART=> UART
// <SL_IOSTREAM_TYPE_VUART=> VUART
// <i> Default: UART
#define APP_LOG_STREAM_TYPE        SL_IOSTREAM_TYPE_UART

// <s APP_LOG_STREAM_INSTANCE> Preferred instance
// <i> Preferred IOStream instance name
#define APP_LOG_STREAM_INSTANCE    "vcom"

// </e>

// <e APP_LOG_PREFIX_ENABLE> Log level prefixes
// <i> Enables for logging.
#define APP_LOG_PREFIX_ENABLE                    1

// <s APP_LOG_LEVEL_DEBUG_PREFIX> Prefix for DEBUG
// <i> Log prefix for DEBUG
#define APP_LOG_LEVEL_DEBUG_PREFIX              "[D]"

// <s APP_LOG_LEVEL_INFO_PREFIX> Prefix for INFO
// <i> Log prefix for INFO
#define APP_LOG_LEVEL_INFO_PREFIX               "[I]"

// <s APP_LOG_LEVEL_WARNING_PREFIX> Prefix for WARNING
// <i> Log prefix for WARNING
#define APP_LOG_LEVEL_WARNING_PREFIX            "[W]"

// <s APP_LOG_LEVEL_ERROR_PREFIX> Prefix for ERROR
// <i> Log prefix for ERROR
#define APP_LOG_LEVEL_ERROR_PREFIX              "[E]"

// <s APP_LOG_LEVEL_CRITICAL_PREFIX> Prefix for CRITICAL
// <i> Log prefix for CRITICAL
#define APP_LOG_LEVEL_CRITICAL_PREFIX           "[C]"

// </e>

// <e APP_LOG_COLOR_ENABLE> Colors for terminal logging
// <i> Enables color prefixing for logging to terminal.
#define APP_LOG_COLOR_ENABLE                    0

// <h> Text color

// <o APP_LOG_LEVEL_DEBUG_COLOR> Color for DEBUG
// <APP_LOG_COLOR_BLACK=> BLACK
// <APP_LOG_COLOR_RED=> RED
// <APP_LOG_COLOR_GREEN=> GREEN
// <APP_LOG_COLOR_BLUE=> BLUE
// <APP_LOG_COLOR_YELLOW=> YELLOW
// <APP_LOG_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_COLOR_CYAN=> CYAN
// <APP_LOG_COLOR_WHITE=> WHITE
// <APP_LOG_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_COLOR_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: BRIGHT_WHITE
#define APP_LOG_LEVEL_DEBUG_COLOR              APP_LOG_COLOR_BRIGHT_WHITE

// <o APP_LOG_LEVEL_INFO_COLOR> Color for INFO
// <APP_LOG_COLOR_BLACK=> BLACK
// <APP_LOG_COLOR_RED=> RED
// <APP_LOG_COLOR_GREEN=> GREEN
// <APP_LOG_COLOR_BLUE=> BLUE
// <APP_LOG_COLOR_YELLOW=> YELLOW
// <APP_LOG_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_COLOR_CYAN=> CYAN
// <APP_LOG_COLOR_WHITE=> WHITE
// <APP_LOG_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_COLOR_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: BRIGHT_CYAN
#define APP_LOG_LEVEL_INFO_COLOR               APP_LOG_COLOR_BRIGHT_CYAN

// <o APP_LOG_LEVEL_WARNING_COLOR> Color for WARNING
// <APP_LOG_COLOR_BLACK=> BLACK
// <APP_LOG_COLOR_RED=> RED
// <APP_LOG_COLOR_GREEN=> GREEN
// <APP_LOG_COLOR_BLUE=> BLUE
// <APP_LOG_COLOR_YELLOW=> YELLOW
// <APP_LOG_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_COLOR_CYAN=> CYAN
// <APP_LOG_COLOR_WHITE=> WHITE
// <APP_LOG_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_COLOR_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: BRIGHT_YELLOW
#define APP_LOG_LEVEL_WARNING_COLOR            APP_LOG_COLOR_BRIGHT_YELLOW

// <o APP_LOG_LEVEL_ERROR_COLOR> Color for ERROR
// <APP_LOG_COLOR_BLACK=> BLACK
// <APP_LOG_COLOR_RED=> RED
// <APP_LOG_COLOR_GREEN=> GREEN
// <APP_LOG_COLOR_BLUE=> BLUE
// <APP_LOG_COLOR_YELLOW=> YELLOW
// <APP_LOG_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_COLOR_CYAN=> CYAN
// <APP_LOG_COLOR_WHITE=> WHITE
// <APP_LOG_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_COLOR_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: BRIGHT_RED
#define APP_LOG_LEVEL_ERROR_COLOR              APP_LOG_COLOR_BRIGHT_RED

// <o APP_LOG_LEVEL_CRITICAL_COLOR> Color for CRITICAL
// <APP_LOG_COLOR_BLACK=> BLACK
// <APP_LOG_COLOR_RED=> RED
// <APP_LOG_COLOR_GREEN=> GREEN
// <APP_LOG_COLOR_BLUE=> BLUE
// <APP_LOG_COLOR_YELLOW=> YELLOW
// <APP_LOG_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_COLOR_CYAN=> CYAN
// <APP_LOG_COLOR_WHITE=> WHITE
// <APP_LOG_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_COLOR_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: BRIGHT_WHITE
#define APP_LOG_LEVEL_CRITICAL_COLOR           APP_LOG_COLOR_BRIGHT_WHITE

// </h>

// <h> Background color

// <o APP_LOG_LEVEL_DEBUG_BACKGROUND_COLOR> Background color for DEBUG
// <APP_LOG_BACKGROUND_COLOR_NONE=> NONE
// <APP_LOG_BACKGROUND_COLOR_BLACK=> BLACK
// <APP_LOG_BACKGROUND_COLOR_RED=> RED
// <APP_LOG_BACKGROUND_COLOR_GREEN=> GREEN
// <APP_LOG_BACKGROUND_COLOR_BLUE=> BLUE
// <APP_LOG_BACKGROUND_COLOR_YELLOW=> YELLOW
// <APP_LOG_BACKGROUND_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_BACKGROUND_COLOR_CYAN=> CYAN
// <APP_LOG_BACKGROUND_COLOR_WHITE=> WHITE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: NONE
#define APP_LOG_LEVEL_DEBUG_BACKGROUND_COLOR              APP_LOG_BACKGROUND_COLOR_NONE

// <o APP_LOG_LEVEL_INFO_BACKGROUND_COLOR> Background color for INFO
// <APP_LOG_BACKGROUND_COLOR_NONE=> NONE
// <APP_LOG_BACKGROUND_COLOR_BLACK=> BLACK
// <APP_LOG_BACKGROUND_COLOR_RED=> RED
// <APP_LOG_BACKGROUND_COLOR_GREEN=> GREEN
// <APP_LOG_BACKGROUND_COLOR_BLUE=> BLUE
// <APP_LOG_BACKGROUND_COLOR_YELLOW=> YELLOW
// <APP_LOG_BACKGROUND_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_BACKGROUND_COLOR_CYAN=> CYAN
// <APP_LOG_BACKGROUND_COLOR_WHITE=> WHITE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: NONE
#define APP_LOG_LEVEL_INFO_BACKGROUND_COLOR               APP_LOG_BACKGROUND_COLOR_NONE

// <o APP_LOG_LEVEL_WARNING_BACKGROUND_COLOR> Background color for WARNING
// <APP_LOG_BACKGROUND_COLOR_NONE=> NONE
// <APP_LOG_BACKGROUND_COLOR_BLACK=> BLACK
// <APP_LOG_BACKGROUND_COLOR_RED=> RED
// <APP_LOG_BACKGROUND_COLOR_GREEN=> GREEN
// <APP_LOG_BACKGROUND_COLOR_BLUE=> BLUE
// <APP_LOG_BACKGROUND_COLOR_YELLOW=> YELLOW
// <APP_LOG_BACKGROUND_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_BACKGROUND_COLOR_CYAN=> CYAN
// <APP_LOG_BACKGROUND_COLOR_WHITE=> WHITE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: NONE
#define APP_LOG_LEVEL_WARNING_BACKGROUND_COLOR            APP_LOG_BACKGROUND_COLOR_NONE

// <o APP_LOG_LEVEL_ERROR_BACKGROUND_COLOR> Background color for ERROR
// <APP_LOG_BACKGROUND_COLOR_NONE=> NONE
// <APP_LOG_BACKGROUND_COLOR_BLACK=> BLACK
// <APP_LOG_BACKGROUND_COLOR_RED=> RED
// <APP_LOG_BACKGROUND_COLOR_GREEN=> GREEN
// <APP_LOG_BACKGROUND_COLOR_BLUE=> BLUE
// <APP_LOG_BACKGROUND_COLOR_YELLOW=> YELLOW
// <APP_LOG_BACKGROUND_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_BACKGROUND_COLOR_CYAN=> CYAN
// <APP_LOG_BACKGROUND_COLOR_WHITE=> WHITE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: NONE
#define APP_LOG_LEVEL_ERROR_BACKGROUND_COLOR              APP_LOG_BACKGROUND_COLOR_NONE

// <o APP_LOG_LEVEL_CRITICAL_BACKGROUND_COLOR> Background color for CRITICAL
// <APP_LOG_BACKGROUND_COLOR_NONE=> NONE
// <APP_LOG_BACKGROUND_COLOR_BLACK=> BLACK
// <APP_LOG_BACKGROUND_COLOR_RED=> RED
// <APP_LOG_BACKGROUND_COLOR_GREEN=> GREEN
// <APP_LOG_BACKGROUND_COLOR_BLUE=> BLUE
// <APP_LOG_BACKGROUND_COLOR_YELLOW=> YELLOW
// <APP_LOG_BACKGROUND_COLOR_MAGENTA=> MAGENTA
// <APP_LOG_BACKGROUND_COLOR_CYAN=> CYAN
// <APP_LOG_BACKGROUND_COLOR_WHITE=> WHITE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_RED=> BRIGHT_RED
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_GREEN=> BRIGHT_GREEN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_BLUE=> BRIGHT_BLUE
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_YELLOW=> BRIGHT_YELLOW
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_MAGENTA=> BRIGHT_MAGENTA
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_CYAN=> BRIGHT_CYAN
// <APP_LOG_BACKGROUND_COLOR_BRIGHT_WHITE=> BRIGHT_WHITE
// <i> Default: BRIGHT_RED
#define APP_LOG_LEVEL_CRITICAL_BACKGROUND_COLOR           APP_LOG_BACKGROUND_COLOR_BRIGHT_RED

// </h>

// </e>

// </e>

// <<< end of configuration section >>>

#endif // APP_LOG_CONFIG_H
