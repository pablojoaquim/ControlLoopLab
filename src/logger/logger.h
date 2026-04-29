#ifndef LOGGER_H
#define LOGGER_H

/*===========================================================================*/
/**
 * @file logger.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION: API for the logger
 *
 * Refactored to remove Singleton and support multi-sink logging.
 *
 * @section ABBR ABBREVIATIONS:
 *   - @todo List any abbreviations, precede each with a dash ('-').
 *
 * @section TRACE TRACEABILITY INFO:
 *   - Design Document(s):
 *     - @todo Update list of design document(s).
 *
 *   - Requirements Document(s):
 *     - @todo Update list of requirements document(s)
 *
 *   - Applicable Standards (in order of precedence: highest first):
 *     - @todo Update list of other applicable standards
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files (Common to C and C++)
 *===========================================================================*/

#ifdef __cplusplus
/*===========================================================================*
 * Header Files (C++ only)
 *===========================================================================*/
#include <vector>
#include <string>
#include "logger_sink.h"
#endif

/*===========================================================================*
 * Exported Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/
#define LOG_DEBUG(msg) \
    if(g_logger) g_logger->debug(msg, __FILE__, __LINE__, __func__)

#define LOG_INFO(msg) \
    if(g_logger) g_logger->info(msg, __FILE__, __LINE__, __func__)

#define LOG_WARN(msg) \
    if(g_logger) g_logger->warn(msg, __FILE__, __LINE__, __func__)

#define LOG_ERROR(msg) \
    if(g_logger) g_logger->error(msg, __FILE__, __LINE__, __func__)

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/
#ifdef __cplusplus

class Logger
{
public:
    Logger();
    ~Logger();

    void setLevel(LogLevel level);
    void addSink(ILogSink* s);

    void debug(std::string msg,
               const char* file,
               int line,
               const char* func);

    void info (std::string msg,
               const char* file,
               int line,
               const char* func);

    void warn (std::string msg,
               const char* file,
               int line,
               const char* func);

    void error(std::string msg,
               const char* file,
               int line,
               const char* func);

private:
    void log(const LogRecord& record);

    LogLevel currentLevel {LogLevel::Info};
    std::vector<ILogSink*> sinks;
};

#endif

/*===========================================================================*
 * Exported C Function Prototypes
 *===========================================================================*/
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

/*===========================================================================*
 * Exported C++ Globals
 *===========================================================================*/
#ifdef __cplusplus
extern Logger* g_logger;
#endif

/*===========================================================================*/
/*===========================================================================*/
#endif /* LOGGER_H */
