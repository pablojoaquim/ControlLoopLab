/*===========================================================================*/
/**
 * @file logger.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Refactored Logger implementation without Singleton.
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files
 *===========================================================================*/
#include "logger.h"

/*===========================================================================*
 * Global Logger Instance Pointer
 *===========================================================================*/
Logger* g_logger = nullptr;

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/*****************************************************************************
 * Name         Logger::Logger
 * Description  Constructor
 *****************************************************************************/
Logger::Logger()
{
    // Optional debug
}

/*****************************************************************************
 * Name         Logger::~Logger
 * Description  Destructor
 *****************************************************************************/
Logger::~Logger()
{
    // Optional debug
}

/*****************************************************************************
 * Name         Logger::setLevel
 * Description  Set the logging level
 *****************************************************************************/
void Logger::setLevel(LogLevel level)
{
    currentLevel = level;
}

/*****************************************************************************
 * Name         Logger::addSink
 * Description  Add a logging sink
 *****************************************************************************/
void Logger::addSink(ILogSink* s)
{
    if (s)
    {
        sinks.push_back(s);
    }
}

/*****************************************************************************
 * Name         Logger::debug
 *****************************************************************************/
void Logger::debug(std::string msg,
                   const char* file,
                   int line,
                   const char* func)
{
    log({LogLevel::Debug, msg, file, line, func});
}

/*****************************************************************************
 * Name         Logger::info
 *****************************************************************************/
void Logger::info(std::string msg,
                  const char* file,
                  int line,
                  const char* func)
{
    log({LogLevel::Info, msg, file, line, func});
}

/*****************************************************************************
 * Name         Logger::warn
 *****************************************************************************/
void Logger::warn(std::string msg,
                  const char* file,
                  int line,
                  const char* func)
{
    log({LogLevel::Warn, msg, file, line, func});
}

/*****************************************************************************
 * Name         Logger::error
 *****************************************************************************/
void Logger::error(std::string msg,
                   const char* file,
                   int line,
                   const char* func)
{
    log({LogLevel::Error, msg, file, line, func});
}

/*****************************************************************************
 * Name         Logger::log
 * Description  Dispatch log record to sinks
 *****************************************************************************/
void Logger::log(const LogRecord& record)
{
    if (record.level < currentLevel)
        return;

    for (auto* s : sinks)
    {
        if (s)
        {
            s->write(record);
        }
    }
}
