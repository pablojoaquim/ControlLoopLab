/*===========================================================================*/
/**
 * @file logger_sink_stdout.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * The implementation of the interface of the logging sink to implement the stdout
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
 * Header Files
 *===========================================================================*/
#include "logger_sink_stdout.h"
#include <iostream>

/*===========================================================================*
 * Local Preprocessor #define Constants
 *===========================================================================*/
// #define NDEBUG

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Object Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Variables Definitions
 *===========================================================================*/

/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/

/*===========================================================================*
 * Local Inline Function Definitions and Function-Like Macros
 *===========================================================================*/

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/
// Helper function to obtain a color depending on the logging level
static const char* levelColor(LogLevel lvl)
{
    switch(lvl)
    {
        case LogLevel::Debug: return "\033[36m"; // cyan
        case LogLevel::Info:  return "\033[32m"; // green
        case LogLevel::Warn:  return "\033[33m"; // yellow
        case LogLevel::Error: return "\033[31m"; // red
        default: return "\033[0m";
    }
}

// Helper function to obtain a string depending on the logging level
static const char* levelToStr(LogLevel lvl)
{
    switch(lvl)
    {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Error: return "ERROR";
        default: return "";
    }
}

/*****************************************************************************
 * Name         write
 * Description  write message implementation to the stdout as the logging output
 *****************************************************************************/
void StdoutSink::write(const LogRecord& r) noexcept
{
    const char* color = levelColor(r.level);
    const char* reset = "\033[0m";

    std::cout
        << color
        << "[" << levelToStr(r.level) << "]\t"
        << "[" << r.file << ":" << r.line << "]\t"
        << "[" << r.function << "]\t"
        << r.message
        << reset
        << "\n";
}

