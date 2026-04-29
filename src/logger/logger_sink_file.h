#ifndef LOGGER_SINK_FILE_H
#define LOGGER_SINK_FILE_H

/*===========================================================================*/
/**
 * @file logger_sink_file.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION: API for the file logger
 *
 * @todo Add full description here
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
 * Header Files (C++ only)
 *===========================================================================*/
#ifdef __cplusplus
#include <fstream>
#include <string>
#include "logger_sink.h"
#endif

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/
#ifdef __cplusplus

class LoggerFileSink : public ILogSink
{
public:
    explicit LoggerFileSink(const std::string& filename,
                      LogLevel minLevel = LogLevel::Info);

    ~LoggerFileSink();

    void write(const LogRecord& record) noexcept override;

private:
    std::ofstream file_;
    LogLevel minLevel_;
};

#endif

/*===========================================================================*/
#endif /* LOGGER_SINK_FILE_H */
