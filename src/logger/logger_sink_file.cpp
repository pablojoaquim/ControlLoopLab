/*===========================================================================*/
/**
 * @file logger_sink_file.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * The implementation of the interface of the logging sink to implement the file output
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
#include "logger_sink_file.h"
#include <iostream>

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/*****************************************************************************
 * Name         FileSink::FileSink
 * Description  Constructor - opens file
 *****************************************************************************/
FileSink::FileSink(const std::string& filename,
                   LogLevel minLevel)
    : file_(filename),
      minLevel_(minLevel)
{
    if (!file_.is_open())
    {
        std::cerr << "ERROR: Cannot open log file: " << filename << std::endl;
    }
}

/*****************************************************************************
 * Name         FileSink::~FileSink
 *****************************************************************************/
FileSink::~FileSink()
{
    if (file_.is_open())
    {
        file_.flush();
        file_.close();
    }
}

/*****************************************************************************
 * Name         FileSink::writeHeader
 *****************************************************************************/
void FileSink::writeHeader(const std::string& header)
{
    if (file_.is_open())
    {
        file_ << header << "\n";
    }
}

/*****************************************************************************
 * Name         FileSink::write
 * Description  Write log record to file
 *****************************************************************************/
void FileSink::write(const LogRecord& record) noexcept
{
    if (!file_.is_open())
        return;

    if (record.level < minLevel_)
        return;

    // CSV limpio: solo mensaje
    file_ << record.message << "\n";
}
