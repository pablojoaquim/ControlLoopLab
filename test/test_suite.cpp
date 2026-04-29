/*===========================================================================*/
/**
 * @file test_suite.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Unit tests
 * 
 * @todo Divide this file content using an abstraction layers concept
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
#include <iostream>
#include <memory>
#include "acutest.h"
#include "katas.h"

/*===========================================================================*
 * Local Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Local Type Declarations
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
 * Test cases
 *===========================================================================*/
#include "acutest.h"
#include <string.h>
#include <string>
#include "katas.h"

void test_SomeTest(void)
{
    TEST_CHECK(strcmp(highAndLow("4 5 29 54 4 0 -214 542 -64 1 -3 6 -6").c_str(),
                      "542 -214") == 0);
}

void test_SortTest(void)
{
    TEST_CHECK(strcmp(highAndLow("10 2 -1 -20").c_str(),
                      "10 -20") == 0);
}

void test_PlusMinusTest(void)
{
    TEST_CHECK(strcmp(highAndLow("1 -1").c_str(),
                      "1 -1") == 0);
}

void test_PlusPlusTest(void)
{
    TEST_CHECK(strcmp(highAndLow("1 1").c_str(),
                      "1 1") == 0);
}

void test_MinusMinusTest(void)
{
    TEST_CHECK(strcmp(highAndLow("-1 -1").c_str(),
                      "-1 -1") == 0);
}

void test_PlusMinusZeroTest(void)
{
    TEST_CHECK(strcmp(highAndLow("1 -1 0").c_str(),
                      "1 -1") == 0);
}

void test_PlusPlusZeroTest(void)
{
    TEST_CHECK(strcmp(highAndLow("1 1 0").c_str(),
                      "1 0") == 0);
}

void test_MinusMinusZeroTest(void)
{
    TEST_CHECK(strcmp(highAndLow("-1 -1 0").c_str(),
                      "0 -1") == 0);
}

void test_SingleTest(void)
{
    TEST_CHECK(strcmp(highAndLow("42").c_str(),
                      "42 42") == 0);
}


/*===========================================================================*
 * Test list
 *===========================================================================*/
TEST_LIST = {
    { "SomeTest", test_SomeTest },
    { "SortTest", test_SortTest },
    { "PlusMinusTest", test_PlusMinusTest },
    { "PlusPlusTest", test_PlusPlusTest },
    { "MinusMinusTest", test_MinusMinusTest },
    { "PlusMinusZeroTest", test_PlusMinusZeroTest },
    { "PlusPlusZeroTest", test_PlusPlusZeroTest },
    { "MinusMinusZeroTest", test_MinusMinusZeroTest },
    { "SingleTest", test_SingleTest },
    { NULL, NULL }
};
