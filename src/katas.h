#ifndef KATAS_H
#define KATAS_H

/*===========================================================================*/
/**
 * @file kata.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION: API for the Person module
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
 * Header Files (Common to C and C++)
 *===========================================================================*/
#include <stdint.h>
#include <stddef.h>
#include <unordered_map>
#include <unordered_set>

#ifdef __cplusplus
/*===========================================================================*
 * Header Files (C++ only)
 *===========================================================================*/
#include <string>
#endif

/*===========================================================================*
 * Exported Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/
#ifdef __cplusplus
template <typename T>
int add(T param1, T param2)
{
  return param1 + param2;
}

template <typename T>
class Box
{
    public:
        Box(T value) 
        {
            this->value = value;
        }
        
        T getValue() const 
        { 
            return value; 
        }

    private:
        T value;
};

template <typename T1,  typename T2>
class Pair
{
    public:
        Pair(T1 first, T2 second) 
        {
            this->first = first;
            this->second = second;
        }
        
        T1 getFirst() const 
        { 
            return first; 
        }

        T2 getSecond() const 
        { 
            return second; 
        }

        void display() const
        {
            std::cout << "First: " << first << ", Second: " << second << std::endl;
        }
        
    private:
        T1 first;
        T2 second;
};

// @todo: Add C++ class declarations here.
class Vehicle
{
    public:
        Vehicle();
        ~Vehicle();

        void SetSpeed(int speed);
        int GetSpeed();

        void SetAccel(int accel);
        void Periodic1SCalc();

        friend std::string operator+(const Vehicle& v1, const Vehicle& v2);
        friend std::ostream& operator<<(std::ostream& os, const Vehicle& v);

        virtual void GetSound() = 0;

    private:
        int Speed;
        int Accel;
};


class Car: public Vehicle
{
    public:
        Car();
        ~Car();

        void SetModel(std::string model);
        void SetModel(std::string model, int model_year);
        void SetModel(std::string model, int model_year, std::string color);
        std::string GetModel();
        int GetModelYear();
        std::string GetColor();

        void GetSound() override;
        
    private:
        std::string Model;
        int ModelYear;
        std::string Color;
};

#endif

/*===========================================================================*
 * Exported C Function Prototypes
 *===========================================================================*/
#ifdef __cplusplus
extern "C"
{
#endif
    // @todo: Add pure C function prototypes here.

#ifdef __cplusplus
} /* extern "C" */
#endif

/*===========================================================================*
 * Exported C++ Function Prototypes
 *===========================================================================*/
#ifdef __cplusplus
// @todo: Add pure C++ function prototypes here.

void filesystem_example();

/*****************************************************************************
 * @fn         encode
 * @brief      Takes a string and encode it
 * @param [in] p_what - The string to encode
 * @return     The encoded string
 ******************************************************************************/
std::string encode(const std::string &p_what);

/*****************************************************************************
 * @fn         decode
 * @brief      Takes an encoded string and decode it
 * @param [in] p_what - The string to decode
 * @return     The decoded string
 ******************************************************************************/
std::string decode(const std::string& p_what);

/*****************************************************************************
 * Name         is_pangram
 * Description  A pangram is a sentence that contains every single letter of 
 *              the alphabet at least once.
 *****************************************************************************/
/*****************************************************************************
 * @fn         is_pangram
 * @brief      A pangram is a sentence that contains every single letter of 
 *             the alphabet at least once. For example, the sentence 
 *             "The quick brown fox jumps over the lazy dog" is a pangram, 
 *             because it uses the letters A-Z at least once (case is irrelevant).
 * @param [in] s - The string to detect whether or not it is a pangram. Ignore 
 *                 numbers and punctuation.
 * @return     True if it is, False if not
 ******************************************************************************/
bool is_pangram(const std::string& s);

/*****************************************************************************
 * @fn         add_binary
 * @brief      Adds two numbers and return the solution in binary represented as strings.
 * @param [in] a - input1
 * @param [in] b - input2
 * @return     A binary number as a string
 ******************************************************************************/
std::string add_binary(uint64_t a, uint64_t b);

/*****************************************************************************
 * @fn         stripComments
 * @brief      Removes comments from a string based on the provided marker characters.
 * @param [in] str - The string from which to remove comments
 * @param [in] markers - A set of characters that indicate the start of a comment
 * @return     The string with comments removed
 ******************************************************************************/
std::string stripComments(const std::string& str, const std::unordered_set<char>& markers);

/*****************************************************************************
 * @fn         snail
 * @brief      Traverses a 2D array in a snail pattern and returns the elements in order.
 *              array = [[1,2,3],
 *              [4,5,6],
 *              [7,8,9]]
 *              snail(array) #=> [1,2,3,6,9,8,7,4,5]
 * @param [in] snail_map - The 2D array to traverse
 * @return     A vector containing the elements in snail order
 ******************************************************************************/
std::vector<int> snail(const std::vector<std::vector<int>> &snail_map);

/*****************************************************************************
 * @fn         permutations
 * @brief      Given a string, return an array of all permutations of the characters in the string.
 * @par Examples:
 * Your function should return ['ab', 'ba']
 * With input 'abc':
 * Your function should return ['abc','acb','bac','bca','cab','cba']
 * With input 'aabb':
 * Your function should return ['aabb', 'abab', 'abba', 'baab', 'baba', 'bbaa']
 * @param [in] s - The string to permute
 * @return     A vector containing all permutations of the input string
 ******************************************************************************/
std::vector<std::string> permutations(std::string s);

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* KATAS_H */
