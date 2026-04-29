/*===========================================================================*/
/**
 * @file katas.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Develop the code to solve the kata in this file
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
#include <cstdint>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include "katas.h"
#include <vector>
#include <fstream>
#include <filesystem>
#include <unordered_set>

/*===========================================================================*
 * Local Preprocessor #define Constants
 *===========================================================================*/
// #define NDEBUG

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/
#define QUEUE_SIZE 50

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/
typedef int q_elem;

typedef struct queue
{
    q_elem buf[QUEUE_SIZE];
    int head;
    int tail;
    int count;
} queue_t;

/*===========================================================================*
 * Local Object Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Variables Definitions
 *===========================================================================*/
// int working_arr[100];

/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/

/*===========================================================================*
 * Local Inline Function Definitions and Function-Like Macros
 *===========================================================================*/

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

void printArr(int curr, size_t len, int *arr)
{
    printf("-- %d --- ", curr);
    for (size_t i = 0; i < len; ++i)
    {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

void q_init(queue_t *q)
{
    q->head = q->tail = q->count = 0;
}

bool q_is_empty(queue_t *q)
{
    return (0 == q->count);
}

bool q_is_full(queue_t *q)
{
    return (QUEUE_SIZE == q->count);
}

bool q_push(queue_t *q, q_elem elem)
{
    if (q_is_full(q))
        return false;
    q->buf[q->tail] = elem;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;
    return true;
}

bool q_pop(queue_t *q, q_elem *out)
{
    if (q_is_empty(q))
        return false;
    *out = q->buf[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count--;
    return true;
}

/*****************************************************************************
 * Name         Methods declaration
 *****************************************************************************/
Vehicle::Vehicle()
{
    std::cout << "Constructor called" << std::endl;
    this->Speed = 0;
    this->Accel = 0;
}

Vehicle::~Vehicle()
{
    std::cout << "Destructor called" << std::endl;
}

void Vehicle::SetSpeed(int speed)
{
    this->Speed = speed;
}

int Vehicle::GetSpeed()
{
    return this->Speed;
}

void Vehicle::SetAccel(int accel)
{
    this->Accel = accel;
}

void Vehicle::Periodic1SCalc()
{
    this->Speed += this->Accel;
}

// void Vehicle::GetSound()
// {
//     std::cout << "Vroom!" << std::endl;
// }

Car::Car()
{
    std::cout << "Constructor called" << std::endl;
    this->Model = "";
}

Car::~Car()
{
    std::cout << "Destructor called" << std::endl;
}

void Car::SetModel(std::string model)
{
    this->Model = model;
}

std::string Car::GetModel()
{
    return this->Model;
}

void Car::SetModel(std::string model, int model_year)
{
    this->Model = model;
    this->ModelYear = model_year;
}

void Car::SetModel(std::string model, int model_year, std::string color)
{
    this->Model = model;
    this->ModelYear = model_year;
    this->Color = color;
}

int Car::GetModelYear()
{
    return this->ModelYear;
}

std::string Car::GetColor()
{
    return this->Color;
}

void Car::GetSound()
{
    std::cout << "Beep!" << std::endl;
}

std::string operator+(Vehicle &v1, Vehicle &v2)
{
    return "Combined speed: " + std::to_string(v1.GetSpeed() + v2.GetSpeed());
}

std::ostream &operator<<(std::ostream &os, const Vehicle &v)
{
    os << "Speed: " << v.Speed << ", Accel: " << v.Accel;
    return os;
}

void filesystem_example()
{
    try
    {
        Car car1, car2, car3, car4;
        car1.SetModel("Sedan", 2020, "Red");
        car2.SetModel("SUV", 2021, "Blue");
        car3.SetModel("Coupe", 2019, "Black");
        car4.SetModel("Hatchback", 2022, "White");
        std::vector<Car> cars;
        cars.push_back(car1);
        cars.push_back(car2);
        cars.push_back(car3);
        cars.push_back(car4);

        std::cout << car1 + car2 << std::endl;
        std::cout << "Vehicle 1: " << car1 << std::endl;
        std::cout << "Vehicle 2: " << car2 << std::endl;

        std::ofstream WriteFile("example.txt");
        // Enable exceptions
        WriteFile.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        std::cout << std::filesystem::current_path() << std::endl;

        if (WriteFile.is_open())
        {
            std::cout << "File opened successfully" << std::endl;
            WriteFile << "Info: System started" << std::endl;
            WriteFile << "Debug: Value received" << std::endl;

            for (auto &car : cars)
            {
                WriteFile << "Model: " << car.GetModel() << ", Year: " << car.GetModelYear() << ", Color: " << car.GetColor() << std::endl;
            }

            WriteFile.close();
        }
        else
        {
            std::cerr << "Failed to open file" << std::endl;
        }

        std::ifstream ReadFile("example.txt");
        // Enable exceptions
        ReadFile.exceptions(std::ifstream::badbit);

        if (ReadFile.is_open())
        {
            std::string line;
            while (std::getline(ReadFile, line))
            {
                std::cout << line << std::endl;
            }
            ReadFile.close();
        }
        else
        {
            std::cerr << "Failed to read file" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

/*****************************************************************************
 * Name         is_pangram
 * Description  A pangram is a sentence that contains every single letter of
 *              the alphabet at least once.
 *****************************************************************************/
bool is_pangram(const std::string &s)
{
    unsigned char abc[26] = {0};

    for (int i = 0; i < s.length(); i++)
    {
        char c = s.at(i);
        if (c >= 'A' && c <= 'Z')
        {
            c -= 'A';
            c += 'a';
        }
        if (c >= 'a' && c <= 'z')
            abc[c - 'a'] = 1;
    }

    for (int i = 0; i < 26; i++)
    {
        if (abc[i] == 0)
            return false;
    }

    return true;
}

/*****************************************************************************
 * Name         add_binary
 * Description  Adds two numbers and return the solution in binary represented as strings.
 *****************************************************************************/
std::string add_binary(uint64_t a, uint64_t b)
{
    uint64_t sum = a + b;

    std::string binary = "";

    if (sum == 0)
        return "0";

    while (sum != 0)
    {
        binary = std::to_string(sum % 2) + binary;
        sum /= 2;
    }

    return binary;
}

/*****************************************************************************
 * Name         rtrim
 * Description  Removes trailing whitespace from a string.
 *****************************************************************************/
std::string rtrim(std::string str)
{
    for (size_t i = str.length(); i > 0; --i)
    {
        if (!std::isspace(str[i - 1]))
        {
            return str.substr(0, i);
        }
    }
    return "";
}

/*****************************************************************************
 * Name         stripComments
 * Description  Removes comments from a string based on the provided marker characters.
 *****************************************************************************/
std::string stripComments(const std::string &str, const std::unordered_set<char> &markers)
{
    std::string result;
    std::vector<std::string> lines;

    for (size_t i = 0, last_pos = 0; i < str.length(); ++i)
    {
        if (str[i] == '\n' || i == str.length() - 1)
        {
            lines.push_back(str.substr(last_pos, i - last_pos + 1)); // Get the line up to the newline
            last_pos = i + 1;                                        // Update the position of the last newline
        }
    }

    size_t pos = 0;
    bool found_marker = false;
    for (const auto &line : lines)
    {
        pos = std::string::npos; // Reset position for each line
        found_marker = false;    // Reset marker found flag for each line

        // Check for each marker in the line and find the earliest occurrence
        for (auto &marker : markers)
        {
            pos = line.find(marker);
            if (pos != std::string::npos)
            {
                // Extract the part of the line before the marker and trim trailing whitespace
                std::string comment_free_line = rtrim(line.substr(0, pos));
                // Append the comment-free line to the result
                result.append(comment_free_line);
                // If the original line had a newline, we need to maintain it in the result
                if(line.find('\n') != std::string::npos)
                {
                    result.append("\n");
                }
                found_marker = true;
                break; // Stop checking other markers for this line
            }
        }

        // No marker found, add the whole line
        if (!found_marker)
        {
            result.append(rtrim(line));
            // If the original line had a newline, we need to maintain it in the result
            if(line.find('\n') != std::string::npos)
            {
                result.append("\n");
            }
        }
    }

    return result;
}

/*****************************************************************************
 * Name         snail
 * Description  Traverses a 2D array in a snail pattern and returns the elements in order.
 *****************************************************************************/
std::vector<int> snail(const std::vector<std::vector<int>> &snail_map) 
{
    std::vector<int> result={};

    int col=0;
    int row=0;

    int max_col=snail_map[0].size();
    int max_row=snail_map.size();

    while (col < max_col && row < max_row) {
        // Traverse right
        for (int i = col; i < max_col; ++i) {
            result.push_back(snail_map[row][i]);
        }
        row++;

        // Traverse down
        for (int i = row; i < max_row; ++i) {
            result.push_back(snail_map[i][max_col - 1]);
        }
        max_col--;

        if (row < max_row) {
            // Traverse left
            for (int i = max_col - 1; i >= col; --i) {
                result.push_back(snail_map[max_row - 1][i]);
            }
            max_row--;
        }

        if (col < max_col) {
            // Traverse up
            for (int i = max_row - 1; i >= row; --i) {
                result.push_back(snail_map[i][col]);
            }
            col++;
        }
    }

  return {result};
}

/*****************************************************************************
 * Name         permutations
 * Description  Given a string, return an array of all permutations of the characters in the string.
 *****************************************************************************/
std::vector<std::string> permutations_helper(std::string s) 
{
    std::vector<std::string> result = {};

    if (s.empty()) {
        result.push_back("");
        return result;
    }
    else if(s.length() == 1) {
        result.push_back(s);
        return result;
    }

    for (int i=0; i<s.length();i++)
    {
        char c = s.at(i);
        std::string remaining = s.substr(0, i) + s.substr(i + 1);
        std::vector<std::string> sub_permutations = permutations_helper(remaining);

        for (const auto& perm : sub_permutations) {
            result.push_back(c + perm);
        }
    }
    return result;
}

std::vector<std::string> permutations(std::string s)
{
    auto result = permutations_helper(s);
    sort(result.begin(), result.end());
    result.erase(unique(result.begin(), result.end()), result.end());
    return result;
}
