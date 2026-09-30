/**
 * @file c_utility.h
 * @brief Common utility functions and assertion helpers
 * @author Veronica Rispo et al.
 * @date 2026
 * @details Provides general-purpose utility functions used across the WeaklyHard framework,
 * including assertion and error handling utilities.
 */

#pragma once
#include <string>

/**
 * @brief Custom assertion function with error message
 * @param msg Error message to display when assertion fails
 * @details This function performs a runtime assertion check and outputs a detailed
 * error message if the assertion fails. Useful for debugging and runtime validation
 * of critical invariants throughout the framework.
 * 
 * @note Implementation details and exact behavior depend on c_utility.cpp
 * 
 * @warning Use for debugging and development; consider implications for production code
 * @see c_utility.cpp
 */
void my_assert(std::string msg);