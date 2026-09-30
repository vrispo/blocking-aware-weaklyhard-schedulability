/**
 * @file c_utility.cpp
 * @brief Implementation of common utility functions
 * @author Veronica Rispo et al.
 * @date 2026
 * @details This file provides general-purpose utility functions used throughout
 * the WeaklyHard framework, including assertion and error handling utilities.
 * 
 * Current utilities:
 * - Custom assertion with error message output
 * 
 * The my_assert function is useful for runtime validation of critical invariants
 * and debugging during development and testing phases.
 * 
 * @see c_utility.h for interface
 */

#include "c_utility.h"
#include <iostream>

void my_assert(std::string msg){
	std::cerr << "My_assert failed. " << msg << std::endl;
	std::abort();
}
