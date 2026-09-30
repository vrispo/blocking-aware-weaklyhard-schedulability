/**
 * @file weaklydatasetJSON.h
 * @brief JSON-based task set generation, import, and result export
 * @author Veronica Rispo
 * @date 2026
 * @details This file provides utilities for working with task sets in JSON format:
 * - Generating synthetic task sets with specified timing and constraint parameters
 * - Importing task sets from JSON files
 * - Exporting analysis results in JSON format
 * The module handles both schedulable and unschedulable task set generation,
 * with support for weakly-hard (m,k) constraints and critical sections.
 */

#pragma once
#include "mixed_tasks.h"
#include <fstream>

/**
 * @brief Import task sets from a JSON file stream
 * @param MyFile Input file stream opened in read mode from a JSON task set file
 * @return Vector of imported task sets (mixedtaskset objects)
 * @details Parses JSON-formatted task set file and reconstructs mixedtaskset objects.
 * Each JSON object in the file should represent a complete task set with all task parameters.
 * @throws std::runtime_error if JSON format is invalid or required fields are missing
 * @see generate_export_tasksets_MILP()
 */
std::vector<mixedtaskset> import_tasksets(std::ifstream& MyFile);

/**
 * @brief Generate and export synthetic task sets to JSON file
 * @param num_taskset Number of task sets to generate
 * @param U Target system utilization (0.0 to 1.0)
 * @param N Number of tasks per task set
 * @param json_file Output file stream for writing JSON task sets
 * @param m Parameter m of (m,k) weakly-hard constraint
 * @param k Parameter k of (m,k) weakly-hard constraint
 * @param crit_sec_perc Critical section length as percentage of WCET (0-100)
 * @param T_range_from Minimum task period for generation
 * @param T_range_to Maximum task period for generation
 * @return Vector of generated task sets stored in memory
 * @details Generates random task sets with:
 * - Specified total utilization U
 * - Periods uniformly distributed in [T_range_from, T_range_to]
 * - At least one task with (m,k) weakly-hard constraint
 * - Deadlines equal to periods (implicit deadline model)
 * - Automatic WCET computation to achieve target utilization
 * 
 * Task sets are written to json_file in JSON format and also returned in memory.
 * @see generate_export_tasksets_MILP_unsch()
 */
std::vector<mixedtaskset> generate_export_tasksets_MILP(const int num_taskset, const float U, const int N, std::ofstream& json_file, const int m, const int k, int crit_sec_perc, const int T_range_from, const int T_range_to);

/**
 * @brief Generate and export task sets ensuring at least one unschedulable task set
 * @param num_taskset Number of task sets to generate
 * @param U Target system utilization (0.0 to 1.0)
 * @param N Number of tasks per task set
 * @param json_file Output file stream for writing JSON task sets
 * @param m Parameter m of (m,k) weakly-hard constraint
 * @param k Parameter k of (m,k) weakly-hard constraint
 * @param crit_sec_perc Critical section length as percentage of WCET (0-100)
 * @param T_range_from Minimum task period for generation
 * @param T_range_to Maximum task period for generation
 * @param N_mk Number of tasks to which (m,k) constraints are applied
 * @return Vector of generated task sets
 * @details Similar to generate_export_tasksets_MILP() but additionally ensures that
 * at least one task set contains an unschedulable task (one that cannot meet all deadlines).
 * This is achieved by forcing the last task in the set to be unschedulable via parameter
 * forcing and utilization adjustment.
 * 
 * Useful for creating diverse test sets that span both schedulable and unschedulable regions.
 * @see generate_export_tasksets_MILP()
 */
std::vector<mixedtaskset> generate_export_tasksets_MILP_unsch(const int num_taskset, const float U, const int N, std::ofstream& json_file, const int m, const int k, int crit_sec_perc, const int T_range_from, const int T_range_to, int N_mk);

/**
 * @brief Export comparative analysis results to JSON file
 * @param our_res Vector of response times from first analysis method
 * @param their_res Vector of response times from second analysis method
 * @param our_sum Sum/count of first method results
 * @param their_sum Sum/count of second method results
 * @param json_file Output file stream for writing results
 * @param min_t_our Minimum runtime (us) for first method
 * @param max_t_our Maximum runtime (us) for first method
 * @param med_t_our Median runtime (us) for first method
 * @param min_t_their Minimum runtime (us) for second method
 * @param max_t_their Maximum runtime (us) for second method
 * @param med_t_their Median runtime (us) for second method
 * @details Writes comparative analysis results including response times and timing statistics
 * from two different analysis methods (e.g., analytic vs MILP) to JSON format.
 */
void export_results(const std::vector<int> our_res, const std::vector<int> their_res, int our_sum, int their_sum, std::ofstream& json_file, long long min_t_our, long long max_t_our, double med_t_our, long long min_t_their, long long max_t_their, double med_t_their);

/**
 * @brief Export comparative analysis results with vector references
 * @param our_res Reference to vector of response times from first analysis method
 * @param their_res Reference to vector of response times from second analysis method
 * @param json_file Output file stream for writing results
 * @param min_t_our Minimum runtime (us) for first method
 * @param max_t_our Maximum runtime (us) for first method
 * @param med_t_our Median runtime (us) for first method
 * @param min_t_their Minimum runtime (us) for second method
 * @param max_t_their Maximum runtime (us) for second method
 * @param med_t_their Median runtime (us) for second method
 * @details Overloaded version using const references to avoid vector copying.
 * Functionality identical to export_results() with value parameters.
 */
void export_results(const std::vector<int>& our_res, const std::vector<int>& their_res, std::ofstream& json_file, long long min_t_our, long long max_t_our, double med_t_our, long long min_t_their, long long max_t_their, double med_t_their);

/**
 * @brief Export single method analysis results to JSON file
 * @param our_res Vector of response times from analysis
 * @param our_sum Sum/count of results
 * @param json_file Output file stream for writing results
 * @param min_t_our Minimum runtime (us)
 * @param max_t_our Maximum runtime (us)
 * @param med_t_our Median runtime (us)
 * @details Simplified export for single analysis method results, useful when comparing
 * against baseline or when only one analysis approach is applied.
 */
void export_results(const std::vector<int> our_res, int our_sum, std::ofstream& json_file, long long min_t_our, long long max_t_our, double med_t_our);