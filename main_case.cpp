/**
 * @file main_case.cpp
 * @brief Case study analysis tool for hand-crafted task sets
 * @author Veronica Rispo et al.
 * @date 2026
 * @details This executable is designed for detailed analysis of specific, pre-defined task sets.
 * Unlike main_experiment.cpp which generates random task sets, this tool analyzes carefully
 * crafted case studies based on real-world systems or specific research scenarios.
 * 
 * Key features:
 * - Support for both preemptive and non-preemptive task models
 * - Mixed execution times and critical sections (NPS)
 * - Weakly-hard (m,k) constraint specification per task
 * - Detailed response time computation via MILP solver
 * - Structured result export to JSON
 * 
 * Case studies can be:
 * - Real systems extracted from literature
 * - Synthetic benchmarks designed to stress-test analysis algorithms
 * - Minimal examples for verification and validation
 * 
 * Task definitions are embedded in source code (commented sections) for easy modification.
 * Results are exported to a specified output directory for downstream analysis and visualization.
 * 
 * Usage: ./WeaklyHardCase <case_number> <results_directory>
 * 
 * @example
 * ./WeaklyHardCase 1 results/case_study_1
 * This generates analysis results for case 1 in results/case_study_1/ directory
 */

#include "json.hpp"
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include <chrono>
#include "mixed_tasks.h"
#include "weaklydatasetJSON.h"

/**
 * @brief Main entry point for case study analysis
 * @param argc Number of command-line arguments
 * @param argv Command-line arguments: [program_name] [results_directory]
 * @return Exit code: 0 on success, -1 on failure
 * @details Reads task definitions (currently hard-coded in source), performs
 * schedulability analysis, and exports results to JSON files in the specified directory.
 * 
 * Workflow:
 * 1. Validate command-line arguments
 * 2. Initialize task set with predefined tasks
 * 3. Perform schedulability analysis (hard real-time and weakly-hard if applicable)
 * 4. Compute response times for each task
 * 5. Export results to JSON files
 * 
 * Error handling:
 * - Missing arguments: prints usage message and exits with -1
 * - Invalid task parameters: throws exception with error details
 * - Directory creation failures: handled by std::filesystem
 * 
 * @see analyze_case_study()
 */
int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "usage: WeaklyHardCase <case_number> <results_dir>" << std::endl;
        return -1;
    }
    int case_number = 0;
    try {
        case_number = std::stoi(argv[1]);
        if (case_number < 0 || case_number > 4) {
            throw std::out_of_range("Invalid case number");
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: Invalid case number." << std::endl;
        return -1;
    }
    std::string res_dir = argv[2];

    bool job_continue = true;

    std::vector<mixedtask> tasks;
    double scale = 1;
    
    /**
     * @brief Lambda function for scaling task parameters
     * @details Useful for testing scenarios with different time scales
     */
    auto S = [&](int wcet) {
        return static_cast<int>(wcet * scale);
    };

    ////////////
    // Group 1: periodic tasks from core 2 + ISR tasks from core 3, suppressed one task to avoid U>1, mixed exec and blocking of other task
    ////////////

    // Setup tasks based on case_number
    switch (case_number)
    {
    case 0:
        // CASE 0 - suppressed overloading task, HRT schedulable (RM, OPTI)
        tasks.emplace_back(S(404),   2000,    2000,    S(404),   0, 1);
        tasks.emplace_back(S(931),   5000,    5000,    S(931),   0, 2);
        tasks.emplace_back(S(10468), 20000,   20000,   S(10468), 0, 3);
        tasks.emplace_back(S(3084),  50000,   50000,   S(3084),  0, 6);
        //tasks.emplace_back(S(9418),  100000,  100000,  S(9418),  0, 7);
        tasks.emplace_back(S(138),   200000,  200000,  S(138),   0, 8);
        tasks.emplace_back(S(137),   1000000, 1000000, S(137),   0, 9);
        // ISRs on core 3
        tasks.emplace_back(S(35),   9500, 9500, S(35),   0, 10);
        tasks.emplace_back(S(18),   9500, 9500, S(18),   0, 11);
        tasks.emplace_back(S(24),   9500, 9500, S(24),   0, 12);
        break;
    case 1: // CASE 1 - added 1597 units of non-preemptive blocking (non HRT sched)
    case 2: // CASE 2 - added 1597 units of non-preemptive blocking (WH sched)       
        tasks.emplace_back(S(404),   2000,    2000,    S(404),   0, 1);
        tasks.emplace_back(S(931),   5000,    5000,    S(931),   0, 2);
        tasks.emplace_back(S(10468), 20000,   20000,   S(8871), 1597, 3);
        tasks.emplace_back(S(3084),  50000,   50000,   S(3084),  0, 6);
        //tasks.emplace_back(S(9418),  100000,  100000,  S(9418),  0, 7);
        tasks.emplace_back(S(138),   200000,  200000,  S(138),   0, 8);
        tasks.emplace_back(S(137),   1000000, 1000000, S(137),   0, 9);
        // ISRs on core 3
        tasks.emplace_back(S(35),   9500, 9500, S(35),   0, 10);
        tasks.emplace_back(S(18),   9500, 9500, S(18),   0, 11);
        tasks.emplace_back(S(24),   9500, 9500, S(24),   0, 12);
        break;  
    case 3: // CASE 3 - added more blocking
    case 4: // CASE 4 - added more blocking 
          
        tasks.emplace_back(S(404),   2000,    2000,    S(404),   0, 1);
        tasks.emplace_back(S(931),   5000,    5000,    S(931),   0, 2);
        tasks.emplace_back(S(10468), 20000,   20000,   S(7275), 3193, 3);
        tasks.emplace_back(S(3084),  50000,   50000,   S(3084),  0, 6);
        //tasks.emplace_back(S(9418),  100000,  100000,  S(9418),  0, 7);
        tasks.emplace_back(S(138),   200000,  200000,  S(138),   0, 8);
        tasks.emplace_back(S(137),   1000000, 1000000, S(137),   0, 9);
        // ISRs on core 3
        tasks.emplace_back(S(35),   9500, 9500, S(35),   0, 10);
        tasks.emplace_back(S(18),   9500, 9500, S(18),   0, 11);
        tasks.emplace_back(S(24),   9500, 9500, S(24),   0, 12);
        break;
    default:
        break;
    }
    mixedtaskset taskset(tasks);
    // Setup (m,k) contraints based on case_number
    switch (case_number)
    {
    case 0: // CASE 0 - no (m,k) constraints (HRT schedulable)
    case 1: // CASE 1 - no (m,k) constraints (non HRT sched)
        break;
    case 2: // CASE 2 - task 0 has (m,k) constraint (2,10) depending on the scenario (WHRT schedulable)
    case 3: // CASE 3 - task 0 has (m,k) constraint (2,10) depending on the scenario (non WHRT schedulable)
        taskset.set_mk(0, std::pair<int,int>(2,10));
        break;
    case 4: // CASE 4 - task 0 and task 1 have (m,k) constraints (4,11) and (4,12) depending on the scenario (WHRT schedulable)
        taskset.set_mk(0, std::pair<int,int>(4,11));
        taskset.set_mk(1, std::pair<int,int>(4,12));
        break;
    default:
        break;
    }
    
    // Print task set details for verification
    taskset.print();
    std::cout << std::endl;
    taskset.set_prioRM();
    taskset.print();
    std::cout << "Utilization: " << taskset.get_U() << std::endl;

    // ---------------- RM ----------------
    auto rm_start = std::chrono::steady_clock::now();
    auto rm_res = compute_schedulability(taskset, job_continue);
    auto rm_end = std::chrono::steady_clock::now();
    int t_rm = std::chrono::duration_cast<std::chrono::microseconds>(rm_end - rm_start).count();
    bool rm_sched = (std::get<0>(rm_res) == Schedulable);
    std::cout << (rm_sched ? "RM schedulable\n" : "RM unschedulable\n");

    // ---------------- PA ----------------
    auto mio_start = std::chrono::steady_clock::now();
    auto mio_res = compute_priority_assignment(taskset, job_continue);
    auto mio_end = std::chrono::steady_clock::now();
    int t_mio = std::chrono::duration_cast<std::chrono::microseconds>(mio_end - mio_start).count();
    bool mio_sched = std::get<0>(mio_res);
    std::cout << (mio_sched ? "PA schedulable\n" : "PA unschedulable\n");

    // ---------------- EXPORT ----------------
    std::vector<int> res_mio = {mio_sched};
    std::vector<int> res_rm  = {rm_sched};

    std::filesystem::create_directories(res_dir);
    std::ofstream file(res_dir + "/results-" + std::to_string(case_number) + ".json");

    export_results(res_mio, res_rm, res_mio[0], res_rm[0], file, t_mio, t_mio,
            (double)t_mio, t_rm, t_rm, (double)t_rm);

    return 0;
}
