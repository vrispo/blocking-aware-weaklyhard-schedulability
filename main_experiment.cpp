/**
 * @file main_experiment.cpp
 * @brief Main driver for experimental campaigns in weakly-hard schedulability analysis
 * @author Veronica Rispo et al.
 * @date 2026
 * @details This executable orchestrates large-scale experiments:
 * - Loads experiment configuration from JSON files
 * - Generates or reuses synthetic task sets
 * - Runs schedulability analysis across a sweep of parameters
 * - Exports results and timing statistics to JSON format
 * - Outputs are organized per sweep value with individual result files
 * 
 * Typical experiment workflow:
 * 1. Load configuration (generation, analysis, experiment parameters)
 * 2. Create experiment directory structure
 * 3. For each sweep value:
 *    a. Generate (or reuse) num_tasksets task sets
 *    b. Run schedulability analysis on each task set
 *    c. Aggregate results and timing statistics
 *    d. Export to JSON files
 * 
 * Usage: ./WeaklyHard <config_file.json>
 * 
 * @see plot_results.py for result visualization
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
 * @struct GenerationConfig
 * @brief Configuration parameters for synthetic task set generation
 * @details Specifies all aspects of how task sets should be randomly generated,
 * including number of tasks, timing parameters, utilization, and constraint settings.
 */
struct GenerationConfig {
    bool force_last_unsched;   ///< If true, force the last task to be unschedulable
    int N;                     ///< Number of tasks in each generated task set
    int N_mk;                  ///< Number of tasks to which (m,k) constraints are applied
    double U;                  ///< Target total system utilization (0.0 to 1.0)
    int T_min;                 ///< Minimum task period for random generation
    int T_max;                 ///< Maximum task period for random generation
    int m;                     ///< Parameter m of (m,k) weakly-hard constraint
    int k;                     ///< Parameter k of (m,k) weakly-hard constraint
    int crit_sec_perc;         ///< Critical section length as percentage of WCET (0-100)
};

/**
 * @struct AnalysisConfig
 * @brief Configuration parameters for schedulability analysis
 * @details Controls which scheduling policy and job handling strategy to use
 * during analysis.
 */
struct AnalysisConfig {
    bool job_continue;  ///< true: preemptive (jobs continue); false: non-preemptive (jobs killed on deadline miss)
};

/**
 * @struct ExperimentConfig
 * @brief Configuration for the experiment campaign and output
 * @details Specifies which parameter to vary (sweep), the range and step size,
 * number of task sets per sweep point, and where to store results.
 */
struct ExperimentConfig {
    std::string sweep_parameter;    ///< Parameter to sweep: "U", "N", "k", "m", or "crit_sec_perc"
    double sweep_start;             ///< Starting value for sweep parameter
    double sweep_end;               ///< Ending value for sweep parameter
    double sweep_step;              ///< Step increment for sweep parameter
    int num_tasksets;               ///< Number of task sets to generate/analyze per sweep point
    bool reuse_tasksets;            ///< true: reuse existing tasksets; false: generate new ones
    std::string results_directory;  ///< Output directory for experiment results
};

/**
 * @struct Configuration
 * @brief Top-level configuration structure combining all experiment parameters
 * @details Contains three configuration sections: generation, analysis, and experiment.
 * Loaded from JSON file via load_config().
 */
struct Configuration {
    GenerationConfig gen;   ///< Task set generation parameters
    AnalysisConfig ana;     ///< Schedulability analysis parameters
    ExperimentConfig exp;   ///< Experiment campaign parameters
};

/**
 * @brief Load experiment configuration from JSON file
 * @param filename Path to JSON configuration file
 * @param cfg Reference to Configuration struct to populate
 * @return true if configuration loaded successfully, false on error
 * @details Parses JSON file and extracts generation, analysis, and experiment parameters.
 * Validates that all required fields are present. Prints error message on parse failure.
 * 
 * Expected JSON structure:
 * @code
 * {
 *   "generation": { ... },
 *   "analysis": { ... },
 *   "experiment": { ... }
 * }
 * @endcode
 * 
 * @see print_config()
 * @throws std::exception on JSON parsing errors (caught internally)
 */
bool load_config(const std::string& filename, Configuration& cfg)
{
    std::ifstream file(filename);
    if (!file) return false;

    try {
        nlohmann::json j = nlohmann::json::parse(file);
        auto gen = j.at("generation");
        cfg.gen.force_last_unsched = gen.at("force_last_unsched");
        cfg.gen.N                  = gen.at("N");
        cfg.gen.U                  = gen.at("U");
        cfg.gen.T_min             = gen.at("T_min");
        cfg.gen.T_max             = gen.at("T_max");
        cfg.gen.m                 = gen.at("m");
        cfg.gen.k                 = gen.at("k");
        cfg.gen.crit_sec_perc     = gen.at("crit_sec_perc");
        cfg.gen.N_mk              = gen.at("N_mk");
        auto ana = j.at("analysis");
        cfg.ana.job_continue = ana.at("job_continue");
        auto exp = j.at("experiment");
        cfg.exp.sweep_parameter = exp.at("sweep_parameter");
        cfg.exp.sweep_start     = exp.at("sweep_start");
        cfg.exp.sweep_end       = exp.at("sweep_end");
        cfg.exp.sweep_step      = exp.at("sweep_step");
        cfg.exp.num_tasksets    = exp.at("num_tasksets");
        cfg.exp.reuse_tasksets  = exp.at("reuse_tasksets");
        cfg.exp.results_directory = exp.at("results_directory");
    }
    catch (const std::exception& e) {
        std::cerr << "Config error: " << e.what() << '\n';
        return false;
    }

    return true;
}

/**
 * @brief Print loaded configuration to standard output
 * @param cfg Reference to Configuration struct to display
 * @details Pretty-prints all configuration parameters organized by section
 * (generation, analysis, experiment). Useful for debugging and configuration verification.
 */
void print_config(const Configuration& cfg)
{
    std::cout << "================ GENERATION ================\n";
    std::cout << "force_last_unsched = " << std::boolalpha << cfg.gen.force_last_unsched << '\n';
    std::cout << "N                  = " << cfg.gen.N << '\n';
    std::cout << "U                  = " << cfg.gen.U << '\n';
    std::cout << "m                  = " << cfg.gen.m << '\n';
    std::cout << "k                  = " << cfg.gen.k << '\n';
    std::cout << "T_min              = " << cfg.gen.T_min << '\n';
    std::cout << "T_max              = " << cfg.gen.T_max << '\n';
    std::cout << "crit_sec_perc      = " << cfg.gen.crit_sec_perc << '\n';
    std::cout << "N_mk               = " << cfg.gen.N_mk << '\n';
    std::cout << "================ ANALYSIS ================\n";
    std::cout << "job_continue       = " << std::boolalpha << cfg.ana.job_continue << '\n';
    std::cout << "================ EXPERIMENT ================\n";
    std::cout << "sweep_parameter    = " << cfg.exp.sweep_parameter << '\n';
    std::cout << "sweep_start        = " << cfg.exp.sweep_start << '\n';
    std::cout << "sweep_end          = " << cfg.exp.sweep_end << '\n';
    std::cout << "sweep_step         = " << cfg.exp.sweep_step << '\n';
    std::cout << "num_tasksets       = " << cfg.exp.num_tasksets << '\n';
    std::cout << "reuse_tasksets     = " << std::boolalpha << cfg.exp.reuse_tasksets << '\n';
    std::cout << "results_directory  = " << cfg.exp.results_directory << '\n';
    std::cout << "============================================\n";
}

std::string make_tag(const std::string& sweep, double v) {
    std::ostringstream ss;

    if (sweep == "U")
        ss << "U_" << std::fixed << std::setprecision(2) << v;
    else if (sweep == "N")
        ss << "N_" << static_cast<int>(v);
    else if (sweep == "m")
        ss << "m_" << static_cast<int>(v);
    else if (sweep == "k")
        ss << "k_" << static_cast<int>(v);
    else if (sweep == "crit_sec_perc")
        ss << "crit_" << static_cast<int>(v);
    else if (sweep == "N_mk")
        ss << "N_mk_" << static_cast<int>(v);
    else
        throw std::runtime_error("Invalid sweep parameter");

    return ss.str();
}
int main(int argc, char* argv[])
{
    bool verbose = false;
    if (argc != 2) {
        std::cerr << "usage: WeaklyHard <config-file-path>" << std::endl;
        return -1;
    }

    std::string config_file_path = argv[1];

    Configuration cfg;
    if (!load_config(config_file_path, cfg)) {
        std::cerr << "ERROR: invalid config file" << std::endl;
        return -1;
    }

    print_config(cfg);

    std::string exp_root = cfg.exp.results_directory;
    std::filesystem::create_directories(exp_root);
    std::filesystem::copy_file(config_file_path, exp_root + "/config.json", std::filesystem::copy_options::overwrite_existing);
    std::string tasksets_root = exp_root + "/tasksets";
    std::string results_root  = exp_root + "/results";
    std::filesystem::create_directories(tasksets_root);
    std::filesystem::create_directories(results_root);

    std::string sweep = cfg.exp.sweep_parameter;
    int steps = static_cast<int>((cfg.exp.sweep_end - cfg.exp.sweep_start) / cfg.exp.sweep_step + 1e-12);

    // =========================
    // GENERATION
    // =========================
    bool reuse = cfg.exp.reuse_tasksets && std::filesystem::exists(tasksets_root);

    if (!reuse) {
        std::cout << "Generating tasksets..." << std::endl;
        for (int i = 0; i <= steps; i++) {
            double v = cfg.exp.sweep_start + i * cfg.exp.sweep_step;
            std::string tag = make_tag(sweep, v);

            int N = cfg.gen.N;
            int N_mk = cfg.gen.N_mk;
            double U = cfg.gen.U;
            int m = cfg.gen.m;
            int k = cfg.gen.k;
            int crit = cfg.gen.crit_sec_perc;

            if (sweep == "U") U = v;
            else if (sweep == "N") N = (int)v;
            else if (sweep == "m") m = (int)v;
            else if (sweep == "k") k = (int)v;
            else if (sweep == "crit_sec_perc") crit = (int)v;
            else if (sweep == "N_mk") N_mk = (int)v;

            std::string file_path = tasksets_root + "/tasksets_" + tag + ".json";

            std::ofstream file(file_path);
            if (!file) {
                std::cerr << "Cannot open file: " << file_path << std::endl;
                return -1;
            }

            if (cfg.gen.force_last_unsched)
                generate_export_tasksets_MILP_unsch(cfg.exp.num_tasksets, U, N, file,
                                                     m, k, crit, cfg.gen.T_min, cfg.gen.T_max, N_mk);
            else
                generate_export_tasksets_MILP(cfg.exp.num_tasksets, U, N, file,
                                              m, k, crit, cfg.gen.T_min, cfg.gen.T_max);

            if (steps > 0 && i % (steps / 100 + 1) == 0) {
                int pct = (100 * i) / steps;
                std::cout << "\rProgress: " << pct << "%" << std::flush;
            }
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "Reusing tasksets..." << std::endl;
    }

    // =========================
    // ANALYSIS
    // =========================
    bool job_continue = cfg.ana.job_continue;
    std::cout << "Analyzing tasksets..." << std::endl;
    for (int i = 0; i <= steps; i++) {
        double v = cfg.exp.sweep_start + i * cfg.exp.sweep_step;
        std::string tag = make_tag(sweep, v);

        std::string file_path = tasksets_root + "/tasksets_" + tag + ".json";

        std::ifstream json_file(file_path);
        if (!json_file) {
            std::cerr << "Cannot open file: " << file_path << std::endl;
            return -1;
        }

        auto tasksets = import_tasksets(json_file);

        std::vector<int> res_our;
        std::vector<int> res_rm;

        long long min_t_our = 0, max_t_our = 0;
        long long min_t_rm  = 0, max_t_rm  = 0;
        double med_t_our = 0.0;
        double med_t_rm  = 0.0;
        int n_avg = 0;

        for (size_t t = 0; t < tasksets.size(); t++) {
            if (cfg.gen.force_last_unsched){
                auto sched_hard_rm = compute_HARD_schedulability(tasksets[t], job_continue);
                if (std::get<0>(sched_hard_rm) == Schedulable) {
                    std::cerr << "ERR:Taskset " << t << " is HARD schedulable under RM aRTA" << std::endl;
                } 
            }
            /* 
            // =========================
            // WEAKLY-HARD REAL-TIME PRIORITY ASSIGNMENT COMPARISON
            // =========================*/
            if( verbose ) {
                std::cout << "Analyzing taskset " << t << " with U = " << v << std::endl;
                tasksets[t].print();
                std::cout << std::endl;
                std::cout << "RM" << std::endl;
            }

            auto begin_rm = std::chrono::steady_clock::now();
            auto sched_rm = compute_schedulability(tasksets[t], job_continue);
            auto end_rm = std::chrono::steady_clock::now();
            res_rm.push_back(std::get<0>(sched_rm) == Schedulable ? 1 : 0);

            if( verbose ){
                std::cout << "RM schedulability: " << (std::get<0>(sched_rm) == Schedulable ? "Schedulable" : "Unschedulable") << std::endl;
                std::cout << "PRIO" << std::endl;
            }

            //Compute priority assignment 
            
            auto begin_our = std::chrono::steady_clock::now();
            auto res = compute_priority_assignment(tasksets[t], job_continue);
            auto end_our = std::chrono::steady_clock::now();

            if (std::get<0>(res)) {
                mixedtaskset res_taskset(std::get<1>(res).get_taskset());

                auto test_res = compute_schedulability(res_taskset, job_continue);

                res_our.push_back(
                    std::get<0>(test_res) == Schedulable ? 1 : 0
                );
            } else {
                if( verbose )
                    std::cout << "Priority assignment failed for taskset " << t << std::endl;
                res_our.push_back(0);
            }


            long long t_rm  = std::chrono::duration_cast<std::chrono::microseconds>(end_rm - begin_rm).count();
            long long t_our = std::chrono::duration_cast<std::chrono::microseconds>(end_our - begin_our).count();

            if (t == 0) {
                min_t_rm = max_t_rm = t_rm;
                min_t_our = max_t_our = t_our;
                med_t_rm = t_rm;
                med_t_our = t_our;
                n_avg = 1;
            } else {
                min_t_rm = std::min(min_t_rm, t_rm);
                max_t_rm = std::max(max_t_rm, t_rm);
                min_t_our = std::min(min_t_our, t_our);
                max_t_our = std::max(max_t_our, t_our);
                n_avg++;
                med_t_rm += (t_rm - med_t_rm) / n_avg;
                med_t_our += (t_our - med_t_our) / n_avg;
            }
        }

        std::ofstream out_res(
            results_root + "/results_" + tag + ".json"
        );

        if (!out_res) {
            std::cerr << "Cannot open results file" << std::endl;
            return -1;
        }

        export_results(res_our, res_rm, out_res, min_t_our, max_t_our, med_t_our, min_t_rm, max_t_rm, med_t_rm);
 
        if (steps > 0 && i % (steps / 100 + 1) == 0) {
            int pct = (100 * i) / steps;
            std::cout << "\rProgress: " << pct << "%" << std::flush;
        }
    }
    std::cout << std::endl;

    return 0;
}
