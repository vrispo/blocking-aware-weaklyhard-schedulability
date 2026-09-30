/**
 * @file weaklydatasetJSON.cpp
 * @brief Implementation of JSON-based task set generation and export
 * @author Veronica Rispo
 * @date 2026
 * @details This file implements synthetic task set generation algorithms and JSON
 * serialization/deserialization for task sets and analysis results.
 * 
 * Key functionalities:
 * - Random task set generation with configurable utilization and period ranges
 * - Period generation using log-uniform distribution
 * - Automatic WCET assignment to achieve target system utilization
 * - JSON export of generated task sets
 * - Support for (m,k) weakly-hard constraints
 * - Support for critical sections (non-preemptive parts)
 * - Forced unschedulability mode for balanced test set generation
 * 
 * The module uses a fixed random seed for reproducibility, though actual generation
 * may not be deterministic in multi-threaded environments.
 * 
 * Random distributions:
 * - Periods: log-uniform distribution (favors shorter periods)
 * - Utilization adjustments: uniform distribution
 * 
 * @see weaklydatasetJSON.h for interface
 */

#include "weaklydatasetJSON.h"
#include "json.hpp"
#include <iostream>
#include <random>

using json = nlohmann::json;

std::mt19937 rng(12345); // fixed seed (task generation is anyways not deterministic if multithreaded processing mode is used)

double draw_uniform_distribution_real(double min, double max)
{
	if (min > max) throw std::invalid_argument("invalid distribution range");
	std::uniform_real_distribution<double> distribution(min, max);
	return distribution(rng);
}

int draw_uniform_distribution_integer(int min, int max)
{
	if (min > max) throw std::invalid_argument("invalid distribution range");
	std::uniform_int_distribution<int> distribution(min, max);
	return distribution(rng);
}

int draw_log_uniform_distribution_integer(int min, int max)
{
    if (min > max) throw std::invalid_argument("invalid distribution range");
	int granularity = 1;

	std::uniform_real_distribution<double> distribution(log(min), log(max + granularity));
	double floor = std::floor(exp(distribution(rng)) / static_cast<double>(granularity));
	int value = static_cast<int>(floor) * granularity;
	if (value < min) {
		value = min;
		std::cerr << "Error: invalid distribution range" << std::endl;
	}
	if (value > max) {
		value = max;
		std::cerr << "Error: invalid distribution range" << std::endl;
	}

	return value;
}

// Implements the UUniFast algorithm by Bini and Buttazzo ("Measuring the performance of schedulability tests", Real-Time Systems, 2005)
std::vector<double> uunifast(int num_tasks, double total_utilization)
{
	std::vector<double> utilizations(num_tasks);
	if (num_tasks < 1) {
		return utilizations;
	}

	double util_sum = total_utilization;
	for (int i = 0; i < num_tasks - 1; i++) {
		double base = draw_uniform_distribution_real(0.0, 1.0);
		double exponent = static_cast<double>(static_cast<double>(1.0) / static_cast<double>(num_tasks - i - 1)); 
		double next_util_sum = util_sum * pow(base, exponent);
		utilizations[i] = util_sum - next_util_sum;
		util_sum = next_util_sum;
	}
	utilizations[num_tasks - 1] = util_sum;

	return utilizations;
}

std::vector<mixedtaskset> import_tasksets(std::ifstream& MyFile){
    std::vector<mixedtaskset> tasksets;
    json j_tasksets;

    MyFile >> j_tasksets;

    for (auto& j_taskset : j_tasksets) {
		std::vector<mixedtask> tasks;
        int i = 0;
		for (auto& j_task : j_taskset) {
            //float U;
            float C;
            int P, D, m=0, k=1, RCT, NPS;
            bool isPeriodic;
            //bool isPreemptive;

			// Extract task parameters from JSON
			//U = j_task["U"];
			P = j_task["P"];
			C = j_task["C"];
            NPS = j_task["NPS"];
            RCT = j_task["RCT"];
			isPeriodic = j_task["isPeriodic"];
            //isPreemptive = j_task["isPreemptive"];
            json j_constraint = j_task["constraint"];
			D = j_constraint["D"];
			if (j_constraint.contains("m")) {
				m = j_constraint["m"];
				k = j_constraint["k"];
			}
            if (isPeriodic == true) {
                pair<int, int> mks = pair<int, int>(m, k);
                tasks.push_back(mixedtask(C, P, D, RCT, NPS, mks, i));
            }
            else {
                tasks.push_back(mixedtask(C, P, D, RCT, NPS, i));
            }
            i++;
		}
        mixedtaskset tmp_taskset = mixedtaskset(tasks);
        tmp_taskset.set_prioRM();
        tasksets.push_back(tmp_taskset);
    }

    return tasksets;
}

/* Randomly schedulable and randomly weakly-hard */
std::vector<mixedtaskset> generate_export_tasksets_MILP(const int num_taskset, const float U, const int N, std::ofstream& json_file, const int m, const int k, int crit_sec_perc, const int T_range_from, const int T_range_to){
    std::vector<mixedtaskset> tasksets;
    tasksets.reserve(static_cast<size_t>(num_taskset)); // avoids repeated reallocations
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::bernoulli_distribution d(0.5);
    int generated_count = 0;
    const int FLUSH_INTERVAL = 10; // flush output to file every N task sets

    if (!json_file.is_open()) {
        std::cerr << "generate_export_tasksets_MILP: could not open json_file\n";
        return std::vector<mixedtaskset>();
    }

    std::vector<int> T; T.reserve(N);
    std::vector<int> wcets; wcets.reserve(N);
    std::vector<int> D; D.reserve(N);
    std::vector<mixedtask> taskset_v; taskset_v.reserve(N);

    json_file << "[\n";
    bool first = true;

    do {
        T.clear(); wcets.clear(); D.clear(); taskset_v.clear();

        //std::vector<float> Uvect = call_dsr(U, N);
        std::vector<double> Uvect = uunifast(N, U);
        float U_effective = 0.0f;

        for (int i = 0; i < N; i++) {
            T.push_back(draw_log_uniform_distribution_integer(T_range_from, T_range_to));
            int tmp_wcet = std::round(T[i] * Uvect[i]);
            if (tmp_wcet == 0) tmp_wcet = 1;
            wcets.push_back(tmp_wcet);
            D.push_back(T[i]);
            U_effective += static_cast<float>(tmp_wcet) / static_cast<float>(T[i]);
	        
            int NPS = ceil((double)wcets[i] * ((double)crit_sec_perc / 100.0));
	        int RCT = wcets[i] - NPS;
            taskset_v.emplace_back(wcets[i], T[i], D[i], RCT, NPS, i);
        }

        if (U_effective > U * 1.01f || U_effective < U * 0.99f) continue;
        if (U_effective >= 1.0f) continue;

        mixedtaskset test_taskset(taskset_v);
        test_taskset.set_prioRM();

        for (auto i = 0; i < (int)test_taskset.get_taskset().size(); i++) {
            bool isPeriodic = d(generator);
            if (isPeriodic==false) 
                test_taskset.set_sporadic(i);
            else 
                test_taskset.set_mk(i, std::pair<int,int>(m , k));
        }

        json j_taskset = json::array();
        for (int i = 0; i < N; i++) {
            mixedtask t_i = test_taskset.get_task(i);
            json j_t;
            j_t["U"] = t_i.get_U();
            j_t["P"] = t_i.get_T();
            j_t["C"] = t_i.get_WCET();
            j_t["RCT"] = t_i.get_RCT();
            j_t["NPS"] = t_i.get_NPS();
            j_t["isPreemptive"] = t_i.get_isPreemptive();
            j_t["isPeriodic"] = t_i.get_isPeriodic();
            json j_const;
            j_const["D"] = t_i.get_D();
            if (t_i.get_isPeriodic()) { j_const["m"] = t_i.get_mk().first; j_const["k"] = t_i.get_mk().second; }
            j_t["constraint"] = j_const;
            j_taskset.push_back(j_t);
        }

        if (!first) json_file << ",\n";
        json_file << j_taskset.dump(4);
        first = false;

        generated_count++;
        //std::cout << "\n\t>> generated: " << generated_count << "/"<< num_taskset << " U: " << U << " #task: " << N << std::endl;

        try {
            tasksets.emplace_back(std::move(test_taskset));
        } catch (const std::bad_alloc& e) {
            std::cerr << "generate_export_tasksets_MILP_unsch: bad_alloc while storing taskset (" << e.what() << "). Returning partial results.\n";
            json_file << "\n]\n";
            json_file.flush();
            return tasksets;
        }

        if ((generated_count % FLUSH_INTERVAL) == 0) {
            json_file.flush();
            if (!json_file.good()) std::cerr << "generate_export_tasksets_MILP_unsch: I/O error after flush\n";
        }
    } while (generated_count < num_taskset);

    json_file << "\n]\n";
    json_file.flush();
    return tasksets;
}

/* Unschedulable and ONLY LAST mk  */
std::vector<mixedtaskset> generate_export_tasksets_MILP_unsch(const int num_taskset, const float U, const int N, std::ofstream& json_file, const int m, const int k, int crit_sec_perc, const int T_range_from, const int T_range_to, int N_mk){ 
    std::vector<mixedtaskset> tasksets;
    tasksets.reserve(static_cast<size_t>(num_taskset)); // avoid repeated reallocations
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    int generated_count = 0;
    const int FLUSH_INTERVAL = 10; // flush output to file every N task sets

    if (!json_file.is_open()) {
        std::cerr << "generate_export_tasksets_MILP_unsch: could not open json_file\n";
        return std::vector<mixedtaskset>();
    }

    std::vector<int> T; T.reserve(N);
    std::vector<int> wcets; wcets.reserve(N);
    std::vector<int> D; D.reserve(N);
    std::vector<mixedtask> taskset_v; taskset_v.reserve(N);

    json_file << "[\n";
    bool first = true;

    do {
        T.clear(); wcets.clear(); D.clear(); taskset_v.clear();

        //std::vector<float> Uvect = call_dsr(U, N);
        std::vector<double> Uvect = uunifast(N, U);
        float U_effective = 0.0f;

        for (int i = 0; i < N; i++) {
            T.push_back(draw_log_uniform_distribution_integer(T_range_from, T_range_to));
            int tmp_wcet = std::round(T[i] * Uvect[i]);
            if (tmp_wcet == 0) tmp_wcet = 1;
            wcets.push_back(tmp_wcet);
            D.push_back(T[i]);
            U_effective += static_cast<float>(tmp_wcet) / static_cast<float>(T[i]);
	        
            int NPS = ceil((double)wcets[i] * ((double)crit_sec_perc / 100.0));
	        int RCT = wcets[i] - NPS;
            taskset_v.emplace_back(wcets[i], T[i], D[i], RCT, NPS, i);
        }

        if (U_effective > U * 1.01f || U_effective < U * 0.99f) continue;
        if (U_effective >= 1.0f) continue;

        mixedtaskset test_taskset(taskset_v);
        test_taskset.set_prioRM();

        //bool only_last_unsc = true;
        bool only_one_unschedulable = true;
        bool discard_taskset = false;
        bool unschedulable = false;
        int unschedulable_count = 0;
        std::vector<int>unschedulable_indexes;
        for (int i = 0; i < (int)test_taskset.get_taskset().size(); i++) {
            mixedtask t_i = test_taskset.get_task(i);
            int R_i = test_taskset.compute_aRi(i);
            if (R_i == std::numeric_limits<int>::max()) { discard_taskset = true; break; }
            if ((R_i > t_i.get_D())) {
                unschedulable = true;
                unschedulable_count++;
                unschedulable_indexes.push_back(i);
               // if (i < (int)test_taskset.get_taskset().size() - N_mk) { only_last_unsc = false; }
            }
            //cout << ">> WCRT for task" << i << "=" << R_i << " (D=" << t_i.get_D() << ")" << std::endl;
        }
        if (unschedulable_count > N_mk) {
            only_one_unschedulable = false;
            //cout << ">> WARNING: more than one unschedulable task in the taskset." << std::endl;
        }
        //cout << "\r unschedulable" << unschedulable << " only_last_unsc" << only_last_unsc << " discard_taskset" << discard_taskset << fflush;
        // Compute the validity of the taskset
        if (discard_taskset) continue;
        bool valid_taskset = unschedulable && only_one_unschedulable; //(unschedulable && only_last_unsc && m_k_unsch);
        if (!valid_taskset) {
            //cout << ">> WARNING: invalid taskset generated. Discarding it." << std::endl;
            continue;
        }
        //cout << "\n\t>> generated: " << generated_count << "/"<< num_taskset << " U: " << U << " #task: " << N << std::endl;
        // ======================================================
        // Set the sporadic and periodic tasks in the taskset
        // ======================================================
        // Select N_mk indexes from the taskset
        if (N_mk > (int)test_taskset.get_taskset().size()) N_mk = test_taskset.get_taskset().size();
        std::vector<int> indexes(test_taskset.get_taskset().size());
        std::iota(indexes.begin(), indexes.end(), 0);
        std::shuffle(indexes.begin(), indexes.end(), generator);
        indexes.resize(N_mk);
        // Select N_mk indexes from the unschedulable indexes
        if (N_mk > (int)unschedulable_indexes.size())  { // add additional indexes to reach N_mk
            std::vector<int> additional_indexes;
            std::set_difference(indexes.begin(), indexes.end(), unschedulable_indexes.begin(), unschedulable_indexes.end(), std::inserter(additional_indexes, additional_indexes.begin()));
            std::shuffle(additional_indexes.begin(), additional_indexes.end(), generator);
            additional_indexes.resize(N_mk - unschedulable_indexes.size());
            unschedulable_indexes.insert(unschedulable_indexes.end(), additional_indexes.begin(), additional_indexes.end());
        }
        else{ // reduce the unschedulable indexes to N_mk
            std::sort(unschedulable_indexes.begin(), unschedulable_indexes.end());
            std::shuffle(unschedulable_indexes.begin(), unschedulable_indexes.end(), generator);
            unschedulable_indexes.resize(N_mk);
        }
        // Set the tasks in the taskset
        for (auto i = 0; i < (int)test_taskset.get_taskset().size(); i++) { 
            // Set the task as periodic if it is in the unschedulable selected indexes, otherwise set it as sporadic         
            //if (std::find(indexes.begin(), indexes.end(), i) != indexes.end()) test_taskset.set_mk(i, std::pair<int,int>(m , k));
            if (std::find(unschedulable_indexes.begin(), unschedulable_indexes.end(), i) != unschedulable_indexes.end()) test_taskset.set_mk(i, std::pair<int,int>(m , k));
            else test_taskset.set_sporadic(i);
        }

        // Export the taskset to JSON
        json j_taskset = json::array();
        for (int i = 0; i < N; i++) {
            mixedtask t_i = test_taskset.get_task(i);
            json j_t;
            j_t["U"] = t_i.get_U();
            j_t["P"] = t_i.get_T();
            j_t["C"] = t_i.get_WCET();
            j_t["RCT"] = t_i.get_RCT();
            j_t["NPS"] = t_i.get_NPS();
            j_t["isPreemptive"] = t_i.get_isPreemptive();
            j_t["isPeriodic"] = t_i.get_isPeriodic();
            json j_const;
            j_const["D"] = t_i.get_D();
            if (t_i.get_isPeriodic()) { j_const["m"] = t_i.get_mk().first; j_const["k"] = t_i.get_mk().second; }
            j_t["constraint"] = j_const;
            j_taskset.push_back(j_t);
        }

        if (!first) json_file << ",\n";
        json_file << j_taskset.dump(4);
        first = false;

        generated_count++;
        //std::cout << "\n\t>> generated: " << generated_count << "/"<< num_taskset << " U: " << U << " #task: " << N << std::endl;

        try {
            tasksets.emplace_back(std::move(test_taskset));
        } catch (const std::bad_alloc& e) {
            std::cerr << "generate_export_tasksets_MILP_unsch: bad_alloc while storing taskset (" << e.what() << "). Returning partial results.\n";
            json_file << "\n]\n";
            json_file.flush();
            return tasksets;
        }

        if ((generated_count % FLUSH_INTERVAL) == 0) {
            json_file.flush();
            if (!json_file.good()) std::cerr << "generate_export_tasksets_MILP_unsch: I/O error after flush\n";
        }
    } while (generated_count < num_taskset);

    json_file << "\n]\n";
    json_file.flush();
    return tasksets;
}

void export_results(const std::vector<int> our_res, const std::vector<int> their_res, int our_sum, int their_sum, std::ofstream& json_file, long long min_t_our, long long max_t_our, double med_t_our, long long min_t_their, long long max_t_their, double med_t_their){
    json j_results;
    j_results["our_sum"] = (100.0 * (double)our_sum) / (double)our_res.size();
    j_results["their_sum"] = (100.0 * (double)their_sum) / (double)their_res.size();
    j_results["our_min_t"] = min_t_our;
    j_results["our_max_t"] = max_t_our;
    j_results["our_avg_t"] = med_t_our;
    j_results["their_min_t"] = min_t_their;
    j_results["their_max_t"] = max_t_their;
    j_results["their_avg_t"] = med_t_their;
    json j_result;
    int or_sum = 0;
    for(size_t i = 0; i < our_res.size(); i++) {
        json j_i;
        j_i["sched_our"] = our_res[i];
        j_i["sched_their"] = their_res[i];
        j_result.push_back(j_i);
        if (our_res[i] + their_res[i] != 0) {
            or_sum++;
        }
    }
    j_results["or_sum"] = (100.0 *(double)or_sum) / (double)our_res.size();
    j_results["schedulability"] = j_result;

    json_file << std::setw(4) << j_results << std::endl;
    return;
}

void export_results(const std::vector<int>& our_res, const std::vector<int>& their_res, std::ofstream& json_file, long long min_t_our, long long max_t_our, double med_t_our, long long min_t_their, long long max_t_their, double med_t_their){
    json j_results;

    int our_sum = 0;
    int their_sum = 0;
    int or_sum = 0;

    json j_result = json::array();

    for(size_t i = 0; i < our_res.size(); i++){
        json j_i;
        j_i["sched_our"] = our_res[i];
        j_i["sched_their"] = their_res[i];
        j_result.push_back(j_i);

        our_sum += our_res[i];
        their_sum += their_res[i];

        if(our_res[i] || their_res[i]){
            or_sum++;
        }
    }

    j_results["our_sum"] = (100.0 * our_sum) / our_res.size();
    j_results["their_sum"] = (100.0 * their_sum) / their_res.size();
    j_results["or_sum"] = (100.0 * or_sum) / our_res.size();

    j_results["our_min_t"] = min_t_our;
    j_results["our_max_t"] = max_t_our;
    j_results["our_avg_t"] = med_t_our;

    j_results["their_min_t"] = min_t_their;
    j_results["their_max_t"] = max_t_their;
    j_results["their_avg_t"] = med_t_their;

    j_results["schedulability"] = j_result;

    json_file << std::setw(4) << j_results << std::endl;
}

void export_results(const std::vector<int> our_res, int our_sum, std::ofstream& json_file, long long min_t_our, long long max_t_our, double med_t_our) {
    json j_results;
    j_results["our_sum"] = (100.0 * (double)our_sum) / (double)our_res.size();
    j_results["our_min_t"] = min_t_our;
    j_results["our_max_t"] = max_t_our;
    j_results["our_avg_t"] = med_t_our;
    json j_result;
    for (size_t i = 0; i < our_res.size(); i++) {
        json j_i;
        j_i["sched_our"] = our_res[i];
        j_result.push_back(j_i);
    }
    j_results["schedulability"] = j_result;

    json_file << std::setw(4) << j_results << std::endl;
    return;
}
