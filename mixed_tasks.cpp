/**
 * @file mixed_tasks.cpp
 * @brief Implementation of task and task set management classes
 * @author Veronica Rispo
 * @date 2026
 * @details This file implements the core task representation classes: mixedtask (individual tasks)
 * and mixedtaskset (collections of tasks). It provides:
 * 
 * Task Management:
 * - Task parameter validation and initialization
 * - Getters/setters for task attributes (WCET, period, deadline, priorities, (m,k) constraints)
 * - Task type checking (periodic vs. sporadic, preemptive vs. non-preemptive)
 * - Priority modification and management
 * 
 * Task Set Analysis:
 * - Response time computation via fixed-point iteration (compute_Ri)
 * - Abstract response time analysis (compute_aRi)
 * - Utilization calculation
 * - Blocking multiset computation
 * - Schedulability verification
 * - Priority assignment for rate-monotonic (RM) scheduling
 * 
 * Global Analysis Functions:
 * - Schedulability verification for individual tasks and entire task sets
 * - Hard real-time analysis (all jobs meet deadlines)
 * - Weakly-hard constraint verification with (m,k) analysis
 * - Sequence reachability checking for miss subsequence validation
 * - Priority assignment search
 * 
 * The module extensively uses the MILP class for complex scenarios requiring
 * optimal response time bounds.
 * 
 * @see mixed_tasks.h for class interfaces
 */

#include "mixed_tasks.h"
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <string>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <limits>
#include "MILP.h"


mixedtask::mixedtask() {
	this->isPeriodic = false; // default value
	this->D = 0; // default value

	this->WCET = 0; // default value
	this->T = 0; // default value
	this->U = 0; // default value

	this->prio = 0; // default value
	this->NPS = 0; // default value
	this->RCT = this->WCET; // default value
	this->isPreemptive = true; // default value
}

mixedtask::mixedtask(int WCET, int T, int D, int RCT, int NPS, int prio) {
	if (prio < 0) {
		throw invalid_argument("mixedtask(): invalid argument -> prio < 0");
		return;
	}
	if (D < 0) {
		throw invalid_argument("mixedtask(): invalid argument -> D < 0");
		return;
	}
	if (T < 0) {
		throw invalid_argument("mixedtask(): invalid argument -> T < 0");
		return;
	}
	if (T < D) {
		throw invalid_argument("mixedtask(): invalid argument -> T < D");
		return;
	}
	if (WCET <= 0) {
		throw invalid_argument("mixedtask(): invalid argument -> WCET <= 0");
		return;
	}
	if (WCET > T) {
		throw invalid_argument("mixedtask(): invalid argument -> WCET > T");
		return;
	}
	if (NPS + RCT != WCET) {
		throw invalid_argument("mixedtask(): invalid argument -> NPS + RCT != WCET");
		return;
	}

	this->isPeriodic = false;
	this->D = D;

	this->WCET = WCET;

	this->T = T;
	this->U = (double)WCET / (double)T;
	this->prio = prio;

	this->RCT = RCT;
	this->NPS = NPS;
	if (this->NPS > 0) {
		this->isPreemptive = false;
	}
	else {
		this->isPreemptive = true;	
	}
	return;
}

mixedtask::mixedtask(int WCET, int T, int D, int RCT, int NPS, pair<int, int> mk, int prio) {
	if (D < 0) {
		throw invalid_argument("mixedtask(): invalid argument -> D < 0");
		return;
	}
	if (T < 0) {
		throw invalid_argument("mixedtask(): invalid argument -> T < 0");
		return;
	}
	if (T < D) {
		throw invalid_argument("mixedtask(): invalid argument -> T < D");
		return;
	}
	if (WCET <= 0) {
		throw invalid_argument("mixedtask(): invalid argument -> WCET <= 0");
		return;
	}
	if (WCET > T) {
		throw invalid_argument("mixedtask(): invalid argument -> WCET > T");
		return;
	}
	if ((mk.first <= 0) || (mk.second <=0)) {
		throw invalid_argument("mixedtask(): invalid argument -> m or k <= 0");
		return;
	}
	if ((mk.first > mk.second)) {
		throw invalid_argument("mixedtask(): invalid argument -> m > k");
		return;
	}
	this->isPeriodic = true;
	this->D = D;
	this->mk = mk;

	this->WCET = WCET;

	this->T = T;
	this->U = (double)WCET / (double)T;
	this->prio = prio;

	this->RCT = RCT;
	this->NPS = NPS;
	if (this->NPS > 0) {
		this->isPreemptive = false;
	}
	else {
		this->isPreemptive = true;	
	}
}

int mixedtask::get_WCET() const{
	return this->WCET;
}

int mixedtask::get_RCT() const{
	return this->RCT;
}

int mixedtask::get_NPS() const{
	return this->NPS;
}

int mixedtask::get_T() const{
	return this->T;
}

float mixedtask::get_U() const{
	return this->U;
}

int mixedtask::get_prio() const{
	return this->prio;
}

bool mixedtask::get_isPeriodic() const{
	return this->isPeriodic;
}

int mixedtask::get_D() const{
	return this->D;
}

pair<int, int> mixedtask::get_mk() const{
	if (this->isPeriodic)
		return this->mk;
	else
		return pair<int, int>();
}

int mixedtask::get_m() const{
	if (this->isPeriodic)
		return this->mk.first;
	else
		return 0;
}

bool mixedtask::get_isPreemptive() const{
	return this->isPreemptive;
}

void mixedtask::set_prio(int prio) {
	if (prio < 0) {
		throw invalid_argument("set_prio(): invalid argument -> prio < 0");
		return;
	}
	this->prio = prio;
}

void mixedtask::set_mk(pair<int, int> mk) {
	this->isPeriodic = true;
	this->mk = mk;
}

void mixedtask::set_sporadic() {
	this->isPeriodic = false;
	this->mk = pair<int, int>();
}

void mixedtask::print() const{
	std::cout << " ";
	std::cout << std::setw(SETWval) << "| C = " << std::setw(SETWval) << this->WCET;
	std::cout << std::setw(SETWval) << "| nps = " << std::setw(SETWval) << this->NPS;
	std::cout << std::setw(SETWval) << "| T = " << std::setw(SETWval) << this->T;
	std::cout << std::setw(SETWval) << "| D = " << std::setw(SETWval) << this->D;
	std::cout << std::setw(SETWval) << "| p = " << std::setw(SETWval) << this->prio;
	if(this->isPeriodic){
		string mk_str = "(" + to_string(this->mk.first) + " " + to_string(this->mk.second) + ")";
		std::cout << std::setw(SETWval) << "| (m k) = " << std::setw(SETWval) << mk_str;
	}
	else
		std::cout << std::setw(SETWval) << "| (m k) = " << std::setw(SETWval) << "-";
	std::cout << std::setw(SETWval) << "| R = " << std::setw(SETWval) << "-";
	std::cout << "|" << std::endl;
}

void mixedtask::print(int R) const{
	std::cout << " ";
	std::cout << std::setw(SETWval) << "| C = " << std::setw(SETWval) << this->WCET;
	std::cout << std::setw(SETWval) << "| T = " << std::setw(SETWval) << this->T;
	std::cout << std::setw(SETWval) << "| D = " << std::setw(SETWval) << this->D;
	std::cout << std::setw(SETWval) << "| p = " << std::setw(SETWval) << this->prio;
	if (this->isPeriodic) {
		string mk_str = "(" + to_string(this->mk.first) + " " + to_string(this->mk.second) + ")";
		std::cout << std::setw(SETWval) << "| (m k) = " << std::setw(SETWval) << mk_str;
	}
	else
		std::cout << std::setw(SETWval) << "| (m k) = " << std::setw(SETWval) << "-";
	std::cout << std::setw(SETWval) << "| R = " << std::setw(SETWval) << R;
	std::cout << "|" << std::endl;
}

float mixedtaskset::compute_Bi(int index, float A){
	float Bi = 0;
	if (this->sched == FP) {
		for (int j = 0; j < (int)this->taskset.size(); j++) {
			if ((index != j) && (this->taskset[index].get_prio() < this->taskset[j].get_prio())) { // tj lower priority task
				if (Bi < this->taskset[j].get_NPS()) {
					Bi = this->taskset[j].get_NPS();
				}
			}
		}
	}
	if (int(Bi) < 0) {
		my_assert("In compute_Bi. Negative Bi.");
	}
	return Bi;
}

float mixedtaskset::compute_IBFi(int index, float A, float delta) {
	float total_interference = 0.0f;

	// Get the priority of the task at 'index'
	int prio_i = taskset[index].get_prio();

	// Calculate the interval for which we are computing interference
	float interval = A + delta;

	// If the interval is non-positive, return 0 interference
	if (interval <= 0.0f) {
		return 0.0f;
	}

	// Iterate over all tasks to compute interference from higher-priority tasks
	for (int j = 0; j < (int)taskset.size(); ++j) {
		// Skip the task itself
		if (j == index) {
			continue;
		}

		mixedtask task_j = taskset[j];
		int prio_j = task_j.get_prio();

		// Only consider tasks with higher priority (lower priority number)
		if (prio_j < prio_i) {
			// Retrieve the WCET and period of the higher-priority task
			int Cj = task_j.get_WCET();
			int Tj = task_j.get_T();

			// If the period is non-positive, skip this task (should not happen for periodic/sporadic tasks)
			if (Tj <= 0) {
				// This is an unexpected case, as Tj should be positive for valid tasks.
				continue;
			}

			float Tj_f = (float)Tj;

			// Calculate Equation (10) for the higher-priority task 'j'

			// Calculate the number of complete jobs of task 'j' that can arrive in the interval [0, A + delta]
			int N = (int)std::floor(interval / Tj_f);

			// Calculate the remainder of the interval after accounting for N complete jobs
			float remainder = interval - (float)(N * Tj);

			// Calculate the carry-in interference, which is the minimum of Cj and the positive remainder
			float carry_in = std::min((float)Cj, std::max(0.0f, remainder));

			// Calculate the total interference contributed by task 'j' in the interval [0, A + delta]
			float alpha_j = (float)(N * Cj) + carry_in;

			// Accumulate the interference from task 'j' into the total interference
			total_interference += alpha_j;
		}
	}

	return total_interference;
}

vector<int> mixedtaskset::compute_multiset(int index, int size_multiset) {
	vector<int> multiset;
	if (this->preemptive == false) {
		for (auto i = 0; i < (int)this->taskset.size(); i++) {
			if ((this->taskset[i].get_prio() >= this->taskset[index].get_prio()) && (i != index)) { // tau_i prio less than tau_index prio
				int t = ((size_multiset - 1) * this->taskset[index].get_T()) + this->taskset[index].get_D();
				int n = (int)ceil((double)(t + compute_aRi(i)) / (double)this->taskset[i].get_T());
				for (auto c = 0; c < n; c++) {
					//multiset.push_back(this->taskset[i].get_WCET());
					multiset.push_back(this->taskset[i].get_NPS());
				}
			}
		}
		// Sort the vector
		sort(multiset.begin(), multiset.end(), greater<>());
	}
	
	// Resize the vector
	multiset.resize(size_multiset, 0);

	if (multiset.size() != size_multiset) {
		cout << "In compute multiset for task index " << index << " with prio = " << this->taskset[index].get_prio() << ". ";
		cout << multiset.size() << " != " << size_multiset << ". ";
		my_assert(" multiset.size() != size_multiset");
	}
	
	return multiset;
}

mixedtaskset::mixedtaskset() {
	this->U = 0;
	this->schedulable = none;
}

mixedtaskset::mixedtaskset(vector<mixedtask> taskset) {
	this->taskset = taskset;
	this->U = 0;
	for (int i = 0; i < (int)this->taskset.size(); i++) {
		this->U += this->taskset[i].get_U();
	}
	if (U >= 1.0) {
		my_assert("ERROR: U>=1");
	}
	this->schedulable = none;
	this->R.resize(this->taskset.size());

	this->preemptive = true;
	for (auto task : taskset) {
		if (!task.get_isPreemptive()) {
			this->preemptive = false;
			break;
		}
	}
}

vector<mixedtask> mixedtaskset::get_taskset() const{
	return this->taskset;
}

vector<int> mixedtaskset::get_R() const{
	return this->R;
}

Schedulability mixedtaskset::get_schedulable() const{
	return this->schedulable;
}

float mixedtaskset::get_U() const{
	return this->U;
}

mixedtask mixedtaskset::get_task(int index) const{
	if((index < 0) ||(index >=this->taskset.size())){
		throw invalid_argument("get_task(): invalid argument -> index not exist");
		return mixedtask();
	}
	else {
		return this->taskset[index];
	}
}

int mixedtaskset::get_H() const{
	vector<int> Ts;
	for (int i = 0; i < (int)this->taskset.size(); i++) {
		int T = this->taskset[i].get_T();
		Ts.push_back(T);
	}

	if (Ts.size() != 0) {
		int H = Ts[0];
		for (int i = 1; i < (int)Ts.size(); i++)
			H = std::lcm(H, Ts[i]);

		return H;
	}
	else {
		return 0;
	}
}

int mixedtaskset::get_R(int index) const{
	if ((index < 0) || (index >= (int)this->taskset.size())) {
		return -1;
	}
	return this->R[index];
}

int mixedtaskset::get_B(int index) const{

	int prio_i = taskset[index].get_prio();
	int max_blocking_time = 0;

	// Iterate over all tasks to find the maximum NPS of lower-priority tasks
	for (int j = 0; j < (int)taskset.size(); ++j) {

		// A task cannot block itself
		if (j == index) {
			continue;
		}

		int prio_j = taskset[j].get_prio();

		// Search for task 'j' with LOWER priority than 'i'
		if (prio_j > prio_i) {
			// The blocking is the max NPS (Longest Non-Preemptive Section)
			// of all tasks with lower priority.
			max_blocking_time = std::max(max_blocking_time, taskset[j].get_NPS());
		}
	}

	return max_blocking_time;
}

bool mixedtaskset::isPreemptive() const{
	return this->preemptive;
}

void mixedtaskset::set_prioRM() {
	stable_sort(this->taskset.begin(), this->taskset.end(),
		[](mixedtask x, mixedtask y) { return x.get_U() < y.get_U(); });
	stable_sort(this->taskset.begin(), this->taskset.end(),
		[](mixedtask x, mixedtask y) { return x.get_T() < y.get_T(); });
	for (int i = 0; i < (int)this->taskset.size(); i++) {
		this->taskset[i].set_prio(i);
	}
}

void mixedtaskset::set_mk(int index, pair<int, int> mk) {
	this->taskset[index].set_mk(mk);
}

void mixedtaskset::set_sporadic(int index) {
	this->taskset[index].set_sporadic();
}

void mixedtaskset::set_R(vector<int> R, Schedulability sched) {
	if ((int)R.size() != (int)this->taskset.size()) {
		throw invalid_argument("set_R(): invalid argument -> size of R != size of taskset");
		return;
	}
	this->R = R;
	this->schedulable = sched;
}

void mixedtaskset::set_Ri(int index, int Ri) {
	if ((index > ((int)this->taskset.size()-1))|| index < 0) {
		throw invalid_argument("set_Ri(): invalid argument -> index not exist");
		return;
	}
	this->R[index] = Ri;
	return;
}

int mixedtaskset::compute_aRi(int index) {
	// 1. Retrieve task parameters
	mixedtask task_i = taskset[index];
	int Ci = task_i.get_WCET();
	int RCTi = task_i.get_RCT();
	int Bi = get_B(index);

	// 2. Prepare for fixed-point iteration
	const int MAX_ITERATIONS = 5000; // reasonable upper bound
	const double EPS = 1e-6; // convergence tolerance
	const double MAX_AR_LIMIT = static_cast<double>(std::numeric_limits<int>::max()) / 4.0; // conservative safety threshold
	int iteration_count = 0;

	// 3. Initialize aR_current and aR_next
	double aR_current = (double)(RCTi + Bi); // Base case: aR starts at RCTi + Bi
	double aR_next = 0.0;

	// 4. Fixed point iteration on aR (the preemptable window) add NPS at the end
	while (true) {
		// Compute the interference bound function (IBF) for the current aR
		float ibf = compute_IBFi(index, (float)aR_current, 0.0f); // delta = 0.0f for the preemptable window
		aR_next = (double)(RCTi + Bi) + (double)ibf;

		// Check for non-finite or excessively large values to prevent overflow
		if (!std::isfinite(aR_next) || aR_next > MAX_AR_LIMIT) {
			std::cerr << "compute_aRi: non-finite or too large value encountered for task " << index
				<< " (aR_next=" << aR_next << "). Returning INT_MAX.\n";
			return std::numeric_limits<int>::max();
		}

		// Check for convergence within the specified tolerance
		if (std::fabs(aR_next - aR_current) <= EPS) {
			return (int)std::ceil(aR_next) + (Ci - RCTi); // add back the non-preemptive section NPS_i 
		}

		// Safety break: if the number of iterations exceeds MAX_ITERATIONS, return INT_MAX
		if (iteration_count >= MAX_ITERATIONS) {
			std::cerr << "compute_aRi: safety break after " << iteration_count
				<< " iterations for task " << index << ". Returning INT_MAX.\n";
			return std::numeric_limits<int>::max();
		}

		// Update for the next iteration
		aR_current = aR_next;
		iteration_count++;
	}
}

int mixedtaskset::compute_Ri(int index){
	int R = this->get_task(index).get_WCET() + get_B(index);
	int R0 = R;
	do {
		R0 = R;
		R = this->get_task(index).get_WCET() + get_B(index);
		for (size_t k = 0; k < this->taskset.size(); k++) {
			if ((k != index) && (this->get_task(k).get_prio() < this->get_task(index).get_prio())) { // higher-priority task
				R += (int)std::ceil((double)R0 / (double)this->get_task(k).get_T()) * this->get_task(k).get_WCET();
			}
		}
	} while (R0 != R);

	if ((int)std::round(R) < 0) {
		my_assert("Negative WCRT.");
	}
	return (int)std::round(R);
}

void mixedtaskset::print() const{
	std::cout << "  -----------------------------------------------------------------------------" << std::endl;
	std::cout << std::setw(SETWval * 7) << "TASKSET" << std::endl;;
	std::cout << "  -----------------------------------------------------------------------------" << std::endl;
	std::cout << "Preemptive: " << this->preemptive << endl;
	if (this->schedulable == none) {
		for (int i = 0; i < (int)this->taskset.size(); i++) {
			this->taskset[i].print();
		}
	}
	else {
		for (int i = 0; i < (int)this->taskset.size(); i++) {
			this->taskset[i].print(this->R[i]);
		}
	}
	std::cout << "  -----------------------------------------------------------------------------" << std::endl;
}

/**
 * @brief Invokes MILP solver for weakly-hard constraint verification
 * @param tmp_seq_i Sequence of miss/success indicators (binary vector) for the task
 * @param index Index of the specific task in the task set
 * @param taskset The task set containing the task to analyze
 * @param job_continue If true, uses job-continue policy; if false, uses job-discard policy
 * @return true if the MILP solver found a feasible solution, false otherwise
 * @details Creates a blocking multiset based on the sequence and invokes the MILP optimizer
 * to verify if the given miss sequence is realizable. The solver computes worst-case response
 * times considering the specific task behavior specified by tmp_seq_i.
 * @see realizable_sequence()
 * @see MILP::call_solver()
 */
bool call_MILP(vector<int> tmp_seq_i, int index, mixedtaskset taskset, bool job_continue) {
	vector<int> multiset = taskset.compute_multiset(index, tmp_seq_i.size());
	MILP optimizer(taskset, tmp_seq_i, index, multiset, job_continue);

	std::pair<bool,int> result =optimizer.call_solver();
	return result.first;
}

bool cache_controls(vector<int> seq, vector<vector<int>> cache) {
	int n = seq.size();

	for (int c = 0; c < (int)cache.size(); c++) {
		int m = cache[c].size();

		int i = 0, j = 0;
		while (i < n && j < m) {
			if (seq[i] == cache[c][j]) {
				j++;
				if (j == m) {
					return false; // Subsequence found: sequence is not valid
				}
			}
			else {
				// Reset the cache[c] index only if there was a partial match
				if (j > 0) {
					j = 0;
				}
			}
			i++;
		}
	}

	return true;
}

bool realizable_sequence(vector<int> tmp_seq_i, vector<vector<int>> cache, int index, vector<int> M_upperbound, mixedtaskset taskset, bool job_continue) {
	bool seq_ok = cache_controls(tmp_seq_i, cache);
	if (seq_ok == false) {
		return false;
	}
	bool realizable = call_MILP(tmp_seq_i, index, taskset, job_continue);
	return realizable;
}


int compute_M_upperbound(int index, vector<int> M_upperbound, mixedtaskset taskset, bool job_continue) {
	pair<int, int> mk_i = taskset.get_task(index).get_mk();
	int k_i_r = mk_i.second;

	vector<vector<int>> S_i;
	S_i.push_back(vector<int>(1,1)); // Miss 1, Hit 0
	int M_upperbound_i = 1;
	vector<vector<int>> cache;
	
	for (int k = 2; k <= k_i_r; k++) {
		bool M_added = false;
		vector<vector<int>> Tmp_S_i;
		for (int i = 0; i < (int)S_i.size(); i++) {
			vector<int> tmp_seq_i = S_i[i];
			tmp_seq_i.push_back(1); // add a miss
			bool realizable = realizable_sequence(tmp_seq_i, cache, index, M_upperbound, taskset, job_continue);
			if (realizable) {
				Tmp_S_i.push_back(tmp_seq_i);
				M_added = true;
			}
			else {
				cache.push_back(tmp_seq_i);
			}

			tmp_seq_i.clear();
			tmp_seq_i = S_i[i];
			tmp_seq_i.push_back(0); // add an hit
			realizable = realizable_sequence(tmp_seq_i, cache, index, M_upperbound, taskset, job_continue);
			if (realizable) {
				Tmp_S_i.push_back(tmp_seq_i);
			}
			else {
				cache.push_back(tmp_seq_i);
			}
		}
		if (M_added) {
			for (int i = 0; i < (int)Tmp_S_i.size(); i++) {
				int sum = 0;
				for (int s = 0; s < (int)Tmp_S_i[i].size(); s++) {
					sum += Tmp_S_i[i][s];
				}
				if (sum > M_upperbound_i)
					M_upperbound_i = sum;
			}
		}
		S_i = Tmp_S_i;

		/* If we do not want the exact upper bound, but only to know if it is greater than m_i */
		if (M_upperbound_i > mk_i.first) {
			break;
		}
	}
	if (M_upperbound_i <= mk_i.first) {
		if (S_i.size() == 0) {
			my_assert("Sequence too short.");
		}
		for (int i = 0; i < (int)S_i.size(); i++) {
			if ((int)S_i[i].size() < mk_i.second) {
				my_assert("Sequence too short.");
			}
		}
	}

	return M_upperbound_i;
}

tuple<Schedulability, vector<int>, vector<int>> compute_schedulability(mixedtaskset taskset, bool job_continue) {
	//taskset.print();
	//std::cout << ">> Computing schedulability" << std::endl;
	Schedulability sched = Schedulable;
	vector<int> M_upperbound;
	vector<int> R;

	for (int i = 0; i < (int)taskset.get_taskset().size(); i++) {
		int R_i = taskset.compute_aRi(i);
		//std::cout << ">> WCRT for task" << i << "=" << R_i << std::endl;
		//std::cout << ">> WCRT for task" << i << "=" << R_i << std::endl;
		R.push_back(R_i);
		taskset.set_Ri(i, R_i);
	}
	if ((int)taskset.get_taskset().size() != (int)R.size()) {
		my_assert("Error: size of R not valid");
	}

	for (int i = 0; i < (int)R.size(); i++) {
		mixedtask t_i = taskset.get_task(i);
		int R_i = R[i];
		if (R_i > t_i.get_D()) {
			//std::cout << ">> WCRT > D for task " << i << std::endl;
			if (t_i.get_isPeriodic()) {
				int M_value = compute_M_upperbound(i,M_upperbound, taskset, job_continue);
				M_upperbound.push_back(M_value);
				//cout << "M " << M_value << endl;
				if (M_value > t_i.get_m()) {
					sched = Unschedulable;
				}
			}
			else {
				sched = Unschedulable;
			}
		}
		else {
			if (t_i.get_isPeriodic()) {
				M_upperbound.push_back(0);
			}
		}
	}
	taskset.set_R(R, sched);
	return tuple<Schedulability, vector<int>, vector<int>>(sched, M_upperbound, R);
}

tuple<Schedulability, vector<int>, vector<int>> compute_schedulability(mixedtaskset taskset, int index, bool job_continue) {
	//taskset.print();
	//std::cout << ">> Computing schedulability of " <<index << std::endl;
	Schedulability sched = Schedulable;
	vector<int> M_upperbound;
	vector<int> R;
	for (int i = 0; i < (int)taskset.get_taskset().size(); i++) {
		mixedtask t_i = taskset.get_task(i);
		int R_i = taskset.compute_aRi(i);
		//std::cout << ">> WCRT for task" << i << "=" << R_i << std::endl;
		R.push_back(R_i);
		taskset.set_Ri(i, R_i);
	}

	if (R[index] > taskset.get_task(index).get_D()) {
		//std::cout << ">> WCRT > D for task " << index << std::endl;
		if (taskset.get_task(index).get_isPeriodic()) {
			int M_value = compute_M_upperbound(index, M_upperbound, taskset, job_continue);
			M_upperbound.push_back(M_value);
			if (M_value > taskset.get_task(index).get_m()) {
				//std::cout << ">> Weakly-hard not verified for task " << index << std::endl;
				sched = Unschedulable;
			}
		}
		else {
			sched = Unschedulable;
		}
	}
	else {
		if (taskset.get_task(index).get_isPeriodic()) {
			M_upperbound.push_back(0);
		}
	}
	return tuple<Schedulability, vector<int>, vector<int>>(sched, M_upperbound, R);
}

tuple<Schedulability, vector<int>, vector<int>> compute_HARD_schedulability(mixedtaskset taskset, bool job_continue){
	//taskset.print();
	//std::cout << ">> Computing HARD schedulability" << std::endl;
	Schedulability sched = Schedulable;
	vector<int> M_upperbound;
	vector<int> R;

	for (int i = 0; i < (int)taskset.get_taskset().size(); i++) {
		int R_i = taskset.compute_aRi(i);
		//std::cout << ">> WCRT for task" << i << "=" << R_i << std::endl;
		float testR_i = taskset.compute_Ri(i);
		if (R_i > testR_i) {
			std::cout << "Ri :" << testR_i << " aRi: " << R_i << std::endl;
		}
		//std::cout << ">> WCRT for task" << i << "=" << R_i << std::endl;
		R.push_back(R_i);
		taskset.set_Ri(i, R_i);
		mixedtask t_i = taskset.get_task(i);
		if (R_i > t_i.get_D()) {
			//std::cout << ">> WCRT > D for task " << i << std::endl;
			sched = Unschedulable;
		}
	}
	taskset.set_R(R, sched);
	return tuple<Schedulability, vector<int>, vector<int>>(sched, M_upperbound, R);
}

tuple<Schedulability, vector<int>, vector<int>> compute_HARD_CAN_schedulability(mixedtaskset taskset, bool job_continue){
	//taskset.print();
	//std::cout << ">> Computing HARD schedulability" << std::endl;
	Schedulability sched = Schedulable;
	vector<int> M_upperbound;
	vector<int> R;

	for (int i = 0; i < (int)taskset.get_taskset().size(); i++) {
		mixedtask t_i = taskset.get_task(i);
		int B_i = taskset.get_B(i);
		int C_i = t_i.get_WCET();
		int T_i = t_i.get_T();
		int prio_i = t_i.get_prio();

		// Sanity check on Um
		float Um = 0.0;
		for (int j = 0; j < (int)taskset.get_taskset().size(); j++) {
			if ((j != i) && (taskset.get_task(j).get_prio() < prio_i)) { // higher-priority task
				Um += (float)taskset.get_task(j).get_WCET() / (float)taskset.get_task(j).get_T();
			}
		}
		if (Um >= 1.0) {
			std::cerr << "compute_HARD_CAN_schedulability: Um >= 1.0 for task " << i << ". Returning Unschedulable.\n";
			sched = Unschedulable;
			break;
		}

		// Compute t_m : length of the priority level-m busy period
		//std::cout << ">> Computing t_m for task " << i << std::endl;
		int tm = C_i;
		int tm_new = tm;
		do{ 
			tm = tm_new;
			tm_new = B_i;
			for (int j = 0; j < (int)taskset.get_taskset().size(); j++) {
				if ((j != i) && (taskset.get_task(j).get_prio() < prio_i)) { // higher-priority task
					tm_new += (int)std::ceil((double)tm / (double)taskset.get_task(j).get_T()) * taskset.get_task(j).get_WCET();
				}
			}
		} while (tm != tm_new);
		//std::cout << ">> t_m for task " << i << " = " << tm_new << std::endl;

		// Compute Q_m : number of instances of task_i ready before of the end of the busy period
		//std::cout << ">> Computing Q_m for task " << i << std::endl;
		int Qm = std::ceil((double)tm_new / (double)T_i);
		
		//std::cout << ">> Q_m for task " << i << " = " << Qm << std::endl;

		// Compute w_m(q) : longest time from the start of the busy period to the completion of the q-th instance of task_i
		//std::cout << ">> Computing w_m(q) for task " << i << std::endl;
		std::vector<int> wm(Qm, 0);
		for (int q = 0; q < (int)wm.size(); q++) {
			//std::cout << ">> Computing w_m(" << q << ") for task " << i << std::endl;
			int wm_q = B_i+ (q* C_i);
			if (q > 0) {
				wm_q = wm[q-1] + C_i;
			}
			int wm_q_new = wm_q;
			do {
				wm_q = wm_q_new;
				wm_q_new = B_i + (q * C_i);
				for (int j = 0; j < (int)taskset.get_taskset().size(); j++) {
					if ((j != i) && (taskset.get_task(j).get_prio() < prio_i)) { // higher-priority task
						wm_q_new += (int)std::ceil((double)wm_q / (double)taskset.get_task(j).get_T()) * taskset.get_task(j).get_WCET();
					}
				}
				//std::cout << ">> w_m(" << q << ") for task " << i << " = " << wm_q_new << std::endl;
				//std::cout << ">> w_m(" << q << ") for task " << i << " previous = " << wm_q << std::endl;	
				//std::cout << ">> w_m(" << q << ") for task " << i << " deadline check = " << (wm_q_new - (q * T_i) + C_i) << std::endl;
			}while ((wm_q != wm_q_new)&&((wm_q_new - (q * T_i) + C_i) <= taskset.get_task(i).get_D()));
			wm[q] = wm_q_new;
			//std::cout << ">> w_m(" << q << ") for task " << i << " = " << wm_q_new << std::endl;
		}

		// Compute R_m(q) : response time of the q-th instance of task_i
		//std::cout << ">> Computing R_m(q) for task " << i << std::endl;
		std::vector<int> Rm(Qm, 0);
		for (int q = 0; q < (int)Rm.size(); q++) {
			Rm[q] = wm[q] - (q * T_i) + C_i;
			//std::cout << ">> R_m(" << q << ") for task " << i << " = " << Rm[q] << std::endl;
		}

		// Compute R_i : maximum response time of task_i
		int R_i = *std::max_element(Rm.begin(), Rm.end());
		//std::cout << ">> R_i for task " << i << " = " << R_i << std::endl;
		R.push_back(R_i);
		if (R_i > t_i.get_D()) {
			//std::cout << ">> WCRT > D for task " << i << std::endl;
			sched = Unschedulable;
		}
	}
    taskset.set_R(R, sched);
	return tuple<Schedulability, vector<int>, vector<int>>(sched, M_upperbound, R);
}

tuple<bool, mixedtaskset> compute_priority_assignment(mixedtaskset taskset, bool job_continue) {

	tuple<Schedulability, vector<int>, vector<int>> res = compute_schedulability(taskset, job_continue);
	Schedulability result = std::get<0>(res);	
	if (result == Schedulable) {
		return tuple<bool, mixedtaskset>(true, taskset);
	}
	vector<mixedtask> tasks = taskset.get_taskset();

	stable_sort(tasks.begin(), tasks.end(),
		[](mixedtask x, mixedtask y) { return x.get_U() < y.get_U(); });

	for (int p = ((int)tasks.size() -1 ); p >= 0; p--) {
		bool found = false;
		for (int i = 0; i < (int)tasks.size(); i++) {
			if (tasks[i].get_prio() <= p) {
				tasks[i].set_prio(p);			
			
				int tmp_p = 0;
				for (int j = 0; j < (int)tasks.size(); j++) {
					if ((j != i) && (tasks[j].get_prio() <= p)) {
						tasks[j].set_prio(tmp_p);
						tmp_p++;
					}
				}

				vector<mixedtask> tmp_tasks = tasks;
				mixedtaskset tmp_taskset = mixedtaskset(tmp_tasks);
				//tmp_taskset.print();
				tuple<Schedulability, vector<int>, vector<int>> res = compute_schedulability(tmp_taskset, i, job_continue);
				if (std::get<0>(res) == Schedulable) {
					tasks = tmp_tasks;
					found = true;
					//cout << "Found task prio " << p << endl;
					break;
				}
			}
		}
		if (found == false) {
			//cout << "NOT found task prio " << p << endl;
			return tuple<bool, mixedtaskset>(false, taskset);			
		}
	}
	mixedtaskset tmp_taskset = mixedtaskset(tasks);
	res = compute_schedulability(tmp_taskset, job_continue);
	if (std::get<0>(res) == Schedulable)
		return tuple<bool, mixedtaskset>(true, mixedtaskset(tasks));
	else {
		return tuple<bool, mixedtaskset>(false, taskset);
	}
}