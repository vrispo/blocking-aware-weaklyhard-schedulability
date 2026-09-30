/**
 * @file mixed_tasks.h
 * @brief Task representation and management for weakly-hard scheduling analysis
 * @author Veronica Rispo
 * @date 2026
 * @details This file defines data structures and classes for representing mixed real-time tasks
 * with both preemptive and non-preemptive characteristics, including support for (m,k) weakly-hard
 * constraints, critical sections, and periodic/sporadic task types. It provides core abstractions
 * for the schedulability analysis framework.
 */

#pragma once
#include <utility>
#include <vector>
#include <tuple>
#include <string>
#include <iostream>
#include "c_utility.h"

#define SETWval 6
using namespace std;

/**
 * @enum Schedulability
 * @brief Enumeration representing the schedulability status of a task or task set
 * @details Defines three possible schedulability states for response time analysis results.
 */
enum Schedulability { 
    Schedulable,    ///< Task set is schedulable (all jobs meet deadlines)
    Unschedulable,  ///< Task set is unschedulable (at least one job misses deadline)
    none            ///< Unknown or uncomputed schedulability status
};

/**
 * @class mixedtask
 * @brief Represents a single real-time task with mixed preemption characteristics
 * @details This class encapsulates a real-time task that can exhibit both preemptive
 * and non-preemptive execution patterns. Supports tasks with (m,k) weakly-hard constraints,
 * critical sections, and both periodic and sporadic activation patterns.
 * 
 * Key attributes:
 * - WCET (Worst-Case Execution Time): Maximum execution time per job
 * - RCT (Run-to-Completion Threshold): Jobs executing longer than RCT can be preempted
 * - NPS (Non-Preemptive Section): Longest critical section where task cannot be preempted
 * - Period T and Deadline D
 * - (m,k) constraint: Task must complete m out of every k consecutive jobs (weakly-hard)
 * - Utilization U: Ratio of WCET to period
 * 
 * @see mixedtaskset
 */
class mixedtask {
private:
	int WCET;
	int RCT; // run-to-completion threshold - RCT=WCET if preemptive , RCT = epsilon if non preemptive
	int NPS; // longest non-preemptive section - NPS=WCET if non-preemptive , NPS = epsilon if preemptive
	int T;
	float U;
	int prio;
	bool isPeriodic;
	bool isPreemptive; // true if NPS == 0 (task is preemptive), false if NPS > 0 (non-preemptive section present)
	int D;
	pair<int, int> mk;

public:
	/**
	 * @brief Default constructor — creates a task with all attributes set to zero/false.
	 */
	mixedtask();

	/**
	 * @brief Constructs a hard real-time task without weakly-hard constraints.
	 * @param WCET  Worst-case execution time.
	 * @param T     Task period.
	 * @param D     Relative deadline.
	 * @param RCT   Run-to-completion threshold (equals WCET for fully preemptive tasks,
	 *              or a small epsilon for non-preemptive tasks).
	 * @param NPS   Longest non-preemptive section (equals WCET for non-preemptive tasks,
	 *              or a small epsilon for fully preemptive tasks).
	 * @param prio  Fixed priority assigned to the task (lower value = higher priority).
	 */
	mixedtask(int WCET, int T, int D, int RCT, int NPS, int prio);

	/**
	 * @brief Constructs a weakly-hard task with an (m,k) constraint.
	 * @param WCET  Worst-case execution time.
	 * @param T     Task period.
	 * @param D     Relative deadline.
	 * @param RCT   Run-to-completion threshold.
	 * @param NPS   Longest non-preemptive section.
	 * @param mk    Pair (m, k) expressing the weakly-hard constraint: at least m out of
	 *              every k consecutive jobs must complete within their deadline.
	 * @param prio  Fixed priority assigned to the task.
	 */
	mixedtask(int WCET, int T, int D, int RCT, int NPS, pair<int, int> mk, int prio);

	/** @brief Returns the worst-case execution time (WCET). */
	int get_WCET() const;

	/**
	 * @brief Returns the run-to-completion threshold (RCT).
	 * @details Equals WCET for fully preemptive tasks; equals a small epsilon for
	 *          non-preemptive tasks, indicating no mid-job preemption is allowed.
	 */
	int get_RCT() const;

	/**
	 * @brief Returns the longest non-preemptive section (NPS).
	 * @details Equals WCET for non-preemptive tasks; equals a small epsilon for
	 *          fully preemptive tasks.
	 */
	int get_NPS() const;

	/** @brief Returns the task period T. */
	int get_T() const;

	/** @brief Returns the task utilization U = WCET / T. */
	float get_U() const;

	/** @brief Returns the fixed priority of the task (lower value = higher priority). */
	int get_prio() const;

	/** @brief Returns true if the task has periodic activation, false if sporadic. */
	bool get_isPeriodic() const;

	/** @brief Returns the relative deadline D of the task. */
	int get_D() const;

	/**
	 * @brief Returns the (m,k) weakly-hard constraint as a pair.
	 * @return Pair (m, k) where m is the minimum number of jobs that must complete
	 *         within every window of k consecutive jobs.
	 */
	pair<int, int> get_mk() const;

	/**
	 * @brief Returns the m value of the (m,k) weakly-hard constraint.
	 * @return Minimum number of jobs that must complete within every k-job window.
	 */
	int get_m() const;

	/**
	 * @brief Returns true if the task is preemptive (NPS negligibly small).
	 * @details Returns false when the task has a non-trivial non-preemptive section (NPS > 0),
	 *          meaning it holds resources without being preempted for a non-negligible duration.
	 */
	bool get_isPreemptive() const;

	/**
	 * @brief Sets the fixed priority of the task.
	 * @param prio New priority value (lower value = higher priority).
	 */
	void set_prio(int prio);

	/**
	 * @brief Sets the (m,k) weakly-hard constraint.
	 * @param mk Pair (m, k) defining the new weakly-hard constraint.
	 */
	void set_mk(pair<int, int> mk);

	/**
	 * @brief Converts the task from periodic to sporadic activation.
	 * @details Sets the isPeriodic flag to false; T becomes the minimum inter-arrival time.
	 */
	void set_sporadic();

	/**
	 * @brief Prints the task parameters to standard output.
	 * @details Displays WCET, RCT, NPS, period, deadline, priority, utilization,
	 *          and (m,k) constraint in a formatted table row.
	 */
	void print() const;

	/**
	 * @brief Prints the task parameters together with its computed response time.
	 * @param R Worst-case response time to display alongside the task attributes.
	 */
	void print(int R) const;
};

/**
 * @enum Scheduler
 * @brief Enumeration of supported scheduling policies
 * @details Defines the scheduling algorithms available for task prioritization and analysis.
 */
enum Scheduler {
    FP,    ///< Fixed Priority scheduling
    EDF,   ///< Earliest Deadline First
    FIFO   ///< First-In-First-Out
};

/**
 * @class mixedtaskset
 * @brief Collection of real-time tasks with joint schedulability analysis
 * @details This class represents a set of mixed tasks and provides methods for
 * computing response times, checking schedulability, and managing task priorities.
 * Supports various scheduling algorithms (FP, EDF, FIFO) and joint schedulability
 * analysis considering inter-task interference and blocking times.
 * 
 * Key features:
 * - Storage and management of multiple mixed tasks
 * - Response time computation via fixed-point iteration and abstract analysis
 * - Schedulability verification (hard real-time, weakly-hard)
 * - Automatic priority assignment (RM: Rate Monotonic)
 * - Blocking multiset computation for non-preemptive sections
 * - Support for job continuation policies in weakly-hard analysis
 * 
 * @note Preemptive property is determined by the most restrictive task in the set
 * @see mixedtask
 * @see realizable_sequence()
 */
class mixedtaskset {
private:
	vector<mixedtask> taskset;
	vector<int> R;
	Schedulability schedulable;
	float U;
	Scheduler sched = FP;
	bool preemptive; // false if at least one task is "non-preemptive" (NPS>0)

	vector<float> compute_Ai(int index);

	/**
	 * @brief Computes the blocking time for a specific task in the set
	 * @param index Index of the task in the taskset
	 * @return Maximum blocking time experienced by the task due to lower-priority tasks
	 * @details The blocking time is determined by the longest non-preemptive section (NPS)
 *          of any lower-priority task that is active at the time task i is released.
	 */
	float compute_Bi(int index, float A);

	/**
	 * @brief Compute the interference bound (IBF) for the task at index 'index'.
	 * * This function implements Equation (17) from the paper:
	 * \alpha_i^IBF(A, \delta) = \sum_{j \in hp(i)} \alpha_j^IBF(A, \delta)
	 * * Where hp(i) is the set of tasks with higher priority than 'i'.
	 * The \alpha_j^IBF for a single task 'j' is calculated using Equation (10).
	 * * It is assumed that higher priority = lower priority number.
	 *
	 * @param index The index of the task 'i' that is experiencing interference.
	 * @param A The time interval 'A' (in this context, the current aR_i), preemtable window.
	 * @param delta The "push-forward" delta (in this context, \delta_i^RTA).
	 * @return (float) The total interference experienced by the task 'i'.
	 */
	float compute_IBFi(int index,float A,float delta);
	
public:
	/**
	 * @brief Default constructor for mixedtaskset
	 * @details Initializes an empty task set with default properties.
	 */
	mixedtaskset();
	/**
	 * @brief Constructor for mixedtaskset with a given vector of tasks
	 * @param taskset Vector of mixedtask objects to initialize the task set
	 * @details Initializes the task set with the provided tasks and computes initial properties.
	 */
	mixedtaskset(vector<mixedtask>taskset);

	/**
	 * @brief Retrieves the vector of tasks in the task set
	 * @return Vector of mixedtask objects representing the task set
	 */
	vector<mixedtask> get_taskset() const;

	/**
	 * @brief Retrieves the vector of computed response times for each task
	 * @return Vector of integers representing the worst-case response times (R_i) for each task
	 */
	vector<int> get_R() const;

	/**
	 * @brief Retrieves the schedulability status of the task set
	 * @return Schedulability enum value indicating if the task set is schedulable, unschedulable, or unknown
	 */
	Schedulability get_schedulable() const;

	/**
	 * @brief Retrieves the total utilization of the task set
	 * @return Float value representing the sum of utilizations (U) of all tasks in the set
	 */
	float get_U() const;

	/**
	 * @brief Retrieves the task at a specific index in the task set
	 * @param index Index of the task to retrieve
	 * @return mixedtask object representing the task at the specified index
	 * @throws std::out_of_range if the index is invalid
	 * @details Provides access to individual tasks for analysis or modification.
	 * @see mixedtask
	 */
	mixedtask get_task(int index) const;

	/**
	 * @brief Retrieves the hyperperiod (H) of the task set
	 * @return Integer value representing the least common multiple of all task periods (T)
	 * @details The hyperperiod is used in schedulability analysis to determine the time frame
	 * over which the task set behavior repeats.
	 */
	int get_H() const;

	/**
	 * @brief Retrieves the computed response time (R_i) for a specific task
	 * @param index Index of the task in the task set
	 * @return Integer value representing the worst-case response time for the task at 'index'
	 * @throws std::out_of_range if the index is invalid
	 */
	int get_R(int index) const;

	/**
	 * @brief Computes the blocking time (B_i) for a specific task in the set
	 * @param index Index of the task in the task set
	 * @return Integer value representing the maximum blocking time experienced by the task
	 * @details The blocking time is determined by the longest non-preemptive section (NPS)
	 * of lower-priority tasks that can block the execution of the task at 'index'.
	 */
	int get_B(int index) const;

	/**
	 * @brief Checks if the task set is preemptive
	 * @return true if all tasks are preemptive (NPS=0), false if at least one task is non-preemptive
	 * @details The preemptive property of the task set is determined by the most restrictive task.
	 */
	bool isPreemptive() const;

	/**
	 * @brief Assigns priorities to tasks based on Rate Monotonic (RM) policy
	 * @details Sorts tasks by period (T) and assigns lower priority numbers to tasks with shorter periods.
	 * This method modifies the internal priority values of the tasks in the set.
	 */
	void set_prioRM();

	/**
	 * @brief Sets the (m,k) weakly-hard constraint for a specific task
	 * @param index Index of the task in the task set
	 * @param mk Pair representing the (m,k) constraint to assign
	 * @details Updates the specified task's weakly-hard constraint, which defines how many jobs
	 * must complete successfully within a window of k consecutive jobs.
	 */
	void set_mk(int index, pair<int, int> mk);

	/**
	 * @brief Sets a specific task to be sporadic (non-periodic)
	 * @param index Index of the task in the task set
	 * @details Updates the specified task's activation pattern to be sporadic, removing any periodicity.
	 */
	void set_sporadic(int index);

	/**
	 * @brief Sets the vector of computed response times (R_i) for the task set
	 * @param R Vector of integers representing the worst-case response times for each task
	 * @param sched Schedulability status to assign to the task set
	 * @details Updates the internal response time vector and schedulability status based on analysis results.
	 */
	void set_R(vector<int> R, Schedulability sched);

	/**
	 * @brief Sets the computed response time (R_i) for a specific task
	 * @param index Index of the task in the task set
	 * @param Ri Integer value representing the worst-case response time to assign
	 * @details Updates the internal response time for the specified task, allowing for incremental analysis.
	 */
	void set_Ri(int index, int Ri);

	/**
	 * @brief Computes the abstract response time (aR_i) for a specific task
	 * @param index Index of the task in the task set
	 * @return Integer value representing the abstract response time for the task at 'index'
	 * @details The abstract response time is computed using an abstract analysis method that
	 * considers interference and blocking from other tasks in the set.
	 * @note This method may compute aR_i even if it exceeds the task's deadline, but includes safeguards
	 * to prevent infinite loops in cases where the total utilization exceeds 1.
	 */
	int compute_aRi(int index);

	/**
	 * @brief Computes the worst-case response time (R_i) for a specific task
	 * @param index Index of the task in the task set
	 * @return Integer value representing the worst-case response time for the task at 'index'
	 * @details The worst-case response time is computed using fixed-point iteration, accounting
	 * for execution demand, blocking, and interference from higher-priority tasks.
	 */
	int compute_Ri(int index);

	/**
	 * @brief Computes the multiset of blocking times for a specific task
	 * @param index Index of the task in the taskset
	 * @param size_multiset Desired size of the multiset (number of jobs to consider)
	 * @return Vector containing the blocking times contributed by lower-priority tasks
	 * @details The multiset is constructed based on the non-preemptive sections (NPS) of
	 * lower-priority tasks that can block the execution of the task at 'index'.
	 * The vector is sorted in descending order and resized to 'size_multiset'.
	 */
	vector<int> compute_multiset(int index, int size_multiset);

	/**
	 * @brief Prints the task set and its properties to standard output
	 * @details Displays a formatted table of all tasks in the set, including their
	 * parameters (WCET, period, deadline, priority, etc.) and computed response times.
	 */
	void print() const;
};

/**
 * @brief Checks if a given miss sequence is realizable under job-continue policy
 * @param tmp_seq_i Vector representing a potential miss sequence for a specific task
 * @param cache Cached intermediate results to optimize computation
 * @param index Index of the task in the taskset
 * @param M_upperbound Upper bounds on the window length for each task
 * @param taskset The task set to analyze
 * @param job_continue If true, use job-continue policy; if false, use job-discard policy
 * @return true if the sequence is realizable, false otherwise
 * @details This function implements the backward reachability analysis to verify if a miss
 * sequence can actually occur given the system dynamics and scheduling policy.
 * @see compute_M_upperbound()
 * @see realizable_sequence()
 */
bool realizable_sequence(vector<int> tmp_seq_i, vector<vector<int>> cache, int index, vector<int> M_upperbound, mixedtaskset taskset, bool job_continue);

/**
 * @brief Computes the upper bound on the analysis window length for weakly-hard verification
 * @param index Index of the task in the taskset
 * @param M_upperbound Current upper bounds for all tasks
 * @param taskset The task set to analyze
 * @param job_continue If true, use job-continue policy; if false, use job-discard policy
 * @return Integer representing the computed upper bound on analysis window
 * @details Determines how many consecutive jobs need to be analyzed to verify (m,k) constraints.
 * @see realizable_sequence()
 */
int compute_M_upperbound(int index, vector<int> M_upperbound, mixedtaskset taskset, bool job_continue);

/**
 * @brief Computes schedulability of entire task set under weakly-hard constraints
 * @param taskset The task set to analyze
 * @param job_continue If true, use job-continue policy; if false, use job-discard policy
 * @return Tuple containing:
 *   - Schedulability status (Schedulable, Unschedulable, or none)
 *   - Vector of response times for each task
 *   - Vector of blocking times for each task
 * @details Performs complete schedulability analysis considering (m,k) weakly-hard constraints
 * for all tasks in the set, using sequence enumeration and reachability analysis.
 * @see compute_schedulability()
 * @see compute_HARD_schedulability()
 */
tuple<Schedulability,vector<int>, vector<int>> compute_schedulability(mixedtaskset taskset, bool job_continue);

/**
 * @brief Computes schedulability of a specific task under weakly-hard constraints
 * @param taskset The task set to analyze
 * @param index Index of the specific task to verify
 * @param job_continue If true, use job-continue policy; if false, use job-discard policy
 * @return Tuple containing:
 *   - Schedulability status
 *   - Vector of response times
 *   - Vector of blocking times
 * @details Focuses schedulability analysis on a single task while considering interference
 * from higher-priority tasks.
 * @see compute_schedulability(mixedtaskset, bool)
 */
tuple<Schedulability, vector<int>, vector<int>> compute_schedulability(mixedtaskset taskset, int index, bool job_continue);

/**
 * @brief Computes hard real-time (100% guarantee) schedulability
 * @param taskset The task set to analyze
 * @param job_continue If true, use job-continue policy; if false, use job-discard policy
 * @return Tuple containing:
 *   - Schedulability status
 *   - Vector of response times
 *   - Vector of blocking times
 * @details Analyzes schedulability assuming all jobs must meet their deadlines (no weakly-hard
 * constraints). Uses standard response time analysis techniques.
 * @see compute_HARD_CAN_schedulability()
 */
tuple<Schedulability,vector<int>, vector<int>> compute_HARD_schedulability(mixedtaskset taskset, bool job_continue);

/**
 * @brief Computes hard real-time schedulability using CAN
 * @param taskset The task set to analyze
 * @param job_continue If true, use job-continue policy; if false, use job-discard policy
 * @return Tuple containing:
 *   - Schedulability status
 *   - Vector of response times
 *   - Vector of blocking times
 * @details Alternative hard real-time analysis using canonical busy-window analysis.
 * @see compute_HARD_schedulability()
 */
tuple<Schedulability,vector<int>, vector<int>> compute_HARD_CAN_schedulability(mixedtaskset taskset, bool job_continue);

/**
 * @brief Performs optimal priority assignment for task set
 * @param taskset The task set for which to assign priorities
 * @param job_continue If true, use job-continue policy; if false, use job-discard policy
 * @return Tuple containing:
 *   - Boolean: true if a feasible priority assignment exists, false otherwise
 *   - Modified taskset with assigned priorities
 * @details Searches the space of priority assignments to find one that makes the task set
 * schedulable under the specified job continuation policy. Uses heuristics and backtracking.
 * @see set_prioRM()
 */
tuple<bool, mixedtaskset> compute_priority_assignment(mixedtaskset taskset, bool job_continue);