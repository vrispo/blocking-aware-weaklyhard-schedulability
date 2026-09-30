/**
 * @file MILP.h
 * @brief Mixed Integer Linear Programming (MILP) based schedulability analysis
 * @author Veronica Rispo et al.
 * @date 2026
 * @details This file implements worst-case response time analysis for real-time tasks
 * using Mixed Integer Linear Programming (MILP). The implementation leverages IBM CPLEX Optimizer
 * to solve complex schedulability problems with support for preemption, blocking, weakly-hard constraints,
 * and job continuation policies. The solver models the problem using decision variables for response times,
 * interference, blocking, and auxiliary linearization variables for nonlinear constraints.
 * 
 * The formulation supports:
 * - Preemptive and non-preemptive tasks
 * - (m,k) weakly-hard constraints
 * - Job-continue and job-discard policies
 * - Critical sections and blocking times
 */

#pragma once
#include "mixed_tasks.h"
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

/**
 * @class MILP
 * @brief MILP-based worst-case response time analyzer for real-time tasks
 * @details This class implements a complete MILP formulation for computing worst-case response times
 * of individual tasks in a mixed real-time task set. The solver is particularly useful for complex
 * scenarios where traditional analytic methods are insufficient, such as:
 * - Tasks with both preemptive and non-preemptive sections
 * - Weakly-hard (m,k) timing constraints
 * - Job continuation policies with spillover analysis
 * - Complex inter-task blocking patterns
 * 
 * The class encapsulates the entire CPLEX/Concert workflow:
 * 1. Variable definition (response times, interference, blocking, auxiliary variables)
 * 2. Constraint generation (feasibility, interference bounds, blocking rules)
 * 3. Objective function setup (minimize response time)
 * 4. Solver invocation and result extraction
 * 
 * Solver Configuration (tunable):
 * - Time limit: 20.0 seconds per problem instance
 * - Numerical precision emphasis: disabled
 * - Thread count: configurable (default 40)
 * - Epsilon for linearization: 0.0001
 * - Big-M constant for linearization: 100000000000.0
 * 
 * @warning The solver performs complex nonlinear constraint linearization;
 * numerical precision may be affected with very large task periods or execution times.
 * 
 * @note This class manages all CPLEX/Concert resources (Env, Model, Cplex) and ensures
 * proper cleanup through RAII pattern in the destructor.
 * 
 * @see mixedtaskset
 * @see mixedtask
 * @see compute_schedulability()
 */
class MILP {
 private:
	// model inputs
	mixedtaskset taskset;
	vector<int> seq_i;  // sequence to evaluate
	int task_index;
	vector<int> multiset;
	bool job_continue;

	// milp configuration
	bool verbose_setup = false;
	bool verbose_solver = false;

	const bool apply_numerical_precision = false; // activate numerical precision emphasis
	const bool apply_solver_time_limit = true; // apply solver time limit parameter (false: use default)
	const double solver_time_limit = 20.0; // time limit in seconds (double)
	const bool apply_solver_thread_num = false; // apply solver thread number parameter (false: use default)
	const int solver_thread_num = 40; // thread number

	const double epsilon = 0.0001; // ceiling linearization constant
	const double M = 100000000000.0; // maximum linearization constant

	// solver status
	bool milp_error = false; // true if solver returned an error
	bool solved = false; // true if solver correctly solved the problem
	double result = -1.0; // optimization value returned in case of errors

	// --- Concert/CPLEX typedefs used in the implementation ---
	typedef IloArray<IloNumVarArray> IloNumVarArray2;
	typedef IloArray<IloNumVarArray2> IloNumVarArray3;
	typedef IloArray<IloBoolVarArray> IloBoolVarArray2;
	typedef IloArray<IloBoolVarArray2> IloBoolVarArray3;

	// --- CPLEX objects (non-static members) ---
	IloEnv env;
	IloModel milp_model;
	IloCplex cplex;

	IloBool solve_result;
	IloCplex::CplexStatus solver_status;

	/**
	 * @defgroup ProblemVariables Problem Variables
	 * @brief Primary decision variables of the MILP formulation for worst-case response time analysis.
	 * @{
	 */

	/**< @brief Execution time of job j: \f$E_{i,j} \in [0, C_i]\f$ where \f$C_i\f$ is the worst-case execution time of task i */
	IloNumVarArray execution_time_vars; 

	/** @brief Response time of job j: \f$R_{i,j} \in [C_i, R_i]\f$ where \f$R_i\f$ is the static WCRT */
	IloNumVarArray response_time_vars;

	/** @brief Spillover/carry-over from job j-1: \f$S_{i,j} = \max(0, R_{i,j-1} - T_i)\f$ 
	 *  Models the workload continuation from the previous job if it exceeds one period */
	IloNumVarArray spillover_vars;

	/** @brief Real interference from higher-priority task k job l: \f$I_{i,j,k,l} \in [0, C_k]\f$ 
	 *  Represents the actual execution time contributed by \f$\tau_{k,l}\f$ during the busy window of \f$\tau_{i,j}\f$ */
	IloNumVarArray3 interference_real_vars;

	/** @brief Arrival blocking time of job j: \f$B_{i,j} \in [0, B_i]\f$ 
	 *  Bounded by the multiset of blocking candidates under non-preemptive or resource-sharing policies */
	IloNumVarArray blocking_time_vars;

	/** @brief Binary selection variables for blocking multiset: \f$x^B_{i,j,y} \in \{0,1\}\f$ 
	 *  At most one per job, selecting a single blocking candidate \f$b_y\f$ from \f$\mathcal{B}_i\f$ */
	IloBoolVarArray2 selected_blocking_time_vars;

	/** @} */ // end of ProblemVariables group

	/**
	 * @defgroup AuxiliaryVariables Auxiliary Variables
	 * @brief Integer and binary auxiliary variables introduced for linearization of nonlinear constraints.
	 * @{
	 */

	/** @brief Spillover indicator: \f$y^3_{i,j} \in \{0,1\}\f$ 
	 *  Tracks whether \f$R_{i,j-1} \geq T_i\f$ for modeling positive spillover in constraint perjobSpillover */
	IloBoolVarArray spillover_indicator_vars;

	/** @brief Positive response difference: \f$\max(0, R_{i,j-1} - T_i)\f$ 
	 *  Linearized as the positive part of response time exceeding the period via Big-M linearization */
	IloNumVarArray positive_response_diff_vars;

	/** @brief HP interference ceiling: \f$y^6_{j,k} \in \mathbb{Z}\f$ 
	 *  Linearizes \f$\lceil (R_k + t) / T_k \rceil\f$ for bounding per-job HP interference in constraint interfUBperHPjobsInWindow */
	IloArray<IloNumVarArray> hp_interference_ceiling_vars;

	/** @brief Aggregate interference ceiling: \f$y^{10}_{t,j,k} \in \mathbb{Z}\f$ 
	 *  Linearizes \f$\lceil (R_{i,j} + (t-1)T_i + R_k) / T_k \rceil\f$ for time-shifted window interference bounds in constraint UBinterfWindow */
	IloNumVarArray3 aggregate_interference_ceiling_vars;

	/** @brief Response exceeds period indicator: \f$y^9_{j} \in \{0,1\}\f$ 
	 *  Tracks whether \f$R_{i,j} \geq T_i\f$ in constraint MissSubSeq for bounding terminal miss job response times 
	 *  under job-continue policy when all preceding jobs in the miss subsequence are continuously pending */
	IloBoolVarArray response_exceeds_period_vars;

	/** @brief Blocking precedent miss indicator: \f$y^{blockingMiss}_{j} \in \{0,1\}\f$ 
	 *  Binary indicator tracking whether \f$R_{i,j} \geq T_i\f$ in constraint blockingPrecedentMiss 
	 *  to linearize the conditional blocking constraint via Big-M: if \f$y=1\f$ then \f$B_{i,j+1} = 0\f$ */
	IloBoolVarArray blocking_precedent_miss_indicator_vars;

	/** @} */ // end of AuxiliaryVariables group

	// resource management flag
	bool resources_closed = false; // true when CPLEX/Model/Env have been ended

	/**
	 * @brief Defines all MILP decision variables for the problem instance.
	 * @details This function initializes the primary decision variables:
	 * - Execution time variables
	 * - Response time variables
	 * - Spillover variables
	 * - Interference variables
	 * - Blocking time variables
	 * 
	 * It also initializes auxiliary linearization variables for nonlinear constraints.
	 */
	void define_variables();

	/**
	 * @brief Defines auxiliary variables used for linearization of nonlinear constraints.
	 * @details This function initializes the following auxiliary variables:
	 * - Spillover indicator variables
	 * - Positive response difference variables
	 * - HP interference ceiling variables
	 * - Aggregate interference ceiling variables
	 * - Response exceeds period indicator variables
	 * - Blocking precedent miss indicator variables
	 */
	void define_auxiliary_variables();

	/**
	 * @brief Defines the objective function for the MILP problem.
	 * @details The objective function is set to minimize the response time of the last job in the sequence:
	 * \f[
	 * \text{minimize } R_{i, n-1}
	 * \f]
	 * where \f$n\f$ is the number of jobs in the sequence for task \f$\tau_i\f$.
	 */
	void define_objective_function();

	/**
	 * @brief Defines the per-job deadline miss/hit constraint for each job in the sequence.
	 * 
	 * This constraint enforces the basic conditions for deadline misses and hits:
	 * - If s_i[j] = M (MISS): R_{i,j} > D_i (implemented as R_{i,j} >= D_i + 1 for integer constraints)
	 * - If s_i[j] = H (HIT): R_{i,j} <= D_i
	 * 
	 * Where R_{i,j} is computed as the sum of interference, WCET, spillover, and blocking time.
	 * 
	 * Under job-continue semantics, R_{i,j} is the actual response time of tau_{i,j}.
	 * Under job-kill semantics, R_{i,j} represents the untruncated response time value,
	 * so the job meets its deadline iff R_{i,j} <= D_i, and misses it otherwise.
	 * 
	 * @note The sequence seq_i encodes the pattern to verify: 1 = MISS, 0 = HIT.
	 */
	void define_constraint_perjobDmiss();

	/**
	 * @brief Enforces a static upper bound on per-job higher-priority interference.
	 *
	 * This function implements a valid inequality constraint that bounds the total interference 
	 * a single job \f$\tau_{i,j}\f$ can suffer from higher-priority tasks. It leverages the 
	 * global static Worst-Case Response Time (WCRT) \f$R_i\f$ as an absolute envelope, 
	 * factoring out the job's execution time, arrival blocking, and spillover.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * \sum_{\tau_h \in hp(\tau_i)}\sum_{\tau_{h, l} \in J_{h}} I_{i,j}^{h,l} \leq R_i - C_{i,j} - B_{i,j} - S_{i,j} \quad \forall \tau_{i, j} \in J_{i}
	 * \f]
	 * Derived directly from the core response time definition (\f$R_{i,j} = \sum I_{i,j}^{h,l} + C_{i,j} + B_{i,j} + S_{i,j}\f$) 
	 * under the property that the actual job response time cannot exceed the static baseline (\f$R_{i,j} \leq R_i\f$). 
	 * 
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, the temporary 
	 *       `IloExpr` objects (`constr_lhs` and `constr_rhs`) are explicitly freed via `.end()` 
	 *       at the end of each iteration.
	 * 
	 * @see define_constraint_perjobWCRTdef()
	 */
	void define_constraint_interferenceStaticWCRT();

	/**
	 * @brief Sets the constraints for managing the spillover of each job in the sequence.
	 *
	 * This function implements the MILP constraints bounding the spillover carried into 
	 * each job \f$\tau_{i,j}\f$ based on the response time of the immediately preceding 
	 * job \f$\tau_{i,j-1}\f$ and the active task miss policy.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * S_{i,j} \leq \max\{0, R_{i,j-1} - T_i\} \quad \forall \tau_{i,j} \in J_i \text{ with } j \geq 1
	 * \f]
	 * subject to the boundary condition \f$S_{i,0} = 0\f$.
	 * 
	 * Specifically, the function applies the following logic:
	 * - **Job-Kill Policy:** The spillover is forced to 0 for all jobs (\f$S_{i,j} = 0\f$).
	 * - **Job-Continue Policy:**
	 *   - If \f$j = 0\f$, the spillover is forced to 0.
	 *   - If \f$j > 0\f$ and the preceding job is a **MISS** (\f$seq\_i[j-1] == 1\f$), a Big-M 
	 *     linearization of the \f$\max\f$ operator is applied using auxiliary variables.
	 *   - If the preceding job is a **HIT**, the spillover is bounded to 0.
	 *
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, all temporary 
	 *       `IloExpr` objects allocated inside this function are explicitly freed via `.end()`.
	 * 
	 * @see define_constraint_perjobDmiss()
	 */
	void define_constraint_perjobSpillover();

	/**
	 * @brief Defines the per-job WCRT constraint for each job in the analysis window.
	 * 
	 * This constraint enforces the definition of the response time R_{i,j} for each job
	 * tau_{i,j} as the sum of:
	 * - Interference from all higher-priority tasks: sum over all h in hp(i) and l of I_{i,j}^{h,l}
	 * - Execution time: C_{i,j}
	 * - Spillover from the previous job: S_{i,j}
	 * - Arrival blocking time: B_{i,j}
	 * 
	 * The constraint is implemented as an equality: R_{i,j} = sum(I_{i,j}^{h,l}) + C_i + S_{i,j} + B_{i,j}
	 * 
	 * Under job-continue semantics, R_{i,j} represents the actual modeled response time.
	 * Under job-kill semantics, it represents the response time if the job were allowed to
	 * complete, while the effective activity is bounded by R_{i,j}^*.
	 */
	void define_constraint_perjobWCRTdef();

	/**
	 * @brief Bounds the total interference contribution from each individual higher-priority job.
	 *
	 * This function implements the MILP constraint enforcing that the cumulative interference 
	 * assigned from a single job \f$\tau_{h,l}\f$ of a higher-priority task \f$\tau_h\f$ to 
	 * all jobs \f$\tau_{i,j}\f$ of the task under analysis \f$\tau_i\f$ cannot exceed its 
	 * worst-case execution time (WCET) \f$C_h\f$.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * \sum_{\tau_{i, j} \in J_{i}} I^{h,l}_{i,j} \leq C_{h} \quad \forall \tau_{h, l} \in J_{h}
	 * \f]
	 * where \f$I^{h,l}_{i,j}\f$ maps to `interference_real_vars[j][k][l]`. 
	 * 
	 * This valid inequality guarantees the core conservation principle of the workload allocation 
	 * semantics: since each unit of execution of \f$\tau_{h,l}\f$ can be assigned as direct 
	 * interference at most once across the entire sequence window, the total physical assignment 
	 * is bounded by \f$C_h\f$. Any subsequent indirect delayed effect is structurally captured 
	 * by job spillover variables (\f$S_{i,j}\f$) rather than overlapping interference terms.
	 *
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, the temporary 
	 *       `IloExpr` objects (`constr_lhs` and `constr_rhs`) are explicitly freed via `.end()` 
	 *       at the end of each iteration of the inner loop.
	 * 
	 * @see define_constraint_perjobWCRTdef()
	 * @see define_constraint_interferenceStaticWCRT()
	 */
	void define_constraint_perHPjobInterfUB();

	/**
	 * @brief Bounds the total interference assigned to a job from all instances of a single higher-priority task.
	 *
	 * This function implements a dynamic workload-enveloping MILP constraint. It restricts the cumulative 
	 * interference imposed on a job \f$\tau_{i,j}\f$ by any single higher-priority task \f$\tau_h\f$ 
	 * to the maximum number of activation instances of \f$\tau_h\f$ that can concurrently overlap with 
	 * the effective active window of \f$\tau_{i,j}\f$.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * \sum_{\tau_{h, l} \in J_{h}} I^{h,l}_{i,j} \leq \left\lceil \frac{R_h^\star + R_{i,j}^\star}{T_h} \right\rceil C_h \quad \forall \tau_{i, j} \in J_{i}, \, \tau_h \in hp(\tau_i)
	 * \f]
	 * To implement the non-linear ceiling (\f$\lceil \cdot \rceil\f$) function inside the linear solver, 
	 * the integer decision variables `hp_interference_ceiling_vars[j][k]` (\f$y\f$) are bound via standard 
	 * MILP linearization constraints:
	 * \f[
	 * X \leq y \leq X + 1 - \epsilon
	 * \f]
	 * where \f$X = \frac{R_h^\star + R_{i,j}^\star}{T_h}\f$ and \f$\epsilon\f$ is a small positive tolerance.
	 *
	 * The active windows are accurately truncated based on the configured scheduling policy:
	 * - \f$R_h^\star\f$ bounds the active execution window of the high-priority task, mapping to \f$R_h\f$ 
	 *   under \emph{job-continue} and \f$\min\{R_h, D_h\}\f$ under \emph{job-kill}.
	 * - \f$R_{i,j}^\star\f$ represents the effective response time of the analyzed job. Under \emph{job-kill}, 
	 *   if the job is pre-characterized as a deadline miss (`seq_i[j] == 1`), its window is structurally 
	 *   bounded by \f$D_i\f$.
	 *
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, all temporary 
	 *       `IloExpr` structures are explicitly deallocated via `.end()` within their loop scopes.
	 * 
	 * @see define_constraint_perjobWCRTdef()
	 * @see define_constraint_perHPjobInterfUB()
	 */
	void define_constraint_interfUBperHPjobsInWindow();

	/**
	 * @brief Bounds the cumulative interference assigned from a higher-priority task to all jobs in the sequence window.
	 *
	 * This function enforces a global valid inequality constraint that restricts the total aggregated 
	 * interference injected by all activation instances of an interfering task \f$\tau_h\f$ into the entire 
	 * job sequence under analysis \f$J_i\f$.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * \sum_{\tau_{i, j} \in J_{i}} \sum_{\tau_{h, l} \in J_{h}} I_{i,j}^{h,l} \leq \left\lceil \frac{R_{h}^\star + t^\star}{T_{h}}\right\rceil C_{h} \quad \forall \tau_{h} \in hp(\tau_i)
	 * \f]
	 * where the sequence window length \f$t^\star\f$ is defined as \f$(|s_i|-1)T_i+D_i\f$, representing the right-open 
	 * interval from the release of the first job to the absolute deadline of the last job in the sequence.
	 * 
	 * Since all terms inside the ceiling operator (\f$\lceil \cdot \rceil\f$) are static constants known at model-generation 
	 * time, the bound is pre-calculated using `std::ceil` without introducing additional binary or integer 
	 * decision variables, keeping the MILP matrix compact.
	 *
	 * The boundary condition \f$R_h^\star\f$ dynamically accounts for the real-time execution policy:
	 * - \textbf{job-continue}: \f$R_h^\star = R_h\f$.
	 * - \textbf{job-kill}: \f$R_h^\star = \min\{R_h, D_h\}\f$.
	 *
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, the temporary 
	 *       `IloExpr` objects (`constr_lhs` and `constr_rhs`) are explicitly cleared via `.end()` 
	 *       at the end of each iteration.
	 * 
	 * @see define_constraint_interfUBperHPjobsInWindow()
	 * @see define_constraint_perHPjobInterfUB()
	 */
	void define_constraint_interfAllToAll();

	/**
	 * @brief Enforces an upper bound on the cumulative interference assigned to any sliding window of consecutive jobs.
	 *
	 * This function implements a generalized workload-enveloping constraint over a sliding window of 
	 * \f$w\f$ consecutive jobs of the analyzed task \f$\tau_i\f$, starting at job index \f$j\f$ and ending 
	 * at \f$j+w-1\f$. It bounds the total interference injected by all higher-priority tasks.
	 *
	 * @details 
	 * The mathematical formulation implemented maps to:
	 * \f[
	 * \sum_{\tau_h\in hp(\tau_i)} \sum_{\tau_{h,l}\in J_h} \sum_{q=j}^{j+w-1} I_{i,q}^{h,l} \leq 
	 * \sum_{\tau_h\in hp(\tau_i)} \left\lceil \frac{R_h^\star + (w-1)T_i + R_{i,j+w-1}^{\star}}{T_h} \right\rceil C_h
	 * \f]
	 * where:
	 * - \f$w\f$ varies from 2 to the total sequence length \f$|s_i|\f$ (mapped to loop variable `t`).
	 * - \f$j\f$ tracks the starting job index of the sliding window.
	 * - \f$(w-1)T_i + R_{i,j+w-1}^{\star}\f$ defines the exact physical time interval spanning from the 
	 *   release of \f$\tau_{i,j}\f$ to the completion of the final job in the window \f$\tau_{i,j+w-1}\f$.
	 *
	 * The non-linear ceiling function for each higher-priority task \f$\tau_h\f$ within the window is 
	 * linearized via the 3D integer decision variable matrix `aggregate_interference_ceiling_vars[t][j][k]` (\f$y\f$) 
	 * using the bounded tolerance technique:
	 * \f[
	 * X \leq y \leq X + 1 - \epsilon
	 * \f]
	 * where \f$X = \frac{R_h^\star + (w-1)T_i + R_{i,j+w-1}^{\star}}{T_h}\f$.
	 *
	 * The active execution windows are contextually truncated according to the scheduling policies:
	 * - \f$R_h^\star = \min\{R_h, D_h\}\f$ under \emph{job-kill} for interfering tasks.
	 * - \f$R_{i,j+w-1}^{\star} = D_i\f$ under \emph{job-kill} if the terminal job of the window is a 
	 *   predetermined deadline miss (`seq_i[j+t-1] == 1`).
	 *
	 * @note Out-of-loop and in-loop `IloExpr` objects are systematically cleared using `.end()` to 
	 *       guarantee zero memory overhead during the CPLEX model populating phase.
	 * 
	 * @see define_constraint_interfUBperHPjobsInWindow()
	 * @see define_constraint_interfAllToAll()
	 */
	void define_constraint_UBinterfWindow();

	/**
	 * @brief Enforces response-time upper bounds for the terminal job of a deadline-miss subsequence under job-continue.
	 *
	 * This function models a specialized valid inequality applicable exclusively under the \b job-continue scheduling policy. 
	 * It bounds the response time of the last job (\f$\tau_{i,\overline M}\f$) in a prefix of consecutive deadline misses, 
	 * leveraging an aggregate workload abstraction.
	 *
	 * @details 
	 * The constraint identifies any subsequence \f$ss_i^M\f$ of \f$n = |ss_i^M|\f$ consecutive jobs pre-marked as deadline 
	 * misses (\f$seq\_i[j] == 1\f$) originating after a hit. 
	 * 
	 * Crucially, the mathematical formulation dictates a conditional implication (*if-then* statement): the upper bound is 
	 * enforced if and only if the response time of every job preceding the terminal one inside the window satisfies \f$R_{i,q} \geq T_i\f$ 
	 * (meaning they remain continuously pending, preventing any internal arrival blocking).
	 * 
	 * The implication is implemented via Big-M linearization with auxiliary binary variables.
	 * For each job \f$q\f$ in the miss subsequence, an auxiliary variable \f$y_q\f$ tracks whether \f$R_{i,q} \geq T_i\f$:
	 * \f[
	 * R_{i,q} + M(1 - y_q) \geq T_i \quad \text{(forward: } y_q = 1 \Rightarrow R_{i,q} \geq T_i \text{)}
	 * \f]
	 * \f[
	 * R_{i,q} - M \cdot y_q \leq T_i - \epsilon \quad \text{(reverse: } R_{i,q} \geq T_i \Rightarrow y_q = 1 \text{)}
	 * \f]
	 * 
	 * The consequent constraint then becomes:
	 * \f[
	 * R_{i,\overline M} \leq R_i^n - (n-1)T_i + M \left( (n-1) - \sum_{q=\underline{M}}^{\overline{M}-1} y_q \right)
	 * \f]
	 * 
	 * This enforces the upper bound only when all \f$n-1\f$ conditions hold (i.e., when all \f$y_q = 1\f$), 
	 * where \f$R_i^n\f$ is the static response-time upper bound derived for an equivalent aggregated task \f$\tau_M\f$ defined by:
	 * - \f$C_M = n \cdot C_i\f$ (Aggregate execution demand)
	 * - \f$D_M = D_i + (n-1)T_i\f$ (Relative deadline expansion to encompass the whole window)
	 * - \f$T_M = n \cdot T_i\f$ (Aggregated period structure)
	 *
	 * This aggregate task \f$\tau_M\f$ undergoes worst-case response time analysis via `compute_aRi()`, fully accounting for 
	 * higher-priority preemptive interference and initial arrival blocking. 
	 * 
	 * @see mixedtaskset::compute_aRi()
	 */
	void define_constraint_MissSubSeq();

	/**
	 * @brief Sets the upper bound constraints for the arrival-blocking time of each job.
	 *
	 * This function implements the MILP constraint bounding the actual arrival-blocking 
	 * time \f$B_{i,j}\f$ assigned to a job \f$\tau_{i,j}\f$ based on the selected candidate 
	 * blocking terms from the pre-computed blocking multiset.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * B_{i,j} \leq \sum_{b_y \in \mathcal{B}_i} x^B_{i,j,y} \cdot b_y \quad \forall \tau_{i,j} \in J_{i}
	 * \f]
	 * where \f$x^B_{i,j,y}\f$ is `selected_blocking_time_vars[j][y]` and \f$b_y\f$ is `this->multiset[y]`.
	 * 
	 * The function applies different mapping strategies based on the scheduling policy:
	 * - **Non-Preemptive / Resource-Sharing Taskset (`isPreemptive == false`):** Bounds \f$B_{i,j}\f$ 
	 *   by the sum of the products of the binary selection variables and their respective 
	 *   multiset blocking values. Since at most one binary variable can be 1 (by structural constraints), 
	 *   this effectively assigns the single chosen upper bound.
	 * - **Purely Preemptive Taskset (`isPreemptive == true`):** Arrival blocking is impossible. 
	 *   The function hard-constraints \f$B_{i,j} \leq 0\f$. Given the lower bound of 0, this 
	 *   forces \f$B_{i,j} = 0\f$, neutralizing any floating behavior of unconstrained selection variables.
	 *
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, all temporary 
	 *       `IloExpr` objects allocated inside this function are explicitly freed via `.end()`.
	 * 
	 * @see define_constraint_blockingOnePerJob()
	 * @see define_constraint_perjobDmiss()
	 */
	void define_constraint_blockingUB();

	/**
	 * @brief Sets the constraints to ensure at most one candidate blocking bound is assigned per job.
	 *
	 * This function implements the MILP constraint enforcing that each job \f$\tau_{i,j}\f$ 
	 * selects no more than one candidate upper bound for its arrival blocking time \f$B_{i,j}\f$ 
	 * via the corresponding binary decision variables.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * \sum_{b_y \in \mathcal{B}_i} x^B_{i,j,y} \leq 1 \quad \forall \tau_{i,j} \in J_{i}
	 * \f]
	 * where \f$x^B_{i,j,y}\f$ represents `selected_blocking_time_vars[j][y]`.
	 * 
	 * The constraint generation depends on the system's blocking behavior:
	 * - **Non-Preemptive / Resource-Sharing Taskset:** If any task has a non-zero Non-Preemptive 
	 *   Section parameter (\f$NPS > 0\f$), it implies the presence of arrival blocking. This 
	 *   encompasses both physical non-preemptive execution and resource contention/critical 
	 *   sections. For such configurations (`isPreemptive == false`), the 
	 *   function limits the sum of candidate blocking selection variables to at most 1.
	 * - **Purely Preemptive Taskset (No Resources):** If all tasks have \f$NPS == 0\f$, arrival 
	 *   blocking cannot occur. The function safely bypasses constraint generation to minimize 
	 *   the MILP model size.
	 *
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, all temporary 
	 *       `IloExpr` objects allocated inside this function are explicitly freed via `.end()`.
	 * 
	 * @see define_constraint_perjobDmiss()
	 * @see define_constraint_perjobSpillover()
	 */
	void define_constraint_blockingOnePerJob();

	/**
	 * @brief Sets the constraints to ensure each blocking multiset occurrence is assigned to at most one job.
	 *
	 * This function implements the MILP constraint enforcing that each unique blocking occurrence 
	 * \f$b_y\f$ from the pre-computed blocking multiset \f$\mathcal{B}_i\f$ is allocated to no more 
	 * than one job \f$\tau_{i,j}\f$ of the task under analysis. This prevents the same physical 
	 * blocking event from being multi-counted across different jobs in the sequence.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * \sum_{\tau_{i, j} \in J_{i}} x^B_{i,j,y} \leq 1 \quad \forall b_{y} \in \mathcal{B}_i
	 * \f]
	 * where \f$x^B_{i,j,y}\f$ represents `selected_blocking_time_vars[j][y]`.
	 * 
	 * The function applies the following execution logic:
	 * - **Non-Preemptive / Resource-Sharing Taskset (`isPreemptive == false`):** Iterates over each 
	 *   candidate blocking bound index \f$y\f$ and bounds the summation of its selection variables 
	 *   across all jobs \f$j\f$ to at most 1.
	 * - **Purely Preemptive Taskset (`isPreemptive == true`):** The constraint generation is skipped, 
	 *   as arrival blocking is forced to zero at the job level via the upper bound definition.
	 *
	 * @note To prevent memory leaks within IBM ILOG CPLEX Concert Technology, all temporary 
	 *       `IloExpr` objects allocated inside the loops are explicitly freed via `.end()`.
	 * 
	 * @see define_constraint_blockingOnePerJob()
	 * @see define_constraint_blockingUB()
	 */
	void define_constraint_OneJobPerBlocking();

	/**
	 * @brief Sets the constraints preventing arrival blocking if the preceding job overlaps the next release.
	 *
	 * This function implements the MILP constraint enforcing that a job \f$\tau_{i,j+1}\f$ cannot 
	 * incur arrival blocking if the immediately preceding job \f$\tau_{i,j}\f$ remains active 
	 * up to or past the release instant of \f$\tau_{i,j+1}\f$.
	 *
	 * @details 
	 * The mathematical formulation implemented corresponds to:
	 * \f[
	 * R_{i,j}^{\star} \geq T_i \implies B_{i,j+1} = 0 \quad \forall j \in [0, |s_i|-2]
	 * \f]
	 * where \f$R_{i,j}^{\star}\f$ is the effective (truncated) response time and \f$T_i\f$ is the task period.
	 * 
	 * @note **Critical Implementation Note:** This constraint strictly requires the *effective* 
	 *       response time \f$R_{i,j}^{\star}\f$ rather than the untruncated value \f$R_{i,j}\f$. Under 
	 *       a \emph{job-kill} policy, since missed jobs are aborted at their deadline, the active 
	 *       duration is physically bounded by \f$D_i\f$. Ensure `response_time_vars[j]` correctly 
	 *       reflects this truncation when \emph{job-kill} is active to prevent unsafe optimization steps.
	 * 
	 * The function applies the following execution logic:
	 * - **Non-Preemptive / Resource-Sharing Taskset (`isPreemptive == false`):** Iterates through 
	 *   consecutive job pairs using an `IloIfThen` logical construct to conditionally force 
	 *   \f$B_{i,j+1} = 0\f$.
	 * - **Purely Preemptive Taskset (`isPreemptive == true`):** The constraint is skipped since 
	 *   arrival blocking variables are already structurally forced to zero.
	 * 
	 * @see define_constraint_blockingUB()
	 * @see define_constraint_perjobDmiss()
	 */
	void define_constraint_blockingPrecedentMiss();

	/**
	 * @brief Solves the MILP model
	 * @details Invokes the CPLEX solver to find an optimal solution to the current MILP model.
	 * @note This function should only be called after the model is properly initialized.
	 */
	void solve_milp();

	/**
	 * @brief Prints the current values of all decision variables
	 * @details Outputs the values of response time, interference, spillover, and blocking variables
	 * to the console for debugging and verification purposes.
	 * @note This function should only be called after a successful call to solve_milp().
	 */
	void print_variables();

public:
	/**
	 * @brief Default constructor
	 * @details Initializes an empty MILP solver instance with default configuration.
	 * Must call initialize_milp() before solving.
	 */
	MILP();

	/**
	 * @brief Full constructor with immediate initialization
	 * @param taskset The task set containing the task to analyze
	 * @param seq_i Sequence of miss/success indicators for the task (binary vector)
	 * @param task_index Index of the specific task within taskset to analyze
	 * @param multiset Multiset of candidate blocking times to consider
	 * @param job_continue If true, uses job-continue policy; if false, uses job-discard policy
	 * @details This constructor immediately sets up the MILP model with the given inputs.
	 * The solver is ready for call_solver() after construction.
	 * @throws std::runtime_error if CPLEX initialization fails
	 */
	MILP(mixedtaskset taskset, vector<int> seq_i, int task_index, vector<int> multiset, bool job_continue);

	/**
	 * @brief Destructor
	 * @details Properly releases all CPLEX/Concert resources (Env, Model, Cplex solver).
	 * Ensures no memory leaks or resource handle exhaustion.
	 * @note Safe to call even if initialize_milp() or call_solver() were never invoked.
	 */
	~MILP();

	/**
	 * @brief Reinitialize solver with new problem instance
	 * @param taskset The task set containing the task to analyze
	 * @param seq_i Sequence of miss/success indicators for the task (binary vector)
	 * @param task_index Index of the specific task within taskset to analyze
	 * @param multiset Multiset of candidate blocking times to consider
	 * @param job_continue If true, uses job-continue policy; if false, uses job-discard policy
	 * @details Clears the previous model and rebuilds with new parameters. Useful for
	 * analyzing multiple task indices or sequences without creating new solver instances.
	 * @note This method closes resources from the previous model before rebuilding.
	 */
	void initialize_milp(mixedtaskset taskset, vector<int> seq_i, int task_index, vector<int> multiset, bool job_continue);

	/**
	 * @brief Solve the MILP instance and extract worst-case response time
	 * @return Pair of (solver_status, response_time):
	 *   - solver_status (bool): true if solver found an optimal solution, false on error/timeout
	 *   - response_time (int): computed worst-case response time (valid only if solver_status=true)
	 * @details Invokes the CPLEX solver and retrieves the objective function value (WCRT).
	 * Updates internal solver_status and milp_error flags.
	 * 
	 * Possible outcomes:
	 * - Optimal solution found: returns (true, WCRT)
	 * - Solver timeout (20s): returns (false, -1)
	 * - Infeasible problem: returns (false, -1)
	 * - Solver error: returns (false, -1)
	 * 
	 * @note The response time is only meaningful when solver_status=true.
	 * @see initialize_milp()
	 */
	std::pair<bool, int> call_solver();
};