/**
 * @file MILP.cpp
 * @brief Implementation of MILP-based schedulability analysis
 * @author Veronica Rispo et al.
 * @date 2026
 * @details This file contains the complete implementation of the MILP class
 * for worst-case response time analysis using IBM CPLEX Optimizer.
 * 
 * Key implementation components:
 * - Variable definition for response times, interference, blocking, and auxiliary variables
 * - Constraint formulation including interference bounds, blocking rules, spillover analysis
 * - CPLEX solver configuration and invocation
 * - Resource management for CPLEX objects (Env, Model, Cplex)
 * 
 * The implementation handles linearization of nonlinear constraints using Big-M methods
 * and auxiliary binary variables. The solver is time-limited to 20 seconds per instance
 * to ensure practical execution times.
 * 
 * @see MILP.h for class interface
 */

#include "MILP.h"
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

void MILP::define_variables() {
	if (verbose_setup) {
		std::cout << "1> Setting variables." << std::endl;
	}

	// Execution time variables
	execution_time_vars = IloNumVarArray(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) { // define variables for task i and job j (one  variables per job in the window of task i under analysis)
		std::string var_name = "E_{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";
		IloNumVar execution_time_vars_j(env, 0, this->taskset.get_task(this->task_index).get_WCET(), var_name.c_str());
		execution_time_vars.add(execution_time_vars_j);
	}

	//Response time variables
	response_time_vars = IloNumVarArray(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) { // define variables for task i and job j (one  variables per job in the window of task i under analysis)
		std::string var_name = "R_{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";
		IloNumVar response_time_vars_j(env, this->taskset.get_task(this->task_index).get_WCET(), this->taskset.get_R(this->task_index), var_name.c_str());
		response_time_vars.add(response_time_vars_j);
	}

	//Spillover variables
	spillover_vars = IloNumVarArray(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) { // define variables for task i and job j (one  variables per job in the window of task i under analysis)
		std::string var_name = "S_{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";
		IloNumVar spillover_vars_j(env, 0,IloNumMax, var_name.c_str());
		spillover_vars.add(spillover_vars_j);
	}

	//Interference real variables 
	interference_real_vars = IloNumVarArray3(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) { // define variables for task i and job j (one  variables per job in the window of task i under analysis)
		IloNumVarArray2 interference_real_vars_i_vars_j(env);
		for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
			IloNumVarArray interference_real_vars_i_vars_jk(env);
			int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
			for (int l = 0; l < tot_l; l++) {
				std::string var_name = "I_{" + std::to_string(this->task_index) + "," + std::to_string(j) + " - " + std::to_string(k) + "," + std::to_string(l) + "}";
				IloNumVar interference_real_vars_i_vars_jkl(env, 0, this->taskset.get_task(k).get_WCET(), var_name.c_str());
				interference_real_vars_i_vars_jk.add(interference_real_vars_i_vars_jkl);
			}
			interference_real_vars_i_vars_j.add(interference_real_vars_i_vars_jk);
		}
		interference_real_vars.add(interference_real_vars_i_vars_j);
	}

	//Blocking time variables
	blocking_time_vars = IloNumVarArray(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) { // define variables for task i and job j (one  variables per job in the window of task i under analysis)
		std::string var_name = "B_{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";
		IloNumVar blocking_time_vars_j(env, 0, this->taskset.get_B(this->task_index), var_name.c_str());
		blocking_time_vars.add(blocking_time_vars_j);
	}

	//Selected blocking time variables
	selected_blocking_time_vars = IloBoolVarArray2(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) { // define variables for task i and job j (one  variables per job in the window of task i under analysis)
		IloBoolVarArray selected_blocking_time_vars_ij(env);
		for (int y = 0; y < (int)this->seq_i.size(); y++) {
			std::string var_name = "x_{" + std::to_string(this->task_index) + "," + std::to_string(j) + "," + std::to_string(y) + "^B}";
			IloBoolVar selected_blocking_time_vars_ijy(env, var_name.c_str());
			selected_blocking_time_vars_ij.add(selected_blocking_time_vars_ijy);
		}
		selected_blocking_time_vars.add(selected_blocking_time_vars_ij);
	}

	define_auxiliary_variables();

	if (verbose_setup) {
		std::cout << "1> Variables ready." << std::endl;
	}
}

void MILP::define_auxiliary_variables() {
	if (verbose_setup) {
		std::cout << "1> Setting auxiliary variables." << std::endl;
	}

	spillover_indicator_vars = IloBoolVarArray(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		std::string var_name = "y_3_r{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";

		IloBoolVar auxiliary_3_r_vars_j(env, var_name.c_str());
		spillover_indicator_vars.add(auxiliary_3_r_vars_j);
	}
	
	positive_response_diff_vars = IloNumVarArray(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		std::string var_name = "Max_3_r{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";

		IloNumVar max_auxiliary_3_r_vars_j(env, var_name.c_str());
		positive_response_diff_vars.add(max_auxiliary_3_r_vars_j);
	}

	hp_interference_ceiling_vars = IloArray<IloNumVarArray>(env);

	for (int j = 0; j < (int)this->seq_i.size(); j++) { // define variables for task i and job j (one  variables per job in the window of task i under analysis)
		IloNumVarArray hp_interference_ceiling_vars_j(env);
		for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
			std::string var_name = "y_6_r{" + std::to_string(this->task_index) + std::to_string(j) + "," + std::to_string(k) + "}";
			IloIntVar hp_interference_ceiling_vars_ijk(env, var_name.c_str());
			hp_interference_ceiling_vars_j.add(hp_interference_ceiling_vars_ijk);
		}
		hp_interference_ceiling_vars.add(hp_interference_ceiling_vars_j);
	}

	aggregate_interference_ceiling_vars = IloNumVarArray3(env);	
	for (int t = 0; t <= (int)this->seq_i.size(); t++) { 
		IloNumVarArray2 auxiliary_10_r_vars_t(env);
		for (int j = 0; j < (int)this->seq_i.size() ; j++) {
			IloNumVarArray auxiliary_10_r_vars_t_j(env);
			for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++){
				std::string var_name = "y_10_r_{" + std::to_string(t) + "," + std::to_string(j) + " , " + std::to_string(k) + "}";
				IloNumVar auxiliary_10_r_vars_t_j_k(env, var_name.c_str());
				auxiliary_10_r_vars_t_j.add(auxiliary_10_r_vars_t_j_k);
			}
			auxiliary_10_r_vars_t.add(auxiliary_10_r_vars_t_j);
		}
		aggregate_interference_ceiling_vars.add(auxiliary_10_r_vars_t);
	}

	// Auxiliary binary variables for Constraint 9: y_9_miss{i, j} = 1 <==> R_{i,j} >= T_i
	response_exceeds_period_vars = IloBoolVarArray(env);
	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		std::string var_name = "y_9_miss{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";
		IloBoolVar y_9_j(env, var_name.c_str());
		response_exceeds_period_vars.add(y_9_j);
	}

	// Auxiliary binary variables for blocking precedent miss constraint: y_blockingMiss{i, j} = 1 <==> R_{i,j} >= T_i
	blocking_precedent_miss_indicator_vars = IloBoolVarArray(env);
	for (int j = 0; j < (int)this->seq_i.size() - 1; j++) {
		std::string var_name = "y_blockingMiss{" + std::to_string(this->task_index) + "," + std::to_string(j) + "}";
		IloBoolVar y_blocking_miss_j(env, var_name.c_str());
		blocking_precedent_miss_indicator_vars.add(y_blocking_miss_j);
	}
}

void MILP::define_objective_function() {
	if (verbose_setup) {
		std::cout << "1> Setting objective function." << std::endl;
	}
	//Since we check the feasibility of the problem, we can not set an objective function.
}

void MILP::define_constraint_perjobDmiss() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint perjobDmiss." << std::endl;
	}
	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		IloExpr constr_lhs(env);
		IloExpr constr_rhs(env);
		for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
			if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
				int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
				for (int l = 0; l < tot_l; l++) {
					constr_lhs += interference_real_vars[j][k][l];
				}
			}
		}
		constr_lhs += spillover_vars[j] + execution_time_vars[j] + blocking_time_vars[j];
		if (seq_i[j] == 1) { // It is a MISS
			constr_rhs += this->taskset.get_task(this->task_index).get_D() + 1;
			//set inequality contraint
			milp_model.add(constr_lhs >= constr_rhs);
		}
		else { // It is a HIT
			constr_rhs += this->taskset.get_task(this->task_index).get_D();
			//set inequality contraint
			milp_model.add(constr_lhs <= constr_rhs);
		}
		// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
		constr_lhs.end();
		constr_rhs.end();
	}
}

void MILP::define_constraint_interferenceStaticWCRT() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint interferenceStaticWCRT." << std::endl;
	}

	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		IloExpr constr_lhs(env);
		IloExpr constr_rhs(env);
		for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
			if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
				int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
				for (int l = 0; l < tot_l; l++) {
					constr_lhs += interference_real_vars[j][k][l];
				}
			}
		}
		constr_rhs += this->taskset.get_R(this->task_index) - execution_time_vars[j] - blocking_time_vars[j] - spillover_vars[j];
		milp_model.add(constr_lhs <= constr_rhs);
		// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
		constr_lhs.end();
		constr_rhs.end();
	}
}

void MILP::define_constraint_perjobSpillover() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint perjobSpillover." << std::endl;
	}
	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		IloExpr constr_lhs(env); 
		IloExpr constr_rhs(env);

		constr_lhs += spillover_vars[j];

		constr_rhs += 0;
		if (job_continue) {
			if (j != 0) {
				if (this->seq_i[j - 1] == 1) {
					milp_model.add(positive_response_diff_vars[j] >= 0); // positive_response_diff_vars[j] >= 0
					milp_model.add(positive_response_diff_vars[j] >= response_time_vars[j - 1] - this->taskset.get_task(this->task_index).get_T()); // positive_response_diff_vars[j] >= R_{i,j-1} - T_i
					
					milp_model.add(positive_response_diff_vars[j] <= M * (1 - spillover_indicator_vars[j])); // positive_response_diff_vars[j] <= M * (1 - y_{i,j}^3)
					milp_model.add(positive_response_diff_vars[j] <= (response_time_vars[j - 1] - this->taskset.get_task(this->task_index).get_T()) + M * spillover_indicator_vars[j]); // positive_response_diff_vars[j] <= (R_{i,j-1} - T_i) + M * y_{i,j}^3

					constr_rhs += positive_response_diff_vars[j];
				}
			}
		}

		milp_model.add(constr_lhs <= constr_rhs);
		// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
		constr_lhs.end();
		constr_rhs.end();
	}
}

void MILP::define_constraint_perjobWCRTdef() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint perjobWCRTdef." << std::endl;
	}
	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		IloExpr constr_lhs(env);
		IloExpr constr_rhs(env);

		constr_lhs += response_time_vars[j];

		for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
			if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
				int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
				for (int l = 0; l < tot_l; l++) {
					constr_rhs += interference_real_vars[j][k][l];
				}
			}
		}
		constr_rhs += execution_time_vars[j] + spillover_vars[j] + blocking_time_vars[j];

		milp_model.add(constr_lhs <= constr_rhs);
		milp_model.add(constr_lhs >= constr_rhs);
		// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
		constr_lhs.end();
		constr_rhs.end();
	}
}

void MILP::define_constraint_perHPjobInterfUB() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint perHPjobInterfUB." << std::endl;
	}
	for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
		if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
			int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
			for (int l = 0; l < tot_l; l++) {
				IloExpr constr_lhs(env);
				IloExpr constr_rhs(env);
				for (int j = 0; j < (int)this->seq_i.size(); j++) {
					constr_lhs += interference_real_vars[j][k][l];
				}
				constr_rhs += this->taskset.get_task(k).get_WCET();
				milp_model.add(constr_lhs <= constr_rhs);
				// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
				constr_lhs.end();
				constr_rhs.end();
			}
		}
	}
}

void MILP::define_constraint_interfUBperHPjobsInWindow() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint interfUBperHPjobsInWindow." << std::endl;
	}
	for (int j = 0; j < (int)this->seq_i.size(); j++) {
		for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
			if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
				IloExpr constr_lhs(env);
				IloExpr constr_rhs(env);
				int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
				for (int l = 0; l < tot_l; l++) {
					constr_lhs += interference_real_vars[j][k][l];
				}
				// ceil of (Rk + Rij / Tk)
				constr_rhs += (this->taskset.get_task(k).get_WCET() * hp_interference_ceiling_vars[j][k]);
				milp_model.add(constr_lhs <= constr_rhs);
				// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
				constr_lhs.end();
				constr_rhs.end();

				IloExpr constr_lhs2(env);
				IloExpr constr_rhs2(env);
				int R_k_star;
				if (job_continue) {
					R_k_star = this->taskset.get_R(k);
				}
				else {
					// R_k_star is the minimum between the WCRT of task k and its relative deadline D_k, due to the fact that in the job-kill semantics, the response time of a higher priority task k is truncated to its relative deadline D_k.
					R_k_star = std::min(this->taskset.get_R(k), this->taskset.get_task(k).get_D());
				}

				// The response term R_{i,j}^\star is defined as follows:
				IloExpr response_term(env);
				if (job_continue || this->seq_i[j] == 0) {
					response_term += response_time_vars[j];
				} else {
					response_term += this->taskset.get_task(this->task_index).get_D();
				}

				constr_lhs2 += (((double)R_k_star + response_term) / (double)this->taskset.get_task(k).get_T());
				milp_model.add(constr_lhs2 <= hp_interference_ceiling_vars[j][k]); // ( R_k + t / T_k) <= y6_r
				constr_rhs2 += (((double)R_k_star + response_time_vars[j]) / (double)this->taskset.get_task(k).get_T()) + 1 - epsilon;
				milp_model.add(hp_interference_ceiling_vars[j][k] <= constr_rhs2); // y6_r <= ( R_k + t / T_k) + 1 -epsilon
				// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
				constr_lhs2.end();
				constr_rhs2.end();
			}
		}
	}
}

void MILP::define_constraint_interfAllToAll() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint interfAllToAll." << std::endl;
	}
	for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) { // for each higher priority task k
		if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
			IloExpr constr_lhs(env); // constraint left hand side expression, initially empty
			IloExpr constr_rhs(env); // constraint right side expression, initially empty
			int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
			for (int j = 0; j < (int)this->seq_i.size(); j++) {
				for (int l = 0; l < tot_l; l++) {
					constr_lhs += interference_real_vars[j][k][l];
				}
			}
			int R_k_star;
			if (job_continue) {
				R_k_star = this->taskset.get_R(k);
			}
			else {
				// R_k_star is the minimum between the WCRT of task k and its relative deadline D_k, due to the fact that in the job-kill semantics, the response time of a higher priority task k is truncated to its relative deadline D_k.
				R_k_star = std::min(this->taskset.get_R(k), this->taskset.get_task(k).get_D());
			}
			int t = ((this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T()) + this->taskset.get_task(this->task_index).get_D();
			constr_rhs += (this->taskset.get_task(k).get_WCET() * ceil((double)(R_k_star + t) / (double)this->taskset.get_task(k).get_T()));

			milp_model.add(constr_lhs <= constr_rhs);
			// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
			constr_lhs.end();
			constr_rhs.end();
		}
	}
}

void MILP::define_constraint_UBinterfWindow() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint UBinterfWindow." << std::endl;
	}
	for (int t = 2; t <= (int)this->seq_i.size(); t++) {
		for (int j = 0; j <= (int)(this->seq_i.size()- t); j++) {
			IloExpr constr_lhs(env);
			IloExpr constr_rhs(env);
				
			for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
				if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
					int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
					for (int l = 0; l < tot_l; l++) {
						for (int q = j; q < (j + t); q++) {
							constr_lhs += interference_real_vars[q][k][l];
						}
					}
					
					constr_rhs += aggregate_interference_ceiling_vars[t][j][k] * this->taskset.get_task(k).get_WCET();
					
					int R_k_star;
					if (job_continue) {
						R_k_star = this->taskset.get_R(k);
					}
					else {
						R_k_star = std::min(this->taskset.get_R(k), this->taskset.get_task(k).get_D());
					}

					IloExpr constr_lhs2(env);
					IloExpr constr_rhs2(env);
					
					// The response term R_{i,j+t-1}^\star is defined as follows:
					IloExpr response_term(env);
                    int last_job_idx = j + t - 1;
                    if (job_continue || this->seq_i[last_job_idx] == 0) {
                        response_term += response_time_vars[last_job_idx];
                    } else {
                        response_term += this->taskset.get_task(this->task_index).get_D();
                    }
					constr_lhs2 += ((((t - 1) * this->taskset.get_task(this->task_index).get_T()) + R_k_star + response_term) / (this->taskset.get_task(k).get_T()));
					milp_model.add(constr_lhs2 <= aggregate_interference_ceiling_vars[t][j][k]); // ( R_ij + (t-1)T_i+Rk / T_k) <= y10_r
					
					constr_rhs2 += ((((t - 1) * this->taskset.get_task(this->task_index).get_T()) + R_k_star + response_term) / (this->taskset.get_task(k).get_T())) + 1 - epsilon;
					milp_model.add(aggregate_interference_ceiling_vars[t][j][k] <= constr_rhs2); // y10_r <= (  R_ij + (t-1)T_i+Rk / T_k) + 1 -epsilon
					// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
					constr_lhs2.end();
					constr_rhs2.end();
					response_term.end();
				}
			}		
					
			milp_model.add(constr_lhs <= constr_rhs);
			// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
			constr_lhs.end();
			constr_rhs.end();
		}
	}
}

void MILP::define_constraint_MissSubSeq() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint MissSubSeq." << std::endl;
	}
	int T_i = this->taskset.get_task(this->task_index).get_T();
    int D_i = this->taskset.get_task(this->task_index).get_D();
		// This constraint is exclusively applicable under job-continue semantics
	if(!job_continue) {
		if (D_i < T_i) {
			return;
		}
	}
    int C_i = this->taskset.get_task(this->task_index).get_WCET();

   int streak = 0; // Counter for consecutive deadline misses

    for (int j = 0; j < (int)this->seq_i.size(); j++) {
        // Track consecutive sequence of misses starting after a hit
        if (this->seq_i[j] == 1) {
            streak++;
        } else {
            streak = 0;
        }

        // The theoretical constraint applies to miss subsequences of length n >= 2
        if (streak > 1) {
            int n = streak;
            
            // 1. Correct construction of the equivalent aggregate task tau_M
            std::vector<mixedtask> tmp_taskset = this->taskset.get_taskset();
            
            // The total volume of execution is correctly multiplied by n
            int wcet = C_i * n;
            int period = T_i * n;
            int deadline = D_i + (n - 1) * T_i;
            
            // Only a single NPS section maintains its status as an "uninterruptible shield".
            // The arrival blocking only occurs once at the start of the continuous busy window.
            int NPS = tmp_taskset[this->task_index].get_NPS(); 
            
            // The rest of the execution time becomes preemptable (RCT).
            // This includes the original n * RCT plus the remaining (n-1) NPS sections 
            // that are downgraded to regular preemptable execution.
            int RCT = wcet - NPS; 
            
            pair<int, int> mk = tmp_taskset[this->task_index].get_mk();
            int prio = tmp_taskset[this->task_index].get_prio();
            
            tmp_taskset[this->task_index] = mixedtask(wcet, period, deadline, RCT, NPS, mk, prio);
            mixedtaskset tmp_T(tmp_taskset);

			// 2. Compute the static WCRT of the equivalent task R_i^n
            int R_equivalent = tmp_T.compute_aRi(this->task_index);
            int rhs_bound = R_equivalent - (n - 1) * T_i;
			if (j - n + 1 < 0) {
				std::cout << "ERRORE CRITICO: j = " << j << ", n = " << n << " -> j - n + 1 = " << (j - n + 1) << std::endl;
			}
			int first_miss_idx = j - n +1 ;//std::max(0, j - n + 1);
            IloExpr sum_y(env);

            // 3. Link existing pre-allocated binary variables: R_{i,q} >= T_i <==> response_exceeds_period_vars[q] == 1
			for (int q = first_miss_idx; q < j; q++) {
                IloExpr cond_expr_1(env);
				IloExpr cond_expr_2(env);
				
				// If y_q = 1 => R_{i,q} >= T_i
				cond_expr_1 += response_time_vars[q] + M * (1 - response_exceeds_period_vars[q]);
				milp_model.add(cond_expr_1 >= T_i);
				cond_expr_1.end();

				// 2. If R_{i,q} >= T_i => y_q = 1 
				// (linearized as: R_{i,q} <= T_i - epsilon + M * y_q)
				cond_expr_2 += response_time_vars[q] - M * response_exceeds_period_vars[q];
				milp_model.add(cond_expr_2 <= T_i - epsilon);
				cond_expr_2.end();

				sum_y += response_exceeds_period_vars[q];
            }

			// 4. Consequent: R_{i,j} <= rhs_bound + BigM * ((n-1) - sum(y_q))
            IloExpr constr_lhs(env);
            IloExpr constr_rhs(env);

            constr_lhs += response_time_vars[j];
            constr_rhs += rhs_bound + M * ((n - 1) - sum_y);

            milp_model.add(constr_lhs <= constr_rhs);

            // Free temporary expressions
			constr_lhs.end();
			constr_rhs.end();
			sum_y.end();
		}

	}
}

void MILP::define_constraint_blockingUB() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint blockingUB." << std::endl;
	}
	if (this->taskset.isPreemptive() == false) {
		for (int j = 0; j < (int)this->seq_i.size(); j++) {
			IloExpr constr_lhs(env);
			IloExpr constr_rhs(env);
			constr_lhs += blocking_time_vars[j];
			for (int y = 0; y < (int)this->seq_i.size(); y++) {
				constr_rhs += selected_blocking_time_vars[j][y] * this->multiset[y];
			}
			milp_model.add(constr_lhs <= constr_rhs);
			// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
			constr_lhs.end();
			constr_rhs.end();
		}
	}
	else {
		for (int j = 0; j < (int)this->seq_i.size(); j++) {
			IloExpr constr_lhs(env);
			IloExpr constr_rhs(env);
			constr_lhs += blocking_time_vars[j];
			constr_rhs += 0;
			milp_model.add(constr_lhs <= constr_rhs);
			// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
			constr_lhs.end();
			constr_rhs.end();
		}
	}
}

void MILP::define_constraint_blockingOnePerJob() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint blockingOnePerJob." << std::endl;
	}
	if (this->taskset.isPreemptive() == false) {
		for (int j = 0; j < (int)this->seq_i.size(); j++) {
			IloExpr constr_lhs(env);
			IloExpr constr_rhs(env);
			for (int y = 0; y < (int)this->seq_i.size(); y++) {
				constr_lhs += selected_blocking_time_vars[j][y];
			}
			constr_rhs += 1;
			milp_model.add(constr_lhs <= constr_rhs);
			// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
			constr_lhs.end();
			constr_rhs.end();
		}
	}
}

void MILP::define_constraint_OneJobPerBlocking() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint OneJobPerBlocking." << std::endl;
	}
	if (this->taskset.isPreemptive() == false) {
		for (int y = 0; y < (int)this->seq_i.size(); y++) {
			IloExpr constr_lhs(env);
			IloExpr constr_rhs(env);
			for (int j = 0; j < (int)this->seq_i.size(); j++) {
				constr_lhs += selected_blocking_time_vars[j][y];
			}
			constr_rhs += 1;
			milp_model.add(constr_lhs <= constr_rhs);
			// Free the temporary expressions to prevent memory leaks in CPLEX Concert Technology
			constr_lhs.end();
			constr_rhs.end();
		}
	}
}

void MILP::define_constraint_blockingPrecedentMiss() {
	if (verbose_setup) {
		std::cout << "1> Setting constraint blockingPrecedentMiss." << std::endl;
	}
	if (this->taskset.isPreemptive() == false) {
		int Ti = this->taskset.get_task(this->task_index).get_T();
		int Di = this->taskset.get_task(this->task_index).get_D();

		for (int j = 0; j < ((int)this->seq_i.size() - 1); j++) {
			if(job_continue){
				IloExpr constr_expr1(env);
				constr_expr1 += blocking_time_vars[j+1] - M * (1.0 - blocking_precedent_miss_indicator_vars[j]);
				milp_model.add(constr_expr1 <= 0);
				constr_expr1.end();

				IloExpr constr_expr2(env);
				constr_expr2 += response_time_vars[j] - (Ti - epsilon + M * blocking_precedent_miss_indicator_vars[j]);
				milp_model.add(constr_expr2 <= 0);
				constr_expr2.end();
			}
			else{
				// Under job-kill semantics, the effective response time is bounded by the task's deadline. Therefore, if the deadline equals the period and the job is a MISS, we can safely force the next job's blocking time to zero.
				if(Di == Ti && this->seq_i[j] == 1){
					milp_model.add(blocking_time_vars[j+1] == 0);
				}
				// If the deadline is less than the period, we cannot guarantee that the next job's blocking time should be zero, as the job may have been killed before reaching the period. In this case, we do not add any constraint.
			}
		}		
	}
}

void MILP::solve_milp() {
	// setup solver
	cplex = IloCplex(env);
	if (!verbose_solver) {
		cplex.setOut(env.getNullStream());
		cplex.setWarning(env.getNullStream());
	}

	// visualize model
	cplex.extract(milp_model);
	if (verbose_solver) {
		std::cout << milp_model << std::endl;
	}

	// apply solver parameters
	if (apply_solver_time_limit) {
		cplex.setParam(IloCplex::Param::Tune::TimeLimit, solver_time_limit);
		cplex.setParam(IloCplex::Param::TimeLimit, solver_time_limit);
	}
	if (apply_solver_thread_num) {
		cplex.setParam(IloCplex::Param::Threads, solver_thread_num);
	}
	if (apply_numerical_precision) {
		cplex.setParam(IloCplex::Param::Emphasis::Numerical, CPX_ON); // numerical precision emphasis; on (CPX_ON) or off (CPX_OFF); default: off.
	}

	// solve
	solve_result = cplex.solve();
	solver_status = cplex.getCplexStatus();

	if (solve_result == IloTrue && solver_status == IloCplex::CplexStatus::Optimal) {
		// Retrieve the optimal objective value from the solver and store it in the result variable
		IloNum solver_obj_value = cplex.getObjValue();
		solved = true;
		result = static_cast<double>(solver_obj_value);
		if (verbose_solver) {
			std::cout << "Optimal MILP solution found" << std::endl;
			std::cout << "Solve result: " << solve_result << std::endl;
			std::cout << "Solver status: " << solver_status << std::endl;
			std::cout << "Optimal solution value: " << solver_obj_value << std::endl;
		}
	}
	else {
		solved = false;
		if (verbose_solver) {
			std::cerr << "Optimal MILP solution not found" << std::endl;
		}		
	}
}

void MILP::print_variables() {
	// Check if the MILP has been solved and an optimal solution is available
	if (!(solve_result == IloTrue && solver_status == IloCplex::CplexStatus::Optimal)) {
		std::cerr << "print_variables(): nessuna soluzione ottimale disponibile, stampa saltata." << std::endl;
		return;
	}

	// Quick check to see if the model has been extracted before calling getValue, to avoid potential exceptions.
	try {
		if (!cplex.isExtracted(milp_model)) {
			std::cerr << "print_variables(): modello non estratto nel solver, stampa saltata." << std::endl;
			return;
		}
	}
	catch (...) {
		// If isExtracted is not supported or gives an error, we proceed anyway but will use try/catch for getValue
	}

	std::cout << "Value of variables:" << std::endl;

	try {
		for (int j = 0; j < (int)this->seq_i.size(); j++) {
			const char* var_name = response_time_vars[j].getName();
			IloNum var_value = cplex.getValue(response_time_vars[j]);
			std::cout << var_name << " = " << var_value << std::endl;
		}

		for (int j = 0; j < (int)this->seq_i.size(); j++) {
			for (int k = 0; k < (int)this->taskset.get_taskset().size(); k++) {
				if ((this->task_index != k) && (this->taskset.get_task(k).get_prio() < this->taskset.get_task(this->task_index).get_prio())) {
					int tot_l = (int)ceil(((long long)(this->seq_i.size() - 1) * this->taskset.get_task(this->task_index).get_T() + this->taskset.get_task(this->task_index).get_D() + this->taskset.get_R(k)) / (double)(this->taskset.get_task(k).get_T()));
					for (int l = 0; l < tot_l; l++) {
						const char* var_name = interference_real_vars[j][k][l].getName();
						IloNum var_value = cplex.getValue(interference_real_vars[j][k][l]);
						std::cout << var_name << " = " << var_value << std::endl;
					}
				}
			}
		}

		for (int j = 0; j < (int)this->seq_i.size(); j++) {
			const char* var_name = spillover_vars[j].getName();
			IloNum var_value = cplex.getValue(spillover_vars[j]);
			std::cout << var_name << " = " << var_value << std::endl;
		}
	}
	catch (IloException& ex) {
		std::cerr << "print_variables(): eccezione CPLEX durante lettura valori: " << ex << std::endl;
	}
	catch (...) {
		std::cerr << "print_variables(): eccezione sconosciuta durante lettura valori." << std::endl;
	}
}

MILP::MILP() {
}

MILP::MILP(mixedtaskset taskset, vector<int> seq_i, int task_index, vector<int> multiset, bool job_continue) {
	this->taskset = taskset;
	this->seq_i = seq_i;
	this->task_index = task_index;
	this->multiset = multiset;
	this->job_continue = job_continue;
}

void MILP::initialize_milp(mixedtaskset taskset, vector<int> seq_i, int task_index, vector<int> multiset, bool job_continue) {
	this->taskset = taskset;
	this->seq_i = seq_i;
	this->task_index = task_index;
	this->multiset = multiset;
	this->job_continue = job_continue;
}

std::pair<bool, int> MILP::call_solver() {
	// setup milp environment
	if (verbose_setup) {
		std::cout << "1> Setting up milp" << std::endl;
	}
	try {
		// close previous resources if they were not closed yet
		if (!resources_closed) {
			try { cplex.end(); } catch (...) {}
			try { milp_model.end(); } catch (...) {}
			try { env.end(); } catch (...) {}
			resources_closed = true;
		}

		// create new environment and model
		resources_closed = false;
		env = IloEnv();
		milp_model = IloModel(env);

		// setup status variables
		milp_error = false;
		solved = false;
		result = -1.0; // initialize result to an invalid value

		// set environment output
		if (!verbose_setup) {
			env.setOut(env.getNullStream());
		}
		env.setWarning(env.getNullStream());
		//env.setError(env.getNullStream());

		// prepare milp formulation
		define_variables();
		define_objective_function();

		define_constraint_perjobDmiss();
		define_constraint_interferenceStaticWCRT();
		define_constraint_perjobSpillover();
		define_constraint_perjobWCRTdef();
		define_constraint_perHPjobInterfUB();
		define_constraint_interfUBperHPjobsInWindow();
		define_constraint_interfAllToAll();
		define_constraint_UBinterfWindow();
		define_constraint_MissSubSeq();
		//blocking constraints
		define_constraint_blockingUB();
		define_constraint_blockingOnePerJob();
		define_constraint_OneJobPerBlocking();
		define_constraint_blockingPrecedentMiss();
		
		// solve model
		if (verbose_solver) {
			std::cout << "1> Solving MILP..." << std::endl;
		}

		solve_milp();

		// print variable values
		if (verbose_solver) {
			if (solve_result == IloTrue && solver_status == IloCplex::CplexStatus::Optimal) {
				print_variables();
			}
		}
	}
	catch (IloException& ex) {
		std::cerr << "MILP error: " << ex << std::endl;
		std::cout << "MILP error: " << ex << std::endl;
		if (verbose_solver) {
			std::cout << "MILP error: " << ex << std::endl;
		}
		milp_error = true;
	}
	catch (...) {
		std::cerr << "Generic MILP error" << std::endl;
		std::cout << "Generic MILP error" << std::endl;
		if (verbose_solver) {
			std::cout << "Generic MILP error" << std::endl;
		}
		milp_error = true;
	}

	bool status = true;
	int ret_value = -1;
	if (!solved || milp_error || (result < 0.0)) {
		status = false;
		ret_value = -1;
	} else {
		ret_value = static_cast<int>(ceil(result));
	}

	return std::make_pair(status, ret_value);
}

MILP::~MILP() {
	if (!resources_closed) {
		// Close in reverse order of creation. Use try/catch to be robust.
		try { cplex.end(); } catch (...) {}
		try { milp_model.end(); } catch (...) {}
		try { env.end(); } catch (...) {}
		resources_closed = true;
	}
}
