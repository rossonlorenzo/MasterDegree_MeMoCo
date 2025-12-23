#include "model.h"
#include "../cpxmacro.h"
#include <chrono>
#include <fstream>
#include <iostream>

/* ============================================================
 *  Constructor
 * ============================================================ */
Model::Model(Env env_, Prob lp_)
    : env(env_),
      lp(lp_),
      status(0),
      start_node(0),
      current_config(std::nullopt)
{
    current_run_result = {};
}

Model::~Model()
{
        if (lp)
            CPXfreeprob(env, &lp);
        if (env)
            CPXcloseCPLEX(&env);
}

/* ============================================================
 *  Setters
 * ============================================================ */

void Model::setGraphConfig(const TSP &c)
{
    current_config = c;
}

void Model::setStartNode(int s)
{
    start_node = s;
}

/* ============================================================
 *  setupLP
 * ============================================================ */
int Model::setupLP(TSP config, int run_id, int start_node, const std::string &lp_file)
{
    CHECKED_CPX_CALL(CPXfreeprob, env, &lp);   // free previous LP
    lp = CPXcreateprob(env, &status, "model");

    // Store config
    this->setGraphConfig(config);
    this->setStartNode(start_node);

    // Derived classes build the model
    status = buildVariables();
    if (status)
        throw std::runtime_error("buildVariables() failed");

    status = buildConstraints();
    if (status)
        throw std::runtime_error("buildConstraints() failed");

    status = buildObjective();
    if (status)
        throw std::runtime_error("buildObjective() failed");

    // Write LP file
    CHECKED_CPX_CALL(CPXwriteprob, env, lp, lp_file.c_str(), nullptr);

    return 0;
}

/* ============================================================
 *  solveRun
 * ============================================================ */
Model::RunResult Model::solveRun(int run_id,
                          const std::string &solution_file)
{
    if (!env || !lp)
    throw std::runtime_error("CPLEX not initialized");

    current_run_result.run_index = run_id;
    if (!current_config.has_value())
        throw std::runtime_error("No graph configuration set");
    current_run_result.N = current_config->getN();
    current_run_result.start_node = start_node;

    // Timing (solver)
    auto t0 = std::chrono::high_resolution_clock::now();

    CHECKED_CPX_CALL(CPXmipopt, env, lp);

    auto t1 = std::chrono::high_resolution_clock::now();
    current_run_result.solve_time =
        std::chrono::duration<double>(t1 - t0).count();

    // Status (cannot use CHECKED_CPX_CALL because CPXgetstat has no env in signature)
    int solstat = CPXgetstat(env, lp);
    current_run_result.optimal =
        (solstat == CPXMIP_OPTIMAL || solstat == CPX_STAT_OPTIMAL);

    // MIP gap
    CHECKED_CPX_CALL(CPXgetmiprelgap, env, lp, &current_run_result.gap);
    current_run_result.gap *= 100.0;

    // Objective value
    CHECKED_CPX_CALL(CPXgetobjval, env, lp, &current_run_result.objective_value);

    // Solution export
    if (!solution_file.empty())
    {
        CHECKED_CPX_CALL(CPXsolwrite, env, lp, solution_file.c_str());
    }

    return current_run_result;
}
