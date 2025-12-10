/**
 * @file base_model.cpp
 * @brief Implementation of BaseModel class
 */

#include "base_model.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <random>
#include <chrono>
#include <limits>
#include "../cpxmacro.h"

BaseModel::BaseModel(CPXENVptr env_, CPXLPptr lp_, const std::string &directory)
    : env(env_),
      lp(lp_),
      directory_(directory),
      status(0),
      start_node(0),
      current_N(0)
{}

// setters
void BaseModel::setStartNode(int s)
{
    start_node = s;
}

void BaseModel::setRunCost(const std::vector<std::vector<double>> &C)
{
    run_cost = C;
    current_N = static_cast<int>(C.size());
}

// setup LP model
int BaseModel::setupLP(int N, int run_id)
{
    if (!env || !lp)
        throw std::runtime_error("CPLEX not initialized");

    current_N = N;
    map_x.assign(N, std::vector<int>(N, -1));
    map_y.assign(N, std::vector<int>(N, -1));
    int current_var_position = 0;
    int K = N - 1;

    // 1) Directed x_ij variables (only for j != start_node, as in reference j != 0)
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            if (j == start_node) continue;

            char xtype = 'C';
            double lb = 0.0;
            double ub = CPX_INFBOUND;
            double objx = 0.0;
            const int NAME_SIZE = 64;
            char name_buf[NAME_SIZE];
            std::snprintf(name_buf, NAME_SIZE, "x_%d_%d", i, j);
            char *xname = name_buf;

            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &objx, &lb, &ub, &xtype, &xname);
            map_x[i][j] = current_var_position++;
        }
    }


    // 2) Directed y_ij variables (path/arcs), for all i != j
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;

            char ytype = 'B';
            double lb = 0.0;
            double ub = 1.0;
            double objy = run_cost[i][j]; // note: run_cost must be N x N
            const int NAME_SIZE = 64;
            char name_buf[NAME_SIZE];

            std::snprintf(name_buf, NAME_SIZE, "y_%d_%d", i, j);
            char *yname = name_buf;

            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &objy, &lb, &ub, &ytype, &yname);
            map_y[i][j] = current_var_position++;
        }
    }

    // 3) Flow conservation constraints (for non-depot nodes k = 1..N-1)
    for (int k = 0; k < N; ++k)
    {
        if (k == start_node) continue;

        std::vector<int> idx;
        std::vector<double> coef;

        // incoming flows (i = 0..N-1, i != k)
        for (int i = 0; i < N; ++i)
        {
            if (i == k) continue;
            if (map_x[i][k] >= 0) {
                idx.push_back(map_x[i][k]);
                coef.push_back(1.0);
            }
        }

        // outgoing flows (j = 1..N-1, skip depot/start_node and k)
        for (int j = 0; j < N; ++j)
        {
            if (j == k) continue;
            if (j == start_node) continue; // ensure outgoing excludes depot
            if (map_x[k][j] >= 0) {
                idx.push_back(map_x[k][j]);
                coef.push_back(-1.0);
            }
        }

        if (!idx.empty())
        {
            char sense = 'E';
            double rhs = 1.0;
            int matbeg = 0;
            CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1,
                             static_cast<int>(idx.size()), &rhs, &sense, &matbeg,
                             idx.data(), coef.data(), nullptr, nullptr);
        }
    }

    // 4a) One outgoing arc per node: sum_j y[i][j] = 1
    for (int i = 0; i < N; ++i)
    {
        std::vector<int> idx;
        std::vector<double> coef;
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            if (map_y[i][j] >= 0) {
                idx.push_back(map_y[i][j]);
                coef.push_back(1.0);
            }
        }
        if (!idx.empty()) {
            char sense = 'E';
            double rhs = 1.0;
            int matbeg = 0;
            CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1,
                             static_cast<int>(idx.size()), &rhs, &sense, &matbeg,
                             idx.data(), coef.data(), nullptr, nullptr);
        }
    }

    // 4b) One incoming arc per node: sum_i y[i][j] = 1
    for (int j = 0; j < N; ++j)
    {
        std::vector<int> idx;
        std::vector<double> coef;
        for (int i = 0; i < N; ++i)
        {
            if (i == j) continue;
            if (map_y[i][j] >= 0) {
                idx.push_back(map_y[i][j]);
                coef.push_back(1.0);
            }
        }
        if (!idx.empty()) {
            char sense = 'E';
            double rhs = 1.0;
            int matbeg = 0;
            CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1,
                             static_cast<int>(idx.size()), &rhs, &sense, &matbeg,
                             idx.data(), coef.data(), nullptr, nullptr);
        }
    }


    // 5) Linking constraints x_ij ≤ (N-1) * y_ij for j != start_node
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            if (j == start_node) continue;
            int xv = map_x[i][j];
            int yv = map_y[i][j];

            if (xv >= 0 && yv >= 0)
            {
                std::vector<int> idx = {xv, yv};
                double bigN = static_cast<double>(N);
                std::vector<double> coef = {1.0, -(bigN - 1.0)}; // x - (N-1) y <= 0
                char sense = 'L';
                double rhs = 0.0;
                int matbeg = 0;
                CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1,
                                 static_cast<int>(idx.size()), &rhs, &sense, &matbeg,
                                 idx.data(), coef.data(), nullptr, nullptr);
            }
        }
    }

    return current_var_position;
}

// solve the LP/MIP for one run, write lp and solution file, and return run results
BaseModel::RunResult BaseModel::solveRun(int run_id, const std::string &solution_file, int &start_node_out)
{
    if (!env)
        throw std::runtime_error("CPLEX not initialized");

    // create a fresh LP for this run (do NOT free the member 'lp' that main owns).
    CPXLPptr lp_local = nullptr;
    int local_status = 0;
    lp_local = CPXcreateprob(env, &local_status, "BaseModel_RunLocal");
    if (!lp_local || local_status != 0)
        throw std::runtime_error("Failed to create run-local LP");

    // save original member lp, swap in the local LP so setupLP uses it
    CPXLPptr lp_saved = lp;
    lp = lp_local;

    // ensure we restore lp and free lp_local even if an exception is thrown
    try
    {
        result.N = current_N;
        result.run_index = run_id;

        // RNG for start node
        std::mt19937 rng(static_cast<unsigned int>(std::time(nullptr)) + run_id);
        std::uniform_int_distribution<int> dist(0, std::max(0, current_N - 1));
        start_node = dist(rng);
        start_node_out = start_node;

        result.start_node = start_node;

        // build the model on the fresh lp_local (via member lp)
        setupLP(current_N, run_id);

        // write LP file for debugging
        std::string lpfile = solution_file;
        auto pos = lpfile.find(".sol");
        if (pos != std::string::npos)
            lpfile.replace(pos, 4, ".lp");
        CHECKED_CPX_CALL(CPXwriteprob, env, lp, lpfile.c_str(), nullptr);

        // time using chrono
        auto t0 = std::chrono::steady_clock::now();
        CHECKED_CPX_CALL(CPXmipopt, env, lp);
        auto t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t1 - t0;

        // check solution status
        int solstat = CPXgetstat(env, lp);

        result.optimal = (solstat == CPXMIP_OPTIMAL ||
                      solstat == CPXMIP_OPTIMAL_TOL);

        double objval = 0.0;
        int ret = CPXgetobjval(env, lp, &objval);
        if (ret == 0) {
            CHECKED_CPX_CALL(CPXsolwrite, env, lp, solution_file.c_str());
        } else {
            objval = std::numeric_limits<double>::quiet_NaN();
            std::cerr << "Warning: Could not retrieve objective value, return code: " << ret << std::endl;
        }
        result.objective_value = objval;

        double mip_gap = 0.0;
        CPXgetmiprelgap(env, lp_local, &mip_gap);
        result.gap = mip_gap * 100.0;

        // restore member lp and free the local lp
        lp = lp_saved;
        CPXfreeprob(env, &lp_local); // safe: lp_local == lp_local (not member)
        result.solve_time = elapsed.count();
        return result;
    }
    catch (...)
    {
        // always restore member lp and free the local lp on exception
        lp = lp_saved;
        if (lp_local) CPXfreeprob(env, &lp_local);
        throw; // rethrow
    }
}
