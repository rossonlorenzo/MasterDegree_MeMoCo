/**
 * @file MTZ_model.cpp
 * @brief Implementation of MTZModel (one concrete Model)
 */

#include "MTZ_model.h"

#include <vector>
#include <string>
#include <cstdio>
#include <limits>
#include <iostream>
#include <stdexcept>
#include "../cpxmacro.h" // CHECKED_CPX_CALL macro

// -----------------------------
// buildVariables
// - creates y_ij (binary) variables with objective = cost[i][j]
// - creates u_i (continuous ordering) variables in [1, N], with u_start fixed to 1
// - records variable indices into protected map_y / map_u
// -----------------------------
int MTZModel::buildVariables()
{
    if (!env || !lp)
        throw std::runtime_error("CPLEX not initialized in MTZModel::buildVariables");

    int N = current_config.N;
    if (N <= 0) return 0;

    // init maps
    map_y.assign(N, std::vector<int>(N, -1));
    map_u.assign(N, -1);

    int current_var_position = 0;

    // 1) y_ij: binary variables for all i != j with objective = cost[i][j]
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;

            double objy = 0.0;
            if ((int)current_config.cost.size() == N && (int)current_config.cost[i].size() == N) {
                objy = current_config.cost[i][j];
            }
            double lb = 0.0;
            double ub = 1.0;
            char ytype = 'B'; // binary
            char name_buf[64];
            std::snprintf(name_buf, sizeof(name_buf), "y_%d_%d", i, j);
            char *colname = name_buf;
            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &objy, &lb, &ub, &ytype, &colname);

            map_y[i][j] = current_var_position++;
        }
    }

    // 2) u_i: continuous ordering variables in [1, N]
    for (int i = 0; i < N; ++i)
    {
        double lb = 1.0;
        double ub = static_cast<double>(N);

        // Fix the start node's u to 1 by setting lb = ub = 1
        if (i == start_node) {
            lb = 1.0;
            ub = 1.0;
        }

        char xtype = 'C'; // continuous
        double obju = 0.0; // no contribution to objective
        char name_buf[64];
        std::snprintf(name_buf, sizeof(name_buf), "u_%d", i);
        char *colname = name_buf;

        CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &obju, &lb, &ub, &xtype, &colname);

        map_u[i] = current_var_position++;
    }

    return 0;
}

// -----------------------------
// buildConstraints
// - one outgoing y per node: sum_j y_i,j = 1
// - one incoming y per node: sum_i y_i,j = 1
// - Lifted MTZ constraints:
//   u_i - u_j + (N-1) y_ij + (N-3) y_ji <= N - 2  for i!=j, i!=start, j!=start
// -----------------------------
int MTZModel::buildConstraints()
{
    if (!env || !lp)
        throw std::runtime_error("CPLEX not initialized in MTZModel::buildConstraints");

    int N = current_config.N;
    if (N <= 0) return 0;

    // 1) One outgoing y per node: sum_j y[i][j] = 1
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

    // 2) One incoming y per node: sum_i y[i][j] = 1
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

    // 3) Lifted MTZ constraints:
    // u_i - u_j + (N-1)*y_ij + (N-3)*y_ji <= N - 2
    // for i != j, i != start_node, j != start_node
    for (int i = 0; i < N; ++i)
    {
        if (i == start_node) continue;
        for (int j = 0; j < N; ++j)
        {
            if (j == start_node) continue;
            if (i == j) continue;

            int ui = map_u[i];
            int uj = map_u[j];
            int yij = map_y[i][j];
            int yji = map_y[j][i];

            // ensure indices exist
            if (ui < 0 || uj < 0) continue;
            if (yij < 0 || yji < 0) continue;

            std::vector<int> idx;
            std::vector<double> coef;

            idx.push_back(ui);    coef.push_back(1.0);               // u_i
            idx.push_back(uj);    coef.push_back(-1.0);              // -u_j
            idx.push_back(yij);   coef.push_back(static_cast<double>(N - 1)); // (N-1) y_ij
            idx.push_back(yji);   coef.push_back(static_cast<double>(N - 3)); // (N-3) y_ji

            char sense = 'L';
            double rhs = static_cast<double>(N - 2);
            int matbeg = 0;

            CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1,
                             static_cast<int>(idx.size()), &rhs, &sense, &matbeg,
                             idx.data(), coef.data(), nullptr, nullptr);
        }
    }

    return 0;
}

// -----------------------------
// buildObjective
// - objective coefficients for y variables already set when variables were created
// - ensure objective sense is minimization
// -----------------------------
int MTZModel::buildObjective()
{
    if (!env || !lp)
        throw std::runtime_error("CPLEX not initialized in MTZModel::buildObjective");

    int ret = CPXchgobjsen(env, lp, CPX_MIN);
    if (ret != 0) {
        throw std::runtime_error("Failed to set objective sense in MTZModel::buildObjective");
    }
    return 0;
}
