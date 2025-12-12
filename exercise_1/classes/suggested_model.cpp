/**
 * @file suggested_model.cpp
 * @brief Implementation of SuggestedModel (one concrete Model)
 */

#include "suggested_model.h"

#include <vector>
#include <string>
#include <cstdio>
#include <limits>
#include <random>
#include <chrono>
#include <ctime>
#include <iostream>
#include "../cpxmacro.h" // CHECKED_CPX_CALL macro

// -----------------------------
// buildVariables
// - creates x_ij (continuous) and y_ij (binary) variables
// - records variable indices into protected map_x / map_y
// - sets objective coeffs for y variables to run_cost[i][j]
// -----------------------------
int SuggestedModel::buildVariables()
{
    int N = current_config.N;
    map_x.assign(N, std::vector<int>(N, -1));
    map_y.assign(N, std::vector<int>(N, -1));
    int current_var_position = 0;

    // x_ij: continuous variables, only for j != start_node and i != j
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            if (j == start_node) continue; // skip vars that target start_node

            // prepare CPXnewcols arguments (single-col arrays)
            double objx = 0.0;
            double lb = 0.0;
            double ub = CPX_INFBOUND;
            char xtype = 'C'; // continuous
            char name_buf[64];
            std::snprintf(name_buf, sizeof(name_buf), "x_%d_%d", i, j);
            char *colname = name_buf;
            // CPXnewcols expects arrays; pass pointers to the single elements
            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &objx, &lb, &ub, &xtype, &colname);

            // store var index
            map_x[i][j] = current_var_position++;
        }
    }

    // y_ij: binary variables for all i != j with objective = run_cost[i][j]
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
    
    return 0;
}

// -----------------------------
// buildConstraints
// - flow conservation (for every non-start node): sum_i x_i,k - sum_j x_k,j = 1
// - one outgoing y per node: sum_j y_i,j = 1
// - one incoming y per node: sum_i y_i,j = 1
// - linking x_ij <= (N-1) * y_ij
// -----------------------------
int SuggestedModel::buildConstraints()
{
    if (!env || !lp)
        throw std::runtime_error("CPLEX not initialized in SuggestedModel::buildConstraints");

    int N = current_config.N;

    // 1) Flow conservation for non-start nodes (k != start_node)
    for (int k = 0; k < N; ++k)
    {
        if (k == start_node) continue;

        std::vector<int> idx;
        std::vector<double> coef;

        // incoming x_i,k (i != k)
        for (int i = 0; i < N; ++i)
        {
            if (i == k) continue;
            if (map_x[i][k] >= 0) {
                idx.push_back(map_x[i][k]);
                coef.push_back(1.0);
            }
        }

        // outgoing x_k,j (j != k, j != start_node)
        for (int j = 0; j < N; ++j)
        {
            if (j == k) continue;
            if (j == start_node) continue;
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

    // 2) One outgoing y per node: sum_j y[i][j] = 1
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

    // 3) One incoming y per node: sum_i y[i][j] = 1
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

    // 4) Linking constraints: x_ij <= (N-1) * y_ij for j != start_node
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
                std::vector<int> idx = { xv, yv };
                double bigN = static_cast<double>(N);
                std::vector<double> coef = { 1.0, -(bigN - 1.0) }; // x - (N-1) y <= 0
                char sense = 'L';
                double rhs = 0.0;
                int matbeg = 0;
                CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1,
                                 static_cast<int>(idx.size()), &rhs, &sense, &matbeg,
                                 idx.data(), coef.data(), nullptr, nullptr);
            }
        }
    }

    return 0;
}

// -----------------------------
// buildObjective
// - In this model we already set objective coefficients for y variables during creation.
// - Here we could adjust objective sense, or add additional objective parts if needed.
// -----------------------------
int SuggestedModel::buildObjective()
{
    // ensure minimization (common default) -- set objective sense explicitly
    int ret = CPXchgobjsen(env, lp, CPX_MIN);
    if (ret != 0) {
        throw std::runtime_error("Failed to set objective sense in SuggestedModel::buildObjective");
    }

    // If you need to add more to the objective (e.g., linear terms on x), do it here by
    // calling CPXchgobj or CPXchgobjlist. For now, y-objectives are already set during newcols.
    return 0;
}


