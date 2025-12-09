/**
 * @file main_sym.cpp
 * @brief Symmetry-exploiting flow-based TSP for drilling problem (CPLEX)
 *
 * - Keeps directed flow variables x[i][j] for subtour elimination
 * - Uses a single undirected binary variable y[min(i,j)][max(i,j)] for edge selection
 * - Degree constraints: sum_j y_ij = 2 for every node i
 * - Linking: x_ij <= (N-1) * y_{min(i,j),max(i,j)}
 *
 */

#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <regex>
#include <algorithm>

#include "cpxmacro.h"

namespace fs = std::filesystem;

// global config directory
std::string directory = "samples";

// error status and message buffer
int status;
char errmsg[BUF_SIZE];

// name buffer
const int NAME_SIZE = 512;
char name_buf[NAME_SIZE];

// CSV results storage
struct RunResult
{
    int config_index;
    int N;
    int run_index;
    int start_node;
    double time_sec;
};

std::vector<RunResult> all_results;

// data
struct GraphConfig
{
    int N;
    std::vector<std::vector<double>> cost;
};

// cost matrix (current run)
std::vector<std::vector<double>> run_cost; // C[i][j] = cost from node i to node j
int start_node = 0;                        // will be set per run

// decision variables maps (indices of variables in CPLEX internal array)
// map_x : directed arcs i->j  (size NxN, -1 if not present)
// map_y : undirected edges (only i<j used) stored in map_y[i][j]
std::vector<std::vector<int>> map_x;
std::vector<std::vector<int>> map_y;

// helper to get y variable index for undirected edge (i,j)
inline int y_index(int i, int j)
{
    if (i == j) return -1;
    if (i < j) return map_y[i][j];
    return map_y[j][i];
}

// setup LP model exploiting symmetry
int setupLP(CPXENVptr env, CPXLPptr lp, int N, int run_id)
{
    int current_var_position = 0;

    // initialize maps
    map_x.assign(N, std::vector<int>(N, -1));
    map_y.assign(N, std::vector<int>(N, -1)); // only i<j entries used

    // Add x variables (continuous flows) for all directed arcs i->j where i!=j and cost>0
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            if (run_cost[i][j] == 0.0) // treat 0.0 as no-arc (same convention as original)
                continue;

            char xtype = 'C';
            double lb = 0.0;
            double ub = CPX_INFBOUND;
            std::snprintf(name_buf, NAME_SIZE, "x_%d_%d", i, j);
            char *xname = name_buf;
            double objx = 0.0; // flow variables don't appear in objective
            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &objx, &lb, &ub, &xtype, &xname);

            map_x[i][j] = current_var_position++;
        }
    }

    // Add undirected y variables only for i < j
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            if (run_cost[i][j] == 0.0)
                continue;

            char ytype = 'B';
            double lb = 0.0;
            double ub = 1.0;
            std::snprintf(name_buf, NAME_SIZE, "y_%d_%d", i, j);
            char *yname = name_buf;
            double objy = run_cost[i][j]; // symmetric cost
            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &objy, &lb, &ub, &ytype, &yname);

            map_y[i][j] = current_var_position++;
        }
    }

    int K = N - 1; // multiplier for linking

    // 1) Flow conservation (for all k != start_node)
    //    sum_i x_ik - sum_j (j != start_node) x_kj = 1
    for (int k = 0; k < N; ++k)
    {
        if (k == start_node) continue;

        std::vector<int> idx;
        std::vector<double> coef;

        // incoming to k: sum_i x_i_k
        for (int i = 0; i < N; ++i)
        {
            if (i == k) continue;
            int v = map_x[i][k];
            if (v >= 0)
            {
                idx.push_back(v);
                coef.push_back(1.0);
            }
        }

        // outgoing from k: subtract sum_j x_k_j but exclude arcs to start_node? original excluded j==start_node
        for (int j = 0; j < N; ++j)
        {
            if (j == k) continue;
            if (j == start_node) continue; // keep same behavior as original setup
            int v = map_x[k][j];
            if (v >= 0)
            {
                idx.push_back(v);
                coef.push_back(-1.0);
            }
        }

        if (idx.empty()) continue;

        char sense = 'E';
        double rhs = 1.0;
        int matbeg = 0;
        CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, static_cast<int>(idx.size()), &rhs, &sense,
                         &matbeg, idx.data(), coef.data(), NULL, NULL);
    }

    // 2) Degree constraints for undirected y: sum_j y_ij = 2  for every node i
    for (int i = 0; i < N; ++i)
    {
        std::vector<int> idx;
        std::vector<double> coef;

        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            int yvar = y_index(i, j);
            if (yvar >= 0)
            {
                idx.push_back(yvar);
                coef.push_back(1.0);
            }
        }

        if (idx.empty()) continue;

        char sense = 'E';
        double rhs = 2.0;
        int matbeg = 0;
        CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, static_cast<int>(idx.size()), &rhs, &sense,
                         &matbeg, idx.data(), coef.data(), NULL, NULL);
    }

    // 3) Linking constraints x_ij <= (N-1) * y_{min(i,j),max(i,j)} for all directed arcs
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            int xv = map_x[i][j];
            if (xv < 0) continue;

            int yv = y_index(i, j);
            if (yv < 0) continue; // if undirected edge not present, skip

            int idxArr[2] = { xv, yv };
            double coefArr[2] = { 1.0, -(double)K };
            char sense = 'L';
            double rhs = 0.0;
            int matbeg = 0;
            CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 2, &rhs, &sense, &matbeg,
                             idxArr, coefArr, NULL, NULL);
        }
    }

    return current_var_position;
}

int main(int argc, char const *argv[])
{
    try
    {
        std::cout << "The program will manage all the different configurations from the samples directory\n";

        // ---------- load configs ----------
        std::vector<GraphConfig> configs;

        if (!fs::exists(directory))
        {
            std::cerr << "Directory not found: " << directory << std::endl;
            return 0;
        }

        // collect files first
        std::vector<fs::path> files;
        for (const auto &entry : fs::directory_iterator(directory))
        {
            if (entry.is_regular_file())
                files.push_back(entry.path());
        }

        // sort
        std::sort(files.begin(), files.end(),
                  [](const fs::path &a, const fs::path &b)
                  {
                      std::regex re("(\\d+)");
                      std::smatch ma, mb;
                      std::string sa = a.filename().string();
                      std::string sb = b.filename().string();

                      int na = 0, nb = 0;

                      if (std::regex_search(sa, ma, re))
                          na = std::stoi(ma[1]);

                      if (std::regex_search(sb, mb, re))
                          nb = std::stoi(mb[1]);

                      return na < nb;
                  });

        // now load
        for (const auto &path : files)
        {
            std::ifstream file(path);
            if (!file)
            {
                std::cerr << "Failed to open: " << path << std::endl;
                continue;
            }

            GraphConfig config;
            file >> config.N;
            config.cost.assign(config.N, std::vector<double>(config.N, 0.0));

            for (int i = 0; i < config.N; ++i)
                for (int j = 0; j < config.N; ++j)
                    file >> config.cost[i][j];

            configs.push_back(std::move(config));
            std::cout << "Loaded: " << path.filename()
                      << " (N=" << configs.back().N << ")\n";
        }

        // ensure solutions root exists
        fs::create_directories("solutions");

        // ---------- process configs ----------
        for (size_t cfg = 0; cfg < configs.size(); ++cfg)
        {
            std::string configDir = "solutions/config_" + std::to_string(cfg + 1);
            fs::create_directories(configDir);

            int N_run = configs[cfg].N;
            // A_run is not directly needed, but keep for informational output
            int A_run = 0;
            for (int i = 0; i < N_run; ++i)
                for (int j = i + 1; j < N_run; ++j)
                    if (configs[cfg].cost[i][j] != 0.0) ++A_run;

            run_cost = configs[cfg].cost;

            std::cout << "\n===== CONFIG " << cfg + 1 << " | N = " << N_run << " | undirected edges = " << A_run << " =====\n";

            for (int run = 1; run <= 10; ++run)
            {
                // set global start_node (do NOT redeclare local variable)
                std::srand(static_cast<unsigned int>(std::time(nullptr)) + run);
                start_node = std::rand() % N_run;

                std::cout << "Executing run " << run << " with N=" << N_run << " undirected A=" << A_run
                          << " rand start_node = " << start_node << std::endl;

                ///// init CPLEX env and prob
                DECL_ENV(env);
                DECL_PROB(env, lp);

                ///// setup LP
                int numVars = setupLP(env, lp, N_run, run);

                ///// optimize
                double cpx_time_before = 0.0, cpx_time_after = 0.0;
                CHECKED_CPX_CALL(CPXgetdettime, env, &cpx_time_before);
                CHECKED_CPX_CALL(CPXmipopt, env, lp);
                CHECKED_CPX_CALL(CPXgetdettime, env, &cpx_time_after);

                double objval;
                CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);
                double best_bound;
                CPXgetbestobjval(env, lp, &best_bound);
                double gap = ((objval - best_bound) / objval) * 100.0;

                double cpx_elapsed = (cpx_time_after - cpx_time_before) / 100.0; // dettime is in hundredths of sec
                std::cout << "Run " << run << " - CPLEX CPU time (sec): " << cpx_elapsed << std::endl;

                ///// objective value
                double objval;
                int rc = CPXgetobjval(env, lp, &objval);
                if (rc == 0)
                    std::cout << "Run " << run << " - Objval: " << objval << std::endl;
                else
                    std::cout << "Run " << run << " - Impossible operation (rc=" << rc << ")\n";

                // save run result
                all_results.push_back({static_cast<int>(cfg + 1), N_run, run, start_node, cpx_elapsed});

                // Write solution file
                char solname[512];
                std::snprintf(solname, sizeof(solname), "%s/main_run_%d.sol", configDir.c_str(), run);
                CHECKED_CPX_CALL(CPXsolwrite, env, lp, solname);

                ///// free cplex mem
                CPXfreeprob(env, &lp);
                CPXcloseCPLEX(&env);

                std::cout << "Run " << run << " completed. SOL file: " << solname << std::endl;
            }

            // Write all results to CSV
            std::string results_file = "solutions/results.csv";
            std::ofstream csv(results_file);

            if (!csv)
            {
                std::cerr << "Cannot open " << results_file << " for writing results!" << std::endl;
            }
            else
            {
                // header
                csv << "Config,N,Run,StartNode,TimeSec\n";

                for (const auto &r : all_results)
                    csv << r.config_index << "," << r.N << "," << r.run_index
                        << "," << r.start_node << "," << r.time_sec << "\n";

                // compute and append average per config
                csv << "\nConfig,AverageTimeSec\n";

                for (size_t c = 0; c < configs.size(); ++c)
                {
                    double sum = 0.0;
                    int count = 0;
                    for (const auto &r : all_results)
                    {
                        if (r.config_index == static_cast<int>(c + 1))
                        {
                            sum += r.time_sec;
                            count++;
                        }
                    }
                    double avg = (count > 0) ? sum / count : 0.0;
                    csv << c + 1 << "," << avg << "\n";
                }

                csv.close();
                std::cout << "Results CSV written to: " << results_file << std::endl;
            }

            char towait; std::cout << "\nAll runs completed for config " << (cfg + 1) << ". Press a key to continue...\n"; std::cin >> towait;
        }
    }
    catch (std::exception &e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
    }

    return 0;
}
