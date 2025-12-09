/**
 * @file main.cpp
 * @brief
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
char name[NAME_SIZE];

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

// parameters

// cost matrix (current run)
std::vector<std::vector<double>> run_cost; // C[i][j] = cost from node i to node j
int start_node = 0;                        // will be set per run

// decision variables maps (indices of variables in CPLEX internal array)
std::vector<std::vector<int>> map_x;
std::vector<std::vector<int>> map_y;

// Build initial LP with binary y_ij variables and degree-2 constraints
int setupLP_SEC(CPXENVptr env, CPXLPptr lp, int N)
{
    int current_var_position = 0;
    map_y.assign(N, std::vector<int>(N, -1));

    // Add binary edge variables y_ij (i < j)
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            char ytype = 'B';
            double lb = 0.0;
            double ub = 1.0;
            char name[NAME_SIZE];
            std::snprintf(name, NAME_SIZE, "y_%d_%d", i, j);
            char *yname = name;
            double objy = run_cost[i][j];
            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &objy, &lb, &ub, &ytype, &yname);
            map_y[i][j] = current_var_position++;
        }
    }

    // Degree-2 constraints: sum_j y_ij = 2 for each node
    for (int i = 0; i < N; ++i)
    {
        std::vector<int> idx;
        std::vector<double> coef;
        for (int j = 0; j < N; ++j)
        {
            if (i == j)
                continue;
            int var = (i < j) ? map_y[i][j] : map_y[j][i];
            if (var >= 0)
            {
                idx.push_back(var);
                coef.push_back(1.0);
            }
        }
        if (!idx.empty())
        {
            char sense = 'E';
            double rhs = 2.0;
            int matbeg = 0;
            CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, static_cast<int>(idx.size()), &rhs, &sense,
                             &matbeg, idx.data(), coef.data(), NULL, NULL);
        }
    }

    return current_var_position;
}

//-------------------------------------------------------------
// Detect subtours in the current solution
std::vector<std::vector<int>> findSubtours(const std::vector<double> &yvals, int N)
{
    std::vector<std::vector<int>> subtours;
    std::vector<bool> visited(N, false);

    // adjacency list
    std::vector<std::vector<int>> adj(N);
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            int var = map_y[i][j];
            if (var >= 0 && yvals[var] > 0.5)
            {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }

    // BFS connected components
    for (int i = 0; i < N; ++i)
    {
        if (visited[i])
            continue;
        std::vector<int> comp;
        std::queue<int> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty())
        {
            int u = q.front();
            q.pop();
            comp.push_back(u);
            for (int v : adj[u])
            {
                if (!visited[v])
                {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }

        if (comp.size() < N)
            subtours.push_back(comp);
    }

    return subtours;
}

//-------------------------------------------------------------
// Add Subtour Elimination Constraint (SEC)
void addSEC(CPXENVptr env, CPXLPptr lp, const std::vector<int> &subtour)
{
    std::vector<int> idx;
    std::vector<double> coef;
    int K = subtour.size();

    for (size_t i = 0; i < subtour.size(); ++i)
    {
        for (size_t j = i + 1; j < subtour.size(); ++j)
        {
            int u = subtour[i], v = subtour[j];
            int var = (u < v) ? map_y[u][v] : map_y[v][u];
            if (var >= 0)
            {
                idx.push_back(var);
                coef.push_back(1.0);
            }
        }
    }

    if (!idx.empty())
    {
        char sense = 'L';
        double rhs = double(K - 1);
        int matbeg = 0;
        CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, static_cast<int>(idx.size()), &rhs, &sense,
                         &matbeg, idx.data(), coef.data(), NULL, NULL);
    }
}

//-------------------------------------------------------------
// Solve TSP using branch-and-cut with SECs
double solveTSP(CPXENVptr env, CPXLPptr lp, int N)
{
    double objval = 0.0;
    int ncols = CPXgetnumcols(env, lp);
    std::vector<double> yvals(ncols, 0.0);

    while (true)
    {
        // Solve current ILP/LP
        CHECKED_CPX_CALL(CPXmipopt, env, lp);

        // Get solution
        int ncols = CPXgetnumcols(env, lp);
        std::vector<double> yvals(ncols, 0.0);
        CHECKED_CPX_CALL(CPXgetmipx, env, lp, yvals.data(), 0, ncols - 1);

        // Detect subtours
       auto subtours = findSubtours(yvals, N);
        if (subtours.empty())
            break;  // done

        // Add SECs for all subtours
        for (auto &S : subtours)
            addSEC(env, lp, S);
    }

    // Get final objective value
    CHECKED_CPX_CALL(CPXgetmipobjval, env, lp, &objval);

    return objval;
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
            int A_run = (N_run * (N_run - 1)) / 2;
            run_cost = configs[cfg].cost;

            std::cout << "\n===== CONFIG " << cfg + 1 << " | N = " << N_run << " | A = " << A_run << " =====\n";

            for (int run = 1; run <= 10; ++run)
            {
                std::srand(static_cast<unsigned int>(std::time(nullptr)) + run);
                start_node = std::rand() % N_run;

                std::cout << "Executing run " << run << " with N=" << N_run
                          << " rand start_node = " << start_node << std::endl;

                ///// init CPLEX env and prob
                DECL_ENV(env);
                DECL_PROB(env, lp);

                ///// setup LP with SEC formulation
                int numVars = setupLP_SEC(env, lp, N_run);

                ///// solve TSP with branch-and-cut SEC
                double cpx_time_before = 0.0, cpx_time_after = 0.0;
                CHECKED_CPX_CALL(CPXgetdettime, env, &cpx_time_before);
                double objval = solveTSP(env, lp, N_run);
                CHECKED_CPX_CALL(CPXgetdettime, env, &cpx_time_after);

                double cpx_elapsed = (cpx_time_after - cpx_time_before) / 100.0; // dettime is in hundredths of sec
                std::cout << "Run " << run << " - CPLEX CPU time (sec): " << cpx_elapsed << std::endl;
                std::cout << "Run " << run << " - Objval: " << objval << std::endl;

                // save run result
                all_results.push_back({static_cast<int>(cfg + 1), N_run, run, start_node, cpx_elapsed});

                // Write solution file (optional)
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

                for (size_t cfg = 0; cfg < configs.size(); ++cfg)
                {
                    double sum = 0.0;
                    int count = 0;
                    for (const auto &r : all_results)
                    {
                        if (r.config_index == static_cast<int>(cfg + 1))
                        {
                            sum += r.time_sec;
                            count++;
                        }
                    }
                    double avg = (count > 0) ? sum / count : 0.0;
                    csv << cfg + 1 << "," << avg << "\n";
                }

                csv.close();
                std::cout << "Results CSV written to: " << results_file << std::endl;
            }

            char towait;
            std::cout << "\nAll runs completed for config " << (cfg + 1) << ". Press a key to continue...\n";
            std::cin >> towait;
        }
    }
    catch (std::exception &e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
    }

    return 0;
}
