/**
 * @file main.cpp
 * @brief Simplified main using BaseModel encapsulating CPLEX
 */

#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <regex>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <limits>

#include "classes/base_model.h"
#include "classes/sample_generator.h"

namespace fs = std::filesystem;

int main()
{
    try
    {
        SampleGenerator generator;
        int config_count = 0;
        std::cout << "Number of configurations to generate: (Enter 0 to skip) ";
        std::cin >> config_count;

        std::cout << "The program will manage all the configurations in the samples directory if not empty ... \n";
        for (int config = 1; config <= config_count; ++config)
        {
            int N = SampleGenerator::MAXN;
            std::cout << "Config " << config << " - insert N (<= "
                      << SampleGenerator::MAXN << "): ";
            if (!(std::cin >> N))
            {
                N = SampleGenerator::MAXN;
                std::cin.clear();
                std::cin.ignore(10000, '\n');
            }
            generator.generate(config, N);
        }

        // --------- Declare env & lp ---------
        CPXENVptr env = nullptr;
        CPXLPptr lp = nullptr;

        int status_local = 0;
        env = CPXopenCPLEX(&status_local);
        if (!env || status_local != 0)
            throw std::runtime_error("CPLEX environment creation failed");

        // Initialize problem once (or recreate per run)
        lp = CPXcreateprob(env, &status_local, "BaseModelProblem");
        if (!lp)
        {
            CPXcloseCPLEX(&env);
            throw std::runtime_error("Failed to create LP");
        }

        // --------- Pass handles to model ---------
        BaseModel model(env, lp, "samples");

        // --------- Load configurations ---------
        std::vector<BaseModel::GraphConfig> configs;
        const std::string directory = "samples";
        for (const auto &entry : fs::directory_iterator(directory))
        {
            if (!entry.is_regular_file())
                continue;
            std::ifstream file(entry.path());
            if (!file)
                continue;

            BaseModel::GraphConfig config;
            file >> config.index >> config.N;
            config.cost.assign(config.N, std::vector<double>(config.N));
            for (int i = 0; i < config.N; ++i)
                for (int j = 0; j < config.N; ++j)
                    file >> config.cost[i][j];
            configs.push_back(std::move(config));
        }

        // Sort configs by their index
        std::sort(configs.begin(), configs.end(),
                  [](const BaseModel::GraphConfig &a, const BaseModel::GraphConfig &b)
                  {
                      return a.index < b.index;
                  });

        fs::create_directories("solutions");
        // --------- Process configurations ---------
        std::vector<BaseModel::RunResult> all_results;
        std::ofstream csvfile("solutions/summary_results.csv");
        csvfile << "ConfigIndex,RunIndex,N,StartNode,SolveTime,Optimal,Gap,ObjectiveValue\n";
        for (const auto &cfg : configs)
        {
            std::string configDir = "solutions/config_" + std::to_string(cfg.index) + "_N_" + std::to_string(cfg.N);
            fs::create_directories(configDir);
            std::cout << "\nProcessing Config " << cfg.index << " with N=" << cfg.N << "\n";
            model.setRunCost(cfg.cost);

            double avg_time = 0.0;
            int optimal_count = 0;

            for (int run = 1; run <= 10; ++run)
            {
                int start_node;
                std::string solfile = configDir + "/run_" + std::to_string(run) + ".sol";
                BaseModel::RunResult result = model.solveRun(run, solfile, start_node);
                avg_time += result.solve_time;
                if (result.optimal)
                    ++optimal_count;
                all_results.push_back(result);
                
                std::cout << "Run " << result.run_index << " completed\n"
                          << "- Solve time: " << result.solve_time << " sec\n"
                          << "- Solution status: "
                          << (result.optimal ? "Optimal" : "Not optimal") << "\n"
                          << "- Gap: " << std::fixed << std::setprecision(2)
                          << result.gap << "%\n"
                          << "- Objective: " << result.objective_value << "\n"
                          << "- Start node: " << result.start_node << "\n\n";

                    csvfile << cfg.index << "," << result.run_index << "," << result.N << ","
                            << result.start_node << "," << result.solve_time << ","
                            << (result.optimal ? 1 : 0) << "," << result.gap << ","
                            << result.objective_value << "\n";
                    if (run == 10)
                    {
                        csvfile << "AverageTime,OptimalCount,,,,,,\n";
                        csvfile << avg_time / 10.0 << "," << optimal_count << ",,,,,,\n";
                        csvfile << "\n";
                    }
            }
            char towait; std::cout << "\nAll runs completed for config " << (cfg.index) << ". Press a key to continue...\n"; std::cin >> towait;
        }
        csvfile.close();

        // --------- Cleanup ---------
        if (lp)
            CPXfreeprob(env, &lp);
        if (env)
            CPXcloseCPLEX(&env);
    }
    catch (std::exception &e)
    {
        std::cerr << ">>> EXCEPTION: " << e.what() << "\n";
    }

    return 0;
}
