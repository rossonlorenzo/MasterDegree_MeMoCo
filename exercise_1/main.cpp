/**
 * @file main.cpp
 * @brief Main program for solving exercise 1 using BaseModel and SampleGenerator
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
#include <random>


#include "cpxmacro.h"
#include "classes/model.h"
#include "classes/suggested_model.h"
#include "classes/MTZ_model.h"
#include "classes/sample_generator.h"

namespace fs = std::filesystem;
int status;
char errmsg[BUF_SIZE];

int main()
{
    try
    {
        std::random_device rd;
        std::mt19937 rng(rd());
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

        // declare env & lp
        DECL_ENV(env);
        // initialize problem
        DECL_PROB(env, lp);

        // model selection
        int model_type;
        std::string solution_directory = "solutions_";
        std::cout << "Please state the type of model to use: \n"
                  << "1 - Suggested Model\n"
                  << "2 - MTZ Model\n"
                  << "Enter model type (1 or 2): ";
        std::cin >> model_type;
        Model* model = nullptr;
        switch (model_type)
        {
        case 1:
            model = new SuggestedModel(env, lp);
            solution_directory = solution_directory + "SuggestedModel";
            break;

        case 2:
            model = new MTZModel(env, lp);
            solution_directory = solution_directory + "MTZModel";
            break;

        default:
            throw std::runtime_error("Unknown model type selected");
        }

        // load configurations
        std::vector<Model::GraphConfig> configs;
        const std::string directory = "samples";
        for (const auto &entry : fs::directory_iterator(directory))
        {
            if (!entry.is_regular_file())
                continue;
            std::ifstream file(entry.path());
            if (!file)
                continue;

            Model::GraphConfig config;
            file >> config.index >> config.N;
            config.cost.assign(config.N, std::vector<double>(config.N));
            for (int i = 0; i < config.N; ++i)
                for (int j = 0; j < config.N; ++j)
                    file >> config.cost[i][j];
            configs.push_back(std::move(config));
        }

        // sort configs by their index
        std::sort(configs.begin(), configs.end(),
                  [](const Model::GraphConfig &a, const Model::GraphConfig &b)
                  {
                      return a.index < b.index;
                  });

        // process each configuration
        fs::create_directories(solution_directory);
        std::vector<Model::RunResult> all_results;
        std::ofstream csvfile(solution_directory + "/summary_results.csv");
        csvfile << "ConfigIndex,RunIndex,N,StartNode,SolveTime,Optimal,Gap,ObjectiveValue,AvgTime,OptimalCount\n";

        for (const auto &cfg : configs)
        {

            // create directory for this config's solutions
            std::string configDir = solution_directory + "/config_" + std::to_string(cfg.index) + "_N_" + std::to_string(cfg.N);
            fs::create_directories(configDir);

            std::cout << "\nProcessing Config " << cfg.index << " with N=" << cfg.N << "\n";

            double avg_time = 0.0;
            int optimal_count = 0;

            for (int run = 1; run <= 10; ++run)
            {
                int start_node = std::uniform_int_distribution<int>(0, cfg.N - 1)(rng);


                // setup config
                std::string path = configDir + "/run_" + std::to_string(run) + ".lp";
                model->setupLP(cfg, run, start_node, path);

                // solve run
                path = configDir + "/run_" + std::to_string(run) + ".sol";
                Model::RunResult result = model->solveRun(run, path);
                avg_time += result.solve_time;
                if (result.optimal)
                    ++optimal_count;
                all_results.push_back(result);

                // log results
                std::cout << "Run " << result.run_index << " completed\n"
                          << "- Solve time: " << result.solve_time << " sec\n"
                          << "- Solution status: "
                          << (result.optimal ? "Optimal" : "Not optimal") << "\n"
                          << "- Gap: " << std::fixed << std::setprecision(2)
                          << result.gap << "%\n"
                          << "- Objective: " << result.objective_value << "\n"
                          << "- Start node: " << result.start_node << "\n\n";

                // write to CSV
                csvfile << cfg.index << "," << result.run_index << "," << result.N << ","
                        << result.start_node << "," << result.solve_time << ","
                        << (result.optimal ? 1 : 0) << "," << result.gap << ","
                        << result.objective_value << ",,\n";
                if (run == 10)
                {
                    csvfile << cfg.index << ",,,,,,," << avg_time / 10.0 << "," << optimal_count << "\n";
                }
            }
            char towait;
            std::cout << "\nAll runs completed for config " << (cfg.index) << ". Press a key to continue...\n";
            std::cin >> towait;
        }
        csvfile.close();

        delete model;
        model = nullptr;
    }
    catch (std::exception &e)
    {
        std::cerr << ">>> EXCEPTION: " << e.what() << "\n";
    }

    return 0;
}
