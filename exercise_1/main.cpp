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
#include "classes/TSP.h"
#include "classes/model.h"
#include "classes/suggested_model.h"
#include "classes/MTZ_model.h"
#include "classes/board.h"
#include "classes/pattern_library.h"
#include "classes/sample_generator.h"

namespace fs = std::filesystem;
int status;
char errmsg[BUF_SIZE];


void sampleGeneration(int config_count)
{
    const auto& predefinedPatterns = PatternLibrary::all();

    for (int config = 1; config <= config_count; ++config)
    {
        int board_size;
        std::vector<Pattern> selectedPatterns;

        std::cout << "\nConfig " << config << " - choose a size for the board:\n"
                  << "small  (30x30)\n"
                  << "medium (50x50)\n"
                  << "large  (100x100)\n"
                  << "Enter size option (30, 50 or 100): ";

        std::cin >> board_size;

        int maxPatterns = 0;
        switch (board_size)
        {
            case 30:  maxPatterns = 3; break;
            case 50:  maxPatterns = 6; break;
            case 100: maxPatterns = 10; break;
            default:
                std::cout << "Invalid board size.\n";
                continue;
        }

        int pattern_count;
        std::cout << "Enter number of patterns to place (max "
                  << maxPatterns << "): ";
        std::cin >> pattern_count;

        pattern_count = std::min(pattern_count, maxPatterns);

        for (int i = 0; i < pattern_count; ++i)
        {
            std::cout << "\nAvailable patterns:\n";
            for (size_t j = 0; j < predefinedPatterns.size(); ++j)
            {
                std::cout << j + 1 << ": "
                          << predefinedPatterns[j].name << "\n";
            }

            int choice;
            std::cout << "Select pattern " << i + 1 << ": ";
            std::cin >> choice;

            if (choice < 1 || choice > (int)predefinedPatterns.size())
            {
                std::cout << "Invalid selection. Try again.\n";
                --i;
                continue;
            }

            selectedPatterns.push_back(predefinedPatterns[choice - 1]);
        }

        Board board(board_size, selectedPatterns);

        if (!board.isValid())
        {
            std::cout << "Board configuration invalid. Patterns do not fit.\n";
        }
        else
        {
            std::cout << "Board successfully generated.\n";
            SampleGenerator generator;
            for (int i = 1; i <= 10; ++i){
                generator.generate(i, board);
                std::cout << "Sample graph saved for config " << i << ".\n";
            }
        }
    }
}


int main()
{
    try
    {
        std::random_device rd;
        std::mt19937 rng(rd());
        
        int config_count = 0;
        std::cout << "Number of configurations to generate:\n" 
        << " (Notice:\n"
        << "    - each configuration will generate 10 random samples following the configuration specifics\n"
        << "    - each configuration will be tested 10 times with random starting node): (Enter 0 to skip):\n";
        std::cin >> config_count;
        if (config_count > 0)
            sampleGeneration(config_count);

        std::cout << "The program will manage all the configurations in the samples directory...\n";

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
        Model *model = nullptr;
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
        std::vector<TSP> configs;
        const std::string directory = "samples";
        for (const auto &entry : fs::directory_iterator(directory))
        {
            if (!entry.is_regular_file())
                continue;
            std::ifstream file(entry.path());
            if (!file)
                continue;

            int index;
            int N;
            std::vector<std::vector<double>> cost;

            file >> index >> N;
            cost.assign(N, std::vector<double>(N));
            for (int i = 0; i < N; ++i)
                for (int j = 0; j < N; ++j)
                    file >> cost[i][j];
            TSP config(index, N, cost);
            configs.push_back(std::move(config));
        }

        // sort configs by their index
        std::sort(configs.begin(), configs.end(),
                  [](const TSP &a, const TSP &b)
                  {
                      return a.getIndex() < b.getIndex();
                  });

        // process each configuration
        fs::create_directories(solution_directory);
        std::vector<Model::RunResult> all_results;
        std::ofstream csvfile(solution_directory + "/summary_results.csv");
        csvfile << "ConfigIndex,RunIndex,N,StartNode,SolveTime,Optimal,Gap,ObjectiveValue,AvgTime,OptimalCount\n";

        for (const auto &cfg : configs)
        {

            // create directory for this config's solutions
            std::string configDir = solution_directory + "/config_" + std::to_string(cfg.getIndex()) + "_N_" + std::to_string(cfg.getN());
            fs::create_directories(configDir);

            std::cout << "\nProcessing Config " << cfg.getIndex() << " with N=" << cfg.getN() << "\n";

            double avg_time = 0.0;
            int optimal_count = 0;

            for (int run = 1; run <= 10; ++run)
            {
                int start_node = std::uniform_int_distribution<int>(0, cfg.getN() - 1)(rng);

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
                csvfile << cfg.getIndex() << "," << result.run_index << "," << result.N << ","
                        << result.start_node << "," << result.solve_time << ","
                        << (result.optimal ? 1 : 0) << "," << result.gap << ","
                        << result.objective_value << ",,\n";
                if (run == 10)
                {
                    csvfile << cfg.getIndex() << ",,,,,,," << avg_time / 10.0 << "," << optimal_count << "\n";
                }
            }
            char towait;
            std::cout << "\nAll runs completed for config " << (cfg.getIndex()) << ". Press a key to continue...\n";
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
