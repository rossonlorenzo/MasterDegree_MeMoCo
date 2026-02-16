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
#include "classes/TSPSolver.h"
#include "classes/board.h"
#include "classes/pattern_library.h"
#include "classes/sample_generator.h"

namespace fs = std::filesystem;
int status;
char errmsg[BUF_SIZE];

/* --- Parameter grid --- */
static const std::vector<TSPSolver::SAParameters> paramGrid = {
    {1000, 1e-3, 0.95, 100, 1000},
    {2000, 1e-4, 0.97, 200, 1000},
    {5000, 1e-4, 0.99, 300, 1000}};

void sampleGeneration(int config_count)
{
    const auto &predefinedPatterns = PatternLibrary::all();

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
        case 30:
            maxPatterns = 11;
            break;
        case 50:
            maxPatterns = 24;
            break;
        case 100:
            maxPatterns = 73;
            break;
        default:
            std::cout << "Invalid board size.\n";
            continue;
        }

        int pattern_count;
        std::cout << "Enter number of patterns to place (max "
                  << maxPatterns << "): ";
        std::cin >> pattern_count;

        pattern_count = std::min(pattern_count, maxPatterns);

        for (int i = 0; i < pattern_count;)
        {
            std::cout << "\nAvailable patterns:\n";
            for (size_t j = 0; j < predefinedPatterns.size(); ++j)
            {
                std::cout << j + 1 << ": "
                          << predefinedPatterns[j].name << "\n";
            }

            std::string input;
            std::cout << "Select pattern " << i + 1
                      << " (format: n or n,count): ";
            std::cin >> input;

            int choice = 0;
            int count = 1; // default

            // Parse input
            size_t commaPos = input.find(',');

            try
            {
                if (commaPos == std::string::npos)
                {
                    // Only number (e.g. "2")
                    choice = std::stoi(input);
                }
                else
                {
                    // Format "n,count"
                    choice = std::stoi(input.substr(0, commaPos));
                    count = std::stoi(input.substr(commaPos + 1));
                }
            }
            catch (...)
            {
                std::cout << "Invalid format. Use n or n,count.\n";
                continue;
            }

            // Validate
            if (choice < 1 || choice > (int)predefinedPatterns.size() || count < 1)
            {
                std::cout << "Invalid selection or count.\n";
                continue;
            }

            // Add patterns
            for (int k = 0; k < count && i < pattern_count; ++k)
            {
                selectedPatterns.push_back(predefinedPatterns[choice - 1]);
                ++i;
            }
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
            for (int i = 1; i <= 10; ++i)
            {
                generator.generate(i, board);
                std::cout << "Sample graph saved for config " << i << ".\n";
            }
        }
    }
}

void manageMathModel(Model *model,
                     std::vector<TSP> &configs,
                     std::vector<Model::RunResult> &all_results,
                     const std::string &solution_directory,
                     std::mt19937 &rng)
{
    // process each configuration
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
            model->solveRun(run, path);
            Model::RunResult result = model->getCurrentRunResult();
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
}

void manageOptimizationAlgorithm(std::vector<TSP> &configs,
                                 const std::string &solution_directory,
                                 std::mt19937 &rng)
{
    fs::create_directories(solution_directory);
    std::vector<TSP> tuning_configs;
    std::vector<TSP> test_configs;

    // tuning / test split (7 tuning, 3 test per block of 10)
    std::cout << "\nSplitting configs into tuning and test sets...\n";
    for (const auto &cfg : configs)
    {
        if (cfg.getIndex() % 10 <= 7 && cfg.getIndex() % 10 != 0)
        {
            std::cout << "Tuning config added: " << cfg.getIndex() << "\n";
            tuning_configs.push_back(cfg);
        }
        else
        {
            std::cout << "Test config added: " << cfg.getIndex() << "\n";
            test_configs.push_back(cfg);
        }
    }

    TSPSolver solver(tuning_configs[0], test_configs[0]);

    for (int i = 0; i < 3; ++i)
    {
        TSP tuningCfg = tuning_configs[i * 7];
        std::ofstream csvfile(solution_directory + "/board_" + std::to_string(tuningCfg.getBoardSize()) + "x" + std::to_string(tuningCfg.getBoardSize()) + "_holes_" + std::to_string(tuningCfg.getN()) + "_summary_results.csv");
        csvfile << "ConfigIndex,RunIndex,N,StartNode,InitialCost,SolveTime,Improved,Improvement,ObjectiveValue,AvgTime,AvgBestCost,ImprovementCount\n";

        for (int j = 0; j < 7; ++j)
        {
            tuningCfg = tuning_configs[i * 7 + j];
            std::cout << "\nProcessing Config " << tuningCfg.getIndex() << " with N=" << tuningCfg.getN() << "\n";

            /* ---------- TUNING ---------- */
            for (size_t p = 0; p < paramGrid.size(); ++p)
            {
                solver.setSAParameters(paramGrid[p]);

                std::cout << "\n[Parameter Set " << p + 1 << "/" << paramGrid.size() << "]\n";

                double avg_time = 0.0;
                double avg_best_cost = 0.0;
                int improved_count = 0;

                for (int run = 1; run <= 10; ++run)
                {
                    int start_node =
                        std::uniform_int_distribution<int>(0, tuningCfg.getN() - 1)(rng);

                    solver.setTuningInstance(tuningCfg);
                    solver.runTuning(start_node, run);

                    const auto &r = solver.getCurrentRunResult();

                    avg_best_cost += r.best_cost;
                    avg_time += r.solve_time;
                    if (r.improvement > 0.0)
                        ++improved_count;

                    /* ----- Console log ----- */
                    std::cout << "Run " << r.run_index << " completed\n"
                              << "- Solve time: " << r.solve_time << " sec\n"
                              << "- Initial cost: " << r.initial_cost << "\n"
                              << "- Best cost: " << r.best_cost << "\n"
                              << "- Worst cost: " << r.worst_cost << "\n"
                              << "- Improvement: "
                              << std::fixed << std::setprecision(2)
                              << r.improvement << "%\n"
                              << "- Start node: " << r.start_node << "\n\n";

                    /* ----- CSV ----- */
                    csvfile << tuningCfg.getIndex() << ","
                            << r.run_index << ","
                            << r.N << ","
                            << r.start_node << ","
                            << r.initial_cost << ","
                            << r.solve_time << ","
                            << (r.improvement > 0.0 ? 1 : 0) << ","
                            << r.improvement << ","
                            << r.best_cost << ",,\n";

                    if (run == 10)
                    {
                        csvfile << tuningCfg.getIndex()
                                << ",,,,,,,,"
                                << avg_time / 10.0 << ","
                                << avg_best_cost / 10.0 << ","
                                << improved_count << "\n\n";
                    }
                }
            }
        }
        solver.selectBestParameters(solver.getTuningResults());

        std::cout << ">>> BEST TUNING PARAMETERS SELECTED <<<\n";
        std::cout << "T0: " << solver.getTuningResults().bestParams.T0 << "\n"
                  << "Tmin: " << solver.getTuningResults().bestParams.Tmin << "\n"
                  << "Alpha: " << solver.getTuningResults().bestParams.alpha << "\n"
                  << "ItersPerT: " << solver.getTuningResults().bestParams.itersPerT << "\n"
                  << "MaxNoImprove: " << solver.getTuningResults().bestParams.maxNoImprove << "\n\n";

        csvfile << "Selected Parameters: T0=" << solver.getTuningResults().bestParams.T0
                << ", Tmin=" << solver.getTuningResults().bestParams.Tmin
                << ", Alpha=" << solver.getTuningResults().bestParams.alpha
                << ", ItersPerT=" << solver.getTuningResults().bestParams.itersPerT
                << ", MaxNoImprove=" << solver.getTuningResults().bestParams.maxNoImprove
                << "\n\n";

        /* ---------- TESTING ---------- */

        for (int k = 0; k < 3; ++k)
        {
            TSP testCfg = test_configs[i * 3 + k];
            std::cout << "\nProcess test Config" << testCfg.getIndex()
                      << " (N=" << testCfg.getN() << ")\n";

            double avg_time = 0.0;
            double avg_best_cost = 0.0;
            int improved_count = 0;

            for (int run = 1; run <= 10; run++)
            {
                int start_node =
                    std::uniform_int_distribution<int>(0, testCfg.getN() - 1)(rng);

                solver.setTestInstance(testCfg);
                solver.runTests(start_node, run);

                const auto &r = solver.getCurrentRunResult();

                avg_time += r.solve_time;
                avg_best_cost += r.best_cost;
                if (r.improvement > 0.0)
                    ++improved_count;

                std::cout << "Run " << r.run_index << " completed\n"
                          << "- Solve time: " << r.solve_time << " sec\n"
                          << "- Initial cost: " << r.initial_cost << "\n"
                          << "- Best cost: " << r.best_cost << "\n"
                          << "- Worst cost: " << r.worst_cost << "\n"
                          << "- Improvement: "
                          << std::fixed << std::setprecision(2)
                          << r.improvement << "%\n"
                          << "- Start node: " << r.start_node << "\n";

                csvfile << testCfg.getIndex() << ","
                        << r.run_index << ","
                        << r.N << ","
                        << r.start_node << ","
                        << r.initial_cost << ","
                        << r.solve_time << ","
                        << (r.improvement > 0.0 ? 1 : 0) << ","
                        << r.improvement << ","
                        << r.best_cost << ",,\n";
                if (run == 10)
                {
                    csvfile << testCfg.getIndex()
                            << ",,,,,,,,"
                            << avg_time / 10.0 << ","
                            << avg_best_cost / 10.0 << ","
                            << improved_count << "\n\n";
                }
            }
        }
        csvfile << "\n\n";
        csvfile.close();
        solver.clearResults(solver.getTuningResults());
        solver.clearResults(solver.getTestResults());
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

        // load configurations
        std::vector<TSP> configs;
        const std::string directory = "samples";
        for (const auto &entry : fs::directory_iterator(directory))
        {
            if (!entry.is_regular_file())
                continue;
            if (entry.path().extension() != ".txt")
                continue;
            std::ifstream file(entry.path());
            if (!file)
                continue;

            int index;
            int size;
            int N;
            std::vector<std::vector<double>> cost;

            file >> index;
            file >> size;
            file >> N;
            cost.assign(N, std::vector<double>(N));
            for (int i = 0; i < N; ++i)
                for (int j = 0; j < N; ++j)
                    file >> cost[i][j];
            TSP config(index, size, N, cost);
            configs.push_back(std::move(config));
            std::cout << "Loaded configuration_" << index << "_board_" << size << "x" << size << "_holes_" << N << "\n";
        }

        // sort configs by their index
        std::sort(configs.begin(), configs.end(),
                  [](const TSP &a, const TSP &b)
                  {
                      if (a.getBoardSize() != b.getBoardSize())
                          return a.getBoardSize() < b.getBoardSize();

                      if (a.getN() != b.getN())
                          return a.getN() < b.getN();

                      return a.getIndex() < b.getIndex();
                  });
        std::cout << "Sorted configurations:\n";
        for (const auto &cfg : configs)
        {
            std::cout << "configuration_" << cfg.getIndex() << "_board_" << cfg.getBoardSize() << "x" << cfg.getBoardSize() << "_holes_" << cfg.getN() << "\n";
        }

        // model selection
        std::string solution_directory = "solutions_";
        int method_choice;
        std::cout << "Select the LP solving method:\n"
                  << "1 - Math Model\n"
                  << "2 - Optimization Algorithm\n";
        std::cin >> method_choice;
        if (method_choice == 2)
        {
            solution_directory = solution_directory + "OptimizationAlgorithm_";
            manageOptimizationAlgorithm(configs, solution_directory, rng);
        }
        else
        {
            // declare env & lp
            DECL_ENV(env);
            // initialize problem
            DECL_PROB(env, lp);
            solution_directory = solution_directory + "MathModel_";
            int model_type;
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
            fs::create_directories(solution_directory);
            std::vector<Model::RunResult> all_results;
            manageMathModel(model, configs, all_results, solution_directory, rng);
            delete model;
            model = nullptr;
        }
    }
    catch (std::exception &e)
    {
        std::cerr << ">>> EXCEPTION: " << e.what() << "\n";
    }

    return 0;
}
