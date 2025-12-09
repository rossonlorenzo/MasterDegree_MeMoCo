/**
 * @file sample_generator.cpp
 * @brief
 */

#include <cstdio>
#include <iostream>
#include <vector>
#include <limits>
#include <cstdlib>
#include <ctime>

// sets
const int MAXN = 200; // maximum graph nodes (size of C matrix)

// cost matrix (current run)
std::vector<std::vector<double>> config_cost; // C[i][j] = cost from node i to node j

void setupGraph(int N)
{
    // generate random symmetric costs for all the edges
    std::cout << "Generating symmetric costs for all the edges..." << std::endl;
    config_cost.clear();
    config_cost.resize(N);

    for (int i = 0; i < N; ++i)
    {
        config_cost[i].resize(N, 0.0); // initialize all to 0.0
    }

    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            double cost = double(std::rand() % 10 + 1);
            config_cost[i][j] = cost;
            config_cost[j][i] = cost; // ensure symmetry
        }
    }

    // store run cost to file in the samples directory
    char filename[128];
    std::snprintf(filename, sizeof(filename), "samples/graph_N_%d.txt", N);

    FILE *f = std::fopen(filename, "w");
    if (f == NULL)
    {
        std::cerr << "Error opening file " << filename
                  << " for writing graph sample!" << std::endl;
        return;
    }

    // write header (N) and full matrix of costs
    std::fprintf(f, "%d\n", N);
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            std::fprintf(f, "%.6f", config_cost[i][j]);
            if (j + 1 < N)
                std::fprintf(f, " ");
        }
        std::fprintf(f, "\n");
    }

    std::fclose(f);
    std::cout << "Graph sample written to " << filename << std::endl;
}

int main(int argc, char const *argv[])
{

    // Seed the random number generator with the current time
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    std::cout << "The program will generate 3 different configurations (insert N). "
              << "MAX N = " << MAXN << std::endl;

    for (int config = 1; config <= 3; ++config)
    {
        int N_config = MAXN;

        // fully connected graph
        int A_config = (N_config * (N_config - 1)) / 2;

        std::cout << "\nConfig " << config << " - insert N (<= " << MAXN << "): ";

        if (!(std::cin >> N_config))
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, using default values: N="
                      << MAXN << " A=" << A_config << std::endl;
            N_config = MAXN;
        }

        if (N_config < 2 || N_config > MAXN)
        {
            std::cout << "N out of bound, using N = " << MAXN << std::endl;
            N_config = MAXN;
        }

        A_config = (N_config * (N_config - 1)) / 2;

        // generate a new graph random sample
        setupGraph(N_config);
    }

    return 0;
}
