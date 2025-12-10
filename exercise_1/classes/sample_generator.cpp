/**
 * @file sample_generator.cpp
 * @brief
 */

#include "sample_generator.h"

#include <cstdio>
#include <iostream>
#include <vector>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

SampleGenerator::SampleGenerator()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

void SampleGenerator::generate(int config, int N)
{
    if (N < 2 || N > MAXN)
    {
        std::cout << "N out of bounds, using N = " << MAXN << std::endl;
        N = MAXN;
    }

    // ensure samples directory exists
    fs::create_directories("samples");

    setupGraph(N);
    saveGraph(config, N);
}

void SampleGenerator::setupGraph(int N)
{
    std::cout << "Generating symmetric costs for all edges..." << std::endl;

    costMatrix.assign(N, std::vector<double>(N, 0.0));

    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            double cost = double(std::rand() % 10 + 1);
            costMatrix[i][j] = cost;
            costMatrix[j][i] = cost;
        }
    }
}

void SampleGenerator::saveGraph(int config, int N)
{
    char filename[128];
    std::snprintf(filename, sizeof(filename), "samples/config_%d_holes_%d.txt", config, N);

    FILE* f = std::fopen(filename, "w");
    if (!f)
    {
        std::cerr << "Error opening file " << filename << std::endl;
        return;
    }
    
        std::fprintf(f, "%d\n", config);
    std::fprintf(f, "%d\n", N);

    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            std::fprintf(f, "%.6f", costMatrix[i][j]);
            if (j + 1 < N)
                std::fprintf(f, " ");
        }
        std::fprintf(f, "\n");
    }

    std::fclose(f);
    std::cout << "Graph sample written to " << filename << std::endl;
}
