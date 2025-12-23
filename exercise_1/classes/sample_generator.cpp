/**
 * @file sample_generator.cpp
 * @brief Implementation of SampleGenerator class
 */
#include "sample_generator.h"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <random>

namespace fs = std::filesystem;

SampleGenerator::SampleGenerator() = default;

void SampleGenerator::generate(int config_index, const Board& board)
{
    if (!board.isValid())
    {
        std::cerr << "Cannot generate sample: board is invalid\n";
        return;
    }

    setupGraph(board);
    saveGraph(config_index, board);
}

void SampleGenerator::setupGraph(const Board& board)
{
    const auto& holes = board.getHoles();
    const int n = static_cast<int>(holes.size());

    costMatrix.assign(n, std::vector<double>(n, 0.0));

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> noise(0.9, 1.1);

    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            double dx = holes[i].x - holes[j].x;
            double dy = holes[i].y - holes[j].y;
            double distance = std::sqrt(dx * dx + dy * dy);

            double cost = distance * noise(rng);

            costMatrix[i][j] = cost;
            costMatrix[j][i] = cost;
        }
    }
}

void SampleGenerator::saveGraph(int config_index, const Board& board)
{
    fs::create_directories("samples");

    const int holes = static_cast<int>(board.getHoles().size());
    const int size  = board.getSize();

    std::string filename =
        "samples/config_" + std::to_string(config_index) +
        "_board_" + std::to_string(size) + "x" + std::to_string(size) +
        "_holes_" + std::to_string(holes) + ".txt";

    std::ofstream out(filename);
    if (!out)
    {
        std::cerr << "Failed to write " << filename << "\n";
        return;
    }

    out << config_index << "\n";
    out << holes << "\n";

    for (const auto& row : costMatrix)
    {
        for (size_t j = 0; j < row.size(); ++j)
        {
            out << row[j];
            if (j + 1 < row.size()) out << " ";
        }
        out << "\n";
    }

    std::cout << "Graph sample written to " << filename << "\n";
}

