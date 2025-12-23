/**
 * @file sample_generator.h
 * @brief SampleGenerator class definition
 */
#ifndef SAMPLE_GENERATOR_H
#define SAMPLE_GENERATOR_H

#include <vector>
#include "board.h"

class SampleGenerator
{
public:
    SampleGenerator();

    // Generate a graph from a VALID board
    void generate(int config_index, const Board& board);

private:
    std::vector<std::vector<double>> costMatrix;

    void setupGraph(const Board& board);
    void saveGraph(int config_index, const Board& board);
};

#endif // SAMPLE_GENERATOR_H
