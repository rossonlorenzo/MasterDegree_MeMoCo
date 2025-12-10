/**
 * @file sample_generator.h
 * @brief 
 */

#ifndef SAMPLE_GENERATOR_H
#define SAMPLE_GENERATOR_H

#include <vector>

class SampleGenerator
{
public:
    static const int MAXN = 200;

    SampleGenerator();

    // generate a graph and save it to samples/
    void generate(int config, int N);

private:
    std::vector<std::vector<double>> costMatrix;

    void setupGraph(int N);
    void saveGraph(int config, int N);
};

#endif // SAMPLE_GENERATOR_H
