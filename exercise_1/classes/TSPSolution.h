#ifndef TSPSOLUTION_H
#define TSPSOLUTION_H

#include <vector>
#include <iostream>
#include <stdexcept>
#include <random>
#include <algorithm>

#include "TSP.h"

#define uint unsigned int

class TSPSolution
{
public:
    std::vector<int> sequence;  // tour: size N+1, last == first
    double cost;                // cached tour cost

public:
    TSPSolution() : cost(0.0) {}

    TSPSolution(const TSP& tsp) : TSPSolution(tsp, 0) {}

    TSPSolution(const TSP& tsp, int start_node)
        : cost(0.0)
    {
        buildCanonical(tsp, start_node);
        // cost is not computed automatically here; call recomputeCost(tsp) when needed
    }

    TSPSolution(const TSPSolution& other)
        : sequence(other.sequence), cost(other.cost) {}

    TSPSolution& operator=(const TSPSolution& right)
    {
        if (this == &right) return *this;
        sequence = right.sequence;
        cost = right.cost;
        return *this;
    }

    /* --- Builders --- */

    void buildCanonical(const TSP& tsp, int start_node)
    {
        int n = tsp.getN();
        if (start_node < 0 || start_node >= n)
            throw std::runtime_error("Invalid start node");

        sequence.clear();
        sequence.reserve(n + 1);

        for (int i = 0; i < n; ++i)
            sequence.push_back((start_node + i) % n);

        sequence.push_back(start_node);
        cost = 0.0; // caller can recompute
    }

    // Random tour keeping start node fixed at both ends
    template <class URNG>
    void randomize(const TSP& tsp, int start_node, URNG& rng)
    {
        buildCanonical(tsp, start_node);

        // shuffle positions [1..n-1] (exclude start at index 0 and last)
        if (sequence.size() <= 3) return;

        std::shuffle(sequence.begin() + 1, sequence.end() - 1, rng);

        // ensure last equals first
        sequence.back() = sequence.front();
        recomputeCost(tsp);
    }

    /* --- Cost utilities --- */

    void recomputeCost(const TSP& tsp)
    {
        double total = 0.0;
        for (size_t i = 0; i + 1 < sequence.size(); ++i)
            total += tsp.getCost()[sequence[i]][sequence[i + 1]];
        cost = total;
    }

    // O(1) delta for 2-opt reversal between i..j (1 <= i < j <= n-1)
    double delta2Opt(const TSP& tsp, int i, int j) const
    {
        const auto& seq = sequence;
        const auto& C = tsp.getCost();

        int a = seq[i - 1];
        int b = seq[i];
        int c = seq[j];
        int d = seq[j + 1];

        // remove (a,b) + (c,d), add (a,c) + (b,d)
        return C[a][c] + C[b][d] - C[a][b] - C[c][d];
    }

    // Apply the 2-opt reversal [i..j]
    void apply2Opt(int i, int j)
    {
        std::reverse(sequence.begin() + i, sequence.begin() + j + 1);
    }

    void print() const
    {
        for (uint i = 0; i < sequence.size(); ++i)
            std::cout << sequence[i] << " ";
        std::cout << "\n";
    }
};

#endif /* TSPSOLUTION_H */
