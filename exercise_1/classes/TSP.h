/**
 * @file TSP.h
 * @brief TSP data
 *
 */

#ifndef TSP_H
#define TSP_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>

/**
 * Class that describes a TSP instance (a cost matrix, nodes are identified by integer 0 ... n-1)
 */
class TSP
{
private:
    int index;
    int N;
    std::vector<std::vector<double>> cost;

public:
    TSP(int idx, int n, const std::vector<std::vector<double>>& c)
        : index(idx), N(n), cost(c) {}

    // Getters
    int getIndex() const { return index; }
    int getN() const { return N; }
    const std::vector<std::vector<double>>& getCost() const { return cost; }
    void setIndex(int idx) { index = idx; }
    void setN(int n) { N = n; }
    void setCost(const std::vector<std::vector<double>>& c) { cost = c; }
};

#endif /* TSP_H */
