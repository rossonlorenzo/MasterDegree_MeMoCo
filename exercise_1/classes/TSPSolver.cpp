#include "TSPSolver.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>

/* -------------------------------------------------------
 * SAParameters equality (needed for grouping by params)
 * ------------------------------------------------------- */
bool operator==(const TSPSolver::SAParameters &a, const TSPSolver::SAParameters &b)
{
    return a.T0 == b.T0 &&
           a.Tmin == b.Tmin &&
           a.alpha == b.alpha &&
           a.itersPerT == b.itersPerT &&
           a.maxNoImprove == b.maxNoImprove;
}

/* -------------------------------------------------------
 * Constructor (ONLY ONE)
 * ------------------------------------------------------- */
TSPSolver::TSPSolver(const TSP& tuning, const TSP& test)
    : tuningInstance(tuning),
      testInstance(test),
      paramsReady(false),
      rng_(std::random_device{}())
{}

/* -------------------------------------------------------
 * Instance management
 * ------------------------------------------------------- */
void TSPSolver::setTuningInstance(const TSP &tsp) { tuningInstance = tsp; }
void TSPSolver::setTestInstance(const TSP &tsp)   { testInstance = tsp; }

void TSPSolver::setSAParameters(const SAParameters &params)
{
    tunedParams = params;
    paramsReady = true;
}

void TSPSolver::clearResults(Results &res)
{
    res.runResults.clear();
    res.bestParams = SAParameters{};
    res.averageImprovement = 0.0;
    res.averageCost = 0.0;
    paramsReady = false;
}

TSPSolver::RunResult &TSPSolver::getCurrentRunResult() { return current_run_result; }
TSPSolver::Results &TSPSolver::getTuningResults() { return tuningResult; }
TSPSolver::Results &TSPSolver::getTestResults() { return testResult; }

/* -------------------------------------------------------
 * Tuning run (single run)
 * ------------------------------------------------------- */
void TSPSolver::runTuning(const int &start_node, const int &run_id)
{
    if (tuningInstance.getN() == 0)
    {
        std::cerr << "ERROR: No tuning instance provided.\n";
        return;
    }
    if (!paramsReady)
    {
        std::cerr << "ERROR: SA parameters not set.\n";
        return;
    }

    auto t0 = std::chrono::high_resolution_clock::now();

    TSPSolution sol(tuningInstance, start_node);
    sol.randomize(tuningInstance, start_node, rng_);

    double initialCost = sol.cost;
    double worstCost = initialCost;

    simulatedAnnealing(tuningInstance, sol, tunedParams, true, worstCost);

    auto t1 = std::chrono::high_resolution_clock::now();
    current_run_result.solve_time = std::chrono::duration<double>(t1 - t0).count();

    double bestCost = sol.cost;
    double improvement = (initialCost > 0.0) ? (100.0 * (initialCost - bestCost) / initialCost) : 0.0;

    current_run_result.run_index = run_id;
    current_run_result.N = tuningInstance.getN();
    current_run_result.start_node = start_node;
    current_run_result.initial_cost = initialCost;
    current_run_result.best_cost = bestCost;
    current_run_result.worst_cost = worstCost;
    current_run_result.improvement = improvement;
    current_run_result.paramsUsed = tunedParams;

    tuningResult.runResults.push_back(current_run_result);
}

/* -------------------------------------------------------
 * Test run (single run)
 * ------------------------------------------------------- */
void TSPSolver::runTests(const int &start_node, const int &run_id)
{
    if (testInstance.getN() == 0) return;

    auto t0 = std::chrono::high_resolution_clock::now();

    TSPSolution sol(testInstance, start_node);
    sol.randomize(testInstance, start_node, rng_);

    double initialCost = sol.cost;
    double worstCost = initialCost;

    simulatedAnnealing(testInstance, sol, tuningResult.bestParams, true, worstCost);

    auto t1 = std::chrono::high_resolution_clock::now();
    current_run_result.solve_time = std::chrono::duration<double>(t1 - t0).count();

    double bestCost = sol.cost;
    double improvement = (initialCost > 0.0) ? (100.0 * (initialCost - bestCost) / initialCost) : 0.0;

    current_run_result.run_index = run_id;
    current_run_result.N = testInstance.getN();
    current_run_result.start_node = start_node;
    current_run_result.initial_cost = initialCost;
    current_run_result.best_cost = bestCost;
    current_run_result.worst_cost = worstCost;
    current_run_result.improvement = improvement;
    current_run_result.paramsUsed = tuningResult.bestParams;

    testResult.runResults.push_back(current_run_result);
}

/* -------------------------------------------------------
 * Parameter selection (avg best_cost primary, improvement tiebreak)
 * ------------------------------------------------------- */
void TSPSolver::selectBestParameters(Results &res)
{
    if (res.runResults.empty())
    {
        std::cerr << "ERROR: No results available.\n";
        return;
    }

    struct Stats
    {
        double totalImprovement = 0.0;
        double totalBestCost = 0.0;
        int count = 0;
    };

    std::vector<SAParameters> uniqueParams;
    std::vector<Stats> stats;

    for (const auto &run : res.runResults)
    {
        bool found = false;
        for (size_t i = 0; i < uniqueParams.size(); ++i)
        {
            if (run.paramsUsed == uniqueParams[i])
            {
                stats[i].totalImprovement += run.improvement;
                stats[i].totalBestCost += run.best_cost;
                stats[i].count++;
                found = true;
                break;
            }
        }

        if (!found)
        {
            uniqueParams.push_back(run.paramsUsed);
            stats.push_back({run.improvement, run.best_cost, 1});
        }
    }

    double bestAvgCost = std::numeric_limits<double>::infinity();
    double bestAvgImprovement = -std::numeric_limits<double>::infinity();
    size_t bestIdx = 0;

    for (size_t i = 0; i < uniqueParams.size(); ++i)
    {
        double avgCost = stats[i].totalBestCost / stats[i].count;
        double avgImp = stats[i].totalImprovement / stats[i].count;

        if (avgCost < bestAvgCost || (avgCost == bestAvgCost && avgImp > bestAvgImprovement))
        {
            bestAvgCost = avgCost;
            bestAvgImprovement = avgImp;
            bestIdx = i;
        }
    }

    res.bestParams = uniqueParams[bestIdx];
    res.averageCost = bestAvgCost;
    res.averageImprovement = bestAvgImprovement;
}

/* -------------------------------------------------------
 * Simulated Annealing (delta-based, early stop)
 * ------------------------------------------------------- */
void TSPSolver::simulatedAnnealing(
    const TSP &tsp,
    TSPSolution &sol,
    const SAParameters &params,
    bool shortRun,
    double &worstCost)
{
    double currCost = sol.cost;

    TSPSolution bestSol(sol);
    double bestCost = currCost;

    worstCost = currCost;

    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    double T = params.T0;
    int iters = shortRun ? std::max(1, params.itersPerT / 5) : params.itersPerT;

    int noImprove = 0;

    const int tourSize = (int)sol.sequence.size(); // N+1
    std::uniform_int_distribution<int> distI(1, tourSize - 3);

    while (T > params.Tmin)
    {
        for (int k = 0; k < iters; ++k)
        {
            int i = distI(rng_);
            std::uniform_int_distribution<int> distJ(i + 1, tourSize - 2);
            int j = distJ(rng_);

            double delta = sol.delta2Opt(tsp, i, j);

            bool accept = false;
            if (delta < 0.0) accept = true;
            else accept = (uni01(rng_) < std::exp(-delta / T));

            if (accept)
            {
                sol.apply2Opt(i, j);
                currCost += delta;
                sol.cost = currCost;

                worstCost = std::max(worstCost, currCost);

                if (currCost < bestCost)
                {
                    bestCost = currCost;
                    bestSol = sol;
                    noImprove = 0;
                }
                else
                {
                    ++noImprove;
                }
            }
            else
            {
                ++noImprove;
            }

            if (params.maxNoImprove > 0 && noImprove >= params.maxNoImprove)
            {
                sol = bestSol;
                sol.cost = bestCost;
                return;
            }
        }

        T *= params.alpha;
    }

    sol = bestSol;
    sol.cost = bestCost;
}

/* -------------------------------------------------------
 * Utility: evaluate
 * ------------------------------------------------------- */
double TSPSolver::evaluate(const TSPSolution &sol, const TSP &tsp) const
{
    double total = 0.0;
    for (size_t i = 0; i + 1 < sol.sequence.size(); ++i)
        total += tsp.getCost()[sol.sequence[i]][sol.sequence[i + 1]];
    return total;
}
