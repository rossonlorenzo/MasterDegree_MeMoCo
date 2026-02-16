/*
 * @file TSPSolver.h
 * @brief TSP solver with Simulated Annealing tuning/testing
 */

#ifndef TSPSOLVER_H
#define TSPSOLVER_H

#include <vector>
#include <random>

#include "TSPSolution.h"
#include "TSP.h"

class TSPSolver
{
public:
  struct SAParameters
  {
    double T0 = 1000.0;
    double Tmin = 1e-3;
    double alpha = 0.95;
    int itersPerT = 100;
    int maxNoImprove = 1000;
  };

  struct RunResult
  {
    int run_index = 0;
    int N = 0;
    int start_node = 0;
    double solve_time = 0.0;

    double initial_cost = 0.0;
    double best_cost = 0.0;
    double worst_cost = 0.0;
    double improvement = 0.0;

    SAParameters paramsUsed{};
  };

  struct Results
  {
    std::vector<RunResult> runResults;
    SAParameters bestParams{};
    double averageImprovement = 0.0;
    double averageCost = 0.0;
  };

  /* Constructor (NO default ctor) */
  TSPSolver(const TSP& tuning, const TSP& test);

  /* Instance management */
  void setTuningInstance(const TSP &tsp);
  void setTestInstance(const TSP &tsp);
  void setSAParameters(const SAParameters &params);

  /* Results management */
  void clearResults(Results &res);
  RunResult &getCurrentRunResult();
  Results &getTuningResults();
  Results &getTestResults();

  /* Main workflow (one run per call) */
  void runTuning(const int &start_node, const int &run_id);
  void runTests(const int &start_node, const int &run_id);

  /* Select best parameters out of a Results container */
  void selectBestParameters(Results &res);

protected:
  TSP tuningInstance;
  TSP testInstance;

  SAParameters tunedParams{};
  bool paramsReady = false;

  Results tuningResult;
  Results testResult;
  RunResult current_run_result;

  std::mt19937 rng_;

  /* Core */
  void simulatedAnnealing(
      const TSP &tsp,
      TSPSolution &sol,
      const SAParameters &params,
      bool shortRun,
      double &worstCost);

  double evaluate(const TSPSolution &sol, const TSP &tsp) const;
};

/* Needed for grouping params */
bool operator==(const TSPSolver::SAParameters &a, const TSPSolver::SAParameters &b);

#endif /* TSPSOLVER_H */
