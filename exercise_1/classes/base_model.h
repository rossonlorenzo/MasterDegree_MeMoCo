/**
 * @file base_model.h
 * @brief BaseModel class with full CPLEX integration (declarations only)
 */

#ifndef BASE_MODEL_H
#define BASE_MODEL_H

#include <vector>
#include <string>
#include <ilcplex/cplex.h> // CPLEX types

class BaseModel
{
public:
struct RunResult
{
    int config_index;
    int N;
    int run_index;
    int start_node;
    double solve_time;     // solver time only
    bool optimal;          // 1 = optimal, 0 = not optimal
    double gap;            // MIP gap (%), or 0 for LP
    double objective_value;// Best objective found
};

    struct GraphConfig
    {
        int index;
        int N;
        std::vector<std::vector<double>> cost;
    };

public:
    // Constructor
    explicit BaseModel(CPXENVptr env, CPXLPptr lp, const std::string& directory = "samples");

    // Setters
    void setStartNode(int s);
    void setRunCost(const std::vector<std::vector<double>>& C);

    // Getters
    const std::vector<RunResult>& results() const;
    const std::vector<std::vector<double>>& cost() const;

    // LP/MIP setup and solve
    int setupLP(int N, int run_id);
    RunResult solveRun(int run_id, const std::string& solution_file, int& start_node_out);

private:
    // Helper for y-index (undirected edges)
    int y_index(int i, int j) const;

private:
    std::string directory_;

    CPXENVptr env;
    CPXLPptr lp;
    int status;
    int current_N;
    int start_node;

    RunResult result;
    std::vector<std::vector<double>> run_cost;

    std::vector<std::vector<int>> map_x;
    std::vector<std::vector<int>> map_y;
};

#endif // BASE_MODEL_H
