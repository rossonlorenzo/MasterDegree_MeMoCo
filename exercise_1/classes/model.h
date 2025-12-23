/**
 * @file base_model.h
 * @brief BaseModel class definition
 */

#ifndef BASE_MODEL_H
#define BASE_MODEL_H

#include <vector>
#include <string>
#include "../cpxmacro.h"
#include "TSP.h"

class Model
{
public:
    struct RunResult
    {
        int run_index;
        int N;
        int start_node;
        double solve_time;
        bool optimal;
        double gap;
        double objective_value;
    };

protected:    
    virtual int buildVariables() = 0;
    virtual int buildConstraints() = 0;
    virtual int buildObjective() = 0;

    // shared internals
    Env env;
    Prob lp;
    int status;
    std::optional<TSP> current_config;
    int start_node;

    std::vector<std::vector<int>> map_x;
    std::vector<std::vector<int>> map_y;

    RunResult current_run_result;

private:
    void setGraphConfig(const TSP &c);
    void setStartNode(int s);

public:
    Model(Env env, Prob lp);
    virtual ~Model();
    // LP/MIP setup and solve
    int setupLP(TSP config, int run_id, int start_node, const std::string &lp_file);
    RunResult solveRun(int run_id, const std::string &solution_file);
};

#endif // BASE_MODEL_H
