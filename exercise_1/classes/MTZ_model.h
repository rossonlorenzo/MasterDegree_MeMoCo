#ifndef MTZ_MODEL_H
#define MTZ_MODEL_H

#include "model.h"
#include <vector>

class MTZModel : public Model
{
public:
    // Inherit base constructor
    explicit MTZModel(Env env, Prob lp)
        : Model(env, lp) {}

protected:
    // Overrides required by Model
    int buildVariables() override;
    int buildConstraints() override;
    int buildObjective() override;

    // Additional mapping for MTZ order variables
    std::vector<int> map_u;
};

#endif // MTZ_MODEL_H
