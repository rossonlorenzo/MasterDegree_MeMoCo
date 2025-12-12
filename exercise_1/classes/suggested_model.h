#ifndef SUGGESTED_MODEL_H
#define SUGGESTED_MODEL_H

#include "model.h"

class SuggestedModel : public Model
{
public:
    // Inherit base constructor
    explicit SuggestedModel(Env env, Prob lp)
        : Model(env, lp) {}

protected:
    // Overrides required by Model
    int buildVariables() override;
    int buildConstraints() override;
    int buildObjective() override;
};

#endif // SUGGESTED_MODEL_H
