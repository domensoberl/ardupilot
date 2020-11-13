#include "model.h"
#include "direction.h"

static Model qqcontrol_model_undefinedModel = {
    {UNDEFINED, UNDEFINED, UNDEFINED, UNDEFINED},
    {UNDEFINED, UNDEFINED, UNDEFINED, UNDEFINED},
    {UNDEFINED, UNDEFINED, UNDEFINED, UNDEFINED},
    {UNDEFINED, UNDEFINED, UNDEFINED, UNDEFINED},
    {UNDEFINED, UNDEFINED, UNDEFINED, UNDEFINED},
    {UNDEFINED, UNDEFINED, UNDEFINED, UNDEFINED}
};

static Model qqcontrol_model_defaultModel = {
    {DEC, INC, INC, DEC}, // x' = M-++-(m1, m2, m3, m4)
    {DEC, DEC, INC, INC}, // y' = M--++(m1, m2, m3, m4)
    {INC, INC, INC, INC}, // z' = M++++(m1, m2, m3, m4)
    {INC, INC, DEC, DEC}, // pitch' = M++--(m1, m2, m3, m4)
    {DEC, INC, INC, DEC}, // roll' = M-++-(m1, m2, m3, m4)
    {DEC, INC, DEC, INC}  // yaw' = M-++-(m1, m2, m3, m4)
};

static Model qqcontrol_model_currentModel;

void qqcontrol_model_init()
{
    qqcontrol_model_currentModel = qqcontrol_model_undefinedModel;
}

void qqcontrol_model_setDefaultModel()
{
    qqcontrol_model_currentModel = qqcontrol_model_defaultModel;
}

void qqcontrol_model_setModel(Model model)
{
    qqcontrol_model_currentModel = model;
}

Model qqcontrol_model_getCurrentModel()
{
    return qqcontrol_model_currentModel;
}

void qqcontrol_model_improve(Input *input)
{

}