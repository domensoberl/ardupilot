#include "model.h"
#include "input.h"
#include "action.h"
#include "target.h"
#include "effects.h"

#define QQCONTROL_PARAMETER_NONE 0
#define QQCONTROL_PARAMETER_LEARN 1
#define QQCONTROL_PARAMETER_ADAPT 2

#define QQCONTROL_VALUE_FALSE 0
#define QQCONTROL_VALUE_TRUE 1

static int qqcontrol_parameter_learn = QQCONTROL_VALUE_TRUE;
static int qqcontrol_parameter_adapt = QQCONTROL_VALUE_TRUE;
static int qqcontrol_isStateDetermined = 0;

void qqcontrol_init()
{
    qqcontrol_state_init();
    qqcontrol_model_init();
    qqcontrol_effects_init();
    qqcontrol_observations_clear();
    qqcontrol_isStateDetermined = 0;
}

void qqcontrol_setParameter(int parameter, int value)
{
    switch (parameter) {
        case QQCONTROL_PARAMETER_LEARN:
            qqcontrol_parameter_learn = value;
            break;
        case QQCONTROL_PARAMETER_ADAPT:
            qqcontrol_parameter_adapt = value;
            break;
    }
}

int qqcontrol_getParameter(int parameter)
{
    switch (parameter) {
        case QQCONTROL_PARAMETER_LEARN:
            return qqcontrol_parameter_learn;
            break;
        case QQCONTROL_PARAMETER_ADAPT:
            return qqcontrol_parameter_adapt;
            break;
        default:
            return 0;
    }
}

void qqcontrol_setDefaultModel()
{
    qqcontrol_model_setDefaultModel();
}

void qqcontrol_setModel(Model model)
{
    qqcontrol_model_setModel(model);
}

Model qqcontrol_getModel()
{
    return qqcontrol_model_getCurrentModel();
}

void qqcontrol_setState(State state)
{
    qqcontrol_state_setState(&state);
    qqcontrol_isStateDetermined = 1;
}

State qqcontrol_getState()
{
    return qqcontrol_state_getState();
}

Action qqcontrol_getAction(Input input, Target target)
{
    qqcontrol_state_updateState(&input);
    if (!qqcontrol_isStateDetermined) {
        qqcontrol_isStateDetermined = 1;
        Action action = {{0, 0, 0, 0}};
        return action;
    }

    State state = qqcontrol_state_getState();

    if (qqcontrol_parameter_adapt)
        qqcontrol_observations_make(&state);
    
    if (qqcontrol_parameter_learn)
        qqcontrol_model_improve(&input);
    
    Model model = qqcontrol_model_getCurrentModel();
    Observations observations = qqcontrol_observations_getCurrentObservations();

    qqcontrol_effects_resolve(&model, &target);
    qqcontrol_effects_quantify(&state, &target, &observations);
    return qqcontrol_effects_getBestAction();
}

void qqcontrol_printComputations(char *buffer)
{
    qqcontrol_effects_print(buffer);
}