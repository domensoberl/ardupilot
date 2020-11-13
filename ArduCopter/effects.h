#ifndef QQCONTROL_EFFECT
#define QQCONTROL_EFFECT

#include "action.h"
#include "model.h"
#include "target.h"
#include "observations.h"

void qqcontrol_effects_init();
void qqcontrol_effects_clear();
void qqcontrol_effects_resolve(Model *model, Target *target);
void qqcontrol_effects_quantify(State *state, Target *target, Observations *observations);
Action qqcontrol_effects_getBestAction();
void qqcontrol_effects_print(char *buffer);

#endif