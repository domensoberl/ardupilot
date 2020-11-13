#ifndef QQCONTROL_STATE
#define QQCONTROL_STATE

#include "input.h"

typedef struct {
    double x, y, z;
    double x_v, y_v, z_v;
    double x_a, y_a, z_a;
    double pitch, roll, yaw;
    double pitch_v, roll_v, yaw_v;
    double pitch_a, roll_a, yaw_a;
} State;

void qqcontrol_state_init();
void qqcontrol_state_setState(State *state);
void qqcontrol_state_updateState(Input *input);
State qqcontrol_state_getState();

#endif