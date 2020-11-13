#include "state.h"

static const State qqcontrol_state_initialState = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
State qqcontrol_state_currentState;

void qqcontrol_state_init()
{
    qqcontrol_state_currentState = qqcontrol_state_initialState;
}

void qqcontrol_state_setState(State *state)
{
    qqcontrol_state_currentState = *state;
}

void qqcontrol_state_updateState(Input *input)
{
    double dt = input->dt;
    State lastState = qqcontrol_state_currentState;
    State *newState = &qqcontrol_state_currentState;

    // variable x
    newState->x = input->x;
    newState->x_v = (newState->x - lastState.x) / dt;
    newState->x_a = (newState->x_v - lastState.x_v) / dt;

    // variable y
    newState->y = input->x;
    newState->y_v = (newState->y - lastState.y) / dt;
    newState->y_a = (newState->y_v - lastState.y_v) / dt;

    // variable z
    newState->z = input->z;
    newState->z_v = (newState->z - lastState.z) / dt;
    newState->z_a = (newState->z_v - lastState.z_v) / dt;

    // variable pitch
    newState->pitch = input->pitch;
    newState->pitch_v = (newState->pitch - lastState.pitch) / dt;
    newState->pitch_a = (newState->pitch_v - lastState.pitch_v) / dt;

    // variable roll
    newState->roll = input->roll;
    newState->roll_v = (newState->roll - lastState.roll) / dt;
    newState->roll_a = (newState->roll_v - lastState.roll_v) / dt;

    // variable yaw
    newState->yaw = input->yaw;
    newState->yaw_v = (newState->yaw - lastState.yaw) / dt;
    newState->yaw_a = (newState->yaw_v - lastState.yaw_v) / dt;
}

State qqcontrol_state_getState()
{
    return qqcontrol_state_currentState;
}