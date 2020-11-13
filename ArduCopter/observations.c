#include "observations.h"

static const Observations qqcontrol_observations_initialObservations = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static Observations qqcontrol_observations_currentObservations;

void qqcontrol_observations_clear()
{
    qqcontrol_observations_currentObservations = qqcontrol_observations_initialObservations;
}

Observations qqcontrol_observations_getCurrentObservations()
{
    return qqcontrol_observations_currentObservations;
}

void qqcontrol_observations_make(State *state)
{
    Observations *observations = &qqcontrol_observations_currentObservations;

    // variable x
    if (state->x_v > 0 && state->x_v > observations->x_vp) observations->x_vp = state->x_v;
    else if (state->x_v < 0 && -state->x_v > observations->x_vn) observations->x_vn = -state->x_v;
    if (state->x_v > 0 && state->x_a > observations->x_ap) observations->x_ap = state->x_a;
    else if (state->x_v < 0 && -state->x_a > observations->x_an) observations->x_an = -state->x_a;

    // variable y
    if (state->y_v > 0 && state->y_v > observations->y_vp) observations->y_vp = state->y_v;
    else if (state->y_v < 0 && -state->y_v > observations->y_vn) observations->y_vn = -state->y_v;
    if (state->y_a > 0 && state->y_a > observations->y_ap) observations->y_ap = state->y_a;
    else if (state->y_a < 0 && -state->y_a > observations->y_an) observations->y_an = -state->y_a;

    // variable z
    if (state->z_v > 0 && state->z_v > observations->z_vp) observations->z_vp = state->z_v;
    else if (state->z_v < 0 && -state->z_v > observations->z_vn) observations->z_vn = -state->z_v;
    if (state->z_a > 0 && state->z_a > observations->z_ap) observations->z_ap = state->z_a;
    else if (state->z_a < 0 && -state->z_a > observations->z_an) observations->z_an = -state->z_a;

    // variable pitch
    if (state->pitch_v > 0 && state->pitch_v > observations->pitch_vp) observations->pitch_vp = state->pitch_v;
    else if (state->pitch_v < 0 && -state->pitch_v > observations->pitch_vn) observations->pitch_vn = -state->pitch_v;
    if (state->pitch_a > 0 && state->pitch_a > observations->pitch_ap) observations->pitch_ap = state->pitch_a;
    else if (state->pitch_a < 0 && -state->pitch_a > observations->pitch_an) observations->pitch_an = -state->pitch_a;

    // variable roll
    if (state->roll_v > 0 && state->roll_v > observations->roll_vp) observations->roll_vp = state->roll_v;
    else if (state->roll_v < 0 && -state->roll_v > observations->roll_vn) observations->roll_vn = -state->roll_v;
    if (state->roll_a > 0 && state->roll_a > observations->roll_ap) observations->roll_ap = state->roll_a;
    else if (state->roll_a < 0 && -state->roll_a > observations->roll_an) observations->roll_an = -state->roll_a;

    // variable yaw
    if (state->yaw_v > 0 && state->yaw_v > observations->yaw_vp) observations->yaw_vp = state->yaw_v;
    else if (state->yaw_v < 0 && -state->yaw_v > observations->yaw_vn) observations->yaw_vn = -state->yaw_v;
    if (state->yaw_a > 0 && state->yaw_a > observations->yaw_ap) observations->yaw_ap = state->yaw_a;
    else if (state->yaw_a < 0 && -state->yaw_a > observations->yaw_an) observations->yaw_an = -state->yaw_a;
}