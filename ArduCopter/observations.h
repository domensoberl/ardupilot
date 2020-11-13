#ifndef QQCONTROL_OBSERVATIONS
#define QQCONTROL_OBSERVATIONS

#include "state.h"

typedef struct {
    double x_vp, x_vn, x_ap, x_an;
    double y_vp, y_vn, y_ap, y_an;
    double z_vp, z_vn, z_ap, z_an;
    double pitch_vp, pitch_vn, pitch_ap, pitch_an;
    double roll_vp, roll_vn, roll_ap, roll_an;
    double yaw_vp, yaw_vn, yaw_ap, yaw_an;
} Observations;

void qqcontrol_observations_clear();
Observations qqcontrol_observations_getCurrentObservations();
void qqcontrol_observations_make(State *state);

#endif