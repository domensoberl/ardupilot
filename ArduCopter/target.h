#ifndef QQCONTROL_TARGET
#define QQCONTROL_TARGET

typedef struct {
    char use_x;
    char use_y;
    char use_z;
    char use_pitch;
    char use_roll;
    char use_yaw;
    double x;
    double y;
    double z;
    double pitch;
    double roll;
    double yaw;
} Target;

#endif