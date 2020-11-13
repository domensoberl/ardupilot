#ifndef QQCONTROL_MODEL
#define QQCONTROL_MODEL

#include "input.h"

typedef struct {
    double x[4];
    double y[4];
    double z[4];
    double pitch[4];
    double roll[4];
    double yaw[4];
} Model;

void qqcontrol_model_init();
void qqcontrol_model_setDefaultModel();
void qqcontrol_model_setModel(Model model);
Model qqcontrol_model_getCurrentModel();
void qqcontrol_model_improve(Input *input);

#endif