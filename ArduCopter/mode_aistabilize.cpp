#include "Copter.h"
#include "qqcontrol.h"

static float pitch = 0;
static float roll = 0;
static float yaw = 0;
static float z = 0;
static float dt = 0;

/* Qualitative data */
#define INC QQCONTROL_INC
#define STD QQCONTROL_STD
#define DEC QQCONTROL_DEC

static QQControl_Input input;
static QQControl_Target target;
static QQControl_Action action;
static double qstep = 10;

static QQControl_Model qualitativeModel = {
    {STD, STD, STD, STD}, // x' = M-++-(m1, m2, m3, m4)
    {STD, STD, STD, STD}, // y' = M--++(m1, m2, m3, m4)
    {INC, INC, INC, INC}, // z' = M++++(m1, m2, m3, m4)
    {INC, DEC, INC, DEC}, // pitch' = M++--(m1, m2, m3, m4)
    {DEC, INC, INC, DEC}, // roll' = M-++-(m1, m2, m3, m4)
    {INC, INC, DEC, DEC}  // yaw' = M-++-(m1, m2, m3, m4)
};
/* End of qualitative data */

/*
 * Init and run calls for stabilize flight mode
 */
bool ModeAIStabilize::init(bool ignore_checks)
{
    state = State::IDLE;
    timestamp = AP_HAL::millis();
    for (int i = 0; i < 4; i++)
        motor_pwm[i] = 0;
    loop_iterations = 0;

    init_qualitative_module();

    return true;
}


// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeAIStabilize::run()
{
    uint32_t timestamp1 = AP_HAL::millis();
    dt = (timestamp1 - timestamp) / 1000.0;
    timestamp = timestamp1;

    /* Get sensory input */
    pitch = ToDeg(ahrs.get_pitch());
    roll = ToDeg(ahrs.get_roll());
    yaw = ToDeg(ahrs.get_yaw());
    z = 0;
    ahrs.get_hagl(z);

    if (loop_iterations == 100) {
        hal.console->printf("%f %f %f %f %f\n", pitch, roll, yaw, z, dt);
        //hal.console->printf("%d %d %d %d\n", motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
        loop_iterations = 0;
    }
    else {
        loop_iterations++;
    }

    /* Check current state */
    switch (state) {
        case State::IDLE:
            if (z < target_height - 1) {
                hal.console->printf("Switching to LIFT state.\n");
                state = State::LIFT;
            }
            else {
                hal.console->printf("Switching to STABILIZE state.\n");
                state = State::STABILIZE;
            }
        break;

        case State::LIFT:
            if (z < target_height) {
                for (int i = 0; i < 4; i++)
                    motor_pwm[i] = 1800;
            }
            else {
                for (int i = 0; i < 4; i++)
                    motor_pwm[i] = 1500;
                
                hal.console->printf("Switching to STABILIZE state.\n");
                state = State::STABILIZE;
            }
        break;

        case State::STABILIZE:
            if (z > target_height - 1) {
                /* Call any stabilization method. */
                qualitative_stabilization();
            }
            else {
                hal.console->printf("Switching to LIFT state.\n");
                state = State::LIFT;
            }
        break;
    }

    /* Output motor values */
    motors->set_pwm(1, motor_pwm[0]);
    motors->set_pwm(2, motor_pwm[1]);
    motors->set_pwm(3, motor_pwm[2]);
    motors->set_pwm(4, motor_pwm[3]);
}

/* Qualitative stabilization */
void ModeAIStabilize::init_qualitative_module()
{
    qqcontrol_init();
    qqcontrol_setModel(qualitativeModel);

    target.pitch = 0;
    target.roll = 0;
    target.yaw = 0;
    target.z = 5.0;

    target.use_pitch = target.use_roll = target.use_yaw = target.use_z = 1;
    target.use_x = target.use_y = 0;
}

void ModeAIStabilize::qualitative_stabilization()
{
    input.pitch = (double)pitch;
    input.roll = (double)roll;
    input.yaw = (double)yaw;
    input.z = (double)z;
    input.dt = dt;

    action = qqcontrol_getAction(input, target);

    motor_pwm[0] += action.m[0] * qstep;
    motor_pwm[1] += action.m[1] * qstep;
    motor_pwm[2] += action.m[2] * qstep;
    motor_pwm[3] += action.m[3] * qstep;
}
/* End of qualitative stabilization */