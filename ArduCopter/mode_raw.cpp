#include "Copter.h"


bool ModeRaw::init(bool ignore_checks)
{
    for (int i = 0; i < 4; i++)
        motor_pwm[i] = 0;
    return true;
}


void ModeRaw::run()
{
    GCS_MAVLINK_Copter *mavlink = copter.gcs().chan(0);
    if (mavlink == nullptr) {
        hal.console->printf("Mavlink communication error!\n");
        return;
    }

    if (mavlink->raw_motor_request_pending()) {
        uint16_t *pwm = mavlink->get_raw_motor_request();
        for (int i = 0; i < 4; i++)
            motor_pwm[i] = pwm[i];
    }

    /* Output motor values */
    motors->set_pwm(1, motor_pwm[0]);
    motors->set_pwm(2, motor_pwm[1]);
    motors->set_pwm(3, motor_pwm[2]);
    motors->set_pwm(4, motor_pwm[3]);
}