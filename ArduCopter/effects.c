#include "direction.h"
#include "effects.h"
#include "kinematics.h"

#define SGN(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))
#define ABS(x) ((x >= 0) ? x : -x)

typedef struct {
    double x, y, z;
    double x_v, y_v, z_v;
    double pitch, roll, yaw;
    double pitch_v, roll_v, yaw_v;
} EffectValues;

typedef struct {
    Action action;
    EffectValues directions;
    EffectValues weights;
    double value;
    int stdCount;
} ActionEffect;

static const EffectValues undefinedValues = {
    UNDEFINED, UNDEFINED, UNDEFINED,
    UNDEFINED, UNDEFINED, UNDEFINED,
    UNDEFINED, UNDEFINED, UNDEFINED,
    UNDEFINED, UNDEFINED, UNDEFINED 
};

static const EffectValues zeroValues = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0
};

static ActionEffect qqcontrol_effects_entries[81];

void qqcontrol_effects_init()
{
    int i = 0;
    for (int m1 = -1; m1 <= 1; m1++) {
        for (int m2 = -1; m2 <= 1; m2++) {
            for (int m3 = -1; m3 <= 1; m3++) {
                for (int m4 = -1; m4 <= 1; m4++) {
                    qqcontrol_effects_entries[i].action.m[0] = (double)m1;
                    qqcontrol_effects_entries[i].action.m[1] = (double)m2;
                    qqcontrol_effects_entries[i].action.m[2] = (double)m3;
                    qqcontrol_effects_entries[i].action.m[3] = (double)m4;
                    
                    int stdCount = 0;
                    if (m1 == 0) stdCount++;
                    if (m2 == 0) stdCount++;
                    if (m3 == 0) stdCount++;
                    if (m4 == 0) stdCount++;
                    qqcontrol_effects_entries[i].stdCount = stdCount;

                    i++;
                }
            }
        }
    }
}

void qqcontrol_effects_clear()
{
    for (int i = 0; i < 81; i++) {
        qqcontrol_effects_entries[i].directions = undefinedValues;
        qqcontrol_effects_entries[i].weights = zeroValues;
        qqcontrol_effects_entries[i].value = 0;
    }
}

static double qqcontrol_effects_resolveForAction(Action *action, double *constraint)
{
    double result = UNDEFINED;
    int allSteady = 1;
    for (int i = 0; i < 4; i++) {
        double dir = action->m[i] * constraint[i];
        if (dir == STD) continue;

        allSteady = 0;
        if (dir != result) {
            if (result == UNDEFINED)
                result = dir;
            else
                result = AMB;
        }
    }

    if (allSteady)
        result = STD;

    return result;
}

static double qqcontrol_effects_assignValue(double existing, double computed)
{
    if (existing == UNDEFINED || existing == computed)
        return computed;
    else
        return AMB;
}

void qqcontrol_effects_resolve(Model *model, Target *target)
{
    qqcontrol_effects_clear();
    for (int i = 0; i < 81; i++) {
        ActionEffect *entry = &qqcontrol_effects_entries[i];
        
        if (target->use_x) {
            entry->directions.x = entry->directions.x_v =
                qqcontrol_effects_assignValue(
                    entry->directions.x,
                    qqcontrol_effects_resolveForAction(&entry->action, model->x)
                );
        }
        else {
            entry->directions.x = UNDEFINED;
        }
        
        if (target->use_y) {
            entry->directions.y = entry->directions.y_v =
                qqcontrol_effects_assignValue(
                    entry->directions.y,
                    qqcontrol_effects_resolveForAction(&entry->action, model->y)
                );
        }
        else {
            entry->directions.y = UNDEFINED;
        }

        if (target->use_z) {
            entry->directions.z = entry->directions.z_v =
                qqcontrol_effects_assignValue(
                    entry->directions.z,
                    qqcontrol_effects_resolveForAction(&entry->action, model->z)
                );
        }
        else {
            entry->directions.z = UNDEFINED;
        }

        if (target->use_pitch) {
            entry->directions.pitch = entry->directions.pitch_v =
                qqcontrol_effects_assignValue(
                    entry->directions.pitch,
                    qqcontrol_effects_resolveForAction(&entry->action, model->pitch)
                );
        }
        else {
            entry->directions.pitch = UNDEFINED;
        }
        
        if (target->use_roll) {
            entry->directions.roll = entry->directions.roll_v =
                qqcontrol_effects_assignValue(
                    entry->directions.roll,
                    qqcontrol_effects_resolveForAction(&entry->action, model->roll)
                );
        }
        else {
            entry->directions.roll = UNDEFINED;
        }
        
        if (target->use_yaw) {
            entry->directions.yaw = entry->directions.yaw_v =
                qqcontrol_effects_assignValue(
                    entry->directions.yaw,
                    qqcontrol_effects_resolveForAction(&entry->action, model->yaw)
                );
        }
        else {
            entry->directions.yaw = UNDEFINED;
        }
    }
}

static double qqcontrol_effects_quantifyForAction(
    double currentValue,
    double targetValue,
    double effectDirection,
    double v,
    double vn,
    double vp,
    double an,
    double ap
) {   
    if (effectDirection == INC || effectDirection == DEC) {
        double distance = targetValue - currentValue;
        double targetDirection = SGN(distance);
        
        double eta = 0;
        if (targetDirection > 0) {
            eta = qqcontrol_kinematics_timeToGoal(distance, v, vp, ap, an);
        }
        else if (targetDirection < 0) {
            eta = qqcontrol_kinematics_timeToGoal(-distance, -v, vn, an, ap);
        }
        
        return effectDirection * targetDirection * eta;
    }
    else {
        return 0;
    }
}

void qqcontrol_effects_quantify(State *state, Target *target, Observations *observations)
{
    for (int i = 0; i < 81; i++) {
        EffectValues *directions = &qqcontrol_effects_entries[i].directions;
        EffectValues *weights = &qqcontrol_effects_entries[i].weights;
        double value = 0;

        weights->x = qqcontrol_effects_quantifyForAction(
            state->x, target->x, directions->x, state->x_v,
            observations->x_vn, observations->x_vp,
            observations->x_an, observations->x_ap
        );
        value += weights->x;

        weights->x_v = qqcontrol_effects_quantifyForAction(
            state->x_v, 0, directions->x_v, state->x_a,
            observations->x_an, observations->x_ap,
            0, 0
        );
        value += weights->x_v;

        weights->y = qqcontrol_effects_quantifyForAction(
            state->y, target->y, directions->y, state->y_v,
            observations->y_vn, observations->y_vp,
            observations->y_an, observations->y_ap
        );
        value += weights->y;

        weights->y_v = qqcontrol_effects_quantifyForAction(
            state->y_v, 0, directions->y_v, state->y_a,
            observations->y_an, observations->y_ap,
            0, 0
        );
        value += weights->y_v;

        weights->z = qqcontrol_effects_quantifyForAction(
            state->z, target->z, directions->z, state->z_v,
            observations->z_vn, observations->z_vp,
            observations->z_an, observations->z_ap
        );
        value += weights->z;

        weights->z_v = qqcontrol_effects_quantifyForAction(
            state->z_v, 0, directions->z_v, state->z_a,
            observations->z_an, observations->z_ap,
            0, 0
        );
        value += weights->z_v;

        weights->pitch = qqcontrol_effects_quantifyForAction(
            state->pitch, target->pitch, directions->pitch, state->pitch_v,
            observations->pitch_vn, observations->pitch_vp,
            observations->pitch_an, observations->pitch_ap
        );
        value += weights->pitch;

        weights->pitch_v = qqcontrol_effects_quantifyForAction(
            state->pitch_v, 0, directions->pitch_v, state->pitch_a,
            observations->pitch_an, observations->pitch_ap,
            0, 0
        );
        value += weights->pitch_v;

        weights->roll = qqcontrol_effects_quantifyForAction(
            state->roll, target->roll, directions->roll, state->roll_v,
            observations->roll_vn, observations->roll_vp,
            observations->roll_an, observations->roll_ap
        );
        value += weights->roll;

        weights->roll_v = qqcontrol_effects_quantifyForAction(
            state->roll_v, 0, directions->roll_v, state->roll_a,
            observations->roll_an, observations->roll_ap,
            0, 0
        );
        value += weights->roll_v;

        weights->yaw = qqcontrol_effects_quantifyForAction(
            state->yaw, target->yaw, directions->yaw, state->yaw_v,
            observations->yaw_vn, observations->yaw_vp,
            observations->yaw_an, observations->yaw_ap
        );
        value += weights->yaw;

        weights->yaw_v = qqcontrol_effects_quantifyForAction(
            state->yaw_v, 0, directions->yaw_v, state->yaw_a,
            observations->yaw_an, observations->yaw_ap,
            0, 0
        );
        value += weights->yaw_v;

        qqcontrol_effects_entries[i].value = value;
    }
}

Action qqcontrol_effects_getBestAction()
{
    Action action = {{0, 0, 0, 0}};
    double maxValue = 0;
    int minStd = 4;
    
    for (int i = 0; i < 81; i++) {
        int betterValue = qqcontrol_effects_entries[i].value >= maxValue;
        int equalValue = qqcontrol_effects_entries[i].value == maxValue;
        int betterDynamics = qqcontrol_effects_entries[i].stdCount < minStd;
        if (betterValue || (equalValue && betterDynamics)) {
            action = qqcontrol_effects_entries[i].action;
            maxValue = qqcontrol_effects_entries[i].value;
            minStd = qqcontrol_effects_entries[i].stdCount;
        }
    }

    return action;
}

static char *qqcontrol_effects_appendString(char *dst, const char *src)
{
    while (*src != 0)
        *(dst++) = *(src++);
    return dst;
}

static char *qqcontrol_effects_appendDir(char *dst, double value)
{
    if (value == INC)
        return qqcontrol_effects_appendString(dst, "inc");
    else if (value == STD)
        return qqcontrol_effects_appendString(dst, "std");
    else if (value == DEC)
        return qqcontrol_effects_appendString(dst, "dec");
    else if (value == AMB)
        return qqcontrol_effects_appendString(dst, "amb");
    else
        return qqcontrol_effects_appendString(dst, "   ");
}

static void qqcontrol_effects_itoa(int value, char *dst)
{
    if (value == 0) {
        *(dst++) = '0';
        *dst = 0;
        return;
    }

    char *p = dst;
    while (value != 0) {
        *(p++) = (char)((value % 10) + 48);
        value /= 10;
    }
    *(p--) = 0;

    while (dst < p) {
        char tmp = *dst;
        *dst = *p;
        *p = tmp;
        dst++;
        p--;
    }
}

static char *qqcontrol_effects_appendNumber(char *dst, int value)
{
    char number[3];
    if (value < 10)
        dst = qqcontrol_effects_appendString(dst, " ");
    qqcontrol_effects_itoa(value, number);
    dst = qqcontrol_effects_appendString(dst, number);
    return dst;
}

static char *qqcontrol_effects_appendWeight(char *dst, double value)
{
    char number[7];

    value *= 1000;
    int magnitude = (int)ABS(value);

    if (magnitude == 0) {
        dst = qqcontrol_effects_appendString(dst, "   0.000");
    }
    else if (magnitude <= 999) {    
        qqcontrol_effects_itoa(magnitude, number);
        dst = qqcontrol_effects_appendString(dst, (value >= 0 ? "   0." : "  -0."));
        
        char *p = number;
        int leading = 3;
        while (*(p++) != 0) leading--;
        while (leading-- > 0) *(dst++) = '0';
        p = number;
        while (*p != 0) *(dst++) = *(p++);
    }
    else if (magnitude <= 999999) {
        qqcontrol_effects_itoa(magnitude, number);
        
        char *p = number;
        int leading = 6;
        while (*(p++) != 0) leading--;
        
        int i = 0;
        while (leading-- > 0) {
            if (i++ == 3) *(dst++) = '.';
            *(dst++) = ' ';
        }

        dst = qqcontrol_effects_appendString(dst, (value >= 0 ? " " : "-"));

        p = number;
        while (*p != 0) {
            if (i++ == 3) *(dst++) = '.';
            *(dst++) = *(p++);
        }
    }
    else {
        if (value >= 0)
            dst = qqcontrol_effects_appendString(dst, "huge   ");
        else
            dst = qqcontrol_effects_appendString(dst, "-huge  ");
    }

    return dst;
}

static char *qqcontrol_effects_appendDirWeight(char *dst, double dir, double weight)
{
    if (dir != UNDEFINED) {
        dst = qqcontrol_effects_appendDir(dst, dir);
        dst = qqcontrol_effects_appendString(dst, " ");
        dst = qqcontrol_effects_appendWeight(dst, weight);
    }
    else {
        dst = qqcontrol_effects_appendString(dst, "            ");
    }

    return dst;
}

/* Provide at least 16 KB buffer size .*/
void qqcontrol_effects_print(char *buffer)
{
    char *p = buffer;
    *p = 0;

    static const char *head = " #|m1  m2  m3  m4 |x            x'          |y            y'          |z            z'          |pitch        pitch'      |roll         roll'       |yaw         yaw'         |value   |\n";
    static const char *hbar = "----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";

    p = qqcontrol_effects_appendString(p, hbar);
    p = qqcontrol_effects_appendString(p, head);
    p = qqcontrol_effects_appendString(p, hbar);

    for (int i = 0; i < 81; i++) {
        ActionEffect *entry = &qqcontrol_effects_entries[i];

        p = qqcontrol_effects_appendNumber(p, i);
        p = qqcontrol_effects_appendString(p, "|");
        p = qqcontrol_effects_appendDir(p, entry->action.m[0]);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDir(p, entry->action.m[1]);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDir(p, entry->action.m[2]);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDir(p, entry->action.m[3]);
        p = qqcontrol_effects_appendString(p, "|");

        p = qqcontrol_effects_appendDirWeight(p, entry->directions.x, entry->weights.x);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDirWeight(p, entry->directions.x_v, entry->weights.x_v);
        p = qqcontrol_effects_appendString(p, "|");

        p = qqcontrol_effects_appendDirWeight(p, entry->directions.y, entry->weights.y);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDirWeight(p, entry->directions.y_v, entry->weights.y_v);
        p = qqcontrol_effects_appendString(p, "|");

        p = qqcontrol_effects_appendDirWeight(p, entry->directions.z, entry->weights.z);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDirWeight(p, entry->directions.z_v, entry->weights.z_v);
        p = qqcontrol_effects_appendString(p, "|");

        p = qqcontrol_effects_appendDirWeight(p, entry->directions.pitch, entry->weights.pitch);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDirWeight(p, entry->directions.pitch_v, entry->weights.pitch_v);
        p = qqcontrol_effects_appendString(p, "|");

        p = qqcontrol_effects_appendDirWeight(p, entry->directions.roll, entry->weights.roll);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDirWeight(p, entry->directions.roll_v, entry->weights.roll_v);
        p = qqcontrol_effects_appendString(p, "|");

        p = qqcontrol_effects_appendDirWeight(p, entry->directions.yaw, entry->weights.yaw);
        p = qqcontrol_effects_appendString(p, " ");
        p = qqcontrol_effects_appendDirWeight(p, entry->directions.yaw_v, entry->weights.yaw_v);
        p = qqcontrol_effects_appendString(p, "|");

        p = qqcontrol_effects_appendWeight(p, entry->value);
        p = qqcontrol_effects_appendString(p, "|\n");
    }

    p = qqcontrol_effects_appendString(p, hbar);
    *p = 0;
}