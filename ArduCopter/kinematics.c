#include <math.h>

/*
Time needed to rech the goal value.
x  - Distance to be made.
v  - Current speed (positive, if towards the goal).
v1 - Maximum speed towards the goal (unsigned).
ap - Acceleration (unsigned, towars the goal). Can be zero.
an  - Negative acceleration (unsigned, away from the goal). Can be zero.
Returns time to travel distance x. It returns the default 1 if the value cannot be computed.
*/
double qqcontrol_kinematics_timeToGoal(double x, double v, double v1, double ap, double an)
{
	/* If no distance needs to be travelled, the resulting time is zero. */
	if (x == 0)
		return 0;

	/* We work with positive displacement. If negative, rotate the coordinate system. */
	if (x < 0)
		return qqcontrol_kinematics_timeToGoal(-x, -v, v1, an, ap);

	/* If no acceleration is used. */
	if (ap <= 0) {
		if (v <= 0) return 1;
		return x / v;
	}

	/* Check and adjust top speed. */
	if (v > v1) v1 = v;

	/* If top speed is 0, use the default value 1. */
	if (v1 == 0) v1 = 1;

	/* If v is non-negative, we use equation (v1)^2 = v^2 + 2a(dx). */
	if (v >= 0) { // We go towards the goal.
		double vx = sqrt(pow(v, 2) + 2 * ap * x); // Final speed.
		if (vx <= v1) { // Maximum speed is not exceeded.
			return (vx - v) / ap; // Time to reach the final speed.
		}
		else { // Maximum speed is exceeded.
			double t1 = (v1 - v) / ap; // Time to reach the maximum speed.
			double s = (pow(v1, 2) - pow(v, 2)) / (2 * ap); // Displacement made until maximum speed.
			double t2 = (x - s) / v1; // The rest of the path is made with maximum speed.
			return t1 + t2;
		}
	}

	/* if v is negative, we first compute the stopping point. */
	else { // We go away from the goal.
		double s = pow(v, 2) / (2 * ap); // Displacement made until stopped.
		double t1 = -v / ap; // Time until stopped.
		double t2 = qqcontrol_kinematics_timeToGoal(x + s, 0, v1, ap, an);
		return t1 + t2;
	}
}
