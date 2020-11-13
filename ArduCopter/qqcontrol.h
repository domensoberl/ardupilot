#define QQCONTROL_PARAMETER_NONE 0
#define QQCONTROL_PARAMETER_LEARN 1
#define QQCONTROL_PARAMETER_ADAPT 2

#define QQCONTROL_VALUE_FALSE 0
#define QQCONTROL_VALUE_TRUE 1

#define QQCONTROL_INC 1
#define QQCONTROL_STD 0
#define QQCONTROL_DEC -1
#define QQCONTROL_AMB 2
#define QQCONTROL_UNDEFINED -2

typedef struct {
    double x, y, z;
    double x_v, y_v, z_v;
    double x_a, y_a, z_a;
    double pitch, roll, yaw;
    double pitch_v, roll_v, yaw_v;
    double pitch_a, roll_a, yaw_a;
} QQControl_State;

typedef struct {
    double x[4];
    double y[4];
    double z[4];
    double pitch[4];
    double roll[4];
    double yaw[4];
} QQControl_Model;

typedef struct {
    double x;
    double y;
    double z;
    double pitch;
    double roll;
    double yaw;
    double dt;
} QQControl_Input;

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
} QQControl_Target;

typedef struct {
    double m[4];
} QQControl_Action;

extern "C" void qqcontrol_init();
extern "C" void qqcontrol_setParameter(int parameter, int value);
extern "C" int qqcontrol_getParameter(int parameter);
extern "C" void qqcontrol_setDefaultModel();
extern "C" void qqcontrol_setModel(QQControl_Model model);
extern "C" QQControl_Model qqcontrol_getModel();
extern "C" void qqcontrol_setState(QQControl_State state);
extern "C" QQControl_State qqcontrol_getState();
extern "C" QQControl_Action qqcontrol_getAction(QQControl_Input input, QQControl_Target target);

/*
    This function is for debugging purposes. It prints out the qualitative effects and weights
    of all 81 qualitative actions. It should be used after calling qqcontrol_getAction to
    get the values for the last action.
    
    The provided buffer must have at least 16 KB capacity.
*/
extern "C" void qqcontrol_printComputations(char *buffer);