#include "hardware/pwm.h"
enum stepper_state
{
    STOP,
    ACCEL,
    CONSTANT,
    DEACCEL
};
enum stepper_direction
{
    FORWARD,
    REVERSE
};
struct stepper_struct
{
    int step;
    double accel;
    double speed;
    int accel_val;
    int target_step;
    int accel_step;
    int deaccel_step;
    int c_constant;
    enum stepper_direction stepper_direction;
    enum stepper_state stepper_state;
    int c_step;
    int rest;
    uint stepper_slice;
    uint stepper_channel;
    double c_t;
    pwm_config stepper_pwm_config;
};
void move_to(int pos, double speed, double accel);
void move_distance(int steps, float speed, float accel);
void stepper_init();
int accel_steps(double speed, double accel);
enum stepper_state get_stepper_state();