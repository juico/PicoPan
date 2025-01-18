#include <stdio.h>
#include <math.h>
#include "ak8419.h"
#include "pico/stdlib.h"

#include "stepper.h"
// Data will be copied from src to dst


struct stepper_struct stepper;
bool stop = false;
void __not_in_flash_func(stepper_isr)()
{
    pwm_clear_irq(stepper.stepper_slice);
    if (stepper.stepper_direction == FORWARD)
    {
        stepper.step++;
    }
    else
    {
        stepper.step--;
    }
    if (stepper.step == stepper.accel_step)
    {
        stepper.rest=0;
        stepper.stepper_state = CONSTANT;
    }
    if (stepper.step == stepper.deaccel_step)
    {
        stepper.stepper_state = DEACCEL;
        stepper.accel_val = -(stepper.accel_val);
         //printf("deaccelstarted %d %d\n",stepper.accel_val,stepper.c_constant);
    }
    if (stepper.stepper_state == CONSTANT)
    {
        stepper.c_step = stepper.c_constant;
    }
    else
    {
        stepper.c_step = stepper.c_step - (2 * stepper.c_step+stepper.rest) / (4 * stepper.accel_val + 1);
        stepper.rest = (2 * stepper.c_step+stepper.rest) % (4 * stepper.accel_val + 1);
        stepper.accel_val++;
    }

    pwm_set_chan_level(stepper.stepper_slice, stepper.stepper_channel, (stepper.c_step>>8) / 2);
    pwm_set_wrap(stepper.stepper_slice, (stepper.c_step>>8));
    if (stepper.step == (stepper.target_step))
    {
        stop = true;
        stepper.stepper_state = STOP;
        pwm_set_enabled(stepper.stepper_slice, false);
        gpio_put(ENABLE_PIN, 1);
    }
}
void move_to(int pos, double speed, double accel)
{
    int accel_distance = ((speed * speed) / (2.0 * accel) + 1.0); // Does not take into account not reaching speed in time
    if (pos > stepper.step)
    {
        gpio_put(DIR_PIN, 1);
        stepper.stepper_direction = FORWARD;
        stepper.accel_step = stepper.step + accel_distance;
        stepper.deaccel_step = pos - (accel_distance - 1);
    }
    else
    {
        gpio_put(DIR_PIN, 0);
        stepper.stepper_direction = REVERSE;
        stepper.accel_step = stepper.step - accel_distance;
        stepper.deaccel_step = pos + (accel_distance - 1);
    }
            printf("Moving to %d,accel_step:%d,deaccel_step%d\n", stepper.target_step, stepper.accel_step, stepper.deaccel_step);

    stepper.target_step = pos;
    stepper.rest=0;
    stepper.accel_val = 1;
    stepper.c_constant = 256000000.0 / (speed );
    stepper.stepper_state = ACCEL;
    stepper.c_step = 256000000.0 * (0.676) * sqrt(2.0 / accel)  ;
    pwm_set_wrap(stepper.stepper_slice, stepper.c_step);
    pwm_set_enabled(stepper.stepper_slice, true);
    gpio_put(ENABLE_PIN, 0);

}

void move_distance(int steps, float speed, float accel)
{
    int accel_steps = ((speed * speed / (accel)) + 2.0);
    if (steps < 0)
    {
        accel_steps = -accel_steps;
    }
    move_to(stepper.step + steps + accel_steps, speed, accel);
}
void stepper_init()
{
    stepper.step = 0;
    stepper.stepper_state = STOP;

    gpio_set_function(STEP_PIN, GPIO_FUNC_PWM);
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 1);
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);


    stepper.stepper_slice = pwm_gpio_to_slice_num(STEP_PIN);
    stepper.stepper_channel = pwm_gpio_to_channel(STEP_PIN);
    pwm_clear_irq(stepper.stepper_slice);
    pwm_set_irq_enabled(stepper.stepper_slice, true);
    irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), stepper_isr);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);

    stepper.stepper_pwm_config = pwm_get_default_config();
    pwm_config_set_clkdiv_int_frac(&(stepper.stepper_pwm_config), 150,0);//Run pwm at 1MHz
    pwm_init(stepper.stepper_slice, &(stepper.stepper_pwm_config), false);
}