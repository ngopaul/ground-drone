#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

uint16_t cur_pwm;


// ********* PWM output on pin 2.4
void Servo_Init(void){
    //15000*1.333us=.02s set period to .02 seconds
    //
    cur_pwm = 1075;
    PWM_Init12(15000, 1075, 750);

}
void Servo_high(void) {
//    PWM_Duty1()
    cur_pwm = 1800;
    PWM_Duty1(1800);
}

void Servo_low(void) {
//    PWM_Duty1()
    cur_pwm = 350;
    PWM_Duty1(350);
}

void Servo_Set(uint16_t pos) {
    // Argument: pos in [0,100]
    // maps pos to a position in the servo's range.
    cur_pwm = pos*14.5+350;
    PWM_Duty1(cur_pwm);
}

void Servo_GetPos(uint16_t *pos) {
    *pos = (uint16_t) (cur_pwm - 350)/14.5;
}



