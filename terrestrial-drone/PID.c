#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../../tirslk_max_1_00_00/inc/Motor.h"
#include "../../tirslk_max_1_00_00/inc/Tachometer.h"
#include "../../tirslk_max_1_00_00/inc/TA3InputCapture.h"

float LeftVelocity = 0;
float RightVelocity = 0;
float LeftTargetVelocity;
float RightTargetVelocity;
int16_t LeftPWM = 3000;            //  rotations per minute
int16_t RightPWM = 3000;           // rotations per minute
uint16_t LeftTach;             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;    // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;             // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach;            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;   // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;            // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
uint16_t period = 100; // in ms

float LeftPropK = 20;
float RightPropK = 20;

int clock_period = 10; // in ms

void handle_pid(int counter) {
    if (counter % clock_period == 0) {
        pid_loop();
    }
}

void pid_loop(void) {
    uint16_t PreviousLeftTach = LeftTach;
    enum TachDirection PreviousLeftDir = LeftDir;
    int32_t PreviousLeftSteps = LeftSteps;
    uint16_t PreviousRightTach = RightTach;
    enum TachDirection PreviousRightDir = RightDir;
    int32_t PreviousRightSteps = RightSteps;


    Tachometer_Get(&LeftTach, &LeftDir, &LeftSteps, &RightTach, &RightDir, &RightSteps);
    LeftVelocity = ((float) (LeftSteps - PreviousLeftSteps));
    RightVelocity = ((float) (RightSteps - PreviousRightSteps));

    printf("LeftVelocity: %f | RightVelocity: %f\n", LeftVelocity, RightVelocity);

    printf("LeftSteps: %u | RightSteps: %u\n", LeftSteps, RightSteps);

    float LeftError = LeftTargetVelocity - LeftVelocity;
    float RightError = RightTargetVelocity - RightVelocity;

    printf("LeftError: %f | RightError: %f\n", LeftError, RightError);

    LeftPWM += (int) LeftError * LeftPropK;
    RightPWM += (int) RightError * RightPropK;
    printf("LeftPWM: %d | RightPWM: %d\n", LeftPWM, RightPWM);

    drive_motors(LeftPWM, RightPWM);
}

void drive_motors(int16_t leftPWM, int16_t rightPWM) {
    if (leftPWM >= 0 && rightPWM >= 0) {
        Motor_Forward((uint16_t) leftPWM, (uint16_t) rightPWM);
    } else if(leftPWM >= 0 && rightPWM <= 0) {
        Motor_Right((uint16_t) leftPWM, (uint16_t) -1 * rightPWM);
    } else if(leftPWM <= 0 && rightPWM >= 0) {
        Motor_Left((uint16_t) -1 * leftPWM, (uint16_t) rightPWM);
    } else {
        Motor_Backward((uint16_t) -1 * leftPWM, (uint16_t) -1 * rightPWM);
     }
}

void set_velocity(float LeftTarget, float RightTarget) {
    LeftTargetVelocity = LeftTarget;
    RightTargetVelocity = RightTarget;

    drive_motors(LeftPWM, RightPWM);
}
