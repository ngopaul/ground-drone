// RSLK_MAX_main.c
// Runs on MSP432 with basic TI-RSLK MAX robot
// Starter project for robot competition
// Daniel and Jonathan Valvano
// June 8, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

// Blinker LEDS
// Front right P8.5 Yellow LED
// Front left  P8.0 Yellow LED
// Back right  P8.7 Red LED
// Back left   P8.6 Red LED

// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

// UCA0RXD connected to P1.2, use UART0_InChar UART0_InUDec UART0_InUHex or UART0_InString to receive from virtual serial port (VCP)
// scanf does not work
// UCA0TXD connected to P1.3, printf sent to PC via virtual serial port (VCP)

#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "../../tirslk_max_1_00_00/inc/Bump.h"
#include "../../tirslk_max_1_00_00/inc/Blinker.h"
#include "../../tirslk_max_1_00_00/inc/Clock.h"
#include "../../tirslk_max_1_00_00/inc/CortexM.h"
#include "../../tirslk_max_1_00_00/inc/LaunchPad.h"
#include "../../tirslk_max_1_00_00/inc/Motor.h"
#include "../../tirslk_max_1_00_00/inc/Reflectance.h"
#include "../../tirslk_max_1_00_00/inc/SysTickInts.h"
#include "../../tirslk_max_1_00_00/inc/Tachometer.h"
#include "../../tirslk_max_1_00_00/inc/UART0.h"
#include "../../tirslk_max_1_00_00/inc/UART1.h"

typedef enum robot_state_t {
    OFF,
    USER_DO,
    FIND_TARGET,
    MATCH_TARGET
} robot_state_t;

robot_state_t state = OFF;
char uart_command[50];
char last_uart_command[50];
uint32_t distance_to_target = 0; // 0 is unknown distance to target
bool UART_command_received = false;

volatile uint32_t Time;    // ms
volatile uint8_t Data;     // QTR-8RC
volatile int32_t Position; // -332 to +332 mm from line
void SysTick_Handler(void) { // every 1ms
  Time = Time + 1;
  if(Time%10==1){
    Reflectance_Start(); // start every 10ms
  }
  if(Time%10==2){
    Data = Reflectance_End(); // finish 1ms later
    Position = Reflectance_Position(Data);
  }
}

void PrintBump(void) {
    int i;
    printf("Bump switches: ");
    uint32_t in = Bump_Read();
    uint32_t mask = 0x01;
    for(i=0;i<6;i++){
        if(mask&in) printf("%d ",i);
        mask = mask<<1;
    }
    printf("pressed\n\r");
}

bool ReceiveUART(void) {
    char ch;
    if (UART1_InStatus()) {
        uint8_t buf_pos = 0;
        printf("Collecting from UART: ");
        fflush(stdout);
        do {
            ch = UART1_InChar();
            printf("%c", ch);
            uart_command[buf_pos] = ch;
            buf_pos ++;
        } while (ch != '\0' && buf_pos < 50);
        printf("\nFinished collecting.\n");
        // clear the UART buffer
        printf("Clearing: ");
        fflush(stdout);
        while (UART1_InStatus()) {
            ch = UART1_InChar();
            printf("%c", ch);
        }
        printf("\n");
        // make sure the string is terminated
        uart_command[buf_pos] = '\0';
        printf("Chars received: %s\n", uart_command);
        return true;
    }
    return false;
}

void UpdateStateAndCommand(void) {
    if (UART_command_received) {
        if (uart_command[0] == 'U') {
            state = USER_DO;
            strcpy(uart_command, last_uart_command);
        } else if (uart_command[0] == '?') {
            state = FIND_TARGET;
            distance_to_target = 0;
        } else if (uart_command[0] == 'F') {
            state = MATCH_TARGET;
        }
    }
}

// printf (to PC) used for debugging
void main(void){
    int i = 0;
    float temp = 0;
    DisableInterrupts();
    Clock_Init48MHz();    // set system clock to 48 MHz
    // UART0_Initprintf();   // serial port channel to PC, with printf
    UART1_Init(); // init UART communication with RPi
    LaunchPad_Init();
    Bump_Init();
    Blinker_Init();       // Blinker LED
    Motor_Init();
    Time = 0;
    Reflectance_Init();    // line sensor
    SysTick_Init(48000,3); // set up SysTick for 1kHz interrupts
    EnableInterrupts();    // SysTick is priority 3

    while(1) {
        // PrintBump();
        UART_command_received = ReceiveUART();
        switch (state) {
            case OFF:
                // Turn off the motors, make the servo point straight ahead.
                Motor_Stop();
                LaunchPad_Output(RED);
                UpdateStateAndCommand();
                break;
            case USER_DO:
                // Do the last uart command, which should have a U starting in it.
                int left_drive_val = 0;
                int right_drive_val = 0;
                /* the uart command string should be formatted like so:
                 * [0]: U
                 * [1]: 0 or 1, depending on if trying to drive forward
                 * [2]: 0 or 1, depending on if trying to drive backward
                 * [3]: 0 or 1, depending on if trying to drive left
                 * [4]: 0 or 1, depending on if trying to drive right
                */
                // parse the last uart command
                if (last_uart_command[0] == 'U' && strlen(last_uart_command) == 5) {
                    if (last_uart_command[1] == '1') {
                        left_drive_val = left_drive_val + 3000;
                        right_drive_val = right_drive_val + 3000;
                    }
                    if (last_uart_command[2] == '1') {
                        left_drive_val = left_drive_val - 3000;
                        right_drive_val = right_drive_val - 3000;
                    }
                    if (last_uart_command[3] == '1') {
                        left_drive_val = left_drive_val - 2000;
                        right_drive_val = right_drive_val + 2000;
                    }
                    if (last_uart_command[4] == '1') {
                        left_drive_val = left_drive_val + 2000;
                        right_drive_val = right_drive_val - 2000;
                    }
                }
                // Drive the motors
                if (left_drive_val > 0 && right_drive_val > 0) {
                    Motor_Forward((uint16_t) left_drive_val, (uint16_t) right_drive_val);
                } else if (left_drive_val > 0 && right_drive_val < 0) {
                    Motor_Left((uint16_t) left_drive_val, (uint16_t) -right_drive_val);
                } else if (left_drive_val < 0 && right_drive_val > 0) {
                    Motor_Right((uint16_t) -left_drive_val, (uint16_t) right_drive_val);
                } else if (left_drive_val < 0 && right_drive_val < 0) {
                    Motor_Backward((uint16_t) -left_drive_val, (uint16_t) -right_drive_val);
                }
                LaunchPad_Output(BLUE);
                UpdateStateAndCommand();
                break;
            case FIND_TARGET:
                // turn slowly until receiving a message which gives the location of the object
                Blinker_Output(BK_RGHT);
                Motor_Right(1500, 1500);
                UpdateStateAndCommand();
                break;
            case MATCH_TARGET:
                // get estimation of distance to target based on information in the uart message (do some math, or use the rangefinder)

                // move properly based on more math

                break;
        }
        Clock_Delay1ms(10); // necessary for receiving UART information
    }
}
