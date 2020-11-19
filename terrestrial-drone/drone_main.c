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
#include "..\..\tirslk_max_1_00_00\inc\Bump.h"
#include "..\..\tirslk_max_1_00_00\inc\Blinker.h"
#include "..\..\tirslk_max_1_00_00\inc\Clock.h"
#include "..\..\tirslk_max_1_00_00\inc\CortexM.h"
#include "..\..\tirslk_max_1_00_00\inc\LaunchPad.h"
#include "..\..\tirslk_max_1_00_00\inc\Motor.h"
#include "..\..\tirslk_max_1_00_00\inc\Reflectance.h"
#include "..\..\tirslk_max_1_00_00\inc\SysTickInts.h"
#include "..\..\tirslk_max_1_00_00\inc\Tachometer.h"
#include "..\..\tirslk_max_1_00_00\inc\UART0.h"
#include "..\..\tirslk_max_1_00_00\inc\UART1.h"

typedef enum robot_state_t {
    OFF,
    DRIVE_STRAIGHT,
    TURN_90_RIGHT,
    TURN_90_LEFT,
} robot_state_t;


uint16_t LeftSpeed;            // rotations per minute
uint16_t RightSpeed;           // rotations per minute
uint16_t LeftTach;             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;    // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;             // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach;            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;   // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;            // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
robot_state_t state = OFF;
char buffer[50];
char response[2] = "R\0";
char ch;
bool UART_command_received = false;

volatile uint32_t Time;    // ms
volatile uint8_t Data;     // QTR-8RC
volatile int32_t Position; // -332 to +332 mm from line
void SysTick_Handler(void){ // every 1ms
  Time = Time + 1;
  if(Time%10==1){
    Reflectance_Start(); // start every 10ms
  }
  if(Time%10==2){
    Data = Reflectance_End(); // finish 1ms later
    Position = Reflectance_Position(Data);
  }
}
void PrintBump(void){int i;
  printf("Bump switches: ");
  uint32_t in = Bump_Read();
  uint32_t mask = 0x01;
  for(i=0;i<6;i++){
    if(mask&in) printf("%d ",i);
    mask = mask<<1;
  }
  printf("pressed\n\r");
}
// printf (to PC) used for debugging
void main(void){
    int i = 0;
    float temp = 0;
    DisableInterrupts();
    Clock_Init48MHz();    // set system clock to 48 MHz
    UART0_Initprintf();   // serial port channel to PC, with printf
    UART1_Init(); // init UART communication with RPi
    LaunchPad_Init();
    Bump_Init();
    Tachometer_Init();
    Blinker_Init();       // Blinker LED
    Motor_Init();
    Time = 0;
    Reflectance_Init();    // line sensor
    SysTick_Init(48000,3); // set up SysTick for 1kHz interrupts
    EnableInterrupts();    // SysTick is priority 3
    while(1) {
        UART_command_received = false;
        if (UART1_InStatus()) {
            uint8_t buf_pos = 0;
            printf("Collecting from UART: ");
            fflush(stdout);
            do {
                ch = UART1_InChar();
                printf("%c", ch);
                buffer[buf_pos] = ch;
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
            buffer[buf_pos] = '\0';
            printf("Chars received: %s\n", buffer);
            UART_command_received = true;
        }
        if (UART_command_received) {
            if (buffer[0] == 'F') {
                int face_right = atoi(buffer + 4);
                buffer[4] = '\0';
                int face_left = atoi(buffer + 1);
                if (abs((face_left + face_right) / 2 - 127) < 15) {
                    UART1_OutString(response);
                    state = OFF;
                } else if ((face_left + face_right) / 2 < 127) {
                    state = TURN_90_LEFT;
                    if (i <= 0) {
                        temp = sqrt(127 - (face_left + face_right) / 2) * (3);
                        printf("Value of turn: %f", temp);
                        i = temp;
                    }
                } else if ((face_left + face_right) / 2 > 127) {
                    state = TURN_90_RIGHT;
                    if (i <= 0) {
                        temp = sqrt((face_left + face_right) / 2 - 127) * (3);
                        printf("Value of turn: %f", temp);
                        i = temp;
                    }
                }
            }
        } else if (i <= 0) {
            state = OFF;
        }
        // PrintBump();
        switch (state) {
        case OFF:
            Motor_Stop();
            LaunchPad_Output(RED);
            break;
        case DRIVE_STRAIGHT:
            Motor_Forward(2500, 2500);
            LaunchPad_Output(BLUE);
            break;
        case TURN_90_RIGHT:
            Blinker_Output(BK_RGHT);
            Motor_Right(2500, 2500);
            break;
        case TURN_90_LEFT:
            Blinker_Output(BK_LEFT);
            Motor_Left(2500, 2500);
            break;
        }
        if (i > 0) {
            i--;
        }
        if (i <= 0) {
            UART1_OutString(response);
        }
        Clock_Delay1ms(10);
    }
}
