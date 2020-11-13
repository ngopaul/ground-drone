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
#include "Bump.h"
#include "Blinker.h"
#include "Clock.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "Reflectance.h"
#include "SysTickInts.h"
#include "Tachometer.h"
#include "UART0.h"

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
    int i;
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
        if (UART_command_received){
            if (!strcmp(buffer, "OFF")) {
                state = OFF;
            } else if (!strcmp(buffer, "DRIVE_STRAIGHT")) {
                state = DRIVE_STRAIGHT;
            } else if (!strcmp(buffer, "TURN_90_RIGHT")) {
                state = TURN_90_RIGHT;
            } else if (!strcmp(buffer, "TURN_90_LEFT")) {
                state = TURN_90_LEFT;
            }
        }
        PrintBump();
        switch (state) {
        case OFF:
            Motor_Stop();
            LaunchPad_Output(RED);
            break;
        case DRIVE_STRAIGHT:
            Motor_Forward(3000, 3000);
            LaunchPad_Output(BLUE);
            break;
        case TURN_90_RIGHT:
            Blinker_Output(BK_RGHT);
            Motor_Right(3000, 3000);
            break;
        case TURN_90_LEFT:
            Blinker_Output(BK_LEFT);
            Motor_Left(3000, 3000);
            break;
        }
        Clock_Delay1ms(62);

//        Motor_Stop();
//        printf("\n\rTI-RSLK MAX\n\r");
//        printf("Press bump sensor to go\n\r");
//        i = 0;
//        LaunchPad_Output(RED);
//        Blinker_Output(BK_RGHT+BK_LEFT+FR_RGHT+FR_LEFT);
//        while(Bump_Read() != 0){
//           Clock_Delay1ms(10);    // delay ~0.01 sec at 48 MHz
//        }
//        while(Bump_Read() == 0){   // flash the blue LED while not touched
//          LaunchPad_Output(BLUE);
//          Blinker_Output(BK_RGHT+BK_LEFT);
//          Clock_Delay1ms(100);     // delay ~0.1 sec at 48 MHz
//          LaunchPad_Output(0x00);
//          Blinker_Output(0);
//          Clock_Delay1ms(100);     // delay ~0.1 sec at 48 MHz
//          i++;
//          if((i%10)==0){ // slow down output
//            int32_t tenths = Position%10;
//            if(tenths<0)tenths = -tenths; // absolute value
//            int32_t ones = Position/10;
//            printf("Line sensor = %d.%1d mm\n\r",ones,tenths);
//          }
//        }
//        printf("Release bump sensor\n\r");
//        while(Bump_Read() != 0){   // flash the red LED while touched
//           PrintBump();
//           LaunchPad_Output(RED);
//           Blinker_Output(BK_RGHT+BK_LEFT+FR_RGHT+FR_LEFT);
//           Clock_Delay1ms(500);    // delay ~0.5 sec at 48 MHz
//           LaunchPad_Output(0x00);
//           Blinker_Output(0);
//           Clock_Delay1ms(500);    // delay ~0.5 sec at 48 MHz
//        }
//        printf("Ready, set, ...");
//        for(i=5; i; i=i-1){    // flash the yellow LED
//          LaunchPad_Output(RED+GREEN);       // red LED is in bit 0, green LED is in bit 1
//          Blinker_Output(FR_RGHT+FR_LEFT);
//          Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
//          LaunchPad_Output(0x00);
//          LaunchPad_Output(0x00);
//          Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
//        }
//        printf(" Forward both duty cycles are 3000/15000\n\r");
//        Blinker_Output(FR_RGHT+FR_LEFT);
//        i = 0;
//        while(Bump_Read() == 0){// repeat the control code until the robot hits the wall
//          LaunchPad_Output(GREEN);           // green LED is in bit 1
//          Motor_Forward(3000, 3000);
//          i++;
//          if((i%25)==0){ // slow down output
//            Tachometer_Get(&LeftTach, &LeftDir, &LeftSteps, &RightTach, &RightDir, &RightSteps);
//          // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
//            LeftSpeed = 2000000/LeftTach;
//            RightSpeed = 2000000/RightTach;
//            printf("Tachometer speed Left=%d, Right=%d rps\n\r",LeftSpeed,RightSpeed);
//          }
//          Clock_Delay1ms(62);              // delay ~0.062 sec at 48 MHz
//        }
    }
}
 /*test code
// Voltage current and speed as a function of duty cycle
#include "../inc/SSD1306.h"
int main(void){
  uint32_t duty;
  Clock_Init48MHz();
  LaunchPad_Init();   // built-in switches and LEDs
  Tachometer_Init();
  Blinker_Init();       // Blinker LED
  Bump_Init();        // bump switches
  Motor_Init(); // initialization
  SysTick_Init(48000,3); // set up SysTick for 1kHz interrupts
  Blinker_Output(FR_RGHT+FR_LEFT+BK_RGHT+BK_LEFT); LaunchPad_LED(1);LaunchPad_Output(0x07);
  EnableInterrupts();    // SysTick is priority 3
  for(int i=0;i<2;i++){
    for(duty=3000; duty<=14999; duty=duty+2999){
      while(Bump_Read()==0);  // wait for touch
      Motor_Stop();   // measure current
      Clock_Delay1ms(100);
      while(Bump_Read());     // wait for release
      Motor_Forward(duty,duty);  // measure current
      Clock_Delay1ms(100);
    }
  }
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_DrawString(0,16,"123456789012345678901",WHITE);
  SSD1306_DrawString(0,24,"abcdefghijklmnopqrstu",WHITE);
  SSD1306_DrawString(0,32,"ABCDEFGHIJKLMNOPQRSTU",WHITE);
  SSD1306_DrawString(0,40,"!@#$%^&*()_+-=",WHITE);
  SSD1306_DrawString(0,48,"123456789012345678901",WHITE);
  SSD1306_DrawString(0,56,"vwxyz,./<>?;'\"[]\\{}|",WHITE);
  SSD1306_DisplayBuffer();
  for(int i=0;i<2;i++){
    for(duty=3000; duty<=14999; duty=duty+2999){
      while(Bump_Read()==0);  // wait for touch
      Motor_Stop();   // measure current
      Clock_Delay1ms(100);
      while(Bump_Read());     // wait for release
      SSD1306_SetCursor(0,0);
      SSD1306_OutString("duty =");
      SSD1306_OutUDec(duty);
      SSD1306_OutChar(CR);
      Motor_Forward(duty,duty);  // measure current
      Clock_Delay1ms(100);
    }
  }
  while(1){};
}
*/
