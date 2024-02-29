// MotorSimple.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.
// Student starter code for Lab 12, uses Systick software delay to create PWM
// Daniel Valvano
// July 11, 2019

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

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTick.h"
#include "../inc/Bump.h"

// *******Lab 12 solution*******


void SysTick_Wait1us(uint32_t delay){
    //48 MHz clock
    //clk period= 0.02us

    uint32_t i,real_delay;
    real_delay = delay*48;
    for(i=0; i<real_delay; i++){
        if(real_delay <= 1){return;}
        SysTick->LOAD = (real_delay - 1);// count down to zero
        SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
        while(( SysTick->CTRL&0x00010000) == 0){};
      }
}


void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs

  // write this as part of Lab 12

    SysTick_Init();
    //port5
     P5->SEL0 &= ~0x30;     //5.4L,5.5R out
     P5->SEL1 &= ~0x30; //PH
     P5->DIR  |=  0x30;
     P5->DS   |=  0x30;
     P5->OUT  &= ~0x30;
    //port2
     P2->SEL0 &= ~0xC0;     //2.6L,2.7 out L:????????????????????????????????????????????????????
     P2->SEL1 &= ~0xC0; //EN
     P2->DIR  |=  0xC0;
     P2->DS   |=  0xC0;
     P2->OUT  &= ~0xC0;
    //port3
     P3->SEL0 &= ~0xC0;     //3.7L,3.6R out
     P3->SEL1 &= ~0xC0; //nsleep
     P3->DIR  |=  0xC0;
     P3->DS   |=  0xC0;
     P3->OUT  &= ~0xC0;

}


void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P1->OUT &= ~0xC0;
  P2->OUT &= ~0xC0;   // off
  P3->OUT &= ~0xC0;   // low current sleep mode
}





void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

  // write this as part of Lab 12
  //left
  //5.4=0; 2.7=1
  //right
  //5.5=0; 2.6=1

    uint32_t i=0,j=0;

    P5->OUT &= ~0x30;//PH=0
    P2->OUT = (P2->OUT&0xF8)|0xC0;//EN=1

    for(i=0; i<time; i++){
        for(j=0; j<duty;j++)
        {
        P3->OUT = (P3->OUT&0xF8)|0xC0;//nsleep=1
        //SysTick_Wait1us(duty);
        }
        for(j=0; j<duty;j++)
        {
        P3->OUT &= ~0xC0;//nsleep=0
        //SysTick_Wait1us(10000-duty);
        }
    }
}


void Motor_BackwardSimple(uint16_t duty, uint32_t time){
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms

    // write this as part of Lab 12
    //left
    //5.4=1; 2.7=1
    //right
    //5.5=1; 2.6=1

      uint32_t i=0,j=0;

      P5->OUT = (P5->OUT&0xF8)|0x30;//PH=0
      P2->OUT = (P2->OUT&0xF8)|0xC0;//EN=1

      for(i=0; i<time; i++){
          for(j=0; j<duty;j++)
          {
          P3->OUT = (P3->OUT&0xF8)|0xC0;//nsleep=1
          //SysTick_Wait1us(duty);
          }
          for(j=0; j<duty;j++)
          {
          P3->OUT &= ~0xC0;//nsleep=0
          //SysTick_Wait1us(10000-duty);
          }
      }
}


void Motor_LeftSimple(uint16_t duty, uint32_t time){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    //left
    //5.4=0; 2.7=1
    //right
    //5.5=x; 2.6=0

    uint32_t i=0,j=0;

    P5->OUT &= ~0x30;//PH=0
    P2->OUT = (P2->OUT&0xF8)|0x80;//EN=1

    for(i=0; i<time; i++){
        for(j=0; j<duty;j++)
        {
        P3->OUT = (P3->OUT&0xF8)|0xC0;//nsleep=1
        //SysTick_Wait1us(duty);
        }
        for(j=0; j<duty;j++)
        {
        P3->OUT &= ~0xC0;//nsleep=0
        //SysTick_Wait1us(10000-duty);
        }
    }
}


void Motor_RightSimple(uint16_t duty, uint32_t time){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    //left
    //5.4=x; 2.7=1
    //right
    //5.5=0; 2.6=0

    uint32_t i=0,j=0;

    P5->OUT &= ~0x30;//PH=0
    P2->OUT = (P2->OUT&0xF8)|0x40;//EN=1

    for(i=0; i<time; i++){
        for(j=0; j<duty;j++)
        {
        P3->OUT = (P3->OUT&0xF8)|0xC0;//nsleep=1
        //SysTick_Wait1us(duty);
        }
        for(j=0; j<duty;j++)
        {
        P3->OUT &= ~0xC0;//nsleep=0
        //SysTick_Wait1us(10000-duty);
        }
    }
}
