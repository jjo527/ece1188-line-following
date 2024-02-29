// MotorSimple.c

#include <stdint.h>
#include "msp.h"
#include "../../inc/SysTick.h"
#include "../../inc/Bump.h"
#include "../../inc/Clock.h"

//**************RSLK1.1***************************
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)
// ****************Lab 12 solution****************

//void SysTick_Wait1us(uint32_t delay){
//    SysTick_Wait(48 * delay);
//}

void Motor_InitSimple(void){
    // Initializes the 6 GPIO lines and puts driver to sleep
    // Returns right away
    // initialize P5.4 and P5.5 and make them outputs

    // 2.6, 2.7 as GPIO
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;  // output bc why??
    P2->OUT &= ~0xC0; //sleeps P2 PWMs

    // 3.6, 3.7 as GPIO
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;  // output bc why??
    P3->OUT &= ~0xC0; // sleeps P3 enables

    // 5.4, 5.5 as GPIO
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30; // P5.4, P5.5 as output
}

void Motor_StopSimple(void){
    // Stops both motors, puts driver to sleep
    // Returns right away

    P1->OUT &= ~0xC0;
    P2->OUT &= ~0xC0;   // off
    P3->OUT &= ~0xC0;   // low current sleep mode
}

void Motor_ForwardSimple(uint16_t duty, uint32_t time){
    // P5.4(L) and P5.5(R) - direction
    // P2.6(R) and P2.7(L) - PWM
    // P3.6(R) and P3.7(L) - enable

    // Drives both motors forward at duty (100 to 9900)
    // Runs for time duration (units=10ms), and then stops

    P5->OUT &= ~0x30; // set direction to forward
    P3->OUT |= 0xC0;  // enables P3

    uint32_t H, L, i;
    H = duty;
    L = 10000 - H;

    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        Clock_Delay1us(H);
        P2->OUT &= ~0xC0;
        Clock_Delay1us(L);
    }

    Motor_StopSimple(); // turn off motors after the time given
}

void Motor_BackwardSimple(uint16_t duty, uint32_t time){
    // P5.4(L) and P5.5(R) - direction
    // P2.6(R) and P2.7(L) - PWM
    // P3.6(R) and P3.7(L) - enable

    // Drives both motors backward at duty (100 to 9900)
    // Runs for time duration (units=10ms), and then stops
    // Runs even if any bumper switch is active
    // Returns after time*10ms

    P5->OUT |= 0x30;  // set direction to forward
    P3->OUT |= 0xC0;  // enables P3

    uint32_t H, L, i;
    H = duty;
    L = 10000 - H;

    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        Clock_Delay1us(H);
        P2->OUT &= ~0xC0;
        Clock_Delay1us(L);
    }

    Motor_StopSimple(); // turn off motors after the time given
}

void Motor_LeftSimple(uint16_t duty, uint32_t time){
    // P5.4(L) and P5.5(R) - direction
    // P2.6(R) and P2.7(L) - PWM
    // P3.6(R) and P3.7(L) - enable

    // Drives just the left motor forward at duty (100 to 9900)
    // Right motor is stopped (sleeping)
    // Runs for time duration (units=10ms), and then stops

    // 00XX 0000
    P5->OUT &= ~0x20; // enable right motor
    P5->OUT |= 0x10;  // disable left motor
    P3->OUT |= 0xC0;  // enables motors

    uint32_t H, L, i;
    H = duty;
    L = 10000 - H;

    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        Clock_Delay1us(H);
        P2->OUT &= ~0xC0;
        Clock_Delay1us(L);
    }

    Motor_StopSimple(); // turn off motors after the time given
}

void Motor_RightSimple(uint16_t duty, uint32_t time){
    // P5.4(L) and P5.5(R) - direction
    // P2.6(R) and P2.7(L) - PWM
    // P3.6(R) and P3.7(L) - enable

    // Drives just the right motor forward at duty (100 to 9900)
    // Left motor is stopped (sleeping)
    // Runs for time duration (units=10ms), and then stops

    // 00XX 0000
    P5->OUT &= ~0x10; // enable left motor
    P5->OUT |= 0x20;  // disable right motor
    P3->OUT |= 0xC0;  // enables motors

    uint32_t H, L, i;
    H = duty;
    L = 10000 - H;

    for(i = 0; i < time; i++){
        P2->OUT |= 0xC0;
        Clock_Delay1us(H);
        P2->OUT &= ~0xC0;
        Clock_Delay1us(L);
    }

    Motor_StopSimple(); // turn off motors after the time given
}
