#include <stdint.h>

#include "msp.h"
#include "../inc/clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Texas.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/CortexM.h"
#include "../inc/SysTickInts.h"
#include "../inc/bump.h"
#include "../inc/BumpInt.h"

uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}

/// ---------------------------------------------------
// Color Reference
/*
 *  dark     --- 0x00
 *  red      R-- 0x01
 *  green    -G- 0x02
 *  yellow   RG- 0x03
 *  blue     --B 0x04
 *  pink     R-B 0x05
 *  sky blue -GB 0x06
 *  white    RBG 0x07
*/

// ---------------------------------------------------
//  Custom Types

uint16_t g_speed_gain = 0;

typedef struct State {
    uint8_t  out;                              // LED output to mark state (debug)
    void (*motorFunction)(uint16_t, uint16_t); // reference to desired motor function call
    uint16_t motorSpeed_L;                     // speed of Left motor
    uint16_t motorSpeed_R;                     // speed of Right motor
    const struct State *next[10];               // next if 2-bit input is 0-3
} State_t;

enum FsmInput {
    I_L4,
    I_L3,
    I_L2,
    I_L1,
    I_C,
    I_R1,
    I_R2,
    I_R3,
    I_R4,
    LOST
};


// ---------------------------------------------------
// FSM Initialization
#define FSM_L4  &fsm[0]
#define FSM_L3  &fsm[1]
#define FSM_L2  &fsm[2]
#define FSM_L1  &fsm[3]

#define FSM_C   &fsm[4]

#define FSM_R1  &fsm[5]
#define FSM_R2  &fsm[6]
#define FSM_R3  &fsm[7]
#define FSM_R4  &fsm[8]

#define FSM_LOST  &fsm[9]

// INPUTS
//  77 66 55 44 | 43 | 33 22 11 00
//  HL SL GO SR | GO | SL GO SR HR
//  << <^ ^^ ^> | ^^ | <^ ^^ ^> >>

#define BASE_SPEED 2500
#define TURN_3 1.15 * BASE_SPEED
#define TURN_2 1.1 * BASE_SPEED
#define TURN_1 1.15 * BASE_SPEED
#define TURN_SPEED 2500
#define MULT 2

State_t fsm[10]= {
    {0x01,   &Motor_Left     , TURN_SPEED , TURN_SPEED , { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_L4}},
    {0x02,   &Motor_Forward  , BASE_SPEED, TURN_3 , { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_L4}},
    {0x03,   &Motor_Forward  , TURN_2 , BASE_SPEED , { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_L4}},
    {0x04,   &Motor_Forward  , TURN_1 , BASE_SPEED, { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_LOST, }},

    {0x05,   &Motor_Forward  , BASE_SPEED , BASE_SPEED , { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_L4}},

    {0x04,   &Motor_Forward  ,BASE_SPEED, TURN_1,  { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_LOST}},
    {0x03,   &Motor_Forward  , BASE_SPEED, TURN_2, { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_R4}},
    {0x02,   &Motor_Forward  , TURN_3, BASE_SPEED, { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_R4}},
    {0x01,   &Motor_Right    , TURN_SPEED , TURN_SPEED , { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_R4}},

    {0x01,   &Motor_Backward    , BASE_SPEED , BASE_SPEED , { FSM_L4, FSM_L3, FSM_L2, FSM_L1, FSM_C, FSM_R1, FSM_R2, FSM_R3, FSM_R4, FSM_LOST}},
};

// ---------------------------------------------------
// Other Global Variables

uint64_t g_count = 0;
uint64_t g_count2 = 0;
uint64_t g_count3 = 0;
int g_gain_count =0;
int g_delay_systick = 1;
uint8_t g_LineResult;
uint8_t g_BumpResult;

// ---------------------------------------------------
// Functions

// write output to P1.0
void Led_Output(uint8_t data) {
    P1->OUT = data;
}

// write three outputs bits of P2
void Port2_Output(uint8_t data) {
    P2->OUT = (P2->OUT&0xF8) | data;
}

int FSM_Input(void){
    // if (g_LineResult&0x10 && g_LineResult&0x08) { // center XXX1 1XXX
        // return I_C;
    if(g_LineResult&0x10) { // L1 XXX1 XXXX
        return I_L1;
    } else if(g_LineResult&0x08) { // R1 XXXX 1XXX
        return I_R1;
    } else if(g_LineResult&0x20) { // L2 XX1X XXXX
        return I_L2;
    } else if(g_LineResult&0x04) { // R2 XXXX X1XX
        return I_R2;
    } else if(g_LineResult&0x40) { // L3 X1XX XXXX
        return I_L3;
    } else if(g_LineResult&0x02) { // R3 XXXX XX1X
        return I_R3;
    } else if(g_LineResult&0x80) { // L4 1XXX XXXX
        g_speed_gain = 0;
        g_gain_count = 0;
        g_count2 = 0;
        g_count3 = 0;
        return I_L4;
    } else if(g_LineResult&0x01) { // R4 XXXX XXX1
        g_speed_gain = 0;
        g_gain_count = 0;
        g_count2 = 0;
        g_count3 = 0;
        return I_R4;
    } else {
        return LOST;
    }
}

void SysTick_Handler(void) {
    if(g_count == 0) {
        if (g_speed_gain < 900 && g_count2 % 1 == 0 & g_count2 != 0) {
            g_speed_gain += 1 * g_count3;
            g_count3++;
        }
        g_count2++;
    }


    if (g_count % 10 == 0) {
        Reflectance_Start();
        g_count +=1;;
    }else if (g_count % (10 + g_delay_systick) == 0) {
        g_LineResult = Reflectance_End();
        g_count = 0;
    }
    else {
        g_count += 1;
    }
}

// ---------------------------------------------------
int main(void) {
    Clock_Init48MHz();
    SysTick_Init(45000,2);//64Hz
    BumpInt_Init(&HandleCollision);      // bump switches
    LaunchPad_Init();
    Reflectance_Init();
    Motor_Init();
    EnableInterrupts();

    g_speed_gain = 0;
    g_count2 = 0;
    g_count3 = 0;

    State_t *Spt;  // pointer to the current state
    Spt = FSM_C;

    int temp;

    while(1){
       (*Spt->motorFunction)(Spt->motorSpeed_L + g_speed_gain, Spt->motorSpeed_R + g_speed_gain);

        // Update Debug RGB Output
        Port2_Output(Spt->out);

        temp = FSM_Input();
        Spt = Spt->next[temp];
    }
}
