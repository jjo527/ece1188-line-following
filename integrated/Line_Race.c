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

uint16_t speed_gain = 0;

typedef struct State {
    uint8_t  out;                              // LED output to mark state (debug)
    void (*motorFunction)(uint16_t, uint16_t); // reference to desired motor function call
    uint16_t motorSpeed_L;                     // speed of Left motor
    uint16_t motorSpeed_R;                     // speed of Right motor
    uint32_t time_length;                      // time motors are engaged
    uint16_t gain;
    const struct State *next[8];               // next if 2-bit input is 0-3
} State_t;

enum FsmInput {
    L3,
    L2,
    L1,
    center,
    R1,
    R2,
    R3,
    LOST
};


// ---------------------------------------------------
// FSM Initialization

#define CENTER      &fsm[0] // S1
#define CLOSE_LEFT  &fsm[1] // S2
#define MID_LEFT    &fsm[2] // S2
#define FAR_LEFT    &fsm[3] // S3
#define CLOSE_RIGHT &fsm[4] // S4
#define MID_RIGHT   &fsm[5] // S2
#define FAR_RIGHT   &fsm[6] // S5
#define LostL       &fsm[7] // S6
#define LostR       &fsm[8] // S7
#define LostGo      &fsm[9] // S8
#define LostStop    &fsm[10] // S9


#define MY_MULTIPLIER 4
#define MY_MULTIPLIER_SLIGHT 4
#define MY_MULTIPLIER_MED 4
#define MY_MULTIPLIER_BEND 4
#define MY_MULTIPLIER_LOST 4
#define BASE_SPEED 1000
#define SOFT_TURN 1100
#define MID_TURN 900
#define HARD_TURN 400
#define HARD_BACK 1000
#define BEND_SPEED 1500
#define LOST_BACK 500

State_t fsm[11]= {
    {0x01,   &Motor_Forward  , BASE_SPEED * MY_MULTIPLIER       , BASE_SPEED * MY_MULTIPLIER,  500, 20, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, CENTER}},     // S1 CENTER      red

    {0x02,   &Motor_Forward  , SOFT_TURN * MY_MULTIPLIER_SLIGHT, BASE_SPEED * MY_MULTIPLIER_SLIGHT,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostL}},      // S2 CLOSE_LEFT  green
    {0x03,   &Motor_Forward  , BASE_SPEED * MY_MULTIPLIER_SLIGHT   , SOFT_TURN * MY_MULTIPLIER_SLIGHT,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostL}},      // S2 MID_LEFT  green
    {0x04,   &Motor_Left  , BASE_SPEED * MY_MULTIPLIER_BEND  , BASE_SPEED * MY_MULTIPLIER_BEND, 0, 0, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostL}},      // S3 FAR_LEFT    yellow

    {0x02,   &Motor_Forward    , BASE_SPEED * MY_MULTIPLIER_SLIGHT, SOFT_TURN * MY_MULTIPLIER_SLIGHT,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostR}},      // S4 CLOSE_RIGHT blue
    {0x03,   &Motor_Forward    , SOFT_TURN * MY_MULTIPLIER_SLIGHT   , BASE_SPEED * MY_MULTIPLIER_SLIGHT,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostR}},      // S4 CLOSE_RIGHT blue
    {0x04,   &Motor_Right      , BASE_SPEED * MY_MULTIPLIER_BEND,  BASE_SPEED * MY_MULTIPLIER_BEND,  500, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostR}},      // S5 FAR_RIGHT   pink

    {0x07,   &Motor_Left , BEND_SPEED * MY_MULTIPLIER_LOST, BEND_SPEED * MY_MULTIPLIER_LOST,1000, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostL}},     // S6 LostL       white
    {0x07,   &Motor_Right     , BEND_SPEED * MY_MULTIPLIER_LOST, BEND_SPEED * MY_MULTIPLIER_LOST, 1000, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostR}},     // S7 LostR       white

    {0x07,   &Motor_Right    , 0001 * MY_MULTIPLIER, 0001 * MY_MULTIPLIER,  400, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT}},     // S8 Lost Go     white
    {0x07,   &Motor_Stop_Pars, 0001 * MY_MULTIPLIER, 0001 * MY_MULTIPLIER,  500, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT }}                        // S9 Lost Stop   white
};

// ---------------------------------------------------
// Other Global Variables

int g_count = 0;
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
    if(g_LineResult&0x08 && g_LineResult&0x01){ // center XXX1 1XXX
        return center;
    }else if(g_LineResult&0x20 || g_LineResult&0x10){ // L1 XX1X XXXX
        return L1;
    }else if(g_LineResult&0x04 || g_LineResult&0x10){ // R1 XXXX X1XX
        return R1;
    }else if(g_LineResult&0x80){ // L3 1XXX XXXX
        return L3; // 0
    }else if(g_LineResult&0x01){ // R3 XXXX XXX1
        return R3; // 6
    }else if(g_LineResult&0x40){ // L2 X1XX XXXX
        return L2; // 1
    }else if(g_LineResult&0x02){ // R2 XXXX XX1X
        return R2; // 5
    }else if(g_LineResult&0x20){ // L1 XX1X XXXX
        return center; // 2
    }else if(g_LineResult&0x04){ // R1 XXXX X1XX
        return center; // 4
    }else{
        return LOST;
    }
}

void SysTick_Handler(void) {
    if (g_count % 10 == 0) {
        Reflectance_Start();
        g_count +=1;
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
    SysTick_Init(48000,2);//64Hz
    BumpInt_Init(&HandleCollision);      // bump switches
    LaunchPad_Init();
    Reflectance_Init();
    Motor_Init();
    EnableInterrupts();

    State_t *Spt;  // pointer to the current state
    Spt = CENTER;

    int temp;

    while(1){
       (*Spt->motorFunction)(Spt->motorSpeed_L, Spt->motorSpeed_R);

        // Update Debug RGB Output
        Port2_Output(Spt->out);

        temp = FSM_Input();
        Spt = Spt->next[temp];
    }
}
