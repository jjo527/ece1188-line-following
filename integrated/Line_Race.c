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

State_t fsm[11]= {
    {0x01,   &Motor_Forward  , 7000, 7000,  500, 20, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, CENTER}},     // S1 CENTER      red
    {0x00,   &Motor_Forward  , 2500, 3000,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_LEFT}},      // S2 CLOSE_LEFT  green
    {0x02,   &Motor_Forward  , 1000, 2000,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_LEFT}},      // S2 MID_LEFT  green
    {0x03,   &Motor_Left     , 5000, 5000,  500, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_LEFT}},      // S3 FAR_LEFT    yellow
    {0x00,   &Motor_Forward  , 3000, 2500,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT}},      // S4 CLOSE_RIGHT blue
    {0x04,   &Motor_Forward  , 2000, 1000,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT}},      // S4 CLOSE_RIGHT blue
    {0x05,   &Motor_Right    , 5000, 5000,  500, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT}},      // S5 FAR_RIGHT   pink
    {0x07,   &Motor_Left     , 3000, 3000, 1000, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT}},     // S6 LostL       white
    {0x07,   &Motor_Right    , 3000, 3000, 1000, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT}},     // S7 LostR       white
    {0x07,   &Motor_Backward , 400, 400, 2000, 0,    { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT}},     // S8 Lost Go     white
    {0x07,   &Motor_Stop_Pars,    0,    0,  500, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, FAR_RIGHT }}                        // S9 Lost Stop   white
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

    if(g_LineResult&0x10 && g_LineResult&0x08){ // center
        return center;
    }else if(g_LineResult&0x20){ // L1
        return L1;
    }else if(g_LineResult&0x04){ // R1
        return R1;
    }else if(g_LineResult&0x40){ // L2
        return L2;
    }else if(g_LineResult&0x02){ // R2
        return R2;
    }else if(g_LineResult&0x80){ // L3
        return L3;
    }else if(g_LineResult&0x01){ // R3
        return R3;
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
    LaunchPad_Init();
    Reflectance_Init();
    Motor_Init();
    EnableInterrupts();

    State_t *Spt;  // pointer to the current state
    Spt = CENTER;

    int temp;

    while(1){
       (*Spt->motorFunction)(Spt->motorSpeed_L+speed_gain, Spt->motorSpeed_R+speed_gain);

        // Update Debug RGB Output
        Port2_Output(Spt->out);

        temp = FSM_Input();
        Spt = Spt->next[temp];
    }
}
