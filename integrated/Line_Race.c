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
    {0x01,   &Motor_Forward  , 6000, 6000,  500, 20, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostGo}},     // S1 CENTER      red
    {0x02,   &Motor_Forward  , 5250, 6000,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostL}},      // S2 CLOSE_LEFT  green
    {0x02,   &Motor_Forward  , 4250, 6000,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostL}},      // S2 MID_LEFT  green
    {0x03,   &Motor_Left     , 3000, 6000,  500, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostL}},      // S3 FAR_LEFT    yellow
    {0x04,   &Motor_Forward  , 6000, 5250,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostR}},      // S4 CLOSE_RIGHT blue
    {0x04,   &Motor_Forward  , 6000, 4250,  500, 10, { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostR}},      // S4 CLOSE_RIGHT blue
    {0x05,   &Motor_Right    , 6000, 3000,  500, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostR}},      // S5 FAR_RIGHT   pink
    {0x07,   &Motor_Left     , 3000, 6000, 1000, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostGo}},     // S6 LostL       white
    {0x07,   &Motor_Right    , 6000, 3000, 1000, 0,  { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostGo}},     // S7 LostR       white
    {0x07,   &Motor_Backward , 400, 400, 2000, 0,    { FAR_LEFT, MID_LEFT, CLOSE_LEFT, CENTER, CLOSE_RIGHT, MID_RIGHT, FAR_RIGHT, LostStop}},   // S8 Lost Go     white
    {0x07,   &Motor_Stop_Pars,    0,    0,  500, 0,  { LostStop, LostStop, LostStop, LostStop, LostStop, LostStop, LostStop, LostStop }}                        // S9 Lost Stop   white
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

    if(g_LineResult&18){ // center
        return 3;
    }else if(g_LineResult&20){ // L1
        return 2;
    }else if(g_LineResult&04){ // R1
        return 4;
    }else if(g_LineResult&40){ // L2
        return 1;
    }else if(g_LineResult&02){ // R2
        return 5;
    }else if(g_LineResult&80){ // L3
        return 0;
    }else if(g_LineResult&01){ // R3
        return 6;
    }else{
        return 7;
    }




//    // Checking Center
//    // 7654 33210
//    // XXX1 1XXX
//    if (g_LineResult&0x18) {
//        return 2;
//    }
//    // Checking Far Left
//    // 7654 33210
//    // 11XX XXXX
//    else if (g_LineResult&0xC0) {
//        speed_gain = 0;
//        return 0;
//    }
//    // Checking Far Right
//    // 7654 3210
//    // XXXX XX11
//    else if (g_LineResult&0x03) {
//        speed_gain = 0;
//        return 4;
//    }
//    // Checking Close Left
//    // 7654 3210
//    // XX11 XXXX
//    else if (g_LineResult&0x30) {
//        return 1;
//    }
//    // Checking Close Right
//    // 7654 3210
//    // XXXX 11XX
//    else if (g_LineResult&0x0C) {
//        return 3;
//    }
//    else {
//        return 5;
//    }
}

void SysTick_Handler(void) {
    if (g_count % 10 == 0) {
        Reflectance_Start();
        g_count +=1;
    }else if (g_count % (10 + g_delay_systick) == 0) {
        g_LineResult = Reflectance_End();
        //g_BumpResult = Bump_Read();
        //Led_Refl();//keep one commented out
        //Led_Bump();
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
//        (*Spt->motorFunction)(Spt->motorSpeed_L+speed_gain, Spt->motorSpeed_R+speed_gain);

//        if (g_count%1000 == 0){
//            speed_gain += Spt->gain;
//        }

        // Update Debug RGB Output
        Port2_Output(Spt->out);

        temp = FSM_Input();
        Spt = Spt->next[temp];
    }
}
