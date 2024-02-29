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

typedef struct State {
    uint8_t  out;                              // LED output to mark state (debug)
    void (*motorFunction)(uint16_t, uint16_t); // reference to desired motor function call
    uint16_t motorSpeed_L;                     // speed of Left motor
    uint16_t motorSpeed_R;                     // speed of Right motor
    uint32_t time_length;                      // time motors are engaged
    const struct State *next[4];               // next if 2-bit input is 0-3
} State_t;

enum FsmInput {
    FAR_LEFT,
    CLOSE_LEFT,
    MIDDLE,
    CLOSE_RIGHT,
    FAR_RIGHT,
    LOST
};


// ---------------------------------------------------
// FSM Initialization

#define center      &fsm[0] // S1
#define L1          &fsm[1] // S2
#define L2          &fsm[2] // S3
#define R1          &fsm[3] // S4
#define R2          &fsm[4] // S5
#define LostL       &fsm[5] // S6
#define LostR       &fsm[6] // S7
#define LostGo      &fsm[7] // S8
#define LostStop    &fsm[8] // S9

State_t fsm[9]={
    {0x01,   &Motor_Forward, 5000, 5000,  500, { center, L1, R1, center }},                   // S1 C         red
    {0x02,      &Motor_Left, 3000, 1000,  500, { LostL, L2, R1, center }},                    // S2 L1        green
    {0x02,      &Motor_Left, 3000, 3000,  500, { LostL, L1, R1, center }},                    // S3 L2        green
    {0x03,     &Motor_Right, 1000, 3000,  500, { LostR, L1, R2, center }},                    // S4 R1        yellow
    {0x04,     &Motor_Right, 3000, 3000,  500, { LostR, L1, R1, center }},                    // S5 R2        blue
    {0x05,      &Motor_Left, 1000, 5000, 1000, { LostGo, L1, R1, center }},                   // S6 LostL     pink
    {0x06,     &Motor_Right, 5000, 1000, 1000, { LostGo, L1, R1, center }},                   // S7 LostR     sky blue
    {0x07,  &Motor_Backward, 4000, 4000, 2000, { LostStop, L1, R1, center }},                 // S8 Lost Go   white
    {0x00, &Motor_Stop_Pars,    0,    0,  500, { LostStop, LostStop, LostStop, LostStop }}    // S9 Lost Stop red
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
    // Checking Far Left
    // 7654 33210
    // 11XX XXXX
    if (g_LineResult&0xC0) {
        return FAR_LEFT;
    }
    // Checking Far Right
    // 7654 3210
    // XXXX XX11
    else if (g_LineResult&0x03) {
        return FAR_RIGHT;
    }
    // Checking Close Left
    // 7654 3210
    // XX11 XXXX
    else if (g_LineResult&0x30) {
        return CLOSE_LEFT;
    }
    // Checking Close Right
    // 7654 3210
    // XXXX 11XX
    else if (g_LineResult&0x0C0) {
        return CLOSE_RIGHT;
    }
    else {
        return LOST;
    }
}

void SysTick_Handler(void) {
    if (g_count % 10 == 0) {
        Reflectance_Start();
        g_count +=1;
    }
    else if (g_count % (10 + g_delay_systick) == 0) {
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
    //Bump_Init();

    //adjust motor speeds in testing

    State_t *Spt;  // pointer to the current state
    //int Input;
    //uint32_t Output;
    //uint32_t heart=1;
    Spt = center;


    while(1){
        (*Spt->motorFunction)(Spt->motorSpeed_L, Spt->motorSpeed_R);

        // Update Debug RGB Output
        Port2_Output(Spt->out);

        // Delay for testing
        Clock_Delay1ms(3000);

        Spt = Spt->next[FSM_Input()];
    }
}
