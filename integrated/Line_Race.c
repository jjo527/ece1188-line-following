#include <stdint.h>
#include "msp.h"
#include "../inc/clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Texas.h"
#include "../inc/Reflectance.h"
//#include "../inc/MotorSimple.h"
#include "../inc/Motor.h"

#include "../inc/CortexM.h"
#include "../inc/SysTickInts.h"

#include "../inc/bump.h"


void Led_Output(uint8_t data){  // write output to P1.0
  P1->OUT = data;
}

void Port2_Output(uint8_t data){  // write three outputs bits of P2
  P2->OUT = (P2->OUT&0xF8)|data;
}

uint8_t LineResult,BumpResult;
int FSM_Input(void){
    //Reflectance
    if (LineResult==0x00){ return 0; }//0x00
    else if (LineResult< 0x08){ return 1; }//xxxx,1___ ; too right go left
    else if (LineResult> 0x10){ return 2; }//___1,xxxx ; too left go right
    else                      { return 3; }//xxx1,1xxx ; center
}


int count=0,delay_systick=1;

void SysTick_Handler(void){
  if (count%10==0){
      Reflectance_Start();
      count +=1;
  }else if (count%(10+delay_systick)==0){
      LineResult = Reflectance_End();
      //BumpResult = Bump_Read();
      //Led_Refl();//keep one commented out
      //Led_Bump();
      count=0;
  }else {
      count +=1;
  }
}




// Linked data structure
typedef struct State {
  uint8_t  out;                // LED output to mark state (debug)
  void (*motorFunction)(uint16_t, uint16_t);
  uint16_t motorSpeed_L;       // speed of Left motor
  uint16_t motorSpeed_R;       // speed of Right motor
  uint32_t time_length;        // time motors are enganged
  const struct State *next[4]; // next if 2-bit input is 0-3

} State_t;

#define center      &fsm[0] // S1
#define L1          &fsm[1] // S2
#define L2          &fsm[2] // S3
#define R1          &fsm[3] // S4
#define R2          &fsm[4] // S5
#define LostL       &fsm[5] // S6
#define LostR       &fsm[6] // S7
#define LostGo      &fsm[7] // S8
#define LostStop    &fsm[8] // S9

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

State_t fsm[9]={
    {0x01,  &Motor_Forward, 5000, 5000,  500, { center, L1, R1, center }},                   // S1 C         red
    {0x02,     &Motor_Left, 3000, 1000,  500, { LostL, L2, R1, center }},                    // S2 L1        green
    {0x02,     &Motor_Left, 3000, 3000,  500, { LostL, L1, R1, center }},                    // S3 L2        green
    {0x03,    &Motor_Right, 1000, 3000,  500, { LostR, L1, R2, center }},                    // S4 R1        yellow
    {0x04,    &Motor_Right, 3000, 3000,  500, { LostR, L1, R1, center }},                    // S5 R2        blue
    {0x05,     &Motor_Left, 1000, 5000, 1000, { LostGo, L1, R1, center }},                   // S6 LostL     pink
    {0x06,    &Motor_Right, 5000, 1000, 1000, { LostGo, L1, R1, center }},                   // S7 LostR     sky blue
    {0x07, &Motor_Backward, 4000, 4000, 2000, { LostStop, L1, R1, center }},                 // S8 Lost Go   white
    {0x00,     &Motor_Stop,    0,    0,  500, { LostStop, LostStop, LostStop, LostStop }}    // S9 Lost Stop red
};


int main(void){
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
        //Output = Spt->out;
        Port2_Output(Spt->out);

        Clock_Delay1ms(3000);



        //Input = FSM_Input();
        Spt = Spt->next[FSM_Input()];
        //heart = heart^1;
        //Led_Output(heart); //debug
    }
}
