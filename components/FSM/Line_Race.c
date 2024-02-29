#include <stdint.h>
#include "msp.h"
#include "../inc/clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Texas.h"
#include "..\inc\Reflectance.h"
#include "..\inc\MotorSimple.h"


#include "../inc/CortexM.h"
#include "../inc/SysTickInts.h"

#include "..\inc\bump.h"


void Led_Output(uint8_t data){  // write output to P1.0
  P1->OUT = data;
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
  }
  else if (count%(10+delay_systick)==0){
      LineResult = Reflectance_End();
      //BumpResult = Bump_Read();
      //Led_Refl();//keep one commented out
      //Led_Bump();
      count=0;
  }
  else
  { count +=1; }




// Linked data structure
struct State {
  uint8_t  out;                // LED output to mark state (debug)
  uint16_t motorSpeed_L;       //speed of Left motor
  uint16_t motorSpeed_R;       //speed of Right motor
  uint32_t time_length;        //time motors are enganged
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define S1    &fsm[0]
#define S2    &fsm[1]
#define S3    &fsm[2]
#define S4    &fsm[3]
#define S5    &fsm[4]
#define S6    &fsm[5]
#define S7    &fsm[6]
#define S8    &fsm[7]
#define S9    &fsm[8]


int main(void){
    Clock_Init48MHz();
    SysTick_Init(48000,2);//64Hz
    LaunchPad_Init();
    Reflectance_Init();
    //Bump_Init();

    //adjust motor speeds in testing
    State_t fsm[9]={
        {0x01, 5000, 5000,  500, { S1, S2, S4, S1 }},  // S1 C
        {0x02, 3000, 1000,  500, { S6, S3, S4, S1 }},  // S2 L1
        {0x02, 3000, 3000,  500, { S6, S2, S4, S1 }},  // S3 L2
        {0x03, 1000, 3000,  500, { S7, S2, S5, S1 }},  // S4 R1
        {0x04, 3000, 3000,  500, { S7, S2, S4, S1 }},  // S5 R2
        {0x05, 1000, 5000, 1000, { S8, S2, S4, S1 }},  // S6 LostL
        {0x06, 5000, 1000, 1000, { S8, S2, S4, S1 }},  // S7 LostR
        {0x07, 4000, 4000, 2000, { S9, S2, S4, S1 }},  // S8 Lost Go
        {0x00,    0,    0,  500, { S9, S9, S9, S9 }}   // S9 Lost Stop
    };

    State_t *Spt;  // pointer to the current state
    //int Input;
    //uint32_t Output;
    //uint32_t heart=1;
    Spt = S1;


    while(1)
    {
        //Output = Spt->out;
        LaunchPad_LED(Spt->out);

        Motor_LeftSimple    (Spt->motorSpeed_L,Spt->time_length);
        Motor_RightSimple   (Spt->motorSpeed_R,Spt->time_length);

        //Input = FSM_Input();
        Spt = Spt->next[FSM_Input()];
        //heart = heart^1;
        //Led_Output(heart); //debug
    }
}

// Color    LED(s) Port2
// dark     --- 0
// red      R-- 0x01
// blue     --B 0x04
// green    -G- 0x02
// yellow   RG- 0x03
// sky blue -GB 0x06
// white    RGB 0x07
// pink     R-B 0x05
