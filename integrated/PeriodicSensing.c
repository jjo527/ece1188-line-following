#include <stdbool.h>
#include <stdint.h>
#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\TExaS.h"


void Update_RGB_LED(char color) {
    P2->OUT &=  ~0x07;
    P1->OUT &=  ~0x01;


    switch (color) {
        case 'd': // dark
            P2->OUT |=  0x00;
            break;
        case 'r': // red
            P2->OUT |=  0x01;
            break;
        case 'b': // blue
            P2->OUT |=  0x04;
            break;
        case 'g': // green
            P2->OUT |=  0x02;
            break;
        case 'y': // yellow
            P2->OUT |=  0x03;
            break;
        case 's': // sky blue
            P2->OUT |=  0x06;
            break;
        case 'w': // white
            P2->OUT |=  0x07;
            break;
        case 'p': // pink
            P2->OUT |=  0x05;
            break;
        case 'o': // p1 led ON (different led)
            P1->OUT |=  0x01;
            break;
        default:
            break;
    }
}

bool input_p1_1() {
    int val = P1->IN&0x02;
    if (val == 0) {
        return true;
    }

    return false;
}

bool input_p1_4() {
    int val = P1->IN&0x10;
    if (val == 0) {
        return true;
    }

    return false;
}

void toggle_p1_0() {
    P1->OUT ^= 0x01;
}

void toggle_p2_2() {
    P2->OUT ^= 0x04;
}


volatile uint32_t Time,MainCount;
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
uint32_t High=120000,Low=360000;
uint32_t cycle_count, total_count;

uint8_t Data; // QTRX
int32_t Position;

bool SemaphoreActive;
void SysTick_Handler(void){ // PWM
  cycle_count++;
  total_count++;

  if(cycle_count == 1) {
      Reflectance_Start();
  }

  if(total_count % 100 == 0) {
           SemaphoreActive = false;
   }


  if(cycle_count == 10) {
      cycle_count = 0;

      if(!SemaphoreActive) {
          Data =  Reflectance_End();

    if (Data) {
        SemaphoreActive = true;
    }

    switch (Data) {
        case 0x01:
            Update_RGB_LED('r');
            break;
        case 0x02:
            Update_RGB_LED('b');
            break;
        case 0x04:
            Update_RGB_LED('g');
            break;
        case 0x08:
            Update_RGB_LED('y');
            break;
        case 0x10:
            Update_RGB_LED('s');
            break;
        case 0x20:
            Update_RGB_LED('w');
            break;
        case 0x40:
            Update_RGB_LED('p');
            break;
        case 0x80:
            Update_RGB_LED('o');
            break;
        default:
            break;
    }

    SemaphoreActive = false;
      }
  }

  // bump check
  uint8_t bumpData = Bump_Read();
  if(bumpData < 63 && !SemaphoreActive) {
        SemaphoreActive = true;

        switch (bumpData) {
            case 0x1F: // bit 5 low
                Update_RGB_LED('r');
                break;
            case 0x2F: // bit 4 low
                Update_RGB_LED('b');
                break;
            case 0x37: // bit 3 low
                Update_RGB_LED('g');
                break;
            case 0x3B: // bit 2 low
                Update_RGB_LED('y');
                break;
            case 0x3D: // bit 1 low
                Update_RGB_LED('s');
                break;
            case 0x3E: // bit 0 low
                Update_RGB_LED('w');
                break;
            default:
                break;
        }
    }
}

void GPIO_Init() {
    // make P2.0, P2.1, P2.2 outputs
    // 7654 3210
    // 0000 0111
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR  |=  0x07;
    P2->DS |= 0x07;       // 3) activate increased drive strength
    P2->OUT &=  ~0x07;

    // make P1.0 outputs
    // 7654 3210
    // 0000 0001
    P1->SEL0 &= ~0x01;
    P1->SEL1 &= ~0x01;
    P1->DIR  |=  0x01;
    P1->OUT &=  ~0x01;

    // Initialize input buttons P1.1 (dimes) and P1.4 (nickles)
    // 7654 3210
    // 0001 0010
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;    // 1) configure P1.4 and P1.1 as GPIO
    P1->DIR &= ~0x12;     // 2) make P1.4 and P1.1 in
    P1->REN |= 0x12;      // 3) enable pull resistors on P1.4 and P1.1
    P1->OUT |= 0x12;      //    P1.4 and P1.1 are pull-up
}

//void main() {
//    GPIO_Init();
//
//    Time = MainCount = 0;
//
//    Clock_Init48MHz();
//    Reflectance_Init(); // your initialization
//    TExaS_Init(LOGICANALYZER_P7);
//    SysTick_Init(4800, 2);
//    EnableInterrupts();
//
//    cycle_count = 0;
//    total_count = 0;
//    SemaphoreActive = false;
//
//    Bump_Init();
//
//    while(1)
//    {
//        uint32_t i, j, temp, length;
//        uint32_t a[100]={5000,5308,5614,5918,6219,6514,
//        6804,7086,7361,7626,7880,8123,8354,8572,8776,8964,9137,
//        9294,9434,9556,9660,9746,9813,9861,9890,9900,9890,9861,
//        9813,9746,9660,9556,9434,9294,9137,8964,8776,8572,8354,
//        8123,7880,7626, 7361,7086,6804,6514,6219,5918,5614,
//        5308,5000,4692,4386,4082,3781,3486,3196,2914,2639,2374,
//        2120,1877,1646,1428,1224,1036,863,706,566,444,340,254,
//        187,139,110,100,110,139,187,254,340,444,566,706,863,
//        1036, 1224, 1428, 1646,1877,2120,2374,2639,2914,
//        3196,3486,3781,4082,4386,4692};
//
//        length = 100;
//        for (i = 0; i < length; i++)
//        {
//            for (j = 0; j < length - i - 1; j++)
//            {
//                if (a[j + 1] < a[j])
//                {
//                    temp = a[j];
//                    a[j] = a[j + 1];
//                    a[j + 1] = temp;
//                }
//            }
//        }
//    }
//}
