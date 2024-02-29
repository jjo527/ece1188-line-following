// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include <stdio.h>
#include "msp432.h"
#include "..\inc\Clock.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
//void Reflectance_Start(void){Reflectance_Init();}


void Reflectance_Init(void){
    //CTRL EVEN [5.3] and CTRL ODD [9.2]: outputs (turn on IR LEDs)
        //initally turned off
    P5->SEL0 &= ~0x08; //CTRL EVEN[5.3]
    P5->SEL1 &= ~0x08;
    P5->DIR  |=  0x08;//out
    //P5->DS   |=  0x08;
    P5->OUT  &= ~0x08;//off


    P9->SEL0 &= ~0x02; //CTRL ODD[9.2]
    P9->SEL1 &= ~0x02;
    P9->DIR  |=  0x02;//out
    //P9->DS   |=  0x04;
    P9->OUT  &= ~0x02;//off

    //bits7-0 of QTRX are bidirectional [7.0-7.7]
        //intially inputs
    /*
    P7->SEL0 &= ~0xFF; //7.0-7.7
    P7->SEL1 &= ~0xFF;
    P7->DIR  &= ~0xFF;//inputs
    */

    P7->SEL0 = 0x00;
    P7->SEL1 = 0x00;
    P7->DIR  = 0x00;
    //P7->REN  |=  0xFF;//pull resistors
    //P7->OUT  |=  0xFF;

    //debug
    //P3->SEL0 &= ~0x01;
    //P3->SEL1 &= ~0x01;    //  P4.0 as GPIO
    //P3->DIR |= 0x01;      //  make P4.0 out
}







// ------------Reflectance_Read------------
// Read the eight sensors

// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){
    uint8_t result;
    //turn on leds: 5.3,9.2
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;
    //charge capacitors: P7 outputs, set high (0xFF)
    P7->DIR = 0xFF;
    P7->OUT = 0xFF;
    //wait 10us
    //make sensors inputs: P7 inputs (0xFF)
    //wait time
    Clock_Delay1ms(time);
    //Clock_Delay1us(10);
    //change sensor to binary: for loop P4 = P7
    P7->DIR = 0x00;
    for(int i=0;i<10000;i++){
      //result = P7->IN&0x01; // convert P7.0 input to digital
        P3->OUT = P7->IN;
    }
    result = P3->OUT;
    //turn off leds: 5.3,9.2
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;
    //return binary: return P4
    return result;

}






// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs

// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
    uint8_t result;
    /*
    //turn on leds: 5.3,9.2
    P5->OUT = 0x08;
    P9->OUT = 0x04;
    //charge capacitors: P7 outputs, set high (0x18)
    P7->OUT = 0x18;
    //wait 10us
    Clock_Delay1us(10);
    //make sensors inputs: P7 inputs (0x18)
    P7->SEL0 &= ~0xFF; //7.0-7.7
    P7->SEL1 &= ~0xFF;
    P7->DIR  &= ~0xFF;//inputs
    //P7->REN  |=  0xFF;//pull resistors
    P7->OUT  |=  0xFF;
    //wait time
    Clock_Delay1ms(time);
    //change sensor to binary: for loop P4 = P7
    for(int i=0;i<10000;i++){
      result = P7->IN&0x01; // convert P7.0 input to digital
    }
    //turn off leds: 5.3,9.2
    P5->OUT = 0x00;
    P9->OUT = 0x00;
    //return binary: return P4
    return result;
    */
    P5->OUT |= 0x08;      // turn on 4 even IR LEDs
    P9->OUT |= 0x04;
    P7->DIR = 0xFF;       // make P7.7-P7.0 out
    P7->OUT = 0xFF;       // prime for measurement
    Clock_Delay1us(10);   // wait 10 us
    P7->DIR = 0x00;       // make P7.7-P7.0 in
    Clock_Delay1us(time);
    /*
    for(int i=0;i<10000;i++){
      P3->OUT = P7->IN&0xFF; // convert P7.0 input to digital
    }*/
    result = P7->IN;
    P5->OUT &= ~0x08;     // turn off 4 even IR LEDs
    P9->OUT &= ~0x04;
    return result;

}






// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){
    int32_t result=0;
    int w[] = {-33400,-23800,-14300,-4800, 4800,14300,23800,33400};

    // return (data bit[i] * w[i])/(number of bits in data)
    for(int i = 0; i < sizeof(w) / sizeof(int); i++)
    {
        result += w[i]* ((data >> i)&1);
    }

    //divider, count num of set bits
    int32_t count=0;
    while (data)
    {
        count += data & 1;
        data = data >> 1;
    }


 return (result/count);
}



uint8_t Reflectance_Start(){
    //first half of Reflectance_Read w/o time input
    P5->OUT |= 0x08;
    P9->OUT |= 0x03;
    P7->DIR  = 0xFF;
    P7->OUT  = 0xFF;
    Clock_Delay1us(10);
    P7->DIR  = 0x00;
}


uint8_t Reflectance_End(){
    //second half of Reflectance_Read
    uint8_t result;

    /*
    for(int i=0;i<10000;i++){
      result = P7->IN;
      //P3->OUT = P7->IN&0x01;
    }*/
    result = P7->IN;
    //turn off leds: 5.3,9.2
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x03;
    //return binary: return P4
    return result;
}

