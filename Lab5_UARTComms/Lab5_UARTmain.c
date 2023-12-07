// RSLK Self Test via UART

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"
#include "..\inc/SensorTest.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))

volatile uint8_t bumpData = 0;

//////////////////////////////////////////////////////////////////////////////////////// MOTOR TEST
void MotorTest(void){
// use of TimerCompare to adjust duty cycle of PWM signal
// higher duty cycle => higher motor speed

    char choice;
    int speed;
    speed = 4000;
    char prevChoice = 'r';
//    DisableInterrupts();
    EUSCIA0_OutString("w: FORWARD"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("s: BACKWARD"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("a: LEFT"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("d: RIGHT"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("q: SPEED UP"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("e: SLOW DOWN"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("r: STOP"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

    while(LaunchPad_Input() == 0){
        Clock_Delay1ms(50);
        choice = EUSCIA0_InChar();
        switch(choice){
                case'w':
                        Motor_Forward(speed, speed);
                        prevChoice = choice;
                        break;

                case 's':
                        Motor_Backward(speed,speed);
                        prevChoice = choice;
                        break;
                case'a':
                        Motor_Left(speed,speed);
                        prevChoice = choice;
                        break;
                case'd':
                        Motor_Right(speed,speed);
                        prevChoice = choice;
                        break;
                case'q':
                        speed += 1000;
                        if(speed > 7000){
                            EUSCIA0_OutString("Max Speed Reached. Speed remains at 7000"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                            speed = 7000;
                        }
                        speedChange(prevChoice,speed);
                        break;
                case'e':
                        speed -= 1000;
                        if(speed < 0){
                            EUSCIA0_OutString("Min speed reached. Speed recalibrated to 0"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                            speed = 0;

                        }
                        speedChange(prevChoice,speed);
                        break;
                case'r':
                        Motor_Stop();
                        prevChoice = choice;
                        break;
                default:
                    break;

            }
    }
    Motor_Stop();
    EUSCIA0_OutString("EXITING"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);



}
/////////////////////////////////////////////////////////////////////////////////////// BUMPER TEST


void SysTick_Handler(void){ // every 1ms
    volatile static uint8_t count=0;
    if(count==0){
        Reflectance_Start();
    }
    else if (count==1) {
//        reflectance_data =  Reflectance_End();
        bumpData = Bump_Read();
    }
    count++;
    if(count==10)count=0;
}
void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

volatile uint8_t CollisionData, CollisionFlag;
void HandleCollision(uint8_t bumpSensor) {
    Motor_Stop();
    CollisionData = bumpSensor;
    CollisionFlag = 1;
    P4->IFG &= ~0xED;


}




///////////////////////////////////////////////////////////////////////////////////// SENSOR

volatile uint32_t nr,nc,nl;
volatile uint32_t ADCvalue;
volatile uint32_t ADCflag;
void SensorRead_ISR(void){
    uint32_t raw17, raw12, raw16;
    P1OUT ^= 0x01;         // profile
    P1OUT ^= 0x01;         // profile
    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
    nr = LPF_Calc(raw17);  // right is channel 17 P9.0
    nc = LPF_Calc2(raw12);  // center is channel 12, P4.1
    nl = LPF_Calc3(raw16);  // left is channel 16, P9.1
    ADCflag = 1;           // semaphore
    P1OUT ^= 0x01;

}

void IRSensor_Init(void)
{
    uint32_t raw17, raw12, raw16;
    uint32_t s;
    ADCflag = 0;
    s = 256; // replace with your choice
    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
    LPF_Init(raw17, s);     // P9.0/channel 17
    LPF_Init2(raw12, s);     // P4.1/channel 12
    LPF_Init3(raw16, s);     // P9.1/channel 16
    UART0_Init();          // initialize UART0 115,200 baud rate
    LaunchPad_Init();
    TimerA1_Init(&SensorRead_ISR,250);  // 2000 Hz sampling
}


//////////////////////////////////////////////////////////////////////////////////////////////////// TACHNOMETER
uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0=0;             // Timer A3 first edge, P10.4
uint32_t Done0=0;              // set each rising

uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2=0;             // Timer A3 first edge, P8.2
uint32_t Done2=0;              // set each rising
void PeriodMeasure0(uint16_t time){
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                    // setup for next
  Done0++;
}

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                    // setup for next
  Done2++;
}


///////////////////////////////////////////////////////////////////////////////////////////////////RSLK
void RSLK_Reset(void){
    DisableInterrupts();

    LaunchPad_Init();
    //Initialise modules used e.g. Reflectance Sensor, Bump Switch, Motor, Tachometer etc
    // ... ...
    Motor_Init();
    Motor_Stop();
    EnableInterrupts();
}

// RSLK Self-Test
// Sample program of how the text based menu can be designed.
// Only one entry (RSLK_Reset) is coded in the switch case. Fill up with other menu entries required for Lab5 assessment.
// Init function to various peripherals are commented off.  For reference only. Not the complete list.
volatile uint32_t left;
volatile uint32_t center;
volatile uint32_t right;
int main(void) {
  uint32_t cmd=0xDEAD, menu=0;

//  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  Motor_Init();
  Motor_Stop();
  LaunchPad_Init();
  //Bump_Init();
  BumpInt_Init(&HandleCollision);
  //IRSensor_Init();
  //Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();


  ////////////////////////////////////////////////////////////////////////////////////////////////// PANEL
  while(1){                     // Loop forever
      // write this as part of Lab 5
//      DisableInterrupts();
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] Roomba"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[7] Line Tracing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      EUSCIA0_OutString("CMD: ");
      cmd=EUSCIA0_InUDec();
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      CollisionFlag = 0;

      switch(cmd){
//////////////////////// RESET
          case 0:
              RSLK_Reset();
              menu =1;
              cmd=0xDEAD;
              break;
//////////////////////// REMOTE CONTROL + MOTORTEST
          case 1:
              MotorTest();

              break;
//////////////////////// IR SENSOR
          case 2:
              IRSensor_Init();
              EUSCIA0_OutString("IR Sensor testing ......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              UART0_OutString("GP2Y0A21YK0F test\nValvano Oct 2017\nConnect analog signals to P9.0,P4.1,P9.1\n");
              EnableInterrupts();
              int32_t n;

              while(1)
              {
                  for(n=0; n<2000; n++){
                        while(ADCflag == 0){};
                        ADCflag = 0; // show every 2000th point
                  }
                  UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" mm,");
                  UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm,");
                  UART0_OutUDec5(RightConvert(nr));UART0_OutString(" mm\r\n");



//                  left = LeftConvert(nl);
//                  center = CenterConvert(nc);
//                  right = RightConvert(nr);
//                  EUSCIA0_OutUDec5(LeftConvert(nl));EUSCIA0_OutString(" mm,");
//                  EUSCIA0_OutUDec5(CenterConvert(nc));EUSCIA0_OutString(" mm,");
//                  EUSCIA0_OutUDec5(RightConvert(nr));EUSCIA0_OutString(" mm\r\n");
              }

              menu = 1;
              break;
//////////////////////////////////////// BUMPER
          case 3:
              EnableInterrupts();

              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Bump Switches Testing ......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              BumpInt_Init(&HandleCollision);
              CollisionData = 0x3F;
              CollisionFlag = 0;

              while (1) {
                      if (CollisionFlag == 1) {
                          // Check individual bits and print messages
                          for (int i = 0; i < 6; i++) {
                              int bit = (CollisionData >> i) & 1;
                              if (bit == 0) {
                                  // A bumper is pressed
                                  UART0_OutString("Bumper ");
                                  UART0_OutUDec(i); // Print the value of i
                                  UART0_OutString(" pressed\n");
                              }
                          }
                      }

                      CollisionFlag = 0;
                      WaitForInterrupt();
                  }
                  return 0;
//


              menu = 1;
              cmd = 0xDEAD;
              break;

///////////////////////////////////////// REFLECTANCE
          case 4:
              // Reflectance
              EUSCIA0_OutString("Reflectance testing......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              uint8_t refData = 0;
              while (1){
                  refData = Reflectance_Read(1000);
                  for(int i = 0; i<8;i++){
                      EUSCIA0_OutUDec(refData%2);
                      EUSCIA0_OutString("");
                      refData=refData/2;

                  }
                  EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                  Clock_Delay1ms(100);
              }
              menu = 1;
              break;
/////////////////////////////////////// TACHOMETER


          case 5:
                    EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                    EUSCIA0_OutString("Tachometer testing......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                    EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                    uint32_t main_count=0;
                    TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);
                    Clock_Delay1ms(500);
                        Motor_Forward(1000,3000);
                    while(LaunchPad_Input()==0){
                          WaitForInterrupt();
                          main_count++;
                          if(main_count%1000){
                              UART0_OutString("Period0 = ");UART0_OutUDec5(Period0);UART0_OutString(" Period2 = ");UART0_OutUDec5(Period2);UART0_OutString(" \r\n");
                          }
                        }
                        Motor_Stop();
                                        menu = 1;
                                        cmd = 0xDEAD;
                                        break;

                menu = 1;
                cmd = 0xDEAD;
                break;

///////////////////////////////////////////// ROOMBA
          case 6:
              EnableInterrupts();
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Dumb Roomba"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              BumpInt_Init(&HandleCollision);
              CollisionData = 0x3F;
              CollisionFlag = 0;
              int speed = 2000;
              while(1){
                  for (int i = 0; i < 6; i++) {
                        int bit = (CollisionData >> i) & 1;
                        if (bit == 0) {
                            // A bumper is pressed
                            UART0_OutString("Bumper ");
                            UART0_OutUDec(i); // Print the value of i
                            UART0_OutString(" pressed\n");
                            if (i==4 || i ==5){
                                Motor_Stop();
                                Motor_Backward(speed, speed);
                                Motor_Right(speed, speed);
                            }
                            else if(i==2 || i==3){
                                Motor_Stop();
                            }
                            else if(i==1 || i == 0 ){
                                Motor_Stop();
                                Motor_Backward(speed, speed);
                                Motor_Left(speed, speed);
                            }
                    }
                }



                Motor_Forward(3000, 3000);
                CollisionFlag = 0;
                WaitForInterrupt();

              }
              menu = 1;
              cmd = 0xDEAD;
              break;
/////////////////////////// LINE TRACING
          case 7:
              EUSCIA0_OutString("Line Tracing......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

              speed = 2000;
              uint32_t Position;
              while (1){
                  Motor_Forward(speed,speed);
                  refData = Reflectance_Read(1000);
                  Position = Reflectance_Position(refData);


                  // Turn Right - off left
                  if (Position > -47 && Position < 47)
                        { //center
                              Motor_Forward(2000, 2000);
                         }
                  else if (Position <= -47 && Position > -142)
                         { //slightly off to the left
                             Motor_Left(speed,speed/2);
                             Clock_Delay1ms(50);
//                             Motor_Stop();
                          }
                   else if (Position >= 47 && Position <142)
                          { //slightly off to the right
                              Motor_Right(speed/2,speed);
                              Clock_Delay1ms(50);
//                              Motor_Stop();
                           }
                   else if (Position <= -142 && Position >-237)
                           { //off to the left
                               Motor_Left(speed,speed/2);
                               Clock_Delay1ms(100);
//                               Motor_Stop();
                           }
                    else if (Position >= 142 && Position < 237)
                           { // off to the right
                               Motor_Right(speed/2,speed);
                               Clock_Delay1ms(100);
//                               Motor_Stop();
                           }
                   else if (Position <= -237 && Position > -332)
                            { // way off left
                               Motor_Left(speed,speed/2);
                                Clock_Delay1ms(150);
//                                Motor_Stop();
                            }
                   else if (Position >= 237 && Position < 332)
                            { // way off right
                              Motor_Right(speed/2,speed);
                              Clock_Delay1ms(150);
//                              Motor_Stop();
                            }
                    else if(refData == 0xFF){
                           Motor_Stop();
                            break;
                         }



              }
              menu = 1;
              break;





              // ....
              // ....

          default:
              menu=1;
              break;
      }

      if(!menu)Clock_Delay1ms(3000);
      else{
          menu=0;
      }

      // ....
      // ....
  }
}

#if 0
//Sample program for using the UART related functions.
int Program5_4(void){
//int main(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}
#endif
