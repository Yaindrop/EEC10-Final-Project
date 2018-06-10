/***********************************************************************************
 * Daniel Yuan, 914798275
 **********************************************************************************/

#include "msp.h"
#include "Bump.h"
#include "TExaS.h"
#include "Clock.h"
#include "SysTick.h"
#include "LaunchPad.h"
#include "CortexM.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

/***********************************************************************************
 * Parameters and global variables.
 ***********************************************************************************/
// Test of Periodic interrupt
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))

#define SIZE 1024 //Number of samples to take from ADC RAM.
#define PWM_PERIOD 15000 //total number of clock cycles per PWM waveform period.
                         // 10ms pulse period
#define FORWARD 1
#define BACKWARD -1
#define LEFT 2
#define RIGHT -2

uint16_t x1[SIZE]; //ADC (0 to 16383) P4.7/A6 Integer Buffer.
uint16_t x2[SIZE]; //ADC (0 to 16383) P4.6/A7 Integer Buffer.
float Vin1[SIZE]; //ADC (-1.65 to 1.65) P4.7/A6 Voltage Buffer.
float Vin2[SIZE]; //ADC (-1.65 to 1.65) P4.6/A7 Voltage Buffer.

bool isSampling = true;
int8_t status = FORWARD;
uint32_t count = 0;
uint16_t duty1 = 0, duty2 = 0;

float rms1 = 0, rms2 = 0;
float avg_rms = 0; //average RMS value of left and right channel RMS value.
float delta_rms = 0; //difference of left and right channel RMS value.
float rel_rms = 0; //relative difference of left and right channel RMS value.

//define your own global variables here

/***********************************************************************************
 * End of parameters and global variables.
 ***********************************************************************************/

void LED_Init(void){

    // Port 2 Configuration: Datasheet Page 141

    P2->SEL0 &= ~0x07;    // 0x07 = 0x00000111 and ~0x07 = 0x11111000
    P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
    P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
    P2->DS |= 0x07;       // 3) activate increased drive strength
    P2->OUT &= ~0x07;     //    all LEDs off
}

//***************************PWM_Duty1*******************************
void PWM_Duty1(void) {
    if (duty1 >= TIMER_A0->CCR[0])
        return; // bad input
    TIMER_A0->CCR[1] = duty1;        // CCR1 duty cycle is duty1/period
}

//***************************PWM_Duty2*******************************
// change duty cycle of PWM output on P2.5
// Inputs:  duty2
// Outputs: none// period of P2.5 is 2*period*666.7ns, duty cycle is duty2/period
void PWM_Duty2(void) {
    if (duty2 >= TIMER_A0->CCR[0])
        return; // bad input
    TIMER_A0->CCR[2] = duty2;        // CCR2 duty cycle is duty2/period
}

/***********************************************************************************
 * Vehicle movement functions. You will implement your own here.
 ***********************************************************************************/
void move_forward(void) {
    P1->OUT &= ~0xC0;
    P3->OUT |= 0xC0;
    PWM_Duty1();
    PWM_Duty2();
}

void move_backward(void) {
    P1->OUT |= 0xC0;
    P3->OUT |= 0xC0;
    PWM_Duty1();
    PWM_Duty2();
}

void turn_left(void) {
    P1->OUT |= 0x80;
    P1->OUT &= ~0x40;
    P3->OUT |= 0xC0;
    PWM_Duty1();
    PWM_Duty2();
}

void turn_right(void) {
    P1->OUT &= ~0x80;
    P1->OUT |= 0x40;
    P3->OUT |= 0xC0;
    PWM_Duty1();
    PWM_Duty2();
}

void rotate_left(void) {
    P1->OUT |= 0x80;
    P1->OUT &= ~0x40;
    P3->OUT |= 0xC0;
    PWM_Duty1();
    PWM_Duty2();
}

void rotate_right(void) {
    P1->OUT &= ~0x80;
    P1->OUT |= 0x40;
    P3->OUT |= 0xC0;
    PWM_Duty1();
    PWM_Duty2();
}

void motor_stop(void) {
    duty1 = 0;
    duty2 = 0;
    PWM_Duty1();
    PWM_Duty2();
    P3->OUT &= ~0xC0;
}

void apply_status(void) {
    if (status == FORWARD) {
        move_forward();
        P2->OUT |= 0x01;
    } else if (status == BACKWARD) {
        move_backward();
        P2->OUT |= 0x01;
    } else if (status == LEFT) {
        rotate_left();
        P2->OUT |= 0x02;
    } else if (status == RIGHT) {
        rotate_right();
        P2->OUT |= 0x04;
    }
}

void adjust_status(float newrms1, float newrms2) {
    float change1 = newrms1 - rms1;
    float change2 = newrms2 - rms2;
    float diff = fabs(newrms1 - newrms2);

    rms1 = newrms1;
    rms2 = newrms2;
    avg_rms = (rms1 + rms2) / 2;
    delta_rms = rms1 - rms2;
    rel_rms = delta_rms / avg_rms;

    if (fabs(rel_rms) > 0.08) {
        int8_t newStatus = rel_rms > 0.08 ? LEFT : RIGHT;
        if (status == newStatus) {
            duty1 = duty2 = 4000;
            count = 60000;
        } else {
            duty1 = duty2 = 6000;
            count = 80000;
        }
        status = newStatus;
    } else {
        bool inGoodDir = false;
        if (status == FORWARD || status == BACKWARD) {
            if (change1 < 0 && change2 < 0) {
                status = -status;
            } else if (change1 > 0 && change2 > 0) {
                inGoodDir = true;
            }
        } else {
            status = FORWARD;
        }
        duty1 = duty2 = inGoodDir ? 6000 : 4500;
        count = inGoodDir ? 600000 : 450000;
    }
}

//define additional movement functions here.

/***********************************************************************************
 * End vehicle movement functions.
 ***********************************************************************************/

void TA2_0_IRQHandler(void) {
    if (count > 0) {
        count --;
        return;
    }

    if (!isSampling) {
        isSampling = true;
        P2->OUT &= ~0x07;
        motor_stop();
        count = 200000;
    }

    static uint8_t trial = 0;
    static float newrms1 = 0, newrms2 = 0;
    static uint32_t I1 = SIZE - 1; //Index to loop through x1,x2,Vin1,and Vin2.

   /**************************************************************************************
    * Write your own code to determine how long you need to run the DC motors before you
    * sample the audio signals again. Turn on/off motor accordingly.
    **************************************************************************************/
   //motor time code

   /**************************************************************************************
    * End of motor time code
    **************************************************************************************/

    TIMER_A2->CCTL[0] &= ~0x0001; // ack

    while (ADC14->CTL0 & 0x00010000){}; // 1) wait for BUSY to be zero
    ADC14->CTL0 |= 0x00000001;       // 2) start single conversion
    while ((ADC14->IFGR0 & 0x02) == 0){}; // 3) wait for ADC14IFG1

    x1[I1] = ADC14->MEM[0];          //-8235;// 4) P4.7/A6 result 0 to 16383
    x2[I1] = ADC14->MEM[1]; //-8230; //  P4.6/A7 result 0 to 16383  different offset around 1.65V

    Vin1[I1] = (x1[I1] * 3.3) / 16383 - 1;
    Vin2[I1] = (x2[I1] * 3.3) / 16383 - 1;

    if (I1 == 0) {
        I1 = SIZE-1; //reset index

        for (uint32_t i = 0; i < SIZE; i++) {
            newrms1 += Vin1[i] * Vin1[i];
            newrms2 += Vin2[i] * Vin2[i];
        }

        // EUSCI_A0->TXBUF = rms>>3;              // divide max with 2^6

        P4->OUT ^= 0x20;  // toggle P 4.5

        // if ((newrms1 + newrms2) / 2 < 900) return;

        if (trial < 4) {
            trial ++;
        } else {
            isSampling = false;
            newrms1 = sqrt(newrms1 / SIZE / trial) * 1000;
            newrms2 = sqrt(newrms2 / SIZE / trial) * 1000;
            newrms1 += 50;
            adjust_status(newrms1, newrms2);
            apply_status();
            newrms1 = newrms2 = 0;
            trial = 0;
        }
    } else {
        I1--;                     // make room for data
    }

   /**************************************************************************************
    * Write your own code to make a dicision about turning right or left based on the
    * relative RMS values (rel_rms).
    **************************************************************************************/

   /**************************************************************************************
    * End of decision code
    **************************************************************************************/
}

//***************************PWM_Init12*******************************
// PWM outputs on P2.4, P2.5
// Inputs:  period (1.333us)
//          duty1
//          duty2
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
// Period of P2.4 is period*1.333us, duty cycle is duty1/period
// Period of P2.5 is period*1.333us, duty cycle is duty2/period
void PWM_Init12(uint16_t period, uint16_t d1, uint16_t d2) {
    if (d1 >= period)
        return; // bad input
    if (d2 >= period)
        return; // bad input
    P2->DIR |= 0x30;          // P2.4, P2.5 output
    P2->SEL0 |= 0x30;         // P2.4, P2.5 Timer0A functions
    P2->SEL1 &= ~0x30;        // P2.4, P2.5 Timer0A functions
    TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
    TIMER_A0->CCR[0] = period;   // Period is 2*period*8*83.33ns is 1.333*period
    TIMER_A0->EX0 = 0x0000;        //    divide by 1
    TIMER_A0->CCTL[1] = 0x0040;      // CCR1 toggle/reset
    TIMER_A0->CCR[1] = d1;        // CCR1 duty cycle is duty1/period
    TIMER_A0->CCTL[2] = 0x0040;      // CCR2 toggle/reset
    TIMER_A0->CCR[2] = d2;        // CCR2 duty cycle is duty2/period
    TIMER_A0->CTL = 0x02F0;        // SMCLK=12MHz, divide by 8, up-down mode
    // bit  mode
    // 9-8  10    TASSEL, SMCLK=12MHz
    // 7-6  11    ID, divide by 8
    // 5-4  11    MC, up-down mode
    // 2    0     TACLR, no clear
    // 1    0     TAIE, no interrupt
    // 0          TAIFG
}

void UART_Init(void) {
    if (((P1->SEL0 & 0x0C) == 0x0C) && (EUSCI_A0->BRW == 104))
        return; // already on
    EUSCI_A0->CTLW0 = 0x0001;              // hold the USCI module in reset mode
    // bit15=0,      no parity bits
    // bit14=x,      not used when parity is disabled
    // bit13=0,      LSB first
    // bit12=0,      8-bit data length
    // bit11=0,      1 stop bit
    // bits10-8=000, asynchronous UART mode
    // bits7-6=11,   clock source to SMCLK
    // bit5=0,       reject erroneous characters and do not set flag
    // bit4=0,       do not set flag for break characters
    // bit3=0,       not dormant
    // bit2=0,       transmit data, not address (not used here)
    // bit1=0,       do not transmit break (not used here)
    // bit0=1,       hold logic in reset state while configuring
    EUSCI_A0->CTLW0 = 0x00C1;
    // set the baud rate
    // N = clock/baud rate = 12,000,000/115,200 = 104.1666667
    EUSCI_A0->BRW = 104;        // UCBR = baud rate = int(N) = 104
    // actual baud rate is 12,000,000/104 = 115,385 bps (0.16% error)
    EUSCI_A0->MCTLW &= ~0xFFF1; // clear first and second modulation stage bit fields
                                // configure second modulation stage select (from Table 22-4 on p731 of datasheet)
//  UCA0MCTLW |= (0<<8);      // UCBRS = N - int(N) = 0.0417; plug this in Table 22-4
            // configure first modulation stage select (ignored when oversampling disabled)
//  UCA0MCTLW |= (10<<4);     // UCBRF = int(((N/16) - int(N/16))*16) = 10
//  UCA0MCTLW |= 0x0001;      // enable oversampling mode
    P1->SEL0 |= 0x0C;
    P1->SEL1 &= ~0x0C;     // configure P1.3 and P1.2 as primary module function
    EUSCI_A0->CTLW0 &= ~0x0001; // enable the USCI module
    EUSCI_A0->IE &= ~0x000F; // disable interrupts (transmit ready, start received, transmit empty, receive full)
}

void ADC0_InitSWTriggerCh67(void) {
    ADC14->CTL0 &= ~0x00000002;      // 2) ADC14ENC = 0 to allow programming
    while (ADC14->CTL0 & 0x00010000){}; // 3) wait for BUSY to be zero
    ADC14->CTL0 = 0x04223390;      // 4) single, SMCLK, on, disabled, /1, 32 SHM
    // 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
    // 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
    // 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
    // 25    ADC14ISSH  invert sample-and-hold  0b = not inverted
    // 24-22 ADC14DIVx  clock divider         000b = /1
    // 21-19 ADC14SSELx clock source select   100b = SMCLK
    // 18-17 ADC14CONSEQx mode select          01b = Sequence-of-channels
    // 16    ADC14BUSY  ADC14 busy              0b (read only)
    // 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks
    // 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks
    // 7     ADC14MSC   multiple sample         1b = continue conversions automatically after first SHI signal trigger
    // 6-5   reserved                          00b (reserved)
    // 4     ADC14ON    ADC14 on                1b = powered up
    // 3-2   reserved                          00b (reserved)
    // 1     ADC14ENC   enable conversion       0b = ADC14 disabled
    // 0     ADC14SC    ADC14 start             0b = No start (yet)
    ADC14->CTL1 = 0x00000030;     // 5) ADC14MEM0, 14-bit, ref on, regular power
    // 20-16 STARTADDx  start addr          00000b = ADC14MEM0
    // 15-6  reserved                  0000000000b (reserved)
    // 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
    // 3     ADC14DF    data read-back format   0b = Binary unsigned
    // 2     REFBURST   reference buffer burst  0b = reference on continuously
    // 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode
    ADC14->MCTL[0] = 0x00000006;     // 6a) 0 to 3.3V, channel 6
    // 15   ADC14WINCTH Window comp threshold   0b = not used
    // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
    // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
    // 12   reserved                            0b (reserved)
    // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
    // 7    ADC14EOS    End of sequence         0b = Not end of sequence
    // 6-5  reserved                           00b (reserved)
    // 4-0  ADC14INCHx  Input channel        0110b = A6, P4.7
    ADC14->MCTL[1] = 0x00000087;     // 6b) 0 to 3.3V, channel 7
    // 15   ADC14WINCTH Window comp threshold   0b = not used
    // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
    // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
    // 12   reserved                            0b (reserved)
    // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
    // 7    ADC14EOS    End of sequence         1b = End of sequence
    // 6-5  reserved                           00b (reserved)
    // 4-0  ADC14INCHx  Input channel        0111b = A7, P4.6

    ADC14->IER0 = 0;                 // 7) no interrupts
    ADC14->IER1 = 0;                 //    no interrupts
    P4->SEL1 |= 0xC0;                // 8) analog mode on P4.7/A6 and P4.6/A7
    P4->SEL0 |= 0xC0;
    ADC14->CTL0 |= 0x00000002;       // 9) enable
}

void ADC_In67(uint8_t in) {
    while (ADC14->CTL0 & 0x00010000) continue; // 1) wait for BUSY to be zero
    ADC14->CTL0 |= 0x00000001;       // 2) start single conversion

    while ((ADC14->IFGR0 & 0x03) == 0) continue; // 3) wait for ADC14IFG1

    if (in == 0) {
        EUSCI_A0->TXBUF = ADC14->MEM[0] >> 6;    // 4) P4.7/A6 result 0 to 16383
    } else {
        EUSCI_A0->TXBUF = ADC14->MEM[1] >> 6;    //    P4.6/A7 result 0 to 16383
    }
}

void TimerA2_Init(uint16_t period) {
    TIMER_A2->CTL = 0X0280;
    TIMER_A2->CCTL[0] = 0X0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0X0005;
    NVIC->IP[3] = (NVIC->IP[3] & 0XFFFFFF00) | 0X00000040;
    NVIC->ISER[0] = 0X00001000;
    TIMER_A2->CTL |= 0X0014;
}

uint8_t flag = 0;
int main(void) {
    Clock_Init48MHz(); // makes bus clock 48 MHz
    LaunchPad_Init(); // use buttons to step through frequencies
    UART_Init();
    ADC0_InitSWTriggerCh67();
    TimerA2_Init(50); // period = 50*2us = 100us => sampling frequency 10KHz

    LED_Init();

    PWM_Init12(PWM_PERIOD, 0, 0);

    EnableInterrupts();

    //P1.7(left DIR), P1.6(right DIR)
    P1->SEL0 &= ~0xC0;
    P1->SEL1 &= ~0xC0; // choose P1.6 and P1.7 as GPIO
    P1->DIR |= 0xC0; // choose P1.6 and P1.7 as outputs

    //P3.7(left SLP), P3.6(right SLP)
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0; // choose P3.6 and P3.7 as GPIO
    P3->DIR |= 0xC0; // choose P3.6 and P3.7 as outputs

    P1->OUT &= ~0xC0; // default going forward for both motors.
    P3->OUT &= ~0xC0; // default turning both motors off to save power.

    PWM_Duty1(); // default 0 number of cycles HIGH (no rotation)
    PWM_Duty2(); // default 0 number of cycles HIGH (no rotation)


    while (1) continue;
}
