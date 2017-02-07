// ADCTestMain.c
// Runs on TM4C123
// This program periodically samples ADC channel 0 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.
// Daniel Valvano
// September 5, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// center of X-ohm potentiometer connected to PE3/AIN0
// bottom of X-ohm potentiometer connected to ground
// top of X-ohm potentiometer connected to +3.3V 

#include <stdint.h>
#include "ADCSWTrigger.h"
#include "../ValvanoWareTM4C123/ValvanoWareTM4C123/inc/tm4c123gh6pm.h"
#include "../ValvanoWareTM4C123/ValvanoWareTM4C123/ST7735_4C123/ST7735.c"
#include "../Lab1/fixed.c"
#include "PLL.h"
#include <stdbool.h>

#define PF2             (*((volatile uint32_t *)0x40025010))
#define PF1             (*((volatile uint32_t *)0x40025008))
#define PF4   (*((volatile uint32_t *)0x40025040))

void ProcessTimeData(void);		// Find min and max time differences and jitter
void CreatePMF(void);					// Plot a PMF of the sampled ADC data
void DelayWait10ms(uint32_t);
void ResetScreen(void);
void TestLines(void);

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

volatile uint32_t ADCvalue;

struct ADCvalueCount {
	uint32_t value;
	uint32_t numOccur;
};
typedef struct ADCvalueCount ADCvalueCount;

const char NewLine = 10;
static const int size = 1000;
uint32_t bufIndex = 0;
uint32_t MaxTimeDiff = 0;
uint32_t MinTimeDiff = 0xFFFFFFFF;
uint32_t Jitter;
uint32_t TimeBuf[size];
uint32_t ADCBuf[size];
uint32_t TimeDiffBuf[size-1];
ADCvalueCount ADCvalueList[size];
uint32_t ListSize = 0;
uint32_t min_ADC;
uint32_t max_ADC;

void Pause(void){
  while(PF4==0x00){ 
    DelayWait10ms(10);
  }
  while(PF4==0x10){
    DelayWait10ms(10);
  }
}

// This function initializes all values and counts in the 
// struct of measured ADC values to 0.
void ADCstructInit() {
	for(int i = 0; i < size; i++) {
		ADCvalueList[i].numOccur = 0;
		ADCvalueList[i].value = 0;
	}
}

void PortF_Init() {
	SYSCTL_RCGCGPIO_R |= 0x20;            // activate port F
  ADC0_InitSWTriggerSeq3_Ch9();         // allow time to finish activating
	GPIO_PORTF_DIR_R |= 0x06;             // make PF2, PF1 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x16;          // disable alt funct on PF4, PF2, PF1
  GPIO_PORTF_DEN_R |= 0x16;             // enable digital I/O on PF4, PF2, PF1
                                        // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF00F)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
	GPIO_PORTF_PUR_R |= 0x10;         // 5) pullup for PF4

  PF2 = 0;                  				    // turn off LED
}

// This debug function initializes Timer0A to request interrupts
// at a 100 Hz frequency.  It is similar to FreqMeasure.c.
void Timer0A_Init100HzInt(void){
  volatile uint32_t delay;
  DisableInterrupts();
  // **** general initialization ****
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  delay = SYSCTL_RCGCTIMER_R;      // allow time to finish activating
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = 0;                // configure for 32-bit timer mode
  // **** timer0A initialization ****
                                   // configure for periodic mode
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TAILR_R = 799999;         // start value for 100 Hz interrupts
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interrupt
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
  TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 32-b, periodic, interrupts
  // **** interrupt initialization ****
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R = 1<<19;              // enable interrupt 19 in NVIC
}

void Timer1A_Init80MHzInt(void) {
	volatile uint32_t delay;
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
  //PeriodicTask = task;          // user function
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAILR_R = 0xFFFFFFFE;    // 4) reload value
  TIMER1_TAPR_R = 0;            // 5) bus clock resolution
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
//  TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
//  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00008000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 37, interrupt number 21
//  NVIC_EN0_R = 1<<21;           // 9) enable IRQ 21 in NVIC
  TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
}

void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
  PF2 ^= 0x04;                   // profile
  PF2 ^= 0x04;                   // profile
  ADCvalue = ADC0_InSeq3();
	if(bufIndex < size) {
		TimeBuf[bufIndex] = TIMER1_TAR_R;		// add current time to time dump
		ADCBuf[bufIndex] = ADCvalue;		// add current ADC value to ADC dump
		bufIndex++;
	}
  PF2 ^= 0x04;                   // profile
}

int main(void){
  PLL_Init(Bus80MHz);                   // 80 MHz
	Output_Init();
  PortF_Init();
  Timer0A_Init100HzInt();               // set up Timer0A for 100 Hz interrupts
	Timer1A_Init80MHzInt();								// set up Timer1A for couting every 12.5ns
  ADCstructInit();											// initialize counts of ADC values
	EnableInterrupts();
	
	ST7735_FillScreen(ST7735_BLACK); 
  ST7735_SetCursor(0,0);
	ST7735_OutString("Sampling ADC");
  while(1){
    PF1 ^= 0x02;  // toggles when running in main
		if(bufIndex == size) {
			break;
		}
  }
	ProcessTimeData();
	CreatePMF();
	Pause();
	TestLines();
}


void ProcessTimeData() {
	for(int i = 0; i < size - 3; i++) { // to convert time values to sec, need to divide by 80e6
		uint32_t time_diff = TimeBuf[i] - TimeBuf[i+1];
		if(time_diff > MaxTimeDiff) {
			MaxTimeDiff = time_diff;
		} else if(time_diff < MinTimeDiff) {
			MinTimeDiff = time_diff;
		}
		TimeDiffBuf[i] = time_diff;
	}
	Jitter = MaxTimeDiff - MinTimeDiff;
}

void CreatePMF() {
	min_ADC = ADCBuf[0];
	max_ADC = ADCBuf[0];
	uint32_t max_occur = 0;
	
	for(int i = 0; i < size; i++) {
		uint32_t val = ADCBuf[i];
		
		if(val < min_ADC) { min_ADC = val; }	// new minimum ADC value found
		else if (val > max_ADC) { max_ADC = val; }	// new maximum ADC value found
		
		bool found = false;
		for(int j = 0; j < ListSize; j++) {
			if(ADCvalueList[j].value == val) { // value already entered
				ADCvalueList[j].numOccur++;
				found = true;
				break;
			}
		}
		if(!found) { // value not yet entered
			ADCvalueList[ListSize].value = val;
			ADCvalueList[ListSize].numOccur = 1;
			ListSize++;
		}
	}
	
	for(int i = 0; i < ListSize; i++) { // find the maximum number of occurences
		if(ADCvalueList[i].numOccur > max_occur) { max_occur = ADCvalueList[i].numOccur; }
	}
	
	// plot the PMF
	// x ranges from 0 to 127, height ranges from 0 to 127
	ST7735_FillScreen(ST7735_BLACK); 
  ST7735_SetCursor(0,0);
	for(int i = 0; i < ListSize; i++) {
		int16_t x = 5 + 117*(ADCvalueList[i].value - min_ADC)/(max_ADC - min_ADC);
		int16_t height = (127*ADCvalueList[i].numOccur)/max_occur; 
		int16_t y = 158 - height; // y in DrawFastVLine is num of rows from top, lines grow downward
		ST7735_DrawFastVLine(x, y, height, ST7735_YELLOW);
	}

	ST7735_SetCursor(1,1);
	ST7735_OutString("Min ADC: ");
	ST7735_OutputNumber(min_ADC);
	ST7735_SetCursor(1,2);
	ST7735_OutString("Max ADC: ");
	ST7735_OutputNumber(max_ADC);
}

// Subroutine to wait 10 msec
// Inputs: None
// Outputs: None
// Notes: ...
void DelayWait10ms(uint32_t n){uint32_t volatile time;
	while(n){
    time = 727240*2/91;  // 10msec
    while(time){
	  	time--;
    }
    n--;
  }
}

void ResetScreen() {
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_SetCursor(0,0);	
}

void TestLines() {
	ResetScreen();
	ST7735_Line(5, 5, 120, 120, ST7735_BLUE);
	Pause();
	ResetScreen();
	ST7735_Line(5, 50, 5, 100, ST7735_BLUE);
	Pause();
	ResetScreen();
	ST7735_Line(5, 110, 110, 10, ST7735_BLUE); 
	Pause();
	ResetScreen();
	ST7735_Line(50, 150, 100, 5, ST7735_BLACK);
	Pause();
	ResetScreen();
	ST7735_Line(50, 100, 50, 25, ST7735_BLUE);
	Pause();
	ResetScreen();
	ST7735_Line(25, 50, 100, 50, ST7735_BLUE);
}
