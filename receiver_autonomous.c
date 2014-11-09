//attempt to combine voltage reader + robot movement

#include <stdio.h> 
#include <at89lp51rd2.h>
#include <stdlib.h>
#include <math.h>

// ~C51~  
#define CLK 22118400L
#define BAUD 115200L
#define BRG_VAL (0x100-(CLK/(32L*BAUD)))
#define FREQ 100000L
//#define TIMER0_RELOAD_VALUE (65536L-(CLK/(12L*FREQ)))
#define TIMER0_RELOAD_VALUE 0

#define ITERATIONS 6000 //MAKE SURE THIS IS THE SAME ON BOTH MCUS

// Red 		VCC
// Black	Ground
// Grey 	Forward			P0_0
// Yellow 	Back			P0_1
// Orange 	Right			P0_2
// Blue 	Left			P0_3
// White	L1 / 180		P0_6
// Green	L2 / p_park_J	P0_5

#define P_JF P0_0
#define P_JB P0_1
#define P_JR P0_2
#define P_JL P0_3
#define P_B8 P0_6
#define P_BP P0_5

#define P_OS P0_7

//follow constants
#define BUFFER 0.05
#define LR_BUFFER 0.2
#define FORWARD 1
#define BACKWARD 0
//follow variables
volatile float voltage_R;
volatile float voltage_L;
volatile float distanceV; //voltage at distance robot must keep from transmitter (V)
volatile int orientation;
//follow/voltage functions
void getVoltage(float *par, int i);
void getMaxVoltages(void);


//These variables are used in the ISR
volatile unsigned char pwmcount;
volatile float pwm1;
volatile float pwm2;
volatile float pwm3;
volatile float pwm4;
volatile float tentativepwm1;
volatile float tentativepwm2;
volatile float tentativepwm3;
volatile float tentativepwm4;
volatile float distance;
volatile float distance1;
volatile float distance2;

volatile int i;


volatile int direction; // boolean variable for direction, 0 for CW, 1 for CCW
volatile int orientation; //turn 180 forward and backward
volatile float speed = 100;
volatile float offsetspeed; //offset for 0<speed<5
volatile float rpm;

void Turn180(float speed);
void waithalfus(void);
void follow(void);
void GetDistances(void);
//unsigned int GetADC(unsigned char channel); // adding this here
int rx_byte (int min, int channel);
float convertToD(float max);
float convertToDAMP(float max);
void turn_180 ( void );
void p_park ( void );
void testmove ( void );
int waithalfs (void);
void refresh ( void );

void waithalfus(void){
	_asm    
	;For a 22.1184MHz crystal one machine cycle takes 12/22.1184MHz=0.5425347us
	;mov TR0, #1
	;mov R7, #9
	;mov TR0, #0
	push AR1
	mov R1, #1
	pop AR1
	ret
	_endasm;
}

unsigned char _c51_external_startup(void)
{
	// Configure ports as a bidirectional with internal pull-ups.
	P0M0=0;	P0M1=0;
	P1M0=0;	P1M1=0;
	P2M0=0;	P2M1=0;
	P3M0=0;	P3M1=0;
	AUXR=0B_0001_0001; // 1152 bytes of internal XDATA, P4.4 is a general purpose I/O
	P4M0=0;	P4M1=0;
	
    // Instead of using a timer to generate the clock for the serial
    // port, use the built-in baud rate generator.
	PCON|=0x80;
	SCON = 0x52;
	BDRCON=0;
	BRL=BRG_VAL;
	BDRCON=BRR|TBCK|RBCK|SPD;
	
	
    // Initialize timer 0 for ISR 'pwmcounter()' below
	TR0=0; // Stop timer 0
	TMOD=0x01; // 16-bit timer
	// Use the autoreload feature available in the AT89LP51RB2
	// WARNING: There was an error in at89lp51rd2.h that prevents the
	// autoreload feature to work.  Please download a newer at89lp51rd2.h
	// file and copy it to the crosside\call51\include folder.
	TH0=RH0=TIMER0_RELOAD_VALUE/0x100;
	TL0=RL0=TIMER0_RELOAD_VALUE%0x100;
	TR0=1; // Start timer 0 (bit 4 in TCON)
	ET0=1; // Enable timer 0 interrupt
	EA=1;  // Enable global interrupts
	
	
	pwmcount=0;
	
	return 0;
}

/*
// Interrupt 1 is for timer 0.  This function is executed every time
// timer 0 overflows: 100 us.
void pwmcounter (void) interrupt 1
{
	if(++pwmcount>99) 
		pwmcount=0;
	P1_0=(pwm1>pwmcount)?1:0;
	P1_1=(pwm2>pwmcount)?1:0;
	P1_2=(pwm3>pwmcount)?1:0;
	P1_3=(pwm4>pwmcount)?1:0;
}*/

	void SPIWrite(unsigned char value)
	{
	SPSTA&=(~SPIF); // Clear the SPIF flag in SPSTA
	SPDAT=value;
	while((SPSTA & SPIF)!=SPIF); //Wait for transmission to end
}

unsigned int GetADC(unsigned char channel)
{
	unsigned int adc;

	// initialize the SPI port to read the MCP3004 ADC attached to it.
	SPCON&=(~SPEN); // Disable SPI
	SPCON=MSTR|CPOL|CPHA|SPR1|SPR0|SSDIS;
	SPCON|=SPEN; // Enable SPI

	P1_4=0; // Activate the MCP3004 ADC.
	SPIWrite(channel|0x18);	// Send start bit, single/diff* bit, D2, D1, and D0 bits.
	for(adc=0; adc<10; adc++); // Wait for S/H to setup
	SPIWrite(0x55); // Read bits 9 down to 4
adc=((SPDAT&0x3f)*0x100);
	SPIWrite(0x55);// Read bits 3 down to 0
	P1_4=1; // Deactivate the MCP3004 ADC.
	adc+=(SPDAT&0xf0); // SPDR contains the low part of the result. 
	adc>>=4;

	return adc;
}

//         LP51B    MCP3004
//---------------------------
// MISO  -  P1.5  - pin 10
// SCK   -  P1.6  - pin 11
// MOSI  -  P1.7  - pin 9
// CE*   -  P1.4  - pin 8
// 4.8V  -  VCC   - pins 13, 14
// 0V    -  GND   - pins 7, 12
// CH0   -        - pin 1
// CH1   -        - pin 2
// CH2   -        - pin 3
// CH3   -        - pin 4




void wait_bit_time()
{
	int n=ITERATIONS;
	while (n>0)
	{
		n--;
	}
	return;
}

void wait_time(int iter)
{
	int n=iter;
	while (n>0)
	{
		n--;
	}
	return;
}

void wait_one_and_half_bit_time()
{
	int n=1.5*ITERATIONS;
	while (n>0)
	{
		n--;
	}
	return;
}

void wait_check()
{
	int n=.5*ITERATIONS;
	while (n>0)
	{
		n--;
	}
	return;
}

int rx_byte (int min, int channel)
{
	int j, val;
	int v;

	//skip the start bit
	val = 0;
	wait_one_and_half_bit_time();
	for (j=0; j<8; j++)
	{
		v=(GetADC(channel)*4.95/1023.0);					//read voltage in
		val|=(v>min)?(0x01<<j):0x00;	//if voltage is greater than "min" then the returned val gets a bit at the right position
		wait_bit_time();
	}
	//wait for stop bits
	wait_one_and_half_bit_time();
	return val;
}

void turn_180 ( void )
{
	int j;
	int wait;
	P1_0 = 0;
	P1_1 = 1;
	P1_2 = 0;
	P1_3 = 1;
	
	for(j=0; j<11; j++){
		for (wait = 0; wait < 5000; wait++) {
			waithalfus();
		}
	}
	
	P1_0 = 0;
	P1_1 = 0;
	P1_2 = 0;
	P1_3 = 0;
	
	return;
}


void p_park ( void )
{
	int wait, j;
	
    //Turning 45 degrees to the left
	P1_0 = 0;
	P1_1 = 0;
	P1_2 = 1;
	P1_3 = 0;

	for(j=0; j<4; j++){
		for (wait = 0; wait < 10000; wait++) {
			waithalfus();
		}
	}
	
	
    //move backwards

	P1_0 = 1;
	P1_1 = 0;
	P1_2 = 0;
	P1_3 = 1;

	for(j=0; j<7; j++){
		for (wait = 0; wait < 10000; wait++) {
			waithalfus();
		}
	}

	
    //turn right
	P1_0 = 0;
	P1_1 = 1;
	P1_2 = 0;
	P1_3 = 0;

	for(j=0; j<4; j++){
		for (wait = 0; wait < 10000; wait++) {
			waithalfus();
		}
	}

	 //move backwards

	P1_0 = 1;
	P1_1 = 0;
	P1_2 = 0;
	P1_3 = 1;

	for(j=0; j<4; j++){
		for (wait = 0; wait < 10000; wait++) {
			waithalfus();
		}
	}


	/*
    //move forward

    P1_0 = 0;
    P1_1 = 1;
    P1_2 = 1;
    P1_3 = 0;

    for(j=0; j<7; j++){
    	for (wait = 0; wait < 10000; wait++) {
        	waithalfus();
    	}
    }

	*/

    P1_0 = 0;
    P1_1 = 0;
    P1_2 = 0;
    P1_3 = 0;

}

void refresh ( void )
{
	P1_0=0;
	P1_1=0;
	P1_2=0;
	P1_3=0;
}

void testmove ( void )
{
	P1_0=1;
	P1_1=0;
	P1_2=0;
	P1_3=1;
}

void getVoltage ( float *par, int i )
{
	*par = (GetADC(i)*4.95)/1023.0;
	return;
}


void getMaxVoltages(void)
{
	getVoltage ( &voltage_R, 1);
	getVoltage ( &voltage_L, 0);
	return;
}


//distance_R = distance1, distance_L = distance 2, BUFFER, FORWARD, BACKWARD defined as a constants
void follow(void) 
{
	if (orientation == FORWARD) {
		if ((voltage_L+BUFFER < distanceV) && (voltage_L+BUFFER < voltage_R)) {
	        //Turning 45 degrees to the right (move motor 1 forward)
			P1_0 = 0;
			P1_1 = 1;
			P1_2 = 0;
			P1_3 = 0;
		}
		if ((voltage_R+BUFFER < distanceV) && (voltage_R+BUFFER < voltage_L)) {
	        //Turning 45 degrees to the left (move motor 2 forward)
			P1_0 = 0;
			P1_1 = 0;
			P1_2 = 1;
			P1_3 = 0;
		}
		if ((voltage_L-BUFFER > distanceV) && (voltage_L > voltage_R+BUFFER-0.015)) {
	        //Turning 45 degrees to the left (move motor 1 backward)
			P1_0 = 1;
			P1_1 = 0;
			P1_2 = 0;
			P1_3 = 0;
		}
		if ((voltage_R-BUFFER > distanceV) && (voltage_R > voltage_L+BUFFER-0.015)) {
	        //Turning 45 degrees to the right (move motor 2 backward)
			P1_0 = 0;
			P1_1 = 0;
			P1_2 = 0;
			P1_3 = 1;
		}
		
	    //try to move forwards
		if ((voltage_R-BUFFER < distanceV) && (voltage_L-BUFFER < distanceV) && ( (voltage_R - voltage_L < LR_BUFFER ) || (voltage_L - voltage_R < LR_BUFFER ) ) ) {
	        //move forward straight
			P1_0 = 0;
			P1_1 = 1;
			P1_2 = 1;
			P1_3 = 0;
		}
		
		if ((voltage_R-BUFFER > distanceV) && (voltage_L-BUFFER > distanceV) && ( (voltage_R - voltage_L < LR_BUFFER+0.025 ) || (voltage_L - voltage_R < LR_BUFFER+0.025 ) ) )  {
			P1_0 = 1;
			P1_1 = 0;
			P1_2 = 0;
			P1_3 = 1;
		} // move backwards since too close
		
		/*
		if ((voltage_R-distanceV < 0.01) && (voltage_L-distanceV < 0.01) && ( (voltage_R - voltage_L < 0.01 ) || (voltage_L - voltage_R < 0.01 ) ) )  {
		  	P1_0 = 0;
			P1_1 = 0;
			P1_2 = 0;
			P1_3 = 0;
		} // stay stil if voltages are equal to distance and are equal to one another
		*/
		
		
		
	}
	else if (orientation == BACKWARD) {
		if ((voltage_R-BUFFER > distanceV) && (voltage_R > voltage_L)) {
            //Turning 45 degrees to the left (move motor 2 forward)
			P1_0 = 1;
			P1_1 = 0;
			P1_2 = 0;
			P1_3 = 0;
		}
		if ((voltage_L-BUFFER > distanceV) && (voltage_L > voltage_R)) {
            //Turning 45 degrees to the right(move motor 1 forward)
			P1_0 = 0;
			P1_1 = 0;
			P1_2 = 0;
			P1_3 = 1;
		}
		if ((voltage_R+BUFFER < distanceV) && (voltage_R < voltage_L)) {
            //Turning 45 degrees to the left (move motor 2 backward)
			P1_0 = 0;
			P1_1 = 1;
			P1_2 = 0;
			P1_3 = 0;
		}
		if ((voltage_L+BUFFER < distanceV) && (voltage_L < voltage_R)) {
            //Turning 45 degrees to the right (move motor 2 backward)
			P1_0 = 0;
			P1_1 = 0;
			P1_2 = 1;
			P1_3 = 0;
		}
//try to move forwards
		if ((voltage_R+BUFFER > distanceV) && (voltage_L+BUFFER > distanceV) && (voltage_R - voltage_L < LR_BUFFER )) {
	        //move forward straight
			P1_0 = 0;
			P1_1 = 1;
			P1_2 = 1;
			P1_3 = 0;
		}
		
		if ((voltage_R+BUFFER < distanceV) && (voltage_L+BUFFER < distanceV) && (voltage_R - voltage_L < LR_BUFFER )) {
	        //Turning 45 degrees to the right (move motor 2 backward)
			P1_0 = 1;
			P1_1 = 0;
			P1_2 = 0;
			P1_3 = 1;
	    }//try to move backwards

	}
    // will only stop if distance 1 is not greater than distance buffer, is not less than distance with buffer
    // and distance 2 is not     
}


int waithalfs (void)
{
	_asm	
	;For a 22.1184MHz crystal one machine cycle 
	;takes 12/22.1184MHz=0.5425347us
	mov R2, #1
	L3:	mov R1, #248
	L2:	mov R0, #184
	L1:	djnz R0, L1 ; 2 machine cycles-> 2*0.5425347us*184=200us
	djnz R1, L2 ; 200us*250=0.05s
	djnz R2, L3 ; 0.05s*5=0.25s
	ret
	_endasm;
	
	return 1;
}

void mov_forward_0 ( void )
{
	while (P_JF == 1)
	{
		P1_0=0;
		P1_1=1;
		P1_2=1;
		P1_3=0;
	}
	return;
}

void mov_backward_0 ( void )
{
	while (P_JB == 1)
	{
		P1_0=1;
		P1_1=0;
		P1_2=0;
		P1_3=1;
	}
	return; 
}

void mov_forward_1 ( void )
{
	while (P_JB == 1)
	{
		P1_0=0;
		P1_1=1;
		P1_2=1;
		P1_3=0;
	}
	return;
}

void mov_backward_1 ( void )
{
	while (P_JF == 1)
	{
		P1_0=1;
		P1_1=0;
		P1_2=0;
		P1_3=1;
	}
	return; 
}

void turn_R ( void )
{
	while (P_JR == 1)
	{
		P1_0=0;
		P1_1=1;
		P1_2=0;
		P1_3=1;
	}
	return;
}

void turn_L ( void )
{	
	while (P_JL == 1)
	{
		P1_0=1;
		P1_1=0;
		P1_2=1;
		P1_3=0;
	}
	return; 
}

void turn_180_J ( void )
{
	int wait;
	P1_0=0;
	P1_1=1;
	P1_2=0;
	P1_3=1;
	
	for (wait = 0; wait < 20; wait++)
	{	
		waithalfs();
	}
	refresh();
	return;
}

void p_park_J ( void )
{
	int wait;
	//Turning 45 degrees left
	P1_0=1;
	P1_1=0;
	P1_2=1;
	P1_3=0;

	for (wait = 0; wait < 7; wait++)
	{	
		waithalfs();
	}
	
    //move back
	P1_0=1;
	P1_1=0;
	P1_2=0;
	P1_3=1;

	for (wait = 0; wait < 15; wait++)
	{	
		waithalfs();
	}

    //Turning 45 degrees to the right
	P1_0=0;
	P1_1=0;
	P1_2=0;
	P1_3=1;

	for (wait = 0; wait < 9; wait++)
	{	
		waithalfs();
	}

    //move backwards
	P1_0=1;
	P1_1=0;
	P1_2=0;
	P1_3=1;
	
	for (wait = 0; wait < 6; wait++)
	{	
		waithalfs();
	}

    //move forwards
	P1_0=0;
	P1_1=1;
	P1_2=1;
	P1_3=0;
	
	for (wait = 0; wait < 9; wait++)
	{	
		waithalfs();
	}
	
	refresh();
	return;
}

void manual_overide ( void )
{	
	int orientation = 0;
	
	while(P_OS != 0)
	{
		if (orientation == 0)
		{
			if (P_BP == 1)
				p_park_J();
			if (P_B8 == 1)
			{
				turn_180_J();
				orientation = 1;
			}	
			if (P_JF == 1)
				mov_forward_0();
			if (P_JB == 1)
				mov_backward_0();
			if (P_JR == 1)
				turn_R();
			if (P_JL == 1)
				turn_L();
			
			refresh();
		}
		
		else
		{
			if (P_BP == 1)
				p_park_J();
			if (P_B8 == 1)
			{
				turn_180_J();
				orientation = 0;
			}
			if (P_JB == 1)
				mov_forward_1();
			if (P_JF == 1)
				mov_backward_1();
			if (P_JR == 1)
				turn_R();
			if (P_JL == 1)
				turn_L();
			
			refresh();
		}
		
		refresh();
	}
	return;
}

void main (void)
{
	distanceV = 2.9; // defaults to 110mm
	orientation = FORWARD;
	P3_5 = 0;   


	while(1)
	{
		int wait;
		int byte = 0;
		int iter;
		float toprint;
		iter = 6000;
		toprint = GetADC(1)*4.95/1023.0;

		printf("GetADC: %f\n", toprint);
		//wait_time(iter);
		if (P_OS == 1)
			manual_overide();
		
		if ((GetADC(1)*4.95)/1023.0 <.2 )					//as soon as there's a start bit...
		{
			refresh();
			wait_check();					//wait half a bit to make sure it actually is zero...
			if((GetADC(1)*4.95)/1023.0 < .2)
			{
				byte = rx_byte(0, 1);		//read the rest of the byte!
				printf("Byte: %i\n", byte);
			}

			if ((byte == 251) || (byte == 253) || (byte == 249))  {
				printf("Button 1 Pressed");// do a 180
				turn_180_J();
				if (orientation == FORWARD){
					orientation = BACKWARD;
				}
				else
					orientation = FORWARD;
				
				
			}
			else if ((byte == 24) || (byte == 140) || (byte == 136)) {
				printf("Button 2 Pressed");
				p_park_J(); // parallel park
				
			}
			else if ((byte == 55) || (byte == 177) || (byte == 103) || (byte == 39)) {
				printf("Button 3 Pressed: Come Closer");
				//testmove();
				refresh();
				
				if (distanceV == 2.9) { // defaults to 2.9V, 110mm
					distanceV = 3.75;
				}
				else if (distanceV == 2.5) {
					distanceV = 2.9;
				}
				else if (distanceV == 1.1) {
					distanceV = 2.5;
				}
				printf("Distance now is %f \n", distanceV);
				waithalfus();

			}
			else if ((byte == 198) || (byte == 226) || (byte == 230) || (byte == 134)) {
				printf("Button 4 Pressed: Go Further");
				refresh();
				
				if (distanceV == 3.75) { 
					
					distanceV = 2.9;
				}
				else if (distanceV == 2.9) { // defaults to 2.9V, 110mm
					distanceV = 2.5;
				}
				else if (distanceV == 2.5) {
					distanceV = 1.1;
				}
				printf("Distance now is %f \n", distanceV);
				waithalfus();
				
			}
			else if ((byte == 241) || (byte == 113) || (byte == 30)) {
				printf("Button 5 Pressed");
			} 
		}			

		else{
			getMaxVoltages();
			follow();
		}
	}
}    