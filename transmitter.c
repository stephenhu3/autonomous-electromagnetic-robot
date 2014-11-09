#include <stdio.h>
#include <at89lp51rd2.h>
#include <stdlib.h>
#include <math.h>

// ~C51~ 

#define CLK 22118400L
#define BAUD 115200L
#define BRG_VAL (0x100-(CLK/(32L*BAUD)))

/* PINS:
 * P2_0 = LEFT MOST SWITCH 		= decrease distance
 * P2_1 = next to the right 	= increase distance
 * P2_2 = "						= 180 degree switch
 * P2_3 = RIGHT MOST SWITCH		= parallel park
 * P2_4 = SLIDE SWITCH			= something super cool
 */

#define FREQ 5000000L
#define TIMER0_RELOAD_VALUE 65478L	//65478L is the magic number for f=15.92kHz!

#define ITERATIONS 6000 //MAKE SURE THIS IS THE SAME ON BOTH MCUS

//These variables are used in the ISR
 volatile unsigned char pwmcount;
 volatile unsigned char pwm1;
 volatile unsigned char pwm2;
 unsigned int txon;

 unsigned char _c51_external_startup(void)
 {
	// Configure ports as a bidirectional with internal pull-ups.
 	P0M0=0;	P0M1=0;
 	P1M0=0;	P1M1=0;
 	P2M0=0;	P2M1=0;
 	P3M0=0;	P3M1=0;
	AUXR=0B_0001_0001; // 1152 bytes of internal XDATA, P4.4 is a general purpose I/O
	P4M0=0;	P4M1=0;
	
    // Initialize the serial port and baud rate generator
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

void pwmcounter (void) interrupt 1
{
	if(txon==1)
	{
		P1_0 = P1_0 ^ 1;
		P1_1 = P1_1 ^ 1;
	}
}

void wait_bit_time()
{
	int n=ITERATIONS;
	while (n>0)
	{
		n--;
	}
	return;
}


void tx_byte ( unsigned char val )
{
	unsigned char j;

	//send the start bit
	txon=0;
	wait_bit_time();
	for (j=0; j<8; j++)
	{
		txon=val&(0x01<<j)?1:0;
		wait_bit_time();
	}
	txon = 1;
	//send the stop bits
	wait_bit_time();
	wait_bit_time();
}


void main (void)
{
	P1_0=0;
	P1_1=1; 
	txon = 1;

	while(1)
	{
		//tx_byte();
		if (P2_0==1)
			tx_byte(25); //left most switch
		if (P2_1==1)
			tx_byte(50); //next
		if (P2_2==1)
			tx_byte(75); //next
		if (P2_3==1)
			tx_byte(100); //next
		if (P2_4==1)
			tx_byte(125); //right most switch

	}

}