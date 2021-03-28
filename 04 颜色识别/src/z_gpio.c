#include "z_gpio.h"


void io_init(void) {

	P0M1=0x00;			           
	P0M0=0x30;

	P1M1=0x00;			           
	P1M0=0x00;

	P2M1=0x00;
	P2M0=0x1e;
			          
	P3M1=0x00;
	P3M0=0x90;	

	P4M1=0x00;			          
	P4M0=0x00;

//	P5M1=0x00;			         
//	P5M0=0x00;
	
	key1_led = 1;
	key2_led = 1;
}


void dj_io_init(void) {
	dj0 = 1;
	dj1 = 1;
	dj2 = 1;
	dj3 = 1;
	dj4 = 1;
	dj5 = 1;
}

void dj_io_set(u8 index, u8 level) {
	switch(index) {
		case 0:dj0 = level;break;
		case 1:dj1 = level;break;
		case 2:dj2 = level;break;
		case 3:dj3 = level;break;
		case 4:dj4 = level;break;
		case 5:dj5 = level;break;
		default:break;
	}
}

void beep_on_times(int times, int delay) {
	int i;
	for(i=0;i<times;i++) {
		beep_on();
		mdelay(delay);
		beep_off();
		mdelay(delay);
	}
}