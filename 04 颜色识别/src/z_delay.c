#include "z_delay.h"

void delay(unsigned int t) {
	
	while(t--);
	
}

void udelay(unsigned int t) {
	unsigned char i;
	while(t--) {
		i = 3;
		while (--i);
	}
}

void mdelay(unsigned int t) {
	unsigned int i, j;
	for(i=0;i<t;i++) {
		for(j=0;j<1000;j++);
	}
}