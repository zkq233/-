#ifndef __TIMER_H__
#define __TIMER_H__

#include "z_stc15.h"

#define timer0_open() {TR0 = 1;ET0 = 1;}
#define timer0_close() {TR0 = 0;ET0 = 0;}

#define timer1_open() {TR1 = 1;ET1 = 1;}
#define timer1_close() {TR1 = 0;ET1 = 0;}

void timer0_init(void);
void timer1_init(void);
void timer2_init(void);
void timer3_init(void);

//void timer0_reset(int t_us);
void timer1_reset(int t_us);
void duoji_inc_handle(u8 index);
u32 millis(void);

#endif



