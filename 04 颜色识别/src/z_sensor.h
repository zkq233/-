#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "z_stc15.h"



#define ADC_YSSB	2

sbit io_xj0 = P0^6;
sbit io_xj1 = P0^7;
sbit io_sound = P1^0;	//

sbit Trig  = P1^6; 		//产生脉冲引脚
sbit Echo  = P1^1; 		//回波引脚

#define xj0() 	io_xj0
#define xj1() 	io_xj1
#define sound() io_sound

//处理智能传感器功能
void setup_sensor(void);
void loop_smart_sensor(void);
int get_csb_value(void);

void smart_xunji(void); 			//循迹功能
void smart_soundjiaqu(void);		//静态声音识别夹取
void smart_yssbjiaqu(void);			//静态颜色识别夹取
void smart_xunjiyanse(void);		//循迹颜色夹取
void smart_csbjiaqu(void);			//静态超声波夹取
void smart_xunjibizhang(void);		//循迹超声波避障
void smart_gensui(void);			//超声波跟随功能
void smart_ziyoubizhang(void);		//超声波自由避障
void smart_xunjicsbjiaqu(void);		//循迹超声波夹取
void smart_yssb_verify(void);		//颜色识别校验



#endif