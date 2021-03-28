#include <stdio.h>
#include <string.h>

#include "z_sensor.h"
#include "z_adc.h"
#include "z_global.h"
#include "z_gpio.h"
#include "z_pwm.h"
#include "z_timer.h"
#include "z_uart.h"
#include "z_main.h"
#include "z_w25q64.h"


#define COLOR_RED_BASE 132 //红色基准色
#define COLOR_GRN_BASE 168 //绿色基准色
#define COLOR_BLU_BASE 180 //蓝色基准色
int color_red_base, color_grn_base, color_blu_base;

/*
	智能功能代码
*/

//左 P06 右 P07 默认标准输入 0 0
void setup_xunji(void) {
//	P0M1 &= ~(1<<6);
//	P0M0 &= ~(1<<6);
//	
//	P0M1 &= ~(1<<7);
//	P0M0 &= ~(1<<7);
}

//P10 默认标准输入 0 0
void setup_sound(void) {
//	P1M1 &= ~(1<<0);
//	P1M0 &= ~(1<<0);
}

//P12 AD
void setup_csb(void) {
	//adc_init(ADC_CSB);
//	P1M0 |= 1<<6;
//	P1M1 &= ~(1<<6);
	
//	P1M0 |= 1<<2;
//	P1M1 &= ~(1<<2);
	
	timer0_init();//超声波计数定时器
}

//P16 AD
void setup_yssb(void) {
	adc_init(ADC_YSSB);
	if(eeprom_info.color_base_flag != 0x29) {
		eeprom_info.color_base_flag = 0x29;
		color_red_base = COLOR_RED_BASE;
		color_grn_base = COLOR_GRN_BASE;
		color_blu_base = COLOR_BLU_BASE;
		rewrite_eeprom();
		uart1_send_str("No Color!");
	} else {
		color_red_base = eeprom_info.color_red_base;
		color_grn_base = eeprom_info.color_grn_base;
		color_blu_base = eeprom_info.color_blu_base;
		uart1_send_str("Get Color!");
	}
	sprintf((char *)uart_receive_buf, "red = %d grn = %d blu = %d\r\n", color_red_base, color_grn_base, color_blu_base);
	uart1_send_str(uart_receive_buf);
}

//初始化传感器IO口
void setup_sensor(void) {
	setup_xunji();	//初始化循迹
	setup_sound();	//初始化声音
	setup_csb();	//初始化超声波
	setup_yssb();	//初始化颜色识别
}

//处理智能传感器功能
void loop_smart_sensor(void) {
	if(smart_mode == 0) {
		smart_xunji();				//循迹模式
	} else if(smart_mode == 1) {
		smart_soundjiaqu();			//声控夹取
	} else if(smart_mode == 2) {
		smart_ziyoubizhang();		//自由避障
	} else if(smart_mode == 3) {
		smart_yssbjiaqu();			//颜色识别
	} else if(smart_mode == 4) {
		smart_csbjiaqu();			//定距夹取
	} else if(smart_mode == 5) {
		smart_gensui();				//跟随功能
	} else if(smart_mode == 6) {
		smart_xunjibizhang();		//循迹避障
	} else if(smart_mode == 7) {
		smart_xunjiyanse();			//循迹识别
	} else if(smart_mode == 8) {
		smart_xunjicsbjiaqu();		//循迹定距
	} else if(smart_mode == 9) {
		smart_yssb_verify();		//颜色校验
	}	else if(smart_mode == 10) {
		group_do_ok = 1;	    	//
	}
}
/*************************************************************
函数名称：get_adc_yssb_middle()
功能介绍：获取灰度传感器采集到的值并返回
函数参数：无
返回值：  采集的数据  
*************************************************************/
int get_adc_yssb_middle() {
	u8 i;
	static int ad_value[5] = {0}, ad_value_bak[5] = {0}, ad_index = 0, myvalue;
	ad_value[ad_index] = adc_read(ADC_YSSB);
	ad_value_bak[ad_index] = ad_value[ad_index];
	ad_index ++ ;
	if(ad_index == 5)ad_index = 0;
	selection_sort(ad_value, 5);
	myvalue = ad_value[2];
	for(i=0;i<5;i++)ad_value[i] = ad_value_bak[i];
	return myvalue;  
}
/*************************************************************
函数名称：get_csb_value()
功能介绍：采集超声波数据
函数参数：无
返回值：  采集的数据  
*************************************************************/
int get_csb_value(void) {
		float distance;
		u16 i = 0, k=0, time;
		u8 j=120;

		TH1 = TL1 = csb_cnt = 0; //初始化定时器0
		Trig = 1; //拉高超声波模块触发IO
		while(j--); // 延时20us
		Trig = 0; //拉低超声波模块触发IO
		i = 0;k=0;
		while(!Echo);//等待超声波模块输出IO拉高 
		TR1 = 1; //开启定时器计时
		while(Echo ); //等待超声波模块输出IO拉低
		TR1 = 0; //关闭定时器计时
	
		//判断是否超出模块最大测距 4m
		//计算出时间 转换成US 
		time = (TH1 * 256 + TL1 + csb_cnt*65536) * (1.0000/22.1184);
		//计算出距离 340m/s =  0.34cm/us 半程 0.17mm/us
		distance = time * 0.17;

		//sprintf(cmd_return, "distance = %.2f cm i=%d k=%d\r\n", distance, i, k);
		//uart1_send_str(cmd_return);

		//复位定时器
		TL1 = 0;		//设置定时初值
		TH1 = 0;		//设置定时初值
		TF1 = 0;		//清除TF1标志
		return distance;
}
/*************************************************************
函数名称：get_adc_csb_middle()
功能介绍：处理超声波采集到的数据，取采集到的中间值
函数参数：无
返回值：  处理后的超声波数据  
*************************************************************/
int get_adc_csb_middle() {
	u8 i;
	static int ad_value[5] = {0}, myvalue;
	for(i=0;i<5;i++)ad_value[i] = get_csb_value();
	selection_sort(ad_value, 5);
	myvalue = ad_value[2];
	return myvalue;  
}

/*************************************************************
函数名称：smart_xunji()
功能介绍：实现循迹功能
函数参数：无
返回值：  无  
*************************************************************/
void smart_xunji(void) {
	int speed = 500;
	if((xj0() == 0) && (xj1() == 1)) {
		car_pwm_set(speed+200, 0);
	} else if((xj0() == 1) && (xj1() == 1)) {
		car_pwm_set(speed, speed);
	} else if((xj0() == 1) && (xj1() == 0)) {
		car_pwm_set(0, speed+200);
	}	
}
/*************************************************************
函数名称：smart_xunjibizhang()
功能介绍：在循迹的过程中，检测有障碍物，则停止，否则继续循迹
函数参数：无
返回值：  无  
*************************************************************/
void smart_xunjibizhang(void) {
	static u32 systick_ms_bak = 0;
	int adc_csb;
	
	if(millis() - systick_ms_bak > 50) {
		systick_ms_bak = millis();
		//避障处理
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
		//sprintf((char *)uart_receive_buf, "adc_csb = %d\r\n", adc_csb);
		//uart1_send_str(uart_receive_buf);
		if(adc_csb < 250) {//距离低于250mm就停止
			car_pwm_set(0, 0);
		} else {
			//循迹处理
			smart_xunji();
		}
	}
}

/*************************************************************
函数名称：smart_gensui()
功能介绍：检测物体距离，在一定距离内实现跟随功能
函数参数：无
返回值：  无  
*************************************************************/
void smart_gensui(void) {
	static u32 systick_ms_bak = 0;
	int speed = 600, adc_csb;
	if(millis() - systick_ms_bak > 20) {
		systick_ms_bak = millis();
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
		if((adc_csb > 300) && (adc_csb < 700)  ) {//距离30~70cm前进
			car_pwm_set(speed, speed);
		} else if(adc_csb < 200) {//距离低于20cm就后退
			car_pwm_set(-speed, -speed);
		} else {//其他情况停止
			car_pwm_set(0, 0);
		}
	}
}
/*************************************************************
函数名称：smart_ziyoubizhang()
功能介绍：识别物体距离从而避开物体前进
函数参数：无
返回值：  无  
*************************************************************/
void smart_ziyoubizhang(void) {
	static u32 systick_ms_bak = 0;
	int speed = 900, adc_csb;
	if(millis() - systick_ms_bak > 100) {
		systick_ms_bak = millis();
		
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
//		sprintf((char *)uart_receive_buf, "adc_csb = %d mm\r\n", adc_csb);
//		uart1_send_str(uart_receive_buf);
		if(adc_csb < 500) {//距离低于50cm就右转
			car_pwm_set(speed, -speed);
		} else {
			car_pwm_set(speed, speed);
		}
	}
}
/*************************************************************
函数名称：smart_csbjiaqu()
功能介绍：识别物体距离夹取物体
函数参数：无
返回值：  无  
*************************************************************/
void smart_csbjiaqu(void) {
	static u32 systick_ms_bak = 0;
	int adc_csb;
	if(group_do_ok == 0)return;
	if(check_dj_state())return;

	//每20ms计算一次
	if(millis() - systick_ms_bak > 20) {
		systick_ms_bak = millis();
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
//		sprintf((char *)uart_receive_buf, "adc_csb = %d\r\n", adc_csb);
//		uart1_send_str(uart_receive_buf);
		if((adc_csb > 130) && (adc_csb < 170)) {//距离15cm左右就夹取
			parse_cmd((u8 *)"$DGT:3-11,1!");
			beep_on_times(1, 100);
		} 
	}
}
/*************************************************************
函数名称：smart_yssbjiaqu()
功能介绍：识别木块颜色，夹取分别放到不同位置
函数参数：无
返回值：  无  
*************************************************************/
void smart_yssbjiaqu(void) {
	static u8 getCount = 0, getCountOk = 0, i;
	static u32 systick_ms_bak = 0, systick_ms_bak2 = 0;
	static int adc_yssb;
	
	if(group_do_ok == 0)return;
	if(check_dj_state())return;
	
//	sprintf((char *)uart_receive_buf, "millis() = %ld  systick_ms_bak3 = %ld  nextComeTime = %ld\r\n", millis(), systick_ms_bak3, nextComeTime);
//	uart1_send_str(uart_receive_buf);
		
	//每20ms获取一次数据
	if(millis() - systick_ms_bak2 > 20) {
		systick_ms_bak2 = millis();
		getCount ++;
		adc_yssb = get_adc_yssb_middle();
		if( getCount >= 5) {
			getCount = 0;
			getCountOk = 1;
		}
	}
	
	//每5次识别一次
	if((millis() - systick_ms_bak > 100) && (getCountOk == 1)) {
		systick_ms_bak = millis();
		getCountOk = 0;
		//获取a0的ad值
		sprintf((char *)uart_receive_buf, "adc_yssb = %d\r\n", adc_yssb);
		uart1_send_str(uart_receive_buf);
		
		if((adc_yssb>color_red_base-10) && (adc_yssb<color_red_base+10)) {
			//RED
			for(i=0;i<5;i++) {
				adc_yssb = get_adc_yssb_middle();
				mdelay(20);
			}
			if((adc_yssb>color_red_base-10) && (adc_yssb<color_red_base+10)) {
				//uart1_send_str("RED");
				parse_cmd((u8 *)"$DGT:12-18,1!");
				beep_on_times(1, 100);
			}
		} else if((adc_yssb>color_grn_base-10) && (adc_yssb<color_grn_base+10)) {
			//GRN
			for(i=0;i<5;i++) {
				adc_yssb = get_adc_yssb_middle();
				mdelay(20);
			}
			if((adc_yssb>color_grn_base-10) && (adc_yssb<color_grn_base+10)) {
				//uart1_send_str("GRN");
				parse_cmd((u8 *)"$DGT:19-27,1!");
				beep_on_times(1, 100);
			}
		} else if((adc_yssb>color_blu_base-10) && (adc_yssb<color_blu_base+10)) {
			//BLU
			for(i=0;i<5;i++) {
				adc_yssb = get_adc_yssb_middle();
				mdelay(20);
			}
			if((adc_yssb>color_blu_base-10) && (adc_yssb<color_blu_base+10)) {
				//uart1_send_str("BLU");
				parse_cmd((u8 *)"$DGT:28-36,1!");
				beep_on_times(1, 100);
			}
		}  
		
	}	
}
/*************************************************************
函数名称：smart_soundjiaqu()
功能介绍：检测1S内声音次数，响一次夹取物体放到一边，响俩次放另一边
函数参数：无
返回值：  无  
*************************************************************/
void smart_soundjiaqu(void) {
	
	static u32 systick_ms_bak = 0, systick_ms_bak2 = 0;
	static int soundCount = 0;
//	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	if(group_do_ok == 0)return;//有动作执行，直接返回
	if(check_dj_state())return;

	if(millis() - systick_ms_bak > 10) {//计算在一秒内有几次响声
		systick_ms_bak = millis();
		if(sound() == 0) {
			soundCount++;
			while(sound() == 0);
			if(soundCount == 1)systick_ms_bak2 = millis();
		}
	}
	
	if(millis() - systick_ms_bak2 > 1000) {
		systick_ms_bak2  = millis();
		
//		if(soundCount) {
//			sprintf((char *)uart_receive_buf, "soundCount = %d\r\n", (int)soundCount);
//			uart1_send_str(uart_receive_buf);
//		}	
		
		if(soundCount>1) {			//右放
			parse_cmd((u8 *)"$DGT:28-36,1!");
			beep_on_times(2, 100);
	
		} else if(soundCount>0){	//左放
			parse_cmd((u8 *)"$DGT:19-27,1!");
			beep_on_times(1, 100);
		} 
		soundCount = 0;	
	}

	
	
}
/*************************************************************
函数名称：smart_xunjicsbjiaqu()
功能介绍：在循迹的过程中实现定距夹取功能
函数参数：无
返回值：  无  
*************************************************************/
void smart_xunjicsbjiaqu() {
	static u32 systick_ms_bak = 0, nextStepTime = 50;
	int adc_csb;	
	if(group_do_ok == 0)return;
	if(check_dj_state())return;
	adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
	if(millis() - systick_ms_bak > nextStepTime) {
		systick_ms_bak = millis();
		//sprintf((char *)uart_receive_buf, "adc0 = %d\r\n", adc_a0);
		//uart1_send_str(uart_receive_buf);
		
		if(nextStepTime > 2000)beep_on_times(1, 300);
		
		if((adc_csb > 100-15) && (adc_csb < 100+15)) {//距离290mm左右就夹取
			car_pwm_set(0, 0);
			nextStepTime = 10000;
			parse_cmd((u8 *)"$DGT:3-11,1!");
		} else {
			smart_xunji();
			nextStepTime = 50;
		}
	}
}
/*************************************************************
函数名称：smart_xunjiyanse()
功能介绍：在循迹的过程中实现颜色识别功能，并将颜色模块移开
函数参数：无
返回值：  无  
*************************************************************/
void smart_xunjiyanse(void) {	
	static u8 getCount = 0, getCountOk = 0, i, shibieOK = 0;
	static u32 systick_ms_bak = 0, systick_ms_bak2 = 0,  systick_ms_bak3 = 0;
	static int adc_yssb;
	
	if(group_do_ok == 0)return;
	if(check_dj_state())return;
	
//	sprintf((char *)uart_receive_buf, "millis() = %ld  systick_ms_bak3 = %ld  nextComeTime = %ld\r\n", millis(), systick_ms_bak3, nextComeTime);
//	uart1_send_str(uart_receive_buf);
		
	//每20ms获取一次数据
	if(millis() - systick_ms_bak2 > 20) {
		systick_ms_bak2 = millis();
		getCount ++;
		adc_yssb = get_adc_yssb_middle();
		if( getCount >= 5) {
			getCount = 0;
			getCountOk = 1;
		}
	}
	
	//每5次识别一次
	if((millis() - systick_ms_bak > 100) && (getCountOk == 1)) {
		systick_ms_bak = millis();
		getCountOk = 0;
		//获取a0的ad值
		//sprintf((char *)uart_receive_buf, "adc_yssb = %d\r\n", adc_yssb);
		//uart1_send_str(uart_receive_buf);
		
		if((adc_yssb>color_red_base-10) && (adc_yssb<color_red_base+10)) {
			//RED
			car_pwm_set(0, 0);
			for(i=0;i<5;i++) {
				adc_yssb = get_adc_yssb_middle();
				mdelay(20);
			}
			if((adc_yssb>color_red_base-10) && (adc_yssb<color_red_base+10)) {
				//uart1_send_str("RED");
				shibieOK = 1;
				//parse_cmd((u8 *)"$DGT:12-18,1!");
				
				parse_cmd((u8 *)"$DGT:19-27,1!");
				beep_on_times(1, 100);
			}

		} else if((adc_yssb>color_blu_base-10) && (adc_yssb<color_blu_base+10)) {
			//BLU
			car_pwm_set(0, 0);
			for(i=0;i<5;i++) {
				adc_yssb = get_adc_yssb_middle();
				mdelay(20);
			}
			if((adc_yssb>color_blu_base-10) && (adc_yssb<color_blu_base+10)) {
				//uart1_send_str("BLU");
				shibieOK = 1;
				
				parse_cmd((u8 *)"$DGT:28-36,1!");
				beep_on_times(1, 100);
			}
		} else {
			shibieOK = 0;
		} 
		
	}	
	
	if(shibieOK == 0) {
		if(millis() - systick_ms_bak3 > 50) {
			systick_ms_bak3 = millis();
			smart_xunji();
		}
	}
}
/*************************************************************
函数名称：smart_yssb_verify()
功能介绍：颜色校准函数
函数参数：无
返回值：  无  
*************************************************************/
void smart_yssb_verify(void) {
	u16 adc_yssb, i;
		
	//校验红色
	beep_on_times(1,1000);
	for(i=0;i<500;i++) {
		adc_yssb = get_adc_yssb_middle();
		mdelay(10);
	}
	color_red_base = adc_yssb;
	beep_on_times(1,1000);
	
	//校验绿色
	beep_on_times(2,300);
	for(i=0;i<500;i++) {
		adc_yssb = get_adc_yssb_middle();
		mdelay(10);
	}
	color_grn_base = adc_yssb;
	beep_on_times(1,1000);
	
	//校验蓝色
	beep_on_times(3,200);
	for(i=0;i<500;i++) {
		adc_yssb = get_adc_yssb_middle();
		mdelay(10);
	}
	color_blu_base = adc_yssb;
	beep_on_times(1,1000);

	eeprom_info.color_red_base = color_red_base;
	eeprom_info.color_grn_base = color_grn_base;
	eeprom_info.color_blu_base = color_blu_base;
	
	//存储
	rewrite_eeprom();

	//打印测试
	sprintf((char *)uart_receive_buf, "red = %d grn = %d blu = %d\r\n", color_red_base, color_grn_base, color_blu_base);
	uart1_send_str(uart_receive_buf);
	
	smart_mode = 10;
	
}