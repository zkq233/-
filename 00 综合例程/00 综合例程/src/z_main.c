/*
	1、串口1调试		ok
	2、串口2调试		ok
	3、定时器0调试		ok
	4、定时器1调试		ok
	5、PS2手柄调试		ok
	6、4通道PWM调试		ok
	7、舵机调试			ok
	8、W25Q64存储调试	ok
	
	调试的过程：
	如上，一个一个模块调通，最后组合
	左边的目录结构就是正队每一个模块调试好做成一个模块文件，便于移植
	
	看程序方法：
	看程序的时候，从main文件的main函数看起
	基本的程序思路是
	主函数->各个模块初始化->大循环while(1) 
						  ->中断(串口、定时器等)
	大家在深究本程序时，建议大家先去了解各个模块的原理，然后看懂文件结构和程序结构，最后再细究算法问题
	
	智能传感器版本增加内容：
	所需硬件
	循迹模块 2个
	超声波模块 1个
	颜色识别模块 1个
	声音模块 1个
	木块红、蓝、绿各一个
	
	IO口分布
	循迹左 	P0^6
	循迹右 	P0^7
	超声波 	P1^6
	颜色	P1^2
	声音	P1^0
	
	智能识别功能（手柄绿灯模式 左边 上下左右 和 右边 上下左右）
	
	功能0 循迹模式
	功能1 声控夹取
	功能2 自由避障
	功能3 颜色识别
	功能4 定距夹取
	功能5 跟随功能
	功能6 循迹避障
	功能7 循迹识别
	功能8 循迹定距
	
	手动遥控功能
	1、手柄遥控
	2、APP遥控
	3、WIFI遥控
	
	图形化编程功能
	
	功能切换：绿灯模式下通过左边上下左右键切换功能，通过蜂鸣器的声音的响声播报功能
	
	
*/

#include <stdio.h>
#include <string.h>
#include <intrins.h>
#include "z_stc15.h"
#include "z_main.h"
#include "z_uart.h"
#include "z_delay.h"
#include "z_gpio.h"
#include "z_ps2.h"
#include "z_pwm.h"
#include "z_timer.h"
#include "z_w25q64.h"
#include "z_global.h"
#include "z_adc.h"
#include "z_sensor.h"

u16 adc7_value = 0, adc0_value = 0;
u16 do_start_index, do_time, group_num_start, group_num_end, group_num_times;
u8 i;
u8 car_dw = 1;
u32 bias_systick_ms_bak = 0;
u8 djBiasSaveFlag = 0;
u8 dbt_flag = 0;
float vol_adc = 0;u32 save_addr_sector = 0, save_action_index_bak = 0;
u8 psx_buf[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 	//

code const char *pre_cmd_set_red[PSX_BUTTON_NUM] = {
	"<PS2_RED01:#005P0600T2000!^$DST!>",	//L2						  
	"<PS2_RED02:#005P2400T2000!^$DST!>",	//R2						  
	"<PS2_RED03:#004P0600T2000!^$DST!>",	//L1						  
	"<PS2_RED04:#004P2400T2000!^$DST!>",	//R1			
	"<PS2_RED05:#002P2400T2000!^$DST!>",	//RU						  
	"<PS2_RED06:#003P2400T2000!^$DST!>",	//RR						  
	"<PS2_RED07:#002P0600T2000!^$DST!>",	//RD						  
	"<PS2_RED08:#003P0600T2000!^$DST!>",	//RL				
	"<PS2_RED09:$DJR!>",					        //SE    				  
	"<PS2_RED10:$SMODE10!>",					    //AL						   
	"<PS2_RED11:$SMODE10!>",						  //AR						  
	"<PS2_RED12:$DGS:2!>",					      //ST			
	"<PS2_RED13:#001P0600T2000!^$DST!>",	//LU						  
	"<PS2_RED14:#000P0600T2000!^$DST!>",	//LR								  
	"<PS2_RED15:#001P2400T2000!^$DST!>",	//LD						  
	"<PS2_RED16:#000P2400T2000!^$DST!>",	//LL								
};

code const char *pre_cmd_set_grn[PSX_BUTTON_NUM] = {
	"<PS2_GRN01:$SMODE10!>",		          //L2						  
	"<PS2_GRN02:$SMODE10!>",		          //R2						  
	"<PS2_GRN03:$!>",		                  //L1						  
	"<PS2_GRN04:$SMODE6!>",		            //R1			
	"<PS2_GRN05:$SMODE4!>",	              //RU						  
	"<PS2_GRN06:$SMODE8!>",	              //RR						  
	"<PS2_GRN07:$SMODE5!>",	              //RD						  
	"<PS2_GRN08:$SMODE7!>",	              //RL				
	"<PS2_GRN09:$DJR!>",					        //SE   					  
	"<PS2_GRN10:$SMODE10!>",						  //AL					  
	"<PS2_GRN11:$SMODE10!>",						  //AR					  
	"<PS2_GRN12:$DGS:2!>",					      //ST			
	"<PS2_GRN13:$SMODE0!>",	              //LU						  
	"<PS2_GRN14:$SMODE3!>",	              //LR								  
	"<PS2_GRN15:$SMODE1!>",            	  //LD						  
	"<PS2_GRN16:$SMODE2!>",	              //LL						  
};


code const char *action_pre_group[] = {
    //动作生成数据为(可直接全选复制粘贴到程序中)：
    //偏差调节组
    "{G0000#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}",
    //直立
    "{G0001#000P1500T1500!#001P1500T1500!#002P1500T1500!#003P1500T1500!#004P1500T1500!#005P1500T0000!}",
    //蜷缩
    "{G0002#000P1500T1500!#001P2200T1500!#002P2500T1500!#003P2000T1500!#004P1500T1500!#005P1500T1500!}",
    //大前抓右放 K0001(3-11)
    "{G0003#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1000T1000!}",
    "{G0004#000P1500T1000!#001P1100T1000!#002P1600T1000!#003P2100T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0005#000P1500T1500!#001P1050T1500!#002P1600T1000!#003P2100T1000!#004P1500T1000!#005P2000T0000!}",
    "{G0006#000P1500T1000!#001P1800T1000!#002P1800T1000!#003P2100T1000!#004P1500T1000!#005P2000T0000!}",
    "{G0007#000P0800T1000!#001P1500T1000!#002P1500T1000!#003P2100T1000!#004P1500T1000!#005P2000T0000!}",
    "{G0008#000P0800T1000!#001P1300T1000!#002P1800T1000!#003P2100T1000!#004P1500T1000!#005P2000T0000!}",
    "{G0009#000P0800T1000!#001P1300T1000!#002P1800T1000!#003P2100T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0010#000P0800T1000!#001P1600T1000!#002P1800T1000!#003P2100T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0011#000P1500T1500!#001P2200T1500!#002P2500T1500!#003P2000T1500!#004P1500T1500!#005P1500T1500!}",
    //前爪前放 K0002(12-18)
    "{G0012#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}",
    "{G0013#000P1520T1000!#001P1100T2000!#002P1850T1000!#003P2100T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0014#000P1520T1000!#001P1100T2000!#002P1900T1000!#003P2100T1000!#004P1500T1000!#005P2000T0000!}",
    "{G0015#000P1520T1000!#001P1800T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0016#000P1520T1000!#001P1100T2000!#002P1800T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0017#000P1520T1000!#001P1100T2000!#002P1800T1000!#003P2000T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0018#000P1500T1500!#001P2200T1500!#002P2500T1500!#003P2000T1500!#004P1500T1500!#005P1500T1500!}",
    //前爪左放 K0003(19-27)
    "{G0019#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}",
    "{G0020#000P1520T1000!#001P1100T2000!#002P1850T1000!#003P2100T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0021#000P1520T1000!#001P1100T2000!#002P1900T1000!#003P2100T1000!#004P1500T1000!#005P2000T0000!}",
    "{G0022#000P1520T1000!#001P1800T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0023#000P2200T1000!#001P1800T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0024#000P2200T1000!#001P1200T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0025#000P2200T1000!#001P1200T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0026#000P2200T1000!#001P1800T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P1000T1000!}",
    "{G0027#000P1500T1500!#001P2200T1500!#002P2500T1500!#003P2000T1500!#004P1500T1500!#005P1500T1500!}",
    //前爪右放 K0004(28-36)
    "{G0028#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}",
    "{G0029#000P1520T1000!#001P1100T2000!#002P1850T1000!#003P2100T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0030#000P1520T1000!#001P1100T2000!#002P1900T1000!#003P2100T1000!#004P1500T1000!#005P2000T0000!}",
    "{G0031#000P1520T1000!#001P1800T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0032#000P0800T1000!#001P1800T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0033#000P0800T1000!#001P1200T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P2000T1000!}",
    "{G0034#000P0800T1000!#001P1200T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P1000T0000!}",
    "{G0035#000P0800T1000!#001P1800T2000!#002P2000T1000!#003P2000T1000!#004P1500T1000!#005P1000T1000!}",
    "{G0036#000P1500T1500!#001P2200T1500!#002P2500T1500!#003P2000T1500!#004P1500T1500!#005P1500T1500!}",
};

/*
	代码从main里开始执行
	在进入大循环while(1)之前都为各个模块的初始化
	最后在大循环处理持续执行的事情
	另外注意uart中的串口中断，接收数据处理
	timer中的定时器中断，舵机的脉冲收发就在那里
*/

void main(void) {
	
	setup_global();			//初始化全局变量
	setup_gpio();			//初始化IO口
	setup_nled();			//初始化工作指示灯
	setup_beep();			//初始化定时器
	setup_djio();			//初始化舵机IO口
	setup_w25q64();			//初始化存储器W25Q64
	setup_ps2();			//初始化PS2手柄
	setup_vol();			//初始化电压采集
	setup_car_pwm();		//初始化电机PWM定时
	setup_uart1();			//初始化串口1
	setup_uart2();			//初始化串口2
	setup_uart4();			//初始化串口4
	setup_systick();		//初始化滴答时钟，1S增加一次systick_ms的值
	setup_others();			//初始化其他
	setup_sensor();			//初始化传感器IO口
	
	setup_dj_timer();		//初始化定时器2 处理舵机PWM输出
	setup_interrupt();		//初始化总中断
	
	setup_start();			//初始化启动信号
	
    while (1) {
		loop_nled();			//循环执行工作指示灯，500ms跳动一次.
		loop_uart();			//串口数据接收处理
		loop_action();			//动作组批量执行
		loop_bt_once();			//蓝牙修改波特率和名称
		loop_ps2_data();		//循环读取PS2手柄数据
		loop_ps2_button();		//处理手柄上的按钮
		loop_ps2_car_pwm();		//处理小车电机摇杆控制
		loop_save_something();	//定时保存一些变量
		loop_smart_sensor();	//处理智能传感器功能
	       	//loop_monitor_servo();	//监视舵机增量
		//单个测试功能的时候可以用下面这些函数，功能总控函数为 loop_smart_sensor
		//smart_xunji();			  //循迹功能
		//smart_xunjibizhang();	//循迹避障
		//smart_gensui();			  //跟随功能
		//smart_ziyoubizhang();	//自由避障
		//smart_csbjiaqu();		  //超声波夹取
		//smart_yssbjiaqu();		//颜色识别夹取
		//smart_soundjiaqu();		//声音夹取
		//smart_xunjicsbjiaqu();//循迹超声波
		//smart_xunjiyanse();		//循迹颜色		
		
//		
//		sprintf(cmd_return, "dis=%d\r\n", (int)get_csb_value());
//		uart1_send_str(cmd_return);
//		mdelay(500);
	}
}

//--------------------------------------------------------------------------------
/*
	初始化函数实现
*/
//初始化全局变量
void setup_global(void) {
	//全局变量初始化
	global_init();
}
//初始化IO口
void setup_gpio(void) {
	//IO初始化
	io_init();
}
//初始化工作指示灯 初始化已在io_init中初始化
void setup_nled(void) {
	nled_off();			//工作指示灯关闭
}
//初始化蜂鸣器 初始化已在io_init中初始化
void setup_beep(void) {
	beep_off();			//关闭蜂鸣器
}			
//初始化舵机IO口
void setup_djio(void) {
	dj_io_init();		//舵机IO口初始化
}	

void setup_vol(void) {
	adc_init(ADC_VOL);
}

//初始化存储器W25Q64
void setup_w25q64(void) {
	//存储器初始化，读取ID进行校验，若错误则长鸣不往下执行
	w25x_init();
	while(w25x_readId()!= W25Q64)beep_on();
	
	w25x_read((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info_t));	//读取全局变量
	if(eeprom_info.version != VERSION) {	//判断版本是否是当前版本
		eeprom_info.version = VERSION;		//复制当前版本
		eeprom_info.dj_record_num = 0;		//学习动作组变量赋值0
		rewrite_eeprom();					//写入到存储器
	} 
	
	if(eeprom_info.dj_bias_pwm[DJ_NUM] != FLAG_VERIFY) {
		for(i=0;i<DJ_NUM;i++) {
			eeprom_info.dj_bias_pwm[i] = 0;
		}
		eeprom_info.dj_bias_pwm[DJ_NUM] = FLAG_VERIFY;
	}
	
	
}	

//初始化PS2手柄
void setup_ps2(void) {
	//手柄初始化
	psx_init();
}
//初始化定时器2 处理舵机PWM输出
void setup_dj_timer(void) {
	timer1_init();	//舵机 定时器初始化
}
//初始化电机PWM定时
void setup_car_pwm(void) {
	//小车 pwm 初始化
	pwm_init(CYCLE);
	car_pwm_set(0,0);	//设置小车的左右轮速度为0
}	
//初始化串口1
void setup_uart1(void) {
	//串口1初始化
	uart1_init(115200);
	//uart1_close();
	uart1_open();
	//串口发送测试字符
	uart1_send_str((u8 *)"uart1 check ok!");
}
//初始化串口2
void setup_uart2(void) {
	//串口2初始化
	uart2_init(115200);
	//uart2_close();
	uart2_open();
	//串口发送测试字符
	uart2_send_str((u8 *)"uart2 check ok!");
}	
//初始化串口4
void setup_uart4(void) {
	//串口4初始化
	uart4_init(115200);
	//uart4_close();
	uart4_open();
	
	//串口发送测试字符
	uart4_send_str((u8 *)"uart4 check ok!");
}	
//初始化滴答时钟，1S增加一次systick_ms的值
void setup_systick(void) {
	//系统滴答时钟初始化	
	timer3_init();
}	


//初始化启动信号
void setup_start(void) {
	//蜂鸣器LED 名叫闪烁 示意系统启动
	beep_on();nled_on();mdelay(100);beep_off();nled_off();mdelay(100);
	beep_on();nled_on();mdelay(100);beep_off();nled_off();mdelay(100);
	beep_on();nled_on();mdelay(100);beep_off();nled_off();mdelay(100);
}	
//初始化其他
void setup_others(void) {	
	//机械臂蜷缩 G0002组
	memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
	if(ACTION_USE_ROM) {
		//使用存储在单片机内部rom中的动作组
		strcpy((char *)uart_receive_buf, action_pre_group[2]);
	} else {
		//从存储芯片中读取第group_num个动作组
		w25x_read(uart_receive_buf, 2*ACTION_SIZE, ACTION_SIZE);	
	}
	//把读取出来的动作组传递到do_action执行 {G0001#
	if(uart_receive_buf[0] == '{' && uart_receive_buf[1] == 'G' && uart_receive_buf[6] == '#') {
		for(i=16;i<strlen((char *)uart_receive_buf);i+=15) {
			uart_receive_buf[i] = '0';
			uart_receive_buf[i+1] = '0';
			uart_receive_buf[i+2] = '0';
			uart_receive_buf[i+3] = '0';
		}
		do_action(uart_receive_buf);
	}
	
	//执行预存命令 {G0000#000P1500T1000!#000P1500T1000!}
	if(eeprom_info.pre_cmd[PRE_CMD_SIZE] == FLAG_VERIFY) {
		strcpy((char *)uart_receive_buf, (char *)eeprom_info.pre_cmd);
		if(eeprom_info.pre_cmd[0] == '$') {
			parse_cmd(eeprom_info.pre_cmd);
		} else {
			for(i=16;i<strlen((char *)uart_receive_buf);i+=15) {
				uart_receive_buf[i] = '0';
				uart_receive_buf[i+1] = '0';
				uart_receive_buf[i+2] = '0';
				uart_receive_buf[i+3] = '0';
			}
			do_action(uart_receive_buf);
		}
	}

}

//初始化总中断
void setup_interrupt(void) {
	//串口1设为高优先级
	IP = 0X10;
	//IP2 = 0X01;
	//总中断打开
	EA = 1;
}	
//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------
/*
	主循环函数实现
*/
//循环执行工作指示灯，500ms跳动一次
void loop_nled(void) {
	static u32 systick_ms_bak = 0;
	if(millis() - systick_ms_bak >= 500) {
		systick_ms_bak = millis();
		nled_switch();	
	}
}		
//串口数据接收处理
void loop_uart(void) {
	static u8 do_once1 = 0, do_once2 = 0;
	if(uart1_get_ok) {
		//测试发回去
		//uart1_send_str(uart_receive_buf);
		
		if(uart1_mode == 1) {				//命令模式
			//uart1_send_str(">cmd");
			parse_cmd(uart_receive_buf);			
		} else if(uart1_mode == 2) {		//单个舵机模式
			//uart1_send_str(">sig");
			do_action(uart_receive_buf);
		} else if(uart1_mode == 3) {		//多个舵机模式
			//uart1_send_str(">group:");
			//总线下发
			do_action(uart_receive_buf);
		} else if(uart1_mode == 4) {		//保存模式
			//uart1_send_str(">save");
			//uart1_send_str(uart_receive_buf);
			action_save(uart_receive_buf);
		} 
		uart1_mode = 0;
		uart1_get_ok = 0;
		//uart1_open();
	}
	

	if(millis() - get_uart_timeout() > 100) {
		if(!do_once1) {
			timer1_open();
			do_once1 = 1;
			do_once2 = 0;
		}
	} else {
		if(!do_once2) {
			timer1_close();
			do_once1 = 0;
			do_once2 = 1;
		}
	}
	
	return;
}	

//定时保存一些变量
void loop_save_something(void) {
	static u32 saveTime = 3000;
	if((djBiasSaveFlag == 1) && (millis() - bias_systick_ms_bak > saveTime)) {
		djBiasSaveFlag = 0;
		bias_systick_ms_bak = millis();
		rewrite_eeprom();
	}	
	return;
}	


void loop_ps2_data(void) {
	static u32 systick_ms_bak = 0;
	if(millis() - systick_ms_bak < 20) {
		return;
	}
	systick_ms_bak = millis();
	psx_write_read(psx_buf);
#if 0	
	sprintf(cmd_return, "0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\r\n", 
	(int)psx_buf[0], (int)psx_buf[1], (int)psx_buf[2], (int)psx_buf[3],
	(int)psx_buf[4], (int)psx_buf[5], (int)psx_buf[6], (int)psx_buf[7], (int)psx_buf[8]);
	uart1_send_str(cmd_return);
#endif 	
	
	return;
}


void loop_ps2_car_pwm(void) {
	static int car_left_bak=0, car_right_bak=0;
	int car_left, car_right;
	
	if(psx_buf[1] != PS2_LED_RED)return;
	car_left = (127 - psx_buf[8]) * 8;
	car_right = (127 - psx_buf[6]) * 8;
	
	if(car_left != car_left_bak || car_right != car_right_bak) {
		car_pwm_set(car_left, car_right);
		car_left_bak = car_left;
		car_right_bak = car_right;
	}

}



void loop_ps2_button(void) {
	static unsigned char psx_button_bak[2] = {0};
	static unsigned char mode_bak;

	//处理智能模式 红灯模式下 智能模式取消，此时为遥控模式
	if(mode_bak != psx_buf[1]) {
		mode_bak = psx_buf[1];
		if(PS2_LED_RED == psx_buf[1]) {
			smart_mode = 255;
		}
		car_pwm_set(0,0);
		group_do_ok = 1;
		//beep_on_times(1, 500);
	}

	if((psx_button_bak[0] == psx_buf[3])
	&& (psx_button_bak[1] == psx_buf[4])) {			
	} else {
		parse_psx_buf(psx_buf+3, psx_buf[1]);
		psx_button_bak[0] = psx_buf[3];
		psx_button_bak[1] = psx_buf[4];
	}
	return;
}

void parse_psx_buf(unsigned char *buf, unsigned char mode) {
	u8 i, pos = 0;
	static u16 bak=0xffff, temp, temp2;
	temp = (buf[0]<<8) + buf[1];
	
	if(bak != temp) {
		temp2 = temp;
		temp &= bak;
		for(i=0;i<16;i++) {
			if((1<<i) & temp) {
			} else {
				if((1<<i) & bak) {	//press
															
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
					} else if(mode == PS2_LED_GRN) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));
					} else continue;
					
					pos = str_contain_str(uart_receive_buf, "^");
					if(pos) uart_receive_buf[pos-1] = '\0';
					if(str_contain_str(uart_receive_buf, "$")) {
						//uart1_close();
						//uart1_get_ok = 1;
						//uart1_mode = 1;
						strcpy(cmd_return, uart_receive_buf+11);
						strcpy(uart_receive_buf, cmd_return);
						parse_cmd(uart_receive_buf);
					} else if(str_contain_str(uart_receive_buf, "#")) {
						//uart1_close();
						//uart1_get_ok = 1;
						//uart1_mode = 2;
						strcpy(cmd_return, uart_receive_buf+11);
						strcpy(uart_receive_buf, cmd_return);
						do_action(uart_receive_buf);
					}
					
					//uart1_send_str(uart_receive_buf);
					//zx_uart_send_str(uart_receive_buf);
					
					bak = 0xffff;
				} else {//release
										
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
					} else if(mode == PS2_LED_GRN) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));
					} else continue;	
					
					pos = str_contain_str(uart_receive_buf, "^");
					if(pos) {
						if(str_contain_str(uart_receive_buf+pos, "$")) {
							//uart1_close();
							//uart1_get_ok = 1;
							//uart1_mode = 1;
							strcpy(cmd_return, uart_receive_buf+pos);
							cmd_return[strlen(cmd_return) - 1] = '\0';
							strcpy(uart_receive_buf, cmd_return);
							parse_cmd(uart_receive_buf);
						} else if(str_contain_str(uart_receive_buf+pos, "#")) {
							//uart1_close();
							//uart1_get_ok = 1;
							//uart1_mode = 2;
							strcpy(cmd_return, uart_receive_buf+pos);
							cmd_return[strlen(cmd_return) - 1] = '\0';
							strcpy(uart_receive_buf, cmd_return);
							do_action(uart_receive_buf);
						}
						//uart1_send_str(uart_receive_buf);
						//zx_uart_send_str(uart_receive_buf);
					}	
				}
				//测试执行指令
				//uart1_send_str(uart_receive_buf);

			}
		}
		bak = temp2;
		beep_on();mdelay(50);beep_off();
	}	
	return;
}

void handle_uart(void) {


	return;
}

/*
	$DST!
	$DST:x!
	$RST!
	$CGP:%d-%d!
	$DEG:%d-%d!
	$DGS:x!
	$DGT:%d-%d,%d!
	$DCR:%d,%d!
	$DWA!
	$DWD!
	$DJR!
	$GETA!
*/

void parse_cmd(u8 *cmd) {
	//u32 uint1;
	static u8 djrFlag=0;
	u16 pos, i, index;
	int int1, int2;
	
	uart1_send_str(cmd);
	
	if(pos = str_contain_str(cmd, "$DST!"), pos) {
		group_do_ok  = 1;
		for(i=0;i<DJ_NUM;i++) {
			duoji_doing[i].inc = 0;	
			duoji_doing[i].aim = duoji_doing[i].cur;
		}
		zx_uart_send_str("#255PDST!");
		car_pwm_set(0, 0);
		smart_mode = 10;
	} else if(pos = str_contain_str(cmd, "$DST:"), pos) {
		if(sscanf(cmd, "$DST:%d!", &index)) {
			duoji_doing[index].inc = 0;	
			duoji_doing[index].aim = duoji_doing[index].cur;
			sprintf(cmd_return, "#%03dPDST!", (int)index);
			zx_uart_send_str(cmd_return);
		}
		
		
	} else if(pos = str_contain_str(cmd, "$RST!"), pos) {		
		soft_reset();
	} else if(pos = str_contain_str(cmd, "$CGP:"), pos) {		
		if(sscanf(cmd, "$CGP:%d-%d!", &int1, &int2)) {
			print_group(int1, int2);
		}
	} else if(pos = str_contain_str(cmd, "$DEG:"), pos) {		
		if(sscanf(cmd, "$DEG:%d-%d!", &int1, &int2)) {
			erase_sector(int1, int2);
		}
	} else if(pos = str_contain_str(cmd, "$DGS:"), pos) {		
		if(sscanf(cmd, "$DGS:%d!", &int1)) {
			do_group_once(int1);
			group_do_ok = 1;
		}
	} else if(pos = str_contain_str(cmd, "$DGT:"), pos) {		
		if(sscanf((char *)cmd, "$DGT:%d-%d,%d!", &group_num_start, &group_num_end, &group_num_times)) {
			//uart1_send_str("111111");			
			if(group_num_start != group_num_end) {
				do_start_index = group_num_start;
				do_time = group_num_times;
				group_do_ok = 0;
				//uart1_send_str("22222");
			} else {
				group_do_ok = 1;
				do_group_once(group_num_start);
				//uart1_send_str("33333");
			}
		}
	} else if(pos = str_contain_str(cmd, "$DCR:"), pos) {		
		if(sscanf(cmd, "$DCR:%d,%d!", &int1, &int2)) {
			car_pwm_set(int1, int2);
		}
	} else if(pos = str_contain_str(cmd, "$DWA!"), pos) {		
		car_dw--;
		if(car_dw == 0)car_dw = 1;
		beep_on();mdelay(100);beep_off();
	} else if(pos = str_contain_str(cmd, "$DWD!"), pos) {		
		car_dw++;
		if(car_dw == 4)car_dw = 3;
		beep_on();mdelay(100);beep_off();
	} else if(pos = str_contain_str(cmd, "$CAR_F!"), pos) {		
		car_pwm_set(1000, 1000);
	} else if(pos = str_contain_str(cmd, "$CAR_B!"), pos) {		
		car_pwm_set(-1000, -1000);
	} else if(pos = str_contain_str(cmd, "$CAR_L!"), pos) {		
		car_pwm_set(-1000, 1000);
	} else if(pos = str_contain_str(cmd, "$CAR_R!"), pos) {		
		car_pwm_set(1000, -1000);
	} else if(pos = str_contain_str(cmd, "$CAR_STOP!"), pos) {		
		car_pwm_set(0, 0);
	} else if(pos = str_contain_str(cmd, "$JXB_ZHI!"), pos) {	
		do_group_once(1);
	} else if(pos = str_contain_str(cmd, "$JXB_WAN!"), pos) {	
		do_group_once(2);
	} else if(pos = str_contain_str(cmd, "$DJR!"), pos) {	
		zx_uart_send_str("#255P1500T2000!");		
		for(i=0;i<DJ_NUM;i++) {
			duoji_doing[i].aim  = 1500;
			duoji_doing[i].time = 2000;
			duoji_doing[i].inc = (duoji_doing[i].aim -  duoji_doing[i].cur) / (duoji_doing[i].time/20.000);
		}
	} else if(pos = str_contain_str(cmd, "$JXB_SWITCH!"), pos) {	
		zx_uart_send_str("#255P1500T2000!");		
		for(i=0;i<DJ_NUM;i++) {
			duoji_doing[i].aim  = 1500;
			duoji_doing[i].time = 2000;
			duoji_doing[i].inc = (duoji_doing[i].aim -  duoji_doing[i].cur) / (duoji_doing[i].time/20.000);
		}
		
		if(djrFlag) {
			do_group_once(1);
		} else {
			do_group_once(2);
		}
		djrFlag = !djrFlag;
	} else if(pos = str_contain_str(cmd, "$GETA!"), pos) {		
		uart1_send_str("AAA");
	} else if(pos = str_contain_str(cmd, "$GETS!"), pos) {		
		if(group_do_ok == 0) {
			uart1_send_str("group_do_ok=0");
		} else {
			uart1_send_str("group_do_ok=1");
		}
	} else if(pos = str_contain_str(cmd, "$GETINC!"), pos) {		
		for(i=0;i<8;i++) {
			sprintf(cmd_return, "inc%d = %f \r\n", (int)i, duoji_doing[i].inc);
			uart1_send_str(cmd_return);
		}
	}else if(pos = str_contain_str(uart_receive_buf, "$DBT:"), pos) {		
		if(sscanf(uart_receive_buf, "$DBT:%d,%d!", &int1, &int2)) {
			if(int1 == 1) {
				group_num_start = 1;
				group_num_end = 10;
				group_num_times = int2;
			} else if(int1 == 2) {
				group_num_start = 11;
				group_num_end = 20;
				group_num_times = int2;
			} else if(int1 == 3) {
				group_num_start = 21;
				group_num_end = 30;
				group_num_times = int2;
			} else if(int1 == 4) {
				group_num_start = 31;
				group_num_end = 40;
				group_num_times = int2;
			} else {
				group_num_start = 0;
				group_num_end = 0;
			}
			
			if(group_num_start != group_num_end) {
				do_start_index = group_num_start;
				do_time = group_num_times;
				group_do_ok = 0;
				dbt_flag = 1;
			} else {
				do_group_once(group_num_start);
			}
			
		}
	} else if(pos = str_contain_str(cmd, "$DRS!"), pos) {	
		uart1_send_str("\r\n51MCU-IAP15W4K61S4\r\n");
	} else if(pos = str_contain_str(cmd, (u8 *)"$SMODE"), pos) {		
		if(sscanf((char *)cmd, "$SMODE%d!", &int1)) {
			if(int1 < 10) {
				smart_mode = int1;
				beep_on_times(1, 100);
				car_pwm_set(0,0);
				//uart1_send_str(cmd);
			}
		}
	} else if(pos = str_contain_str(cmd, (u8 *)"$SMART_STOP!"), pos) {		
		smart_mode = 10;
		beep_on_times(1, 100);
		car_pwm_set(0,0);
	} 
}



void action_save(u8 *str) {
	int action_index = 0;
	//预存命令处理
	if(str[1] == '$' && str[2] == '!') {
		eeprom_info.pre_cmd[PRE_CMD_SIZE] = 0;
		rewrite_eeprom();
		uart1_send_str((u8 *)"@CLEAR PRE_CMD OK!");
		return;
	} else if(str[1] == '$') {
		if(sscanf((char *)str, "<$DGT:%d-%d,%d!>", &group_num_start, &group_num_end, &group_num_times)) {
			if(group_num_start == group_num_end) {
				w25x_read(eeprom_info.pre_cmd, group_num_start*ACTION_SIZE, ACTION_SIZE);	
			} else {
				memset(eeprom_info.pre_cmd, 0, sizeof(eeprom_info.pre_cmd));
				strcpy((char *)eeprom_info.pre_cmd, (char *)str+1);
				eeprom_info.pre_cmd[strlen((char *)str) - 2] = '\0';
			}
			eeprom_info.pre_cmd[PRE_CMD_SIZE] = FLAG_VERIFY;
			rewrite_eeprom();
			//uart1_send_str(eeprom_info.pre_cmd);
			uart1_send_str((u8 *)"@SET PRE_CMD OK!");
		}
		return;
	}
	
	action_index = get_action_index(str);
	//<G0001#001...>
	if((action_index == -1) || str[6] != '#'){
	//if( action_index == -1 ){
		uart1_send_str("E");
		return;
	}
	//save_action_index_bak++;
	if(action_index*ACTION_SIZE % W25Q64_SECTOR_SIZE == 0)w25x_erase_sector(action_index*ACTION_SIZE/W25Q64_SECTOR_SIZE);
	replace_char(str, '<', '{');
	replace_char(str, '>', '}');
	w25x_write(str, action_index*ACTION_SIZE, strlen(str) + 1);
	//uart1_send_str(str);
	uart1_send_str("A");
	return;	
}

int get_action_index(u8 *str) {
	int index = 0;
	//uart_send_str(str);
	while(*str) {
		if(*str == 'G') {
			str++;
			while((*str != '#') && (*str != '$')) {
				index = index*10 + *str-'0';
				str++;	
			}
			return index;
		} else {
			str++;
		}
	}
	return -1;
}

void print_group(int start, int end) {
	if(start > end) {
		int_exchange(&start, &end);
	}
	for(;start<=end;start++) {
		memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
		w25x_read(uart_receive_buf, start*ACTION_SIZE, ACTION_SIZE);
		uart1_send_str(uart_receive_buf);
		uart1_send_str("\r\n");
	}
}


void int_exchange(int *int1, int *int2) {
	int int_temp;
	int_temp = *int1;
	*int1 = *int2;
	*int2 = int_temp;
}

void erase_sector(int start, int end) {
	if(start > end) {
		int_exchange(&start, &end);
	}
	if(end >= 127)end = 127;
	for(;start<=end;start++) {
		SpiFlashEraseSector(start);
		sprintf(cmd_return, "@Erase %d OK!", start);
		uart1_send_str(cmd_return);
	}
	save_action_index_bak = 0;
}



void do_group_once(int group_num) {
//	//uart1_close();
//	memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
//	w25x_read(uart_receive_buf, group_num*ACTION_SIZE, ACTION_SIZE);
//	if(dbt_flag) {
//		strcpy(uart_receive_buf, action_pre_group[group_num]);
//	}
//	do_action(uart_receive_buf);
//	sprintf(cmd_return, "@DoGroup %d OK!\r\n\r\n", group_num);
//	uart1_send_str(cmd_return);
//	//uart1_open();
	
	memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
	if(ACTION_USE_ROM) {
		//使用存储在单片机内部rom中的动作组
		strcpy((char *)uart_receive_buf, action_pre_group[group_num]);
	} else {
		//从存储芯片中读取第group_num个动作组
		w25x_read(uart_receive_buf, group_num*ACTION_SIZE, ACTION_SIZE);	
	}
	//把读取出来的动作组传递到do_action执行
	do_action(uart_receive_buf);
	sprintf((char *)cmd_return, "@DoGroup %d OK!\r\n", group_num);
	uart1_send_str(cmd_return);
}


float abs_float(float value) {
	if(value>0) {
		return value;
	}
	return (-value);
}

void duoji_inc_handle(u8 index) {	
	int aim_temp;
	
	if(duoji_doing[index].inc != 0) {
		
		aim_temp = duoji_doing[index].aim;
		
		if(aim_temp > 2500){
			aim_temp = 2500;
		} else if(aim_temp < 500) {
			aim_temp = 500;
		}
	
		if(abs_float(aim_temp - duoji_doing[index].cur) <= abs_float(duoji_doing[index].inc + duoji_doing[index].inc)) {
			duoji_doing[index].cur = aim_temp;
			duoji_doing[index].inc = 0;
		} else {
			duoji_doing[index].cur += duoji_doing[index].inc;
		}
	}
}


void loop_action(void) {
	
	if(timer1_flag_dj) {
		duoji_inc_handle(duoji_index1);
		timer1_flag_dj = 0;
	}
	
	if((check_dj_state() == 0) && (group_do_ok == 0)) {
		do_group_once(do_start_index);
		
		if(group_num_start<group_num_end) {
			if(do_start_index == group_num_end) {
				do_start_index = group_num_start;
				if(group_num_times != 0) {
					do_time--;
					if(do_time == 0) {
						group_do_ok = 1;
						uart1_send_str((u8*)"@GroupDone!");
					}
				}
				return;
			}
			do_start_index++;
		} else {
			if(do_start_index == group_num_end) {
				do_start_index = group_num_start;
				if(group_num_times != 0) {
					do_time--;
					if(do_time == 0) {
						group_do_ok = 1;
						uart1_send_str((u8*)"@GroupDone!");
					}
				}
				return;
			}
			do_start_index--;
		}
	}
	
}

u8 check_dj_state(void) {
	int i;
	for(i=0;i<DJ_NUM;i++) {
		if(duoji_doing[i].inc) return 1;
	}
	return 0;
}

void do_action(u8 *uart_receive_buf) {
	u16 index,  time,i, lst_i, parse_ok;
	int bias;
	float pwm;
	float aim_temp;
	zx_uart_send_str(uart_receive_buf);
	zx_uart_send_str("\r\n");
	
	if(uart_receive_buf[0] == '#' && uart_receive_buf[4] == 'P' && uart_receive_buf[5] == 'S' && uart_receive_buf[6] == 'C' && uart_receive_buf[7] == 'K' && uart_receive_buf[12] == '!') {
		index = (uart_receive_buf[1] - '0')*100 + (uart_receive_buf[2] - '0')*10 + (uart_receive_buf[3] - '0');
		bias = (uart_receive_buf[9] - '0')*100 + (uart_receive_buf[10] - '0')*10 + (uart_receive_buf[11] - '0');
		if((bias >= -500) && (bias <= 500) && (index < DJ_NUM)) {
			if(uart_receive_buf[8] == '+') {
			} else if(uart_receive_buf[8] == '-') {
				bias = -bias;
			}
			aim_temp = duoji_doing[index].cur + 0.043198 - eeprom_info.dj_bias_pwm[index] + bias;
			eeprom_info.dj_bias_pwm[index] = bias;			
			if(aim_temp > 2497){
				aim_temp = 2497;
			} else if(aim_temp < 500) {
				aim_temp = 500;
			}
			
			duoji_doing[index].aim = aim_temp;
			duoji_doing[index].cur = aim_temp;
			duoji_doing[index].inc = 0;
			bias_systick_ms_bak = millis();
			djBiasSaveFlag = 1;
		}
		return;
	} else if(uart_receive_buf[0] == '#' && uart_receive_buf[4] == 'P' && uart_receive_buf[5] == 'D' && uart_receive_buf[6] == 'S' && uart_receive_buf[7] == 'T' ) {
		index = (uart_receive_buf[1] - '0')*100 + (uart_receive_buf[2] - '0')*10 + (uart_receive_buf[3] - '0');
		if(index < DJ_NUM) {
			duoji_doing[index].aim = duoji_doing[index].cur;
			duoji_doing[index].inc = 0;
		}
		return;
	}
	
		
	i = 0;parse_ok = 0;
	while(uart_receive_buf[i]) {
		if(uart_receive_buf[i] == '#') {
			lst_i = i;
			index = 0;i++;
			while(uart_receive_buf[i] && (uart_receive_buf[i] != 'P')) {
				index = index*10 + uart_receive_buf[i]-'0';i++;
			}
		} else if(uart_receive_buf[i] == 'P') {
			pwm = 0;i++;
			while(uart_receive_buf[i] && (uart_receive_buf[i] != 'T')) {
				pwm = pwm*10 + uart_receive_buf[i]-'0';i++;
			}
		} else if(uart_receive_buf[i] == 'T') {
			time = 0;i++;
			while(uart_receive_buf[i] && (uart_receive_buf[i] != '!')) {
				time = time*10 + uart_receive_buf[i]-'0';i++;
			}
			
			
			if(index < DJ_NUM && (pwm<=2500)&& (pwm>=500) && (time<10000)) {
				//duoji_doing[index].inc = 0;
				//uart1_send_str(uart_receive_buf);
				if(duoji_doing[index].cur == pwm){
					pwm = pwm+0.0031;
				} 
				
				pwm += eeprom_info.dj_bias_pwm[index];
				if(pwm>2497)pwm=2497;
				if(pwm<500)pwm=500;
				
				
				if(time < 20) {
					duoji_doing[index].aim = pwm;
					duoji_doing[index].cur = pwm;
					duoji_doing[index].inc = 0;
				} else {
					duoji_doing[index].aim = pwm;
					duoji_doing[index].time = time;
					duoji_doing[index].inc = (duoji_doing[index].aim -  duoji_doing[index].cur) / (duoji_doing[index].time/20.000);
				}
					//sprintf(cmd_return, "#%03dP%04dT%04d! %f \r\n", (int)index, (int)pwm, (int)time, duoji_doing[index].inc);
				//uart1_send_str(cmd_return);
			}
			
		} else {
			i++;
		}
	}	
}

void replace_char(u8*str, u8 ch1, u8 ch2) {
	while(*str) {
		if(*str == ch1) {
			*str = ch2;
		} 
		str++;
	}
	return;
}


void loop_vol_warning(void) {
	//static u8 flag = 0, flag_count = 0;
	static u32 systick_ms_bak=0;
	if(millis() - systick_ms_bak < 500)return;
	systick_ms_bak = millis();
	adc7_value = adc_read(ADC_VOL);
	vol_adc = (adc7_value/1023.0) * 5.0 * 4;
	
}




//把eeprom_info写入到W25Q64_INFO_ADDR_SAVE_STR位置
void rewrite_eeprom(void) {
	w25x_erase_sector(W25Q64_INFO_ADDR_SAVE_STR/W25Q64_SECTOR_SIZE);
	w25x_write((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info_t));
}

void loop_bt_once(void) {
	static u8 first_change = 1, step = 0;
	static u32 systick_ms_bak = 0;
	if(first_change) {
		if((millis() - systick_ms_bak > 500) && (step == 0)) {
			systick_ms_bak = millis();
			uart4_init(9600);
			uart4_open();
			uart4_send_str((u8 *)"AT+BAUD8\r\n");
			step++;
		} else if((millis() - systick_ms_bak > 500) && (step == 1)){
			systick_ms_bak = millis();		
			uart4_init(115200);
			step++;
		}  else if((millis() - systick_ms_bak > 500) && (step == 2)){
			systick_ms_bak = millis();		
			uart4_send_str((u8 *)"AT+SPPNAMEZL-51-BT2.0\r\n");
			step++;
		} else if((millis() - systick_ms_bak > 500) && (step == 3)){
			systick_ms_bak = millis();
			uart4_send_str((u8 *)"AT+LENAMEZL-51-BT4.0\r\n");
			step++;
		} else if((millis() - systick_ms_bak > 500) && (step == 4)){
			systick_ms_bak = millis();
			uart4_send_str((u8 *)"AT+NAMEZL-51-BT2.0\r\n");
			step++;
			first_change = 0;
		} 
	}
}

void soft_reset(void) {
	IAP_CONTR = 0X60;
}

//void loop_monitor_servo(void) {
//	int i, value = 0;
//	for(i=0;i<DJ_NUM;i++) {
//		value += duoji_doing[i].inc;	
//	}
//	if(value == 0) {
//		timer0_close();
//	}
//	
//}


