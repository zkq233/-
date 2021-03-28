#ifndef __MAIN_H__
#define __MAIN_H__

#define VERSION						20180726	//版本定义
#define ACTION_USE_ROM				0			//1：使用内部数组 0:使用下载的动作组
#define PS2_LED_RED  				0x73
#define PS2_LED_GRN  				0x41
#define PSX_BUTTON_NUM 				16
#define PS2_MAX_LEN 				64
#define CAR_PWM						0
#define ADC_VOL						7
#define ACTION_SIZE					256
#define W25Q64_INFO_ADDR_SAVE_STR	(((8<<10)-4)<<10)//(8*1024-4)*1024		//eeprom_info结构体存储的位置
#define FLAG_VERIFY 				0x38
#define CYCLE   					1000  //电机PWM周期

/*
	初始化函数声明
*/
//初始化全局变量
void setup_global(void);			
//初始化IO口
void setup_gpio(void);			
//初始化工作指示灯
void setup_nled(void);
//初始化低压报警
void setup_vol(void);
//初始化定时器
void setup_beep(void);			
//初始化舵机IO口
void setup_djio(void);	
//初始化传感器IO口
void setup_sensor(void);
//初始化存储器W25Q64
void setup_w25q64(void);		
//初始化PS2手柄
void setup_ps2(void);
//初始化adc
void setup_adc(void);			
//初始化定时器2 处理舵机PWM输出
void setup_dj_timer(void);	
//初始化电机PWM定时
void setup_car_pwm(void);	
//初始化串口1
void setup_uart1(void);	
//初始化串口2
void setup_uart2(void);	
//初始化串口4
void setup_uart4(void);	
//初始化滴答时钟，1S增加一次systick_ms的值
void setup_systick(void);	
//初始化OLED
void setup_oled(void);	
//初始化启动信号
void setup_start(void);		
//初始化其他
void setup_others(void);
//初始化总中断
void setup_interrupt(void);		

/*
	主循环函数声明
*/
//循环执行工作指示灯，500ms跳动一次
void loop_nled(void);		
//串口数据接收处理
void loop_uart(void);	
//动作组批量执行
void loop_action(void);	
//蓝牙修改波特率和名称
void loop_bt_once(void);		
//循环读取PS2手柄数据
void loop_ps2_data(void);	
//处理手柄上的按钮
void loop_ps2_button(void);	
//处理小车电机摇杆控制
void loop_ps2_car_pwm(void);
//定时保存一些变量
void loop_save_something(void);	
//处理低压报警
void loop_vol_warning(void);
//处理OLED显示
void loop_oled_display(void);	
//监视舵机增量
void loop_monitor_servo(void);

void soft_reset(void);
void parse_psx_buf(unsigned char *buf, unsigned char mode);
void parse_cmd(u8 *cmd);

void action_save(u8 *str);
int get_action_index(u8 *str);//获取动作序号
void print_group(int start, int end);
void int_exchange(int *int1, int *int2);
void erase_sector(int start, int end);

void do_group_once(int group_num); 
void handle_action(void);
u8 check_dj_state(void);//检查舵机状态，是否全部到位

void do_action(u8 *uart_receive_buf);
void replace_char(u8*str, u8 ch1, u8 ch2);
void oled_dis_init(void);
void rewrite_eeprom(void);

 						
#endif 