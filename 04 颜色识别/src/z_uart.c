#include "z_uart.h"
#include "z_stc15.h"
#include "z_global.h"
#include "z_timer.h"
#include "z_gpio.h"
#include <string.h>

#define FOSC 22118400L          //系统频率
#define BAUD 115200L            //串口波特率
#define BAUD4 115200L            //串口波特率

#define S2RI  0x01    // 宏定义 串口2寄存器
#define S2TI  0x02   // 宏定义 串口2寄存器
#define S4RI  0x01              //S4CON.0
#define S4TI  0x02              //S4CON.1

#define S4_S0 0x04              //P_SW2.2


#define ISPPROGRAM() ((void(code*)())0xF000)()

u8 busy2 = 0, busy4 = 0;
u32 uart_timeout = 0;

u32 get_uart_timeout(void) {
	return uart_timeout;
}

void uart1_init(u32 baud) {
	SCON |= 0x50;       //串口1方式1,接收充许    
	T2L = (65536 - (FOSC/4/baud));
	T2H = (65536 - (FOSC/4/baud))>>8;
	AUXR |= 0x15;       //串口1使用独立波特率发生器，独立波特率发生器1T 
	PCON = 0;//0x7F;    //
	
//	SCON = 0x50;		//8位数据,可变波特率
//	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
//	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
//	T2L = 0xD0;			//设定定时初值
//	T2H = 0xFF;			//设定定时初值
//	AUXR |= 0x10;		//启动定时器2
	
	EA = 1;   
	ES = 1;             //	
}

void uart2_init(u32 baud) {
	S2CON = 0x50;         //
	T2L = (65536 - (FOSC/4/baud));    //
	T2H = (65536 - (FOSC/4/baud))>>8; //
	IE2 = 0x01;
	P_SW2 |= 0x01;	//TX2 4.7 RX2 4.6	
	EA = 1; 
}

void uart4_init(u32 baud) {

//	S4CON = 0xda;
//    T4L = (65536 - (FOSC/4/BAUD4));   //设置波特率重装值
//    T4H = (65536 - (FOSC/4/BAUD4))>>8;
//    T4T3M |= 0x20;              //定时器4为1T模式
//    T4T3M |= 0x80;              //定时器4开始计时
	
	P_SW2 &= ~S4_S0;            //S4_S0=0 (P0.2/RxD4, P0.3/TxD4)
	S4CON = 0x10;               //8位可变波特率
	T2L = (65536 - (FOSC/4/baud));   //设置波特率重装值
	T2H = (65536 - (FOSC/4/baud))>>8;
	AUXR |= 0x14;                //T2为1T模式, 并启动定时器2
    
	IE2 |= 0x10;                 //使能串口4中断
    EA = 1;
}


void uart1_open(void) {
	ES = 1;
}

void uart1_close(void) {
	ES = 0;
}

void uart2_open(void) {
	//ES2 = 1;
	IE2 |= 0x01; 
}

void uart2_close(void) {
	//ES2 = 0;
	IE2 &= (~0x01); 
}

void uart4_open(void) {
	//ES2 = 1;
	IE2 |= 0x10; 
}

void uart4_close(void) {
	//ES2 = 0;
	IE2 &= (~0x10); 
}


/*----------------------------

----------------------------*/
void uart1_send_byte(u8 dat) {
    SBUF = dat;   
    while(TI == 0);   
    TI = 0; 
}

void uart2_send_byte(u8 dat) {
    S2BUF = dat;   
	while(!(S2CON & S2TI));
	S2CON &= ~S2TI; 
}

void uart4_send_byte(u8 dat) {
    S4BUF = dat;                	//写数据到UART4数据寄存器
	while (!(S4CON & S4TI));        //等待前面的数据发送完成
    S4CON &= ~S4TI;        		 	//清除S4TI位
}

/*----------------------------

----------------------------*/
void uart1_send_str(char *s) {
	timer0_close();
    while (*s) {                  	//
        uart1_send_byte(*s++);         //
    }
	timer0_open();
}

void uart2_send_str(char *s) {
    while (*s) {                  		//
        uart2_send_byte(*s++);         //
    }
}

void uart4_send_str(char *s) {
    while (*s) {                  		//
        uart4_send_byte(*s++);         //
    }
}

void zx_uart_send_str(char *s) {
	timer0_close();
	uart1_get_ok = 1;
    while (*s) {                  		//
        uart2_send_byte(*s++);         //
    }
	uart1_get_ok = 0;
	timer0_open();
}


/*----------------------------

数据格式:

命令		$xxx!
单个舵机	#0P1000T1000!
多个舵机	{#0P1000T1000!#1P1000T1000!}
存储命令	<#0P1000T1000!#1P1000T1000!>

-----------------------------*/
void Uart1() interrupt 4 using 1 {
	static u16 buf_index = 0;
	static u8 sbuf_bak, cntf8 = 0;
	
    if (RI) {
		RI = 0;                 //清除RI位
		sbuf_bak = SBUF;
		//uart1_send_byte(sbuf_bak);		

		if(sbuf_bak == 0) {
			cntf8++;
			if(cntf8 >= 15) {
				IAP_CONTR = 0X60;
			}
		} else {
			cntf8 = 0;
		}
		
		if(uart1_get_ok)return;
		
		if(sbuf_bak == '<') {
			uart1_mode = 4;
			buf_index = 0;
			uart_timeout = millis();
		} else if(uart1_mode == 0) {
			if(sbuf_bak == '$') {
				uart1_mode = 1;
			} else if(sbuf_bak == '#') {
				uart1_mode = 2;
			} else if(sbuf_bak == '{') {
				uart1_mode = 3;
				uart_timeout = millis();
			} else if(sbuf_bak == '<') {
				uart1_mode = 4;
			} 
			buf_index = 0;
		}
		
		uart_receive_buf[buf_index++] = sbuf_bak;
		
		if(uart1_mode == 4) {
			if(sbuf_bak == '>') {
				uart_receive_buf[buf_index] = '\0';
				uart1_get_ok = 1;
				buf_index = 0;
			}
		} else if((uart1_mode == 1) && (sbuf_bak == '!')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		} else if((uart1_mode == 2) && (sbuf_bak == '!')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		} else if((uart1_mode == 3) && (sbuf_bak == '}')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		}  
		
		if(buf_index >= UART_BUF_SIZE) {
			buf_index = 0;
		}

    }
	
//    if (TI) {
//        TI = 0;                 //清除TI位
//    }
}

void UART2_Int(void) interrupt  8 using 1 // 串口2中断服务程序
{
	static u16 buf_index = 0;
	static u8 sbuf_bak;
	if(S2CON&S2RI)		  		// 判断是不是接收数据引起的中断
	{   
		sbuf_bak = S2BUF;
		S2CON &= ~S2RI;
		
		if(uart1_get_ok)return;
		
		
		if(sbuf_bak == '<') {
			uart1_mode = 4;
			buf_index = 0;
			uart_timeout = millis();
		} else if(uart1_mode == 0) {
			if(sbuf_bak == '$') {
				uart1_mode = 1;
			} else if(sbuf_bak == '#') {
				uart1_mode = 2;
			} else if(sbuf_bak == '{') {
				uart1_mode = 3;
			} else if(sbuf_bak == '<') {
				uart1_mode = 4;
			} 
			buf_index = 0;
		}
		
		uart_receive_buf[buf_index++] = sbuf_bak;
		
		if(uart1_mode == 4) {
			if(sbuf_bak == '>') {
				//uart1_close();
				uart_receive_buf[buf_index] = '\0';
				uart1_get_ok = 1;
				buf_index = 0;
			}
		} else if((uart1_mode == 1) && (sbuf_bak == '!')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		} else if((uart1_mode == 2) && (sbuf_bak == '!')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		} else if((uart1_mode == 3) && (sbuf_bak == '}')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		}   

		if(buf_index >= UART_BUF_SIZE)buf_index = 0;
		//uart2_send_byte(sbuf_bak);
	}
	
//	if (S2CON&S2TI)// 接收到发送命令
//	{
//		busy2 = 0;
//	}
}

/*----------------------------
UART4 中断服务程序
-----------------------------*/
void Uart4() interrupt 18 using 1  {
	static u16 buf_index = 0;
	static u8 sbuf_bak;
    if (S4CON & S4RI) {
        S4CON &= ~S4RI;         //清除S4RI位
        sbuf_bak = S4BUF;             //P0显示串口数据
//        uart4_send_byte(sbuf_bak);
//		return;
		
		if(uart1_get_ok)return;
		
		if(sbuf_bak == '<') {
			uart1_mode = 4;
			buf_index = 0;
			uart_timeout = millis();
		} else if(uart1_mode == 0) {
			if(sbuf_bak == '$') {
				uart1_mode = 1;
			} else if(sbuf_bak == '#') {
				uart1_mode = 2;
			} else if(sbuf_bak == '{') {
				uart1_mode = 3;
			} else if(sbuf_bak == '<') {
				uart1_mode = 4;
			} 
			buf_index = 0;
		}
		
		uart_receive_buf[buf_index++] = sbuf_bak;
		
		if(uart1_mode == 4) {
			
			if(sbuf_bak == '>') {
				//uart1_close();
				uart_receive_buf[buf_index] = '\0';
				uart1_get_ok = 1;
				buf_index = 0;
			}
		} else if((uart1_mode == 1) && (sbuf_bak == '!')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		} else if((uart1_mode == 2) && (sbuf_bak == '!')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		} else if((uart1_mode == 3) && (sbuf_bak == '}')){
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
			buf_index = 0;
		}   

		if(buf_index >= UART_BUF_SIZE)buf_index = 0;
    }
//    if (S4CON & S4TI)
//    {
//        S4CON &= ~S4TI;         //清除S4TI位
//		busy4 = 0;               //清忙标志
//    }
}


