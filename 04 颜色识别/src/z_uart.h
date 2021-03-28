#ifndef __UART_H__
#define __UART_H__

#include "z_stc15.h"


//typedef unsigned char BYTE;
//typedef unsigned int WORD;


//#define NONE_PARITY     0       //��У��
//#define ODD_PARITY      1       //��У��
//#define EVEN_PARITY     2       //żУ��
//#define MARK_PARITY     3       //���У��
//#define SPACE_PARITY    4       //�հ�У��

//#define PARITYBIT EVEN_PARITY   //����У��λ

void uart1_init(u32 baud);
void uart1_send_byte(u8 dat);
void uart1_send_str(char *s);
void uart1_close(void);
void uart1_open(void);

void uart2_init(u32 baud);
void uart2_send_byte(u8 dat);
void uart2_send_str(char *s);
void uart2_close(void);
void uart2_open(void);

void uart4_init(u32 baud);
void uart4_send_byte(u8 dat);
void uart4_send_str(char *s);
void uart4_close(void);
void uart4_open(void);

void zx_uart_send_str(char *s);

u32 get_uart_timeout(void);


#endif