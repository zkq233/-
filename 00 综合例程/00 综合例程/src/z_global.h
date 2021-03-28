#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "z_stc15.h"

#define DJ_NUM 8

typedef struct {
	float aim;
	float cur;
	float inc;
	int time;
}duoji_doing_t;

#define PRE_CMD_SIZE 158
typedef struct {
	u32 version;
	u32 dj_record_num;
	u8  pre_cmd[PRE_CMD_SIZE + 1];
	int dj_bias_pwm[DJ_NUM+1];
	u8 color_base_flag;
	int color_red_base;
	int color_grn_base;
	int color_blu_base;
}eeprom_info_t;

extern duoji_doing_t duoji_doing[DJ_NUM];
extern eeprom_info_t eeprom_info;

#define UART_BUF_SIZE 512
#define  CMD_BUF_SIZE 100

extern u8 uart_receive_buf[UART_BUF_SIZE+4], uart1_get_ok, uart1_mode, uart_receive_buf_index;
extern u8 cmd_return[CMD_BUF_SIZE];
extern u8 timer1_flag_dj;
extern u8 duoji_index1;
extern u8 smart_mode;
extern u8 group_do_ok;
extern u16 csb_cnt;

void global_init(void);
u16 str_contain_str(u8 *str, u8 *str2);
int abs_int(int int1);
void selection_sort(int *a, int len);

#endif