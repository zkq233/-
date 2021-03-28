#include "z_global.h"

duoji_doing_t duoji_doing[DJ_NUM];
u8 uart_receive_buf[UART_BUF_SIZE+4], uart1_get_ok, uart1_mode, uart_receive_buf_index;
u8 cmd_return[CMD_BUF_SIZE];
eeprom_info_t eeprom_info;
u8 timer1_flag_dj = 0;
u8 duoji_index1 = 0;
u8 smart_mode = 255;
u8 group_do_ok = 1;
u16 csb_cnt = 0;




void global_init(void) {

	u8 i;
	for(i=0;i<DJ_NUM;i++) {
		duoji_doing[i].aim = 1501;
		duoji_doing[i].cur = 1501;
		duoji_doing[i].inc = 0;		
	}
	
	uart1_get_ok = 0;
	uart1_mode = 0;
	uart_receive_buf_index = 0;
	//uart_get_timeout = 0;
}

u16 str_contain_str(u8 *str, u8 *str2) {
	u8 *str_temp, *str_temp2;
	str_temp = str;
	str_temp2 = str2;
	while(*str_temp) {
		if(*str_temp == *str_temp2) {
			while(*str_temp2) {
				if(*str_temp++ != *str_temp2++) {
					str_temp = str_temp - (str_temp2-str2) + 1;
					str_temp2 = str2;
					break;
				}	
			}
			if(!*str_temp2) {
				return (str_temp-str);
			}
			
		} else {
			str_temp++;
		}
	}
	return 0;
}

void selection_sort(int *a, int len) {
    int i,j,mi,t;
    for(i=0;i<len-1;i++) {
        mi = i;
        for(j=i+1;j<len;j++) {
            if(a[mi] > a[j]) {
                mi = j;    
            }    
        }    
		
        if(mi != i) {
            t = a[mi];
            a[mi] = a[i];
            a[i] = t;    
        }
    }
}

//int型 取绝对值函数
int abs_int(int int1) {
	if(int1 > 0)return int1;
	return (-int1);
}
