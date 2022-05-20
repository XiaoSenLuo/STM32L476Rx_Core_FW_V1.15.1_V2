


#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "utils.h"




/**
 *
 * @param str
 * @param c
 * @return
 */
int find_chr(const char* str, const char c){   // 返回字符 c 第一次出现的位置
	int pos = -1;
	do{
		pos += 1;
		if(*(str + pos) == c) return pos;
	}while(*(str+pos) != '\0');
	return -1;
}

int find_last_index_of_chr(const char* str, const char c){
	int pos = -1, pos2 = -1;
	do{
		pos += 1;
		if(*(str + pos) == c) pos2 = pos;
	}while(*(str+pos) != '\0');
	return pos2;
}

int replace_last_index_of_chr(char* str, const char c1, const char c2){
	int pos = -1, pos2 = -1;
	do{
		pos += 1;
		if(*(str + pos) == c1) pos2 = pos;
	}while(*(str+pos) != '\0');

    if(pos2 >= 0) str[pos2] = c2;

	return pos2;
}

/**
 *
 * @param format
 */
void string_format(char* out, const char* format, ...){   // dddd-dd-dd-dd-dd-dd
    va_list arp;

    int n = 0;

    va_start(arp, format);

    n = vsprintf(out, format, arp);

    va_end(arp);
}


void add_time(struct tm* tt, const int add_second){
	tt->tm_sec += add_second;

	tt->tm_min += tt->tm_sec / 60;
	tt->tm_sec = tt->tm_sec % 60;  // s

    tt->tm_hour += tt->tm_min / 60;
    tt->tm_min = tt->tm_min % 60;   // min

    tt->tm_mday += tt->tm_hour / 24;
    tt->tm_hour = tt->tm_hour % 24;  //  hour

    if(tt->tm_mon == 2){
    	if(tt->tm_year % 4){
    		tt->tm_mon += tt->tm_mday / 29;
    		tt->tm_mday = tt->tm_mday % 29;  // day
    	}
    	else{
    		tt->tm_mon += tt->tm_mday / 28;
    		tt->tm_mday = tt->tm_mday % 28;
    	}
    }
    if((tt->tm_mon == 1) || (tt->tm_mon == 3) || (tt->tm_mon == 5) || (tt->tm_mon == 7) || (tt->tm_mon == 8) || (tt->tm_mon == 10) || (tt->tm_mon == 12)){
    	tt->tm_mon += tt->tm_mday / 31;
    	tt->tm_mday = tt->tm_mday % 31;
    }else{
    	tt->tm_mon += tt->tm_mday / 30;
    	tt->tm_mday = tt->tm_mday % 30;
    }

    tt->tm_year += tt->tm_mon / 12;   // year
    tt->tm_mon = tt->tm_mon % 12;  // month
}

void format_date_to_string(char *out_str, uint8_t len, struct tm _tm){ // 2020-01-01

}

void format_time_to_string(char *out_str, uint8_t len, struct tm _tm){  // 23-59-59
#define DEC "0123456789"

    int n = 0, i = 6, j = 0, k = 0;

    n = snprintf(out_str, len, "%d-", _tm.tm_year + 1900);

    if(n == len) return;
    register uint32_t tmp = 0U;
    tmp = (uint32_t)(&(_tm.tm_sec));
    char dec[3] = {0, 0, '-'};
    for(i = 4; i >= 0; i--, j++){
        dec[0] = DEC[(*(int*)(tmp + i * sizeof(int))) / 10];
        dec[1] = DEC[(*(int*)(tmp + i * sizeof(int))) % 10];
        for(k = 0; k < sizeof(dec); k++){   // 判断是否索引越界
        	if((n + j * 3 + k) < (len - 1)){
        		out_str[n + j * 3 + k] = dec[k];
        	}else{
        		out_str[len - 1] = '\0';
        		return;
        	}
        }
    }
    out_str[n + (j - 1) * 3 + k - 1] = '\0';
#undef DEC
}

uint32_t data_to_str(uint8_t* in_data, uint32_t in_size, char* out_str, uint32_t in_str_len){
#define HEX "0123456789ABCDEF"

	uint32_t i = 0, j = 0;
    in_str_len -= 1;
	do{
        if((j) < in_str_len) out_str[j] = HEX[(in_data[i] >> 4) & 0x0F]; // 高四位
        else break;
        j++;
        if((j) < in_str_len) out_str[j] = HEX[in_data[i] & 0x0F]; // 低四位
        else break;
        j++;
        i++;
	}while(i < in_size);

	out_str[j] = '\0';
	return j;
#undef HEX
}


