#ifndef PUMP_H
#define PUMP_H

#define PUMP_CHNNEL 9

#define  PUMP_CW 0
#define  PUMP_CCW 1

#define PUMP_ASP1 9
//#define PUMP_ASP2 2

#define STN_UL 1000//250

#pragma pack(push,1)
 __packed  struct pump
 {
  //uint8_t chnnel[PUMP_CHNNEL];
  //uint8_t dir[PUMP_CHNNEL];
  uint16_t offset[PUMP_CHNNEL];
  uint32_t bath_vol;
  uint32_t bath_time[PUMP_CHNNEL];
 };
#pragma pack(pop)

#pragma pack(push,1)
 __packed  struct pump_time
 {
  uint32_t use_time[PUMP_CHNNEL];
  uint32_t use_time_min[PUMP_CHNNEL];
  uint32_t use_time_s[PUMP_CHNNEL];
  uint32_t  led_vol[4];
 };
#pragma pack(pop)

 
extern struct pump pp;
extern struct pump_time pp_tm;
extern byte prime_oper_cnt;
extern bool prime_oper_flg;
extern uint16_t eve_cmd;

event pump_control(event event);

int32_t pp_val_ch(byte *data, byte size);
void pump_mem_init();
void pp_tim_sum(int32_t devNm, uint32_t time);
uint32_t set_ul_fun(uint32_t set_ul, byte pump_num);

#endif 
