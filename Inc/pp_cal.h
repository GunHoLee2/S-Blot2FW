#ifndef PP_CAL_H
#define PP_CAL_H
enum bath_state
{
  stBathNone,
  stBathFull1,
  stBathEmpty1,
  stBathFull2,
  stBathEmpty2
};

extern enum bath_state bath_st;
void bath_en(uint32_t state);
void bath_polling();
void pp_cal_control(event event);
uint32_t auto_set_ul_fun(uint32_t ms, byte pump_num);
uint32_t bath_averge(byte pump_num);
void auto_cal_result();

extern bool bath_full_flg[2];
extern uint32_t bath_full_cnt;
extern byte bath_oper_cnt;
#endif 
