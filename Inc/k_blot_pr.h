#ifndef K_BLOT_PR_H
#define K_BLOT_PR_H

#define ARRAY_STEP  20
#define PS_MAX_EVENT 20


void pump_all_call(byte dir, uint time );
void dsp_pump_all_call(byte dir, uint time);
void pump_all_stop();
void dsp_pump_all_stop();

#pragma pack(push,1)
 struct prime
 {
  enum cntrl_event event[PS_MAX_EVENT];
  int32_t bath_psn;  // number//  byte maxNum[3];  // number
  uint32_t motor_speed;  // number//  byte maxNum[3];  // number
  uint8_t dsp_pp_num;
  uint32_t dsp_pp_time;
  int32_t dsp_pin_len;
  uint8_t asp_pp_num;
  uint32_t asp_pp_time;
  int32_t asp_pin_len;
  uint8_t rollbk_pp_num;
  uint32_t rollbk_pp_time;
  
 } ;
#pragma pack(pop)

 #pragma pack(push,1)
 struct dispense 
 {
  enum cntrl_event event[PS_MAX_EVENT];
  byte dsp_num;
  byte pum_num;
  uint16_t dsp_vol;
 } ;
#pragma pack(pop)
 
#pragma pack(push,1)
 struct aspiraion 
 {
  enum cntrl_event event[PS_MAX_EVENT];
  byte asp_num;
  byte dry_oper;
  byte pum_num;
  uint16_t asp_time;
  byte cnt;
 } ;
#pragma pack(pop)

#pragma pack(push,1)
 struct shake  
 {
  enum cntrl_event event[PS_MAX_EVENT];
  int32_t speed;
  uint32_t time;
 } ;
#pragma pack(pop)
 
 #pragma pack(push,1)
 struct roll_bk  
 {
  enum cntrl_event event[PS_MAX_EVENT];
  byte pum_num;
  uint32_t time;
 } ;
#pragma pack(pop)
 
#pragma pack(push,1)
 struct etc_fc    
 {
  enum cntrl_event event[PS_MAX_EVENT];
  uint32_t dsp_tm;
  uint32_t asp_tm;
 } ;
#pragma pack(pop)
 
 #pragma pack(push,1)
 struct dryer  
 {
  enum cntrl_event event[PS_MAX_EVENT];
  uint8_t temp;
  uint8_t time;
 } ;
#pragma pack(pop)
 
#pragma pack(push,1)
 struct clean
 {
  enum cntrl_event event[PS_MAX_EVENT];
  uint8_t cnt;
  uint16_t dsp_time;
  uint16_t asp_time; 
 } ;
#pragma pack(pop)

#pragma pack(push,1)

 
extern struct prime pr_ps;
extern struct dispense dsp;
extern struct aspiraion asp; 
extern struct shake shk;
extern struct roll_bk bk;
extern struct etc_fc fc;
extern struct dryer drys;
extern struct clean clean;

extern uint32_t roll_bk_tm;

#endif 
