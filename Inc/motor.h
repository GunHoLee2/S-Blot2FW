#ifndef MOTOR_CNTRL_H
#define MOTOR_CNTRL_H

#define X_AIS_MT 0
#define SHAKE_MT 1


#define ASP_SPD 500
#define ASP_SPD_HONE 100
#define X_AIS_SPD   pr_ps.motor_speed
#define X_AIS_SPD_HOME   150
#define SK_SPD    63//mt_ctrl.shake_speed  
//int32_t DRV88xx_Homming(uint32_t dwDev_num, uint32_t dwSpeed_pps)
//int32_t DRV88xx_AbsMove(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos)

#define DSP_CH_WIDTH mt_ctrl.strip_width

#define HOME_DETECTION         0
#define DEVICE_MAXCNT           2

#define dSPIN_ACC_PM 100
#define dSPIN_DEC_PM 100

#define dSPIN_MAX_SPEED_PM 1000
#define dSPIN_MIN_SPEED_PM 5


#pragma pack(push,1)
 __packed  struct st_motor
 {
  int32_t bath_asp_pos;
  int32_t tray_asp_pos;
  int32_t asp_pos;
  int32_t shake_home_pos;
  uint32_t shake_speed;
  int32_t shake_asp_pos;
  int32_t shake_dsp_pos;
  int32_t shake_dry_pos;
  int32_t shake_sampl_dsp_pos;
  int32_t strip_width;
  int32_t xtray_dsp_pos;
  int32_t xtray_asp_pos;
  int32_t bath_xtray_pos;
  int32_t shake_anly_pos;
  int32_t qr_pos;
  int32_t cam_strip_pos;
  int32_t strip_width_pos;
//#ifdef PLUS
//  uint32_t heat_pram_ch1;
//  uint32_t heat_pram_ch2;
//#endif
 };
#pragma pack(pop)

#pragma pack(push,1)
 __packed  struct heat_temp
 {
  uint32_t heat_pram_ch1;
  uint32_t heat_pram_ch2;
  uint32_t heat_ir_temp_pos;
  uint32_t air_temp_vol;
  uint32_t tray_temp_vol;
 };
#pragma pack(pop)
/*
 __packed  struct st_motor
 {
  int16_t bath_asp_pos;
  int16_t tray_asp_pos;
  int16_t shake_home_pos;
  int16_t shake_speed;
  int16_t shake_asp_pos;
  int16_t shake_dsp_pos;
  int16_t shake_dry_pos;
  int16_t shake_sampl_dsp_pos;
  int16_t strip_width;
  int16_t xtray_dsp_pos;
  int16_t xtray_asp_pos;
  int16_t bath_xtray_pos;
  
  
 };
*/
extern struct st_motor mt_ctrl;
extern struct heat_temp heat_tmp;
event motor_control(event);

int32_t motor_val_ch(byte *data, byte size);
void motor_param_init();
void temp_param_init();
void mt_init_pos();
void Asp_Home();



void Sk_Home();
  

void X_AIS_Home();

void st_init();
int32_t Stmt_Homming(uint32_t dwDev_num, uint32_t dwSpeed_pps);
int32_t Stmt_AbsMove(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos);
int32_t Stmt_ShakeMove(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos);
int32_t Stmt_ShakeMoveCnt( uint32_t dwSpeed_pps,int32_t cnt);
void Stmt_stop_pr();
int32_t motor_val_ch(byte *data, byte size);
uint ref_heat_vol(uint volt, uint th_res);

extern int sk_cnt;
extern bool air_temp_read_flg;
extern void readTrayTemp();
extern void readAirTemp();
#define TEMP_CALL_CNT 120
#endif