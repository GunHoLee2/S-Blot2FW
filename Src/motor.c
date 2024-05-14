#include "main.h"
#include "string.h"

struct st_motor mt_ctrl;
struct heat_temp heat_tmp;

//  hsAT24xx_Config(hsAT24xx_ConfigType *p_tMemConfig, uint16_t wPageSize, uint16_t wDev_Address, uint16_t wMemStart_Adrress, uint16_t wMemAddress_Size, uint16_t wMemMaxSize)
//  hsAT24xx_Config(&atCfg,64,0xa0,100,2,100);
/*
//Motor Setting Mode
eventAspHome=0x0301,  //Asp Pin Home
eventAspJogRun,       //Asp Jog Run
eventBathAspHiSet,    //Bath Asp pin ???? Set
eventReadBathAspHiSet,//Bath Asp pin ???? ???
eventTrayAspHiSet,
eventReadTrayAspHiSet,
eventReserve4,
eventReserve5,
eventReserve6,
eventShakeMaxPos,

eventReadShakeMaxPos,
eventShakeHomePos,
eventReadShakeHomePos,
eventShakeHome,
eventShakeJogRun,
eventShakeSpdSet,     //shake speed

eventReadShakeSpd,
eventShakeAspPos,
eventReadShakeAsp,
eventShakeDspPos,

eventReadShakeDsp,
eventReserve7,
eventReserve8,
eventShakeDryPos,
eventReadShakeDry,
eventShakeSplDspPos,
eventReadShakeSplDsp,
eventShakeSpdPid,     //shake ????? ????
eventReserve9,
eventReserve10,

eventXaxiSpdSet,
eventReadXaxiSpd,
eventXaxiAcclSet,        // X ?????? ????
eventReadXaxiAccl,
eventXaxiHome,
eventXaxiMove,
eventStripWidth,
eventReadStripWidth,
eventReserve11,
eventTrayDispXStrt,

eventReadTrayDispXStrt,
eventReserve12,
eventReserve13,
eventTrayAspXStrt,
eventReadTrayAspXStrt,
eventReserve14,
eventReserve15,
eventReserve16,
eventReserve17,
eventReserve18,

eventReserve19,
eventBathXPos,
eventReadBathXPos,
*/
/*
__packed  struct st_motor
{
int32_t bath_asp_pos;
int32_t tray_asp_pos;
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
 };;*/
void readTrayTemp();
void readAirTemp();


bool air_temp_read_flg = false;
uint rf_volt = 0;
byte temp_call_cnt = TEMP_CALL_CNT;
event motor_control(event event)
{

    byte send_buf[4] = {
        0,
    };
    switch (event)
    {

    case eventAspHome:
        send_mt_cfg(eventAspHome, 0);
        Asp_Home();

        break;
    case eventAspJogRun:
        mt_ctrl.asp_pos = motor_val_ch(usb_data_buf, 2);
        send_mt_cfg(eventAspJogRun, 0);
        servo_mv(mt_ctrl.asp_pos); // 1550
        break;

    case eventShakeHome:
        send_mt_cfg(eventShakeHome, 0);
        Sk_Home();
        break;

    case eventXaxiHome:
        send_mt_cfg(eventXaxiHome, 0);
        X_AIS_Home();
        break;

    case eventBathAspHiSet:
        mt_ctrl.bath_asp_pos = motor_val_ch(usb_data_buf, 2);
        motor_cfg_wirte();
        send_mt_cfg(eventBathAspHiSet, mt_ctrl.bath_asp_pos);
        mt_init_pos();
        Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, mt_ctrl.bath_xtray_pos);
        servo_mv(mt_ctrl.bath_asp_pos);

        // Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.bath_xtray_pos);
        //   Stmt_AbsMove(ASP_MT,ASP_SPD,mt_ctrl.bath_asp_pos);
        break;

    case eventReadBathAspHiSet:
        motor_cfg_read();
        send_mt_cfg(eventReadBathAspHiSet, mt_ctrl.bath_asp_pos);
        break;

    case eventTrayAspHiSet:
        mt_ctrl.tray_asp_pos = motor_val_ch(usb_data_buf, 2);
        send_mt_cfg(eventTrayAspHiSet, mt_ctrl.tray_asp_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, mt_ctrl.shake_speed, mt_ctrl.shake_asp_pos);
        Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, mt_ctrl.xtray_asp_pos);
        servo_mv(mt_ctrl.tray_asp_pos); // 1550
        // Stmt_AbsMove(ASP_MT,ASP_SPD,mt_ctrl.tray_asp_pos);
        break;

    case eventReadTrayAspHiSet:
        motor_cfg_read();
        send_mt_cfg(eventReadTrayAspHiSet, mt_ctrl.tray_asp_pos);
        break;

    case eventShakeHomePos:
        mt_ctrl.shake_home_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventShakeHomePos, mt_ctrl.shake_home_pos);
        motor_cfg_wirte();
        Stmt_Homming(SHAKE_MT, SK_SPD);
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_home_pos);
        break;

    case eventReadShakeHomePos:
        motor_cfg_read();
        send_mt_cfg(eventReadShakeHomePos, mt_ctrl.shake_home_pos);
        break;

    case eventShakeSpdSet:
        mt_ctrl.shake_speed = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventShakeSpdSet, mt_ctrl.shake_speed);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, 300000);
        // Stmt_ShakeMove(SHAKE_MT,SK_SPD,50000);
        Sk_Home();
        break;

    case eventReadShakeSpd:
        motor_cfg_read();
        send_mt_cfg(eventReadShakeSpd, mt_ctrl.shake_speed);
        break;

    case eventShakeAspPos:
        mt_ctrl.shake_asp_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventShakeAspPos, mt_ctrl.shake_asp_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_asp_pos);
        break;

    case eventReadShakeAsp:
        motor_cfg_read();
        send_mt_cfg(eventReadShakeAsp, mt_ctrl.shake_asp_pos);
        break;

    case eventShakeDspPos:
        mt_ctrl.shake_dsp_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventShakeDspPos, mt_ctrl.shake_dsp_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_dsp_pos);
        break;

    case eventReadShakeDsp:
        motor_cfg_read();
        send_mt_cfg(eventReadShakeDsp, mt_ctrl.shake_dsp_pos);
        break;

    case eventShakeDryPos:
        mt_ctrl.shake_dry_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventShakeDryPos, mt_ctrl.shake_dry_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_dry_pos);
        break;
    case eventShakeAnalyPos:
        mt_ctrl.shake_anly_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventShakeDryPos, mt_ctrl.shake_dry_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_anly_pos);
        break;
    case eventReadShakeAnaly:
        motor_cfg_read();
        send_mt_cfg(eventReadShakeAnaly, mt_ctrl.shake_anly_pos);
        break;

    case eventReadShakeDry:
        motor_cfg_read();
        send_mt_cfg(eventReadShakeDry, mt_ctrl.shake_dry_pos);
        break;

    case eventShakeSplDspPos:
        mt_ctrl.shake_sampl_dsp_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventShakeSplDspPos, mt_ctrl.shake_sampl_dsp_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_sampl_dsp_pos);
        break;

    case eventReadShakeSplDsp:
        motor_cfg_read();
        send_mt_cfg(eventReadShakeSplDsp, mt_ctrl.shake_sampl_dsp_pos);
        break;

    case eventStripWidth:
        mt_ctrl.strip_width = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventStripWidth, mt_ctrl.strip_width);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_dsp_pos);
        Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, mt_ctrl.xtray_dsp_pos);
        // Stmt_AbsMove(ASP_MT,ASP_SPD,mt_ctrl.tray_asp_pos);
        BSP_SystemHW_Delay_msec(1000);
        Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, mt_ctrl.strip_width);
        break;

    case eventReadStripWidth:
        motor_cfg_read();
        send_mt_cfg(eventReadStripWidth, mt_ctrl.strip_width);
        break;

    case eventTrayDispXStrt:
        mt_ctrl.xtray_dsp_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventTrayDispXStrt, mt_ctrl.xtray_dsp_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_dsp_pos);
        Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, mt_ctrl.xtray_dsp_pos);
        break;

    case eventReadTrayDispXStrt:
        motor_cfg_read();
        send_mt_cfg(eventReadTrayDispXStrt, mt_ctrl.xtray_dsp_pos);
        break;

    case eventTrayAspXStrt:
        mt_ctrl.xtray_asp_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventTrayAspXStrt, mt_ctrl.xtray_asp_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_asp_pos);
        Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, mt_ctrl.xtray_asp_pos);
        //  Stmt_AbsMove(ASP_MT,ASP_SPD,mt_ctrl.tray_asp_pos);
        servo_mv(mt_ctrl.tray_asp_pos); // 15

        break;

    case eventReadTrayAspXStrt:
        motor_cfg_read();
        send_mt_cfg(eventReadTrayAspXStrt, mt_ctrl.xtray_asp_pos);
        break;

    case eventBathXPos:
        mt_ctrl.bath_xtray_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventBathXPos, mt_ctrl.bath_xtray_pos);
        motor_cfg_wirte();
        mt_init_pos();
        Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, mt_ctrl.bath_xtray_pos);
        servo_mv(mt_ctrl.bath_asp_pos); // 15
        break;

    case eventReadBathXPos:
        motor_cfg_read();
        send_mt_cfg(eventReadBathXPos, mt_ctrl.bath_xtray_pos);
        break;

    case eventTimer1s:
        /*if(state==stReady)
           if(!temp_call_cnt--){
             temp_call_cnt=TEMP_CALL_CNT;
              set_timer_(eventTrayTempRead,10,0);
              set_timer_(eventAirTempRead,1000,0);
           }*/
        break;

    case eventAspTimeOut:
        //  timer_intrrupt_start();
        Servo_MT_stop();
        break;

    case eventReadAll:
        send_mt_cfg(eventReadAll, 0);
        break;
    case eventCamPosWirte:
        mt_ctrl.cam_strip_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventCamPosWirte, mt_ctrl.cam_strip_pos);
        motor_cfg_wirte();
        mt_init_pos();
        dSPIN_Go_To(mt_ctrl.cam_strip_pos, X_AIS_MT);
        // Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.cam_strip_pos);
        break;
    case eventCamPosRead:
        motor_cfg_read();
        send_mt_cfg(eventCamPosRead, mt_ctrl.cam_strip_pos);
        break;
    case eventCamWidthWrite:
        mt_ctrl.strip_width = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventCamPosWirte, mt_ctrl.strip_width);
        motor_cfg_wirte();
        mt_init_pos();
        //   Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.strip_width_pos);
        break;
    case evnetCamWidthRead:
        motor_cfg_read();
        send_mt_cfg(evnetCamWidthRead, mt_ctrl.strip_width);
        break;

#ifdef PLUS
    case eventTrayHeatingPADch1Write:
        heat_tmp.heat_pram_ch1 = motor_val_ch(usb_data_buf, 4);
        heat_pram_wirte();
        temp_ctrl(4, heat_tmp.heat_pram_ch1);
        send_mt_cfg(eventTrayHeatingPADch1Write, heat_tmp.heat_pram_ch1);
        break;
    case eventTrayHeatingPADch1Read:
        heat_pram_read();
        // sort_8bit(mt_ctrl.heat_pram_ch1,dev_send_buf);
        send_mt_cfg(eventTrayHeatingPADch1Read, heat_tmp.heat_pram_ch1);
        break;
    case eventTrayHeatingPADch2Write:
        heat_tmp.heat_pram_ch2 = motor_val_ch(usb_data_buf, 4);
        heat_pram_wirte();
        temp_ctrl(5, heat_tmp.heat_pram_ch2);
        send_mt_cfg(eventTrayHeatingPADch2Write, heat_tmp.heat_pram_ch2);
        break;
    case eventTrayHeatingPADch2Read:

        // sort_8bit(mt_ctrl.heat_pram_ch1,dev_send_buf);
        send_mt_cfg(eventTrayHeatingPADch2Read, heat_tmp.heat_pram_ch2);
        break;
    case eventHeatPadOnOff:
        usb_send_pack(eventHeatPadOnOff, usb_data_buf);
        temp_ctrl(6, usb_data_buf[3]);
        break;
    case eventHeatPadTempRead:
        heat_tmp.air_temp_vol = motor_val_ch(usb_data_buf, 4);
        heat_pram_wirte();
        temp_set(usb_data_buf);
        break;
    case eventHeatPadTempSet:
        heat_tmp.tray_temp_vol = 0;
        heat_tmp.heat_pram_ch1 = 0;
        heat_tmp.tray_temp_vol = usb_data_buf[0] << 8;
        heat_tmp.tray_temp_vol |= usb_data_buf[1];
        heat_tmp.heat_pram_ch1 = usb_data_buf[2] << 8;
        heat_tmp.heat_pram_ch1 |= usb_data_buf[3];
        heat_tmp.heat_pram_ch1 = ref_heat_vol(heat_tmp.tray_temp_vol, heat_tmp.heat_pram_ch1);
        heat_pram_wirte();
        temp_ctrl(4, heat_tmp.heat_pram_ch1);
        send_mt_cfg(eventHeatPadTempSet, heat_tmp.heat_pram_ch1);
        break;

    case eventXaxiBathSave: // 0x034D
        motor_cfg_read();
        send_mt_cfg(eventXaxiBathSave, mt_ctrl.bath_xtray_pos);
        break;
    case eventXaxiAspSave:
        motor_cfg_read();
        send_mt_cfg(eventXaxiAspSave, mt_ctrl.tray_asp_pos);
        break;
    case eventXaxiDspSave:
        motor_cfg_read();
        send_mt_cfg(eventXaxiDspSave, mt_ctrl.xtray_dsp_pos);
        break;
    case eventShakeAspSave:
        motor_cfg_read();
        send_mt_cfg(eventShakeAspSave, mt_ctrl.shake_asp_pos);
        break;
    case eventShakeDspSave:
        motor_cfg_read();
        send_mt_cfg(eventShakeDspSave, mt_ctrl.shake_dsp_pos);
        break;
    case eventShakeDrySave:
        motor_cfg_read();
        send_mt_cfg(eventShakeDrySave, mt_ctrl.shake_dry_pos);
        break;
    case eventShakeAnalysisSave:
        motor_cfg_read();
        send_mt_cfg(eventShakeAnalysisSave, mt_ctrl.shake_anly_pos);
        break;
    case eventShakeSmpDspSave:
        send_mt_cfg(eventShakeSmpDspSave, mt_ctrl.shake_sampl_dsp_pos);
        break;
    case eventShakeHomeSave:
        motor_cfg_read();
        send_mt_cfg(eventShakeHomeSave, mt_ctrl.shake_home_pos);
        break;
    case eventAspBathSave:
        motor_cfg_read();
        send_mt_cfg(eventAspBathSave, mt_ctrl.bath_asp_pos);
        break;
    case eventAspTraySave:
        motor_cfg_read();
        send_mt_cfg(eventAspTraySave, mt_ctrl.tray_asp_pos);
        break;
    case eventMutiConSave:
        send_mt_cfg(eventMutiConSave, 0);
        break;

    case eventTempSensPosSet:
        heat_tmp.heat_ir_temp_pos = motor_val_ch(usb_data_buf, 4);
        send_mt_cfg(eventTempSensPosSet, heat_tmp.heat_ir_temp_pos);
        heat_pram_wirte();
        dSPIN_Go_To(heat_tmp.heat_ir_temp_pos, X_AIS_MT);
        send_mt_cfg(eventTempSensPosSet, heat_tmp.heat_ir_temp_pos);
        break;
    case eventTempSensPosRead:
        heat_pram_read();
        send_mt_cfg(eventTempSensPosRead, heat_tmp.heat_ir_temp_pos);
        break;
    case eventShakerStop:
        sk_cnt = 0;
        dSPIN_Reset_Pos(SHAKE_MT);
        Sk_Home();
        send_mt_cfg(eventShakerStop, 0);
        break;
    case eventTrayTempRead: //
        readTrayTemp();
        break;
    case eventAirTempRead:
        readAirTemp();
        break;

#endif
    }

    return event;
}

uint ref_heat_vol(uint volt, uint th_res)
{
    float cp_res = 0;
    float res_1th = 10000.0, res_2th = 3300.0;
    float input_volt = (float)(volt / 100.0);
    float temp_res = 0;
    uint rf_vol = 0;

    cp_res = (float)(th_res * res_2th) / (res_2th + th_res);
    // temp_res=(uint)((input_volt*cp_res)/(cp_res+res_1th));
    temp_res = cp_res + res_1th;
    temp_res = (input_volt * cp_res) / temp_res;
    rf_vol = (uint)(temp_res * 1000.0);

    if (input_volt == 5)
        rf_vol += 60;
    // if(th_res<10000)    rf_vol= rf_vol+(rf_vol/1063);
    // compos_res=(float)((bat_vol1/1000.0)*res_1th)/(input_volt-(bat_vol1/1000.0));
    // temp_res=(uint)((compos_res*res_2th)/(res_2th-compos_res));

    return rf_vol;
}

/*
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

 };
*/

void motor_param_init()
{
    motor_cfg_read();
    if ((mt_ctrl.shake_speed == 0) || (mt_ctrl.shake_speed == 0xffffffff))
    {
        mt_ctrl.bath_asp_pos = 17500;
        mt_ctrl.tray_asp_pos = 17800;
        mt_ctrl.asp_pos = 17500;
        mt_ctrl.shake_home_pos = 0;
        mt_ctrl.shake_speed = 60;
        mt_ctrl.shake_asp_pos = 9000;
        mt_ctrl.shake_dsp_pos = 18000;
        mt_ctrl.shake_dry_pos = 0;
        mt_ctrl.shake_sampl_dsp_pos = 3200;
        mt_ctrl.strip_width = 3933;
        mt_ctrl.xtray_dsp_pos = 29200;
        mt_ctrl.xtray_asp_pos = 25750;
        mt_ctrl.bath_xtray_pos = 2000;
        mt_ctrl.shake_anly_pos = 100;
        mt_ctrl.qr_pos = 2000;
        mt_ctrl.cam_strip_pos = 2000;
        mt_ctrl.strip_width_pos = 2000;
        motor_cfg_wirte();
    }
    if (mt_ctrl.asp_pos == 17800)
    {
        mt_ctrl.asp_pos = 17500;
        motor_cfg_wirte();
    }

    // mt_ctrl.bath_xtray_pos=2000;
}

void temp_param_init()
{
    heat_pram_read();
    if ((heat_tmp.heat_pram_ch1 == 0) || (heat_tmp.heat_pram_ch1 == 0xffffffff))
    {
        heat_tmp.heat_pram_ch1 = 1040;
        heat_tmp.heat_pram_ch2 = 1000;
        heat_tmp.heat_ir_temp_pos = 17500;
        heat_tmp.air_temp_vol = 50;
        heat_tmp.tray_temp_vol = 50;
        heat_pram_wirte();
    }

    // mt_ctrl.bath_xtray_pos=2000;
}

int32_t
motor_val_ch(byte *data, byte size)
{
    int32_t temp = 0;
    if (size == 4)
    {
        temp |= data[0] << 24;
        temp |= data[1] << 16;
        temp |= data[2] << 8;
        temp |= data[3];
    }
    else
    {
        temp |= data[2] << 8;
        temp |= data[3];
    }
    return temp;
}

void mt_init_pos()
{
    Asp_Home();
    X_AIS_Home();
    Sk_Home();
}

void Asp_Home()
{
    // servo_mv(mt_ctrl.tray_asp_pos);//1550
    servo_mv(mt_ctrl.asp_pos);
}

void Sk_Home()
{
    Stmt_Homming(SHAKE_MT, SK_SPD);
    Stmt_AbsMove(SHAKE_MT, SK_SPD, mt_ctrl.shake_home_pos);
}

void X_AIS_Home()
{
    Stmt_Homming(X_AIS_MT, X_AIS_SPD_HOME);
    // Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.bath_xtray_pos);
    Stmt_AbsMove(X_AIS_MT, X_AIS_SPD, 100);
    // dSPIN_Run(REV, mt_ctrl.bath_xtray_pos,X_AIS_MT);
}

dSPIN_RegsStruct_TypeDef dSPIN_RegsStruct;

void st_init()
{
    //  HAL_GPIO_WritePin(L6470_STB_RST_GPIO_Port, L6470_STB_RST_Pin, GPIO_PIN_RESET);
    //  HAL_Delay(100);
    //  HAL_GPIO_WritePin(L6470_STB_RST_GPIO_Port, L6470_STB_RST_Pin, GPIO_PIN_SET);
    //  HAL_Delay(100);
    //  HAL_GPIO_WritePin(L6470_SW_GPIO_Port, L6470_SW_Pin, GPIO_PIN_SET);
    //  HAL_Delay(100);
    //  dSPIN_Regs_Struct_Reset(&dSPIN_RegsStruct);
    dSPIN_Reset_And_Standby(0);
    dSPIN_Reset_And_Standby(1);
    // dSPIN_Reset_And_Standby(2);

    /* Acceleration rate settings to 466 steps/s2, range 14.55 to 59590 steps/s2 */
    dSPIN_RegsStruct.ACC = AccDec_Steps_to_Par(10000);
    /* Deceleration rate settings to 466 steps/s2, range 14.55 to 59590 steps/s2 */
    dSPIN_RegsStruct.DEC = AccDec_Steps_to_Par(10000);
    /* Maximum speed settings to 488 steps/s, range 15.25 to 15610 steps/s */
    dSPIN_RegsStruct.MAX_SPEED = MaxSpd_Steps_to_Par(1000);
    /* Minimum speed settings to 0 steps/s, range 0 to 976.3 steps/s */
    dSPIN_RegsStruct.MIN_SPEED = MinSpd_Steps_to_Par(30); // MinSpd_Steps_to_Par(100);
    /* Full step speed settings 252 steps/s, range 7.63 to 15625 steps/s */
    dSPIN_RegsStruct.FS_SPD = FSSpd_Steps_to_Par(10000);
    ; // FSSpd_Steps_to_Par(10000); //655
    /* Hold duty cycle (torque) settings to 10%, range 0 to 99.6% */
    dSPIN_RegsStruct.KVAL_HOLD = 50; // 4;//0x04;//Kval_Perc_to_Par(5);
    /* Run duty cycle (torque) settings to 10%, range 0 to 99.6% */
    dSPIN_RegsStruct.KVAL_RUN = 0x36; // Kval_Perc_to_Par(5);
    /* Acceleration duty cycle (torque) settings to 10%, range 0 to 99.6% */
    dSPIN_RegsStruct.KVAL_ACC = 0x36; // Kval_Perc_to_Par(5);
    /* Deceleration duty cycle (torque) settings to 10%, range 0 to 99.6% */
    dSPIN_RegsStruct.KVAL_DEC = 0x36; // Kval_Perc_to_Par(5);
    /* Intersect speed settings for BEMF compensation to 200 steps/s, range 0 to 3906 steps/s */
    dSPIN_RegsStruct.INT_SPD = 0x1BD0; // IntSpd_Steps_to_Par(200);
    /* BEMF start slope settings for BEMF compensation to 0.038% step/s, range 0 to 0.4% s/step */
    dSPIN_RegsStruct.ST_SLP = 0x18; // BEMF_Slope_Perc_to_Par(0.038);
    /* BEMF final acc slope settings for BEMF compensation to 0.063% step/s, range 0 to 0.4% s/step */
    dSPIN_RegsStruct.FN_SLP_ACC = 0x39; // BEMF_Slope_Perc_to_Par(0.063);
    /* BEMF final dec slope settings for BEMF compensation to 0.063% step/s, range 0 to 0.4% s/step */
    dSPIN_RegsStruct.FN_SLP_DEC = 0x39; // BEMF_Slope_Perc_to_Par(0.063);
    /* Thermal compensation param settings to 1, range 1 to 1.46875 */
    dSPIN_RegsStruct.K_THERM = KTherm_to_Par(1);
    /* Overcurrent threshold settings to 1500mA */
    dSPIN_RegsStruct.OCD_TH = dSPIN_OCD_TH_6000mA; // dSPIN_OCD_TH_6000mA;//dSPIN_OCD_TH_3000mA;
    /* Stall threshold settings to 1000mA, range 31.25 to 4000mA */
    dSPIN_RegsStruct.STALL_TH = StallTh_to_Par(3000); // StallTh_to_Par(1000);
    /* Step mode settings to 128 microsteps */
    dSPIN_RegsStruct.STEP_MODE = dSPIN_STEP_SEL_1_128; // dSPIN_STEP_SEL_1;//dSPIN_STEP_SEL_1_128;
    /* Alarm settings - all alarms enabled */
    dSPIN_RegsStruct.ALARM_EN = dSPIN_ALARM_EN_OVERCURRENT | dSPIN_ALARM_EN_THERMAL_SHUTDOWN | dSPIN_ALARM_EN_THERMAL_WARNING | dSPIN_ALARM_EN_UNDER_VOLTAGE | dSPIN_ALARM_EN_STALL_DET_A | dSPIN_ALARM_EN_STALL_DET_B | dSPIN_ALARM_EN_SW_TURN_ON | dSPIN_ALARM_EN_WRONG_NPERF_CMD;
    /* Internal oscillator, 2MHz OSCOUT clock, supply voltage compensation disabled, *
     * overcurrent shutdown enabled, slew-rate = 290 V/us, PWM frequency = 15.6kHz   */
    //  dSPIN_RegsStruct.CONFIG 	= (uint16_t)dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ | (uint16_t)dSPIN_CONFIG_SW_HARD_STOP
    //      | (uint16_t)dSPIN_CONFIG_VS_COMP_DISABLE | (uint16_t)dSPIN_CONFIG_OC_SD_ENABLE | (uint16_t)dSPIN_CONFIG_SR_290V_us
    //      | (uint16_t)dSPIN_CONFIG_PWM_DIV_2 | (uint16_t)dSPIN_CONFIG_PWM_MUL_1;
    dSPIN_RegsStruct.CONFIG = dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ | dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_VS_COMP_DISABLE | dSPIN_CONFIG_OC_SD_ENABLE | dSPIN_CONFIG_SR_110V_us | dSPIN_CONFIG_PWM_DIV_2 | dSPIN_CONFIG_PWM_MUL_1;

    /* Program all dSPIN registers */
    dSPIN_Registers_Set(&dSPIN_RegsStruct, 0);
    dSPIN_Registers_Set(&dSPIN_RegsStruct, 1);
    // dSPIN_Registers_Set(&dSPIN_RegsStruct,2);
}

int32_t Stmt_Homming(uint32_t dwDev_num, uint32_t dwSpeed_pps)
{
    int32_t dwHome_Check = 0;
    byte i = 0;
    byte hme_sens_flg = 0;

    if ((dwSpeed_pps < 5) || (dwDev_num >= DEVICE_MAXCNT))
        return -1; // parameter fail

    // dSPIN_Set_Param(dwDev_num,dSPIN_ACC, AccDec_Steps_to_Par(dwSpeed_pps));
    // dSPIN_Set_Param(dwDev_num,dSPIN_DEC, AccDec_Steps_to_Par(100));
    //  dSPIN_Set_Param(dwDev_num,dSPIN_SPEED, Speed_Steps_to_Par(dwSpeed_pps));
    // dSPIN_Set_Param(dwDev_num,dSPIN_MIN_SPEED, MinSpd_Steps_to_Par(10));
    for (i = 0; i < 150; i++)
    {
        dwHome_Check = DRV88xx_HomeCheck(dwDev_num);
    }

    if (dwHome_Check != HOME_DETECTION)
    {
        // break;
        hme_sens_flg = 1;
    }
    else
    {
        dSPIN_Run(REV, Speed_Steps_to_Par(dwSpeed_pps), dwDev_num);
    }
    set_timer_1s();
    while (1)
    {
        switch (state)
        {
        case stStby:
            if (stop_pre_st == stReady)
            {
                pump_all_stop();
                return 2;
            }
            break;
        }
        //   main_root();

        if (get_timer_1s() > 6)
        {
            // dSPIN_Reset_Device(dwDev_num);
            //  dSPIN_Run(FWD, Speed_Steps_to_Par(dwSpeed_pps),dwDev_num);
            error(eventError, errTMTimeOut);
            //  dSPIN_Run(REV, Speed_Steps_to_Par(dwSpeed_pps),dwDev_num);
            return 3;
            // set_timer_1s();
        }

        dwHome_Check = DRV88xx_HomeCheck(dwDev_num);
        if (dwHome_Check != HOME_DETECTION)
        {
            hme_sens_flg = 1;
            dSPIN_Hard_Stop(dwDev_num);
            dSPIN_Run(FWD, Speed_Steps_to_Par(1), dwDev_num);
            while (hme_sens_flg)
            {
                dwHome_Check = DRV88xx_HomeCheck(dwDev_num);
                if (dwHome_Check == HOME_DETECTION)
                {
                    dSPIN_Hard_Stop(dwDev_num);
                    BSP_SystemHW_Delay_msec(100);
                    dSPIN_Reset_Pos(dwDev_num);
                    return 1;
                }
            }
            //      if(dwHome_Check != HOME_DETECTION)
            //      {
            //        dSPIN_Soft_Stop(dwDev_num);
            //        dSPIN_Hard_Stop(dwDev_num);
            //         BSP_SystemHW_Delay_msec(100);
            //        dSPIN_Reset_Pos(dwDev_num);
            //        return 1 ;
            //      }
        }
    }
}

uint8_t pre_state = 0, curr_state = 0;

int32_t Stmt_AbsMove(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos)
{

    if ((dwSpeed_pps < 5) || (dwDev_num >= DEVICE_MAXCNT))
        return -1; // parameter fail

    dSPIN_Set_Param(dwDev_num, dSPIN_ACC, AccDec_Steps_to_Par(dwSpeed_pps));
    dSPIN_Set_Param(dwDev_num, dSPIN_DEC, AccDec_Steps_to_Par(dwSpeed_pps));
    // dSPIN_Set_Param(dwDev_num,dSPIN_SPEED, Speed_Steps_to_Par(dwSpeed_pps));
    dSPIN_Set_Param(dwDev_num, dSPIN_MAX_SPEED, MaxSpd_Steps_to_Par(dwSpeed_pps));
    dSPIN_Set_Param(dwDev_num, dSPIN_MIN_SPEED, MinSpd_Steps_to_Par(dSPIN_MIN_SPEED_PM));

    dSPIN_Go_To_Dir(FWD, dwABS_Pos, dwDev_num);

    while (1)
    {
        // result=dSPIN_Get_Param(dwDev_num,dSPIN_ABS_POS);
        switch (state)
        {
        case stStby:
            if (stop_pre_st == stReady)
            {
                pump_all_stop();
                return 2;
            }
            break;
        }

        if (!dSPIN_Busy_HW(dwDev_num))
        {
            dSPIN_Hard_Stop(dwDev_num);
            // dSPIN_Soft_Stop(dwDev_num);
            // BSP_SystemHW_Delay_msec(100);
            //  if(dwDev_num!=ASP_MT)
            dSPIN_Reset_Pos(dwDev_num);
            // dSPIN_Reset_Device(dwDev_num);
            return 1;
        }
        // main_root();
    }
}

int32_t Stmt_ShakeMove(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos)
{
    if ((dwSpeed_pps < 5) || (dwDev_num >= DEVICE_MAXCNT))
        return -1; // parameter fail

    // dSPIN_Set_Param(dwDev_num,dSPIN_ACC, AccDec_Steps_to_Par(0));
    // dSPIN_Set_Param(dwDev_num,dSPIN_DEC, AccDec_Steps_to_Par(0));
    //   dSPIN_Set_Param(dwDev_num,dSPIN_SPEED, Speed_Steps_to_Par(dwSpeed_pps));
    // dSPIN_Set_Param(dwDev_num,dSPIN_MAX_SPEED, MaxSpd_Steps_to_Par(dwSpeed_pps));
    // dSPIN_Set_Param(dwDev_num,dSPIN_MIN_SPEED, MinSpd_Steps_to_Par(0));

    // dSPIN_Go_To_Dir(REV, -dwABS_Pos,dwDev_num);
    dSPIN_Run(REV, Speed_Steps_to_Par(dwSpeed_pps), dwDev_num);
    return 1;
    /*  while(1)
    {
    if(!dSPIN_Busy_HW(dwDev_num))
    {
    dSPIN_Hard_Stop(dwDev_num);
    dSPIN_Reset_Pos(dwDev_num);
    // dSPIN_Soft_Stop(dwDev_num);
    // dSPIN_Hard_Stop(dwDev_num);
    //dSPIN_Reset_Device(dwDev_num);
    return 1;
  }
  }*/
}

int32_t Stmt_ShakeMoveCnt(uint32_t dwSpeed_pps, int32_t cnt)
{
    byte cnts;
    int32_t dwHome_Check = 0;

    cnts = cnt;
    dSPIN_Run(REV, Speed_Steps_to_Par(dwSpeed_pps), SHAKE_MT);
    BSP_SystemHW_Delay_msec(50);
    while (cnts)
    {
        dwHome_Check = DRV88xx_HomeCheck(SHAKE_MT);
        if (dwHome_Check != HOME_DETECTION)
        {
            cnts--;
            BSP_SystemHW_Delay_msec(50);
        }
        // main_root();
    }
    dSPIN_Hard_Stop(SHAKE_MT);
    return 1;
}

void Stmt_stop_pr()
{
    dSPIN_Hard_Stop(0);
    BSP_SystemHW_Delay_msec(100);
    dSPIN_Reset_Pos(0);
    BSP_SystemHW_Delay_msec(100);
    dSPIN_Hard_Stop(1);
    BSP_SystemHW_Delay_msec(100);
    dSPIN_Reset_Pos(1);
    BSP_SystemHW_Delay_msec(100);
    //   dSPIN_Hard_Stop(2);
    //   BSP_SystemHW_Delay_msec(100);
    // dSPIN_Reset_Pos(2);
}

 void readTrayTemp()
{
    byte send_buf[4]={0,};
    USB_IT_Disable();
    rf_volt = heat_tmp.tray_temp_vol / 10;
    send_buf[0] |= rf_volt >> 24;
    send_buf[1] |= rf_volt >> 16;
    send_buf[2] |= rf_volt >> 8;
    send_buf[3] |= rf_volt;
    tray_temp_set(send_buf);
}

void readAirTemp()
{
    byte send_buf[4]={0,};
    USB_IT_Disable();
    send_buf[0] |= heat_tmp.air_temp_vol >> 24;
    send_buf[1] |= heat_tmp.air_temp_vol >> 16;
    send_buf[2] |= heat_tmp.air_temp_vol >> 8;
    send_buf[3] |= heat_tmp.air_temp_vol;
    air_temp_read_flg = true;
    temp_set(send_buf);
}
