#include "main.h"
#include <string.h>
enum state state=stPowerUp;
enum kbl_ft_state ft_state=stNone;
uint32_t dsp_start_pos=0;

int32_t asp_strp_wt=0;
byte dsp_cnt=0;


int asp_cnt=0;
int sk_cnt=0;
int dry_cnt=0;

byte flag=0;
byte oper_step=0;
byte oper_pr=0;

byte buzer;
byte stop_pre_st=0;
uint16_t ev_st=0; 
uint32_t pause_cnt=0;
byte incuC_home_ch=0;
byte incuC_cnt=0;
uint16_t clean_rk_time=0;
byte asp_rpeat_fu_flg=0;
bool sq_strt_chk=false;
bool dry_move_flg=false;
byte ac_step=0;
uint16_t horor_asp_time=300;
uint16_t fan_cnt_time=1140;
uint16_t fan_oper_time=10;
bool fan_oper=false;
char *vr_pnt=FW_VER;
extern uint16_t step_time_sec=0,step_time_min=0;
byte strip_con[2]={0,};

event execute_control(event event)
{
  byte asp_cnt_gbg;
  byte send_buf[4]={0,};
  
  switch(event)
  {
  case eventSpuOn:
  //   led_read();
    AirFan_Off;
    bath_en(1);
    //temp_ctrl(4,mt_ctrl.heat_pram_ch1);
    Pump_Device_Init();
    // adc_ctrl();
  //  pump_mem_init();
    mt_init_pos();
    state=stPrepare;
    //servo_mv(1500);
    //  beep(0,100,1); 
    // set_timer_(eventAllstHome,100,0); 
    
    set_timer_(eventAllstHome,1000,0); 

    break;

  case eventAllstHome:
    send_buf[0]=pp_tm.led_vol[0];
    send_buf[1]=pp_tm.led_vol[1];
    send_buf[2]=pp_tm.led_vol[2];
    send_buf[3]=pp_tm.led_vol[3];
    led_light_set(send_buf);
    set_timer_(eventAllPumpStop,1000,0); 
    break;
    
  case eventAllPumpCall:
    if((!bath_full_flg[0])&&(!bath_full_flg[1]))
    {
      mt_init_pos();
      Pump_Single_Stop(PUMP_ASP1);
      //Pump_Single_Stop(PUMP_ASP2);
      set_timer_(eventAllPumpStop,100,0);
    }
    else 
      error(eventError,errBathFull);
    break;
  case eventAllPumpStop:
     Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.bath_xtray_pos);
    //pump_all_stop();
    //bath_en(0);
#ifdef PLUS
    beep(0,100,3);
#else 
    beep(0,100,3);
#endif
    //Pump_Single_Run(PUMP_ASP1,PUMP_CCW,3000); 
    //Pump_Single_Run(PUMP_ASP1,PUMP_CW,3000);  
    state=stStby;  
    led_toggl(LEDOFF);
    HAL_GPIO_WritePin(GPIOB, M1_CS_Pin, GPIO_PIN_SET);//FAN OFF

#ifdef PLUS
    temp_ctrl(4,1018); //strip25(1019) degree
    dwHsSW_Delay_ms(1000);
    temp_ctrl(6,1); //heat on
    dwHsSW_Delay_ms(1000);
#endif   
     //USB_IT_Disable();
   // led_ctrl(0x01,0x1111);
    //  set_timer_(dsp.event[0],500,0);
    //  Pump_Single_Run(1,PUMP_CW,6000); 
    // servo_mv(1500);  //asp
    // lcd_send_pack(1,)
    //  send_lcd_sq_cfg(0x01, 0x01000001); LED
    //beep(0,100,0); 
    //give_event(hseSqId,0); 
    //'
    // set_timer_(eventFunDry,100,0); 
    // usb_send_pack(eventSysState,send_buf);
    //set_timer_(eventStartSq,1000,0); 
   // set_timer_(eventStepTime,1000,0); 
    //printf("%d", 10);
    //bath_en(0);
    break;
   case eventSysState:
     state=(enum state)usb_data_buf[3];
     break;
     
     
  case eventPrimeBathPsn:
    //mt_init_pos();
    //X_AIS_Home(); 

    
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.bath_xtray_pos);
    if(state==stPause){ 
      ev_st=pr_ps.event[1]; 
      break;
    }else 
      set_timer_(pr_ps.event[1],1000,0); //dsp time
    break;
  case eventPrimeDspOut:
    
    if(food){
      if(sq[oper_step].stNum==Antibody){   
        pr_ps.dsp_pp_num=5;
        Pump_Single_Run(3,PUMP_CW,pr_ps.dsp_pp_time);
        Pump_Single_Run(pr_ps.dsp_pp_num,PUMP_CW,pr_ps.dsp_pp_time);
      }else if(sq[oper_step].stNum==Enzyme){    
        pr_ps.dsp_pp_num=6;
        Pump_Single_Run(4,PUMP_CW,pr_ps.dsp_pp_time);
        Pump_Single_Run(pr_ps.dsp_pp_num,PUMP_CW,pr_ps.dsp_pp_time);
      }else  Pump_Single_Run(pr_ps.dsp_pp_num,PUMP_CW,pr_ps.dsp_pp_time); 
    }else
      Pump_Single_Run(pr_ps.dsp_pp_num,PUMP_CW,pr_ps.dsp_pp_time);  //pump dir set up
    
    set_timer_(pr_ps.event[2],pr_ps.dsp_pp_time,0); //dsp time
    set_timer_(pr_ps.event[3],2000,0); //dsp time
    break;
  case eventPrimeDspEnd:
    
    Pump_Single_Break(pr_ps.dsp_pp_num);
    Pump_Single_Break(3);
    Pump_Single_Break(4);
    break;
  case eventPrimeAspOut:
   // Asp_Home();
    servo_mv(mt_ctrl.bath_asp_pos);
    //Stmt_AbsMove(ASP_MT,ASP_SPD, mt_ctrl.bath_asp_pos);
    Pump_Single_Run(PUMP_ASP1,PUMP_CCW,pr_ps.asp_pp_time);  //pump dir set up
    //  Pump_Single_Run(PUMP_ASP2,PUMP_CCW,pr_ps.asp_pp_time);  //pump dir set up
    set_timer_(pr_ps.event[4],pr_ps.asp_pp_time,0); //asp time
    break;
  case eventPrimeAspEnd:
    Pump_Single_Break(PUMP_ASP1);

    //Pump_Single_Stop(PUMP_ASP2);
    //beep(0,100,0); 
    if(state==stPause){ 
      ev_st=pr_ps.event[5]; 
      break;
    }else 
      set_timer_(pr_ps.event[5],1000,0); 
    break;
  case eventPrimeAspHome:
    //   pump_tm_wirte();
    Asp_Home();
    //         Pump_Single_Run(pr_ps.dsp_pp_num,PUMP_CCW);  //pump dir set up
    //         dwHsSW_Delay_ms(pr_ps.rollbk_pp_time);
    //         Pump_Single_Stop(pr_ps.dsp_pp_num);
    
       X_AIS_Home(); 
    if(ft_state==stEngDsp){
      set_timer_(dsp.event[0],3000,0);
      //dsp.dsp_num=STRIP_MAX_NUM;
    }else{
      if(state==stReady){
        send_buf[1]=oper_step;
        send_buf[2]=oper_pr;
        //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
      //  lcd_send_pack(evnetPrEnd,send_buf);
        set_timer_(eventStartSq,100,0); //asp time
        
      }else{
        if(state!=stPrepare){
          if(!prime_oper_cnt)
            
            
            beep(0,500,3);
        }
        
        if(state==stPcEng)
           set_timer_(eventPumpOffSetOper,1000,0); //asp time
        
        if(prime_oper_flg){
          give_event(eventPrimeCntinOper,0);  
        }
        
        
        if((eve_cmd==eventBathVolSet)&&bath_oper_cnt){
          give_event(eventBathVolCntnOper,0);
        }else if((eve_cmd==eventPumpAutoCalSet)&&bath_oper_cnt){
          give_event(eventPumpAutoCalPumpSel,0);
        }
        
      }
      ft_state=stNone;
    }
    
    
    break;
  case eventDspSkAngle: 
   // mt_init_pos();
    Asp_Home();
    Sk_Home();
    
    Stmt_AbsMove(SHAKE_MT,SK_SPD,mt_ctrl.shake_dsp_pos);
    
    if(state==stPause){ 
      ev_st=dsp.event[1]; 
      break;
    }else 
      set_timer_(dsp.event[1] ,1000,0); 
  
    break;
  case eventDspSkEnd:
    dsp_cnt=dsp.dsp_num=totoal_strip;
    if(food && inhalant ){
      strip_con[0]=inhalant;
      strip_con[1]=food;
    }
    set_timer_(dsp.event[2],100,0); 
    break;
  case eventDspStripMove:
    
    /*  if((dsp.pum_num==4)||(dsp.pum_num==6))
      Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos+DSP_CH_WIDTH);
    else if((dsp.pum_num==3)||(dsp.pum_num==8))
      Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos+(DSP_CH_WIDTH*2));
    else*/
    
    
    if(dsp.pum_num%2)
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos);
    else
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos-mt_ctrl.strip_width);
    set_timer_(dsp.event[3],100,0); 
    
    break;
  case eventDspOper:
    
    if((!strip_con[0]--)&&food){
      if(sq[oper_step].stNum==Antibody)   
        dsp.pum_num=5;
      else if(sq[oper_step].stNum==Enzyme)    
        dsp.pum_num=6;
       dsp.dsp_vol=set_ul_fun((sq[oper_step].dword[oper_pr]&0x0000ffff), dsp.pum_num);
      strip_con[0]=0;
    }
    
    Pump_Single_Run(dsp.pum_num,PUMP_CW,dsp.dsp_vol);
    dsp_cnt--;
    set_timer_(dsp.event[4],dsp.dsp_vol,0); 
    break;
  case eventDspEnd:
    Pump_Single_Break(dsp.pum_num);
    if(dsp_cnt){
      
      if(state==stPause){ 
        ev_st=dsp.event[5]; 
        break;
      }else 
        set_timer_(dsp.event[5] ,300,0); 
     // set_timer_(dsp.event[5],300,0); 
    }else{
     // pump_tm_wirte();
    //  Pump_Single_Run(dsp.pum_num,PUMP_CCW,500);
     // dwHsSW_Delay_ms(500);
      Pump_Single_Break(dsp.pum_num);
      X_AIS_Home(); 
      Sk_Home();//dsp end
      if(state==stReady){
        send_buf[1]=oper_step;
        send_buf[2]=oper_pr;
        //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
       // lcd_send_pack(evnetPrEnd,send_buf);
        set_timer_(eventStartSq,100,0); //asp time
      }else{
        //  beep(0,500,3);
        //send_qc_dsp();
      }
      ft_state=stNone;
    }
    break;
  case evnetDspCon:
    if(dsp.dsp_num-dsp_cnt==12){
      Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.strip_width*3);
    }else
      Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.strip_width);
    set_timer_(dsp.event[3],100,0); 
    break;
    
 
  case eventCleanBathPsn:
    mt_init_pos();
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.bath_xtray_pos);
    set_timer_(clean.event[1],1000,0); //dsp time
    break;
  case eventCleanDspOut:
    //Stmt_AbsMove(ASP_MT,ASP_SPD, mt_ctrl.bath_asp_pos);
    servo_mv(mt_ctrl.bath_asp_pos);
    dsp_pump_all_call(PUMP_CW, clean.dsp_time);
    set_timer_(clean.event[2],clean.dsp_time,0); //dsp time
    give_event(clean.event[3],0); //dsp time
    break;
  case eventCleanDspEnd:
    dsp_pump_all_stop();
    break;
  case eventCleanAspOut:
    // Asp_Home();,                                                                                                                                                                                                                      
    Pump_Single_Run(PUMP_ASP1,PUMP_CCW,clean.asp_time);  //pump dir set up
 //   Pump_Single_Run(PUMP_ASP2,PUMP_CCW,clean.asp_time);  //pump dir set up
    set_timer_(clean.event[4],clean.asp_time,0); //asp time
    break;
  case eventCleanAspEnd:
    Pump_Single_Break(PUMP_ASP1);
   // Pump_Single_Stop(PUMP_ASP2);
    set_timer_(clean.event[5],50,0); //asp time
    break;
  case eventCleanAspHome:
  //  pump_tm_wirte();
    Asp_Home();
    if(!clean.cnt--){
      if(state==stReady){
        send_buf[1]=oper_step;
        send_buf[2]=oper_pr;
        //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
       // lcd_send_pack(evnetPrEnd,send_buf);
        set_timer_(eventStartSq,100,0); //asp time
      }else{
        if(ac_step)
          give_event(eventEngAutoClean,0);
        else{
          
          beep(0,500,3);
          if(state==stPcEng)
            give_event(eventPumpOffSetOper,0);
        }
      }
    }else
      set_timer_(clean.event[0],100,0); //asp time  
    break;
    
 

  case eventAspSkAngle:
    
    Pump_Single_Break(PUMP_ASP1);
  //  Pump_Single_Stop(PUMP_ASP2);
    if(!asp_rpeat_fu_flg){
      mt_init_pos();
     // Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.bath_xtray_pos);
      Stmt_AbsMove(SHAKE_MT,SK_SPD,mt_ctrl.shake_asp_pos);
    }else{
      Asp_Home();
      X_AIS_Home();
    }
    
    if(state==stPause){ 
      ev_st=asp.event[1]; 
      break;
    }else 
      set_timer_(asp.event[1] ,500,0); 
    
    break;
  case eventAspSkEnd:
    asp_cnt=asp.asp_num=totoal_strip;
    asp_cnt_gbg=asp_cnt%ASP_NUM;
    asp_cnt=asp_cnt/ASP_NUM;
    asp_cnt+=asp_cnt_gbg;  
    set_timer_(asp.event[2],100,0); 
    break;
  case eventAspStripMove:
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_asp_pos);
    set_timer_(asp.event[3],200,0); 
    break;
  case eventAspOut:
   // Stmt_AbsMove(ASP_MT,ASP_SPD, mt_ctrl.tray_asp_pos);
    servo_mv(mt_ctrl.tray_asp_pos);
    set_timer_(asp.event[4],200,0); //asp time
    break;
  case eventAspOper: 
    asp_cnt--;
    Pump_Single_Run(PUMP_ASP1,PUMP_CCW,asp.asp_time);  //pump dir set up
  //  Pump_Single_Run(PUMP_ASP2,PUMP_CCW,asp.asp_time);  //pump dir set up
    set_timer_(asp.event[5],asp.asp_time,0); //asp time
    break;
  case eventAspEnd:
    Pump_Single_Break(PUMP_ASP1);
 //   Pump_Single_Stop(PUMP_ASP2);
    //DRV88xx_Homming(1,3200);
    Asp_Home();
    if(asp_cnt<0)
      asp_cnt=0;
    
    if(asp_cnt){
      if(state==stPause){ 
        ev_st=asp.event[6]; 
        break;
      }else 
        set_timer_(asp.event[6] ,500,0);      
    }else{
      if(asp.cnt)
        asp.cnt--;
      
      if(!asp.cnt){
        asp_rpeat_fu_flg=0;
        //pump_tm_wirte();
        mt_init_pos();
        
        if(state==stReady){
          send_buf[1]=oper_step;
          send_buf[2]=oper_pr;
          //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
         // lcd_send_pack(evnetPrEnd,send_buf);
          set_timer_(eventStartSq,100,0); //asp time
        }else{
          beep(0,500,3);
          //send_qc_asp();
          //send_qc_asp();
        }
      }else{
        asp_rpeat_fu_flg=1;
        give_event(eventAspSkAngle,0);
      }
      
    }
    break;
  case evnetAspCon:
    asp_strp_wt=mt_ctrl.strip_width*ASP_NUM;
  //  if(((asp.asp_num/ASP_NUM)+(asp.asp_num%ASP_NUM))-(asp_cnt)==6){
   //   Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.strip_width*4);
    //}else
      Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, asp_strp_wt);    
    set_timer_(asp.event[3],100,0); 
    break;

  case eventSkOper:
    //Stmt_AbsMove(SHAKE_MT,mt_ctrl.shake_speed, 3000 );//
    dSPIN_Run(REV,Speed_Steps_to_Par(SK_SPD),SHAKE_MT);
    //set_timer_(shk.event[1],10,0); 
    give_event(shk.event[1],0);     
    break;
  case eventSkEnd:
    if(!sk_cnt){
      //  DRV88xx_Homming(2,3200);
      dSPIN_Reset_Pos(SHAKE_MT);
      Sk_Home();
      
      if(state==stReady){
        send_buf[1]=oper_step;
        send_buf[2]=oper_pr;
        //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
      //  lcd_send_pack(evnetPrEnd,send_buf);
        set_timer_(eventStartSq,100,0); //asp time
      }else break;//{
        //if(!(state==stPause))
         // beep(0,500,3);
   //   }
     // break;
    }
    
      //give_event(shk.event[1],0);         
      break;
    //-------------------------------incubationC------------------------------
  case eventSkOperC:
    Sk_Home();
    Stmt_ShakeMoveCnt(mt_ctrl.shake_speed,incuC_cnt);
    send_buf[1]=oper_step;
    send_buf[2]=oper_pr;
    //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
   // lcd_send_pack(evnetPrEnd,send_buf);
    set_timer_(eventStartSq,100,0); //asp time       
    break;
    //-------------------------------dry------------------------------
  case eventdryOper:
    HAL_GPIO_WritePin(GPIOB, M1_CS_Pin, GPIO_PIN_RESET);
    BSP_ETC_OnOff_Device(HEATER,HEATER_ON);
    Stmt_AbsMove(SHAKE_MT,SK_SPD, mt_ctrl.shake_dry_pos);
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.qr_pos);
    dry_move_flg=false;
    give_event(drys.event[1],0);     
    break;
  case eventdryMove:
    //if(dry_cnt%2)
    if(dry_move_flg){
    //  dSPIN_Run(REV,2000,X_AIS_MT);
      dSPIN_Move(FWD,mt_ctrl.xtray_asp_pos,X_AIS_MT);
      //  Stmt_AbsMove(X_AIS_MT,X_AIS_SPD , -mt_ctrl.xtray_asp_pos);
      dry_move_flg=false;
      
    }else{
      //dSPIN_Go_To_Dir(FWD,mt_ctrl.xtray_asp_pos,X_AIS_MT);
      dSPIN_Move(REV,mt_ctrl.xtray_asp_pos,X_AIS_MT);
      dry_move_flg=true;
    }
    set_timer_(drys.event[2],2000,0);   
    break;
  case eventdryEnd:
    //if((((drys.time*60)-1)/2)>dry_cnt)
     // BSP_ETC_OnOff_Device(HEATER,HEATER_OFF);
    
    
    if(!dry_cnt){
   
      HAL_GPIO_WritePin(GPIOB, M1_CS_Pin, GPIO_PIN_SET);
      BSP_ETC_OnOff_Device(HEATER,HEATER_OFF);
#ifdef PLUS
      temp_ctrl(6,0);    //heat off
      AirFan_Off;
#endif  
       Sk_Home();
      if(state==stReady){
        send_buf[1]=oper_step;
        send_buf[2]=oper_pr;
        //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
      //  lcd_send_pack(evnetPrEnd,send_buf);
        set_timer_(eventStartSq,100,0); //asp time
      }else{
        if(!(state==stPauseLcd))
          beep(0,500,3);
      }
      break;
    }else{
     // dSPIN_Reset_Pos(X_AIS_MT);
      set_timer_(drys.event[1],500,0);
    }
      //give_event(shk.event[1],0);         
      break;
  case eventRbOper:
    
    if(food){
      if(sq[oper_step].stNum==Antibody){   
        bk.pum_num=5;
        Pump_Single_Run(3,PUMP_CCW,bk.time);
        Pump_Single_Run(bk.pum_num,PUMP_CCW,bk.time);
      }else if(sq[oper_step].stNum==Enzyme){    
        bk.pum_num=6;
        Pump_Single_Run(4,PUMP_CCW,bk.time);
        Pump_Single_Run(bk.pum_num,PUMP_CCW,bk.time);
      }else   Pump_Single_Run(bk.pum_num,PUMP_CCW,bk.time);
    }else
       Pump_Single_Run(bk.pum_num,PUMP_CCW,bk.time);  //pump dir set up
    
    //Pump_Single_Run(bk.pum_num,PUMP_CCW,bk.time);
    set_timer_(bk.event[1],bk.time*1000,0);
    break;
  case eventRbEnd:
    Pump_Single_Break(bk.pum_num);  
    Pump_Single_Break(3);
    Pump_Single_Break(4);
    if(state==stReady){
      send_buf[1]=oper_step;
      send_buf[2]=oper_pr;
      //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
      //lcd_send_pack(evnetPrEnd,send_buf);
      set_timer_(eventStartSq,100,0); //asp time
    }else{
      beep(0,500,3);
    }
    break;
 
  case eventCleanRbOper:
    pump_all_call(PUMP_CCW, clean_rk_time);
    set_timer_(eventCleanRbEnd,clean_rk_time,0);
    break;
  case eventCleanRbEnd:
    pump_all_stop();                 
    if(state==stReady){
      send_buf[1]=oper_step;
      send_buf[2]=oper_pr;
      //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
      //lcd_send_pack(evnetPrEnd,send_buf);
      set_timer_(eventStartSq,100,0); //asp time
    }else{
      if(ac_step)
        give_event(eventEngAutoClean,0);
      else  
        beep(0,500,3);
    }
    break;
    //-------------------------------etc_fc----------------------------------
  case eventBathDspOper:
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.bath_xtray_pos);
    Pump_Single_Run(0,0,fc.dsp_tm);
    set_timer_(fc.event[1],fc.dsp_tm,0);
    break;
  case eventBathDspEnd:
    Pump_Single_Break(0);
    beep(0,500,3);
    break;
  case eventBathAspOper:
    Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.bath_xtray_pos);
   // Stmt_AbsMove(ASP_MT,ASP_SPD, mt_ctrl.bath_asp_pos);
    servo_mv(mt_ctrl.bath_asp_pos);
    Pump_Single_Run(PUMP_ASP1,PUMP_CCW,fc.asp_tm);
  //  Pump_Single_Run(PUMP_ASP2,PUMP_CCW,fc.asp_tm);
    set_timer_(fc.event[3],fc.asp_tm,0);
    break;
  case eventBathAspEnd:
    Pump_Single_Break(PUMP_ASP1);
   // Pump_Single_Stop(PUMP_ASP2);
    mt_init_pos();
    beep(0,500,3);
    break;
  case eventSeqStop:
  case eventRunStop:
    usb_send_pack(eventSeqStop,send_buf);
    beep(1,500,1);
    //set_timer_(eventSpuOn,500,0);  
    HAL_NVIC_SystemReset();
    break;
    
    
    //--------------------------------eng------------------------------------
  case eventSopDsp:
    ft_state=stEngDsp;

     send_qc_dsp();
    if(state==stStby){
      pr_ps.dsp_pp_num=dsp.pum_num;
      pr_ps.rollbk_pp_num=dsp.pum_num;
      pr_ps.asp_pp_time=5000;
      pr_ps.dsp_pp_time=5000;  
      
      give_event(pr_ps.event[0],0);
  //   Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.bath_xtray_pos);
    }else if(state==stReady)
      set_timer_(dsp.event[0],100,0);  
    
    break;
  case eventSopAsp:
    send_qc_asp();
    give_event(asp.event[0],0);
    break;
  case eventFunPrime:
    ft_state=stEngPrime;
    
    send_qc_prime();
    give_event(pr_ps.event[0],0);
    break;
  case eventFunShake:
    if(shk.time<2)
      sk_cnt=shk.time;
    else 
      sk_cnt=shk.time-1;
    
    mt_ctrl.shake_speed=shk.speed;//*1600;
  //  send_qc_sk();
     usb_send_pack(eventFunShake,send_buf);
    give_event(shk.event[0],0);
    break;
  case eventFunRollBack:
  //  send_qc_rollbk();
      usb_send_pack(eventFunRollBack,send_buf);
    give_event(bk.event[0],0);                  
    break;
  case eventFunDry:
    drys.temp=usb_data_buf[2];
    drys.time=usb_data_buf[3];
    dry_cnt=(drys.time*60)-1;
    send_dry(eventFunDry,drys.temp,drys.time);
    give_event(drys.event[0],0);
    break;
  case eventBathDsp:
    send_qc_etc(eventBathDsp,fc.dsp_tm);
    X_AIS_Home(); 
    Asp_Home();
    give_event(fc.event[0],0);
    break;
  case eventBathAsp:
    send_qc_etc(eventBathAsp,fc.asp_tm);
    X_AIS_Home(); 
    Asp_Home();
    give_event(fc.event[2],0);
    break;
    //-----------------lcd cmd----------    
  case eventCrrtTotalTime:
    if(state==stReady||state==stPause){
      send_buf[0]=oper_step+1;//sq[oper_step].stNum;
      send_buf[2]=sq[oper_step].dword[19];
      send_buf[3]=step_time_min;
      //set_timer_(eventAirTempRead,1000,0);
    }
    usb_send_pack(eventCrrtTotalTime,send_buf);
    break;
  case evnetCtAndPt:
    if(state==stReady||state==stPause){
      usb_data_buf[1]=sq[oper_step].stNum;
      usb_data_buf[2]=oper_pr;
      usb_data_buf[3]=sq[oper_step].prNum[oper_pr];
     // set_timer_(eventTrayTempRead,1000,0);
    }
    usb_send_pack(evnetCtAndPt,usb_data_buf);
    break;
  case eventCntinRun:
    send_lcd_sq_data(eventCntinRun,0);
    //Sk_Home();
    
    give_event(eventStartSqPr,0);
    break;
  case eventPuse:

    if(usb_data_buf[3]){
      state=stPause;
      beep(0,300,1);
      give_event(eventhesPause,0);
    }else{
      //give_event(eventAppStart,0);
   
      give_event(eventhesPauseReStart,0);
      sq_strt_chk=0;
    }
    
    usb_send_pack(eventPuse,usb_data_buf);
    break;
  case eventAppStart:
    beep(0,100,2);
    give_event(eventStartSqPr,0);
    break;
  case eventAppStartRe:
    send_lcd_sq_data(eventStartSq,0);
    break;
  case eventStartSqPr:
    if(!sq_strt_chk){
      give_event(eventStartSq,0);
      sq_strt_chk=true;
    }else
      break;
    break;
  case eventStartSq:
    
    if(state==stPauseLcd)
      break;
    if(state==stStby){
      beep(0,200,1);
      BSP_SystemHW_Delay_msec(500);
      step_time_sec=0;
      step_time_min=0;
    //  send_lcd_sq_data(eventStartBtnRes,0);
     // asp.asp_num=dsp.dsp_num=usb_data_buf[0];
      oper_step=usb_data_buf[3]-1;
      //oper_pr=usb_data_buf[2];
      state=stReady;
      
          
#ifdef PLUS //heating
  temp_ctrl(4,heat_tmp.heat_pram_ch1); 
  dwHsSW_Delay_ms(1000);
  temp_ctrl(6,1); //heat on
  dwHsSW_Delay_ms(1000);      
#endif    
    }else{//else if(state==stReady)
      //{        
      oper_pr++;
      
      for(byte k=1; k<6; k++)
        Pump_Single_Break(k);
      horor_asp_time=INCU_Roll_Back;  
      
      if(sq[oper_step].prNum[oper_pr]==0){  
        oper_step++;
        step_time_sec=0;
        step_time_min=0;
        oper_pr=0;
      }
      state=stReady;
    }
    switch(sq[oper_step].stNum)
    {
    case analy:
      fan_oper=false;
#ifdef PLUS
      temp_ctrl(6,0); //heat off
      dwHsSW_Delay_ms(500); 
#endif
      send_buf[3]=sq[oper_step].stNum;
      usb_send_pack(eventAlyGo,send_buf);
      break;
    case Specimen_dsp:
      fan_oper=true;
      break;
    case Substrate_Sol:
      fan_oper=true;
      break;
    case Antibody:
      fan_oper=true;
      break;
    case Enzyme:
      fan_oper=true;
      break;
      
    case Washing1:
      fan_oper=false;
      if(sq[oper_step].prNum[oper_pr]==Incubation)
        AirFan_On;
      else AirFan_Off;
      break;
    case Washing2:
      fan_oper=false;
      if(sq[oper_step].prNum[oper_pr]==Incubation)
        AirFan_On;
      else AirFan_Off;
      break;
    case WashingDW:
      fan_oper=false;
      AirFan_On;
#ifdef PLUS
      temp_ctrl(4,687); //37 degree
      dwHsSW_Delay_ms(1000);
      temp_ctrl(6,1); //heat on
      dwHsSW_Delay_ms(1000); 
#endif  
      break;
    case dry:
      fan_oper=false;
      break;
      
      
    default:
      break;
    }

    switch (sq[oper_step].prNum[oper_pr])
    {
    case Prime_D:
    case Prime:

      pr_ps.dsp_pp_num=sq[oper_step].dword[oper_pr]>>24; 
      
      pr_ps.asp_pp_num=sq[oper_step].dword[oper_pr]>>24; 
      pr_ps.asp_pp_time =5000;
      pr_ps.dsp_pp_time =(sq[oper_step].dword[oper_pr]&0x0000ffff)*1000;
   /*   if(5000>pr_ps.asp_pp_time){
        pr_ps.asp_pp_time=5000;
        pr_ps.dsp_pp_time=5000;
      }*/
      //give_event( eventFunPrime,0);
       give_event(pr_ps.event[0],0);
      break;
    case Dispense_D:
    case Dispense:
      dsp.pum_num=sq[oper_step].dword[oper_pr]>>24;
      dsp.dsp_vol=set_ul_fun((sq[oper_step].dword[oper_pr]&0x0000ffff), dsp.pum_num);
      // dsp.dsp_vol=(sq[oper_step].dword[oper_pr]&0x0000ffff);
       set_timer_(dsp.event[0],100,0);  
      break;
    case Aspiration:
      //asp.dry_oper=sq[oper_step].dword[oper_pr]>>24;
      asp_rpeat_fu_flg=0;
      asp.asp_num=totoal_strip;
      asp.asp_time=(sq[oper_step].dword[oper_pr]&0x00ff)*1000;
      asp.cnt=1;//(sq[oper_step].dword[oper_pr]>>16)&0xff;
      give_event(asp.event[0],0);
      //give_event(eventSopAsp,0);
      break;
    case Incubation:
      shk.speed=sq[oper_step].dword[oper_pr]>>16;
      shk.time=(sq[oper_step].dword[oper_pr]&0x0000ffff)*60;
      give_event(eventFunShake,0);
      break;
  /*  case IncubationC:
      shk.speed=sq[oper_step].dword[oper_pr]>>16;
      incuC_cnt=(byte)sq[oper_step].dword[oper_pr]&0x0000ffff;
      give_event(eventSkOperC,0);
      break;*/
    case Buzzer:
      buzer=sq[oper_step].dword[oper_pr];
      if(buzer)
        beep(0,500,buzer);
      
      if(state==stReady){
        send_buf[1]=oper_step;
        send_buf[2]=oper_pr;
        //lcd_send_rc_pack(evnetPrEnd,evnetPrEndRes,send_buf);
        //lcd_send_pack(evnetPrEnd,send_buf);
        set_timer_(eventStartSq,100,0); //asp time
      }
      break;
    case Dry:
      drys.temp=sq[oper_step].dword[oper_pr]>>8;
      drys.time=(sq[oper_step].dword[oper_pr]&0x000000ff);
      dry_cnt=(drys.time*60)-1;
      give_event(drys.event[0],0);

      break;
    case Pause:
      //give_event(eventhesPause,0);
      mt_init_pos();
      Stmt_AbsMove(SHAKE_MT,SK_SPD,mt_ctrl.shake_sampl_dsp_pos);
      state=stPause;
      give_event(eventhesPause,0);
      //send_lcd_sq_data(eventhspPause,0);
      break;
    case Auto_Clean:
      clean.cnt=sq[oper_step].dword[oper_pr]>>16;
      if(clean.cnt)
        clean.cnt--;
      clean.dsp_time=((sq[oper_step].dword[oper_pr]>>8)&0xff)*1000;
      clean.asp_time=(sq[oper_step].dword[oper_pr]&0xff)*1000;
      give_event(clean.event[0],0);
      break;
    case Clean_RollBack:
     // clean_rk_time=(sq[oper_step].dword[oper_pr]&0xffff);
    //  give_event(eventCleanRbOper,0);
      if(food){
        if(sq[oper_step].stNum==Antibody)   bk.pum_num=5;
        else if(sq[oper_step].stNum==Enzyme)    bk.pum_num=6;
        else  bk.pum_num=sq[oper_step].dword[oper_pr]>>24; 
      }else
        bk.pum_num=sq[oper_step].dword[oper_pr]>>24; 
      
     // bk.pum_num=sq[oper_step].dword[oper_pr]>>24;
      bk.time=(sq[oper_step].dword[oper_pr]&0x0000ffff);
      give_event(bk.event[0],0);  
      break;
    case RollBack:
      bk.pum_num=sq[oper_step].dword[oper_pr]>>24;
      bk.time=(sq[oper_step].dword[oper_pr]&0x0000ffff);
      give_event(bk.event[0],0);  
      break;
    default: 
      //send_lcd_sq_data(eventStopSq,0);
      state=stStby;
      oper_step=0;
      oper_pr=0;
      sq_strt_chk=false;
      break;          
    }
    
    //send_lcd_sq_data(hspStart,)
    break;
  case eventhesPause:
   
   // beep(0,100,1);
    //ev_st=0;
    //state=stPauseLcd;
    switch (sq[oper_step].prNum[oper_pr])
    {
    case Aspiration :
      pause_cnt=asp.cnt;
      break;
    case Incubation:
      pause_cnt=sk_cnt;
      dSPIN_Hard_Stop(SHAKE_MT);
      sk_cnt=0;
      break;
    case Dry:
      pause_cnt=dry_cnt;
      BSP_ETC_OnOff_Device(FAN,FAN_OFF);
      BSP_ETC_OnOff_Device(HEATER,HEATER_OFF);
      dry_cnt=0;
      break;
    default:
      beep(0,100,0); 
      break;
    }
    init_evnet();
   // send_lcd_sq_data(eventhspPauseLcd,0);      
    break;
  case eventhesPauseReStart:
    beep(0,100,1);
    state=stReady;
  //  oper_step=usb_data_buf[1];
 //   oper_pr=usb_data_buf[2];
    switch (sq[oper_step].prNum[oper_pr])
    {
    case Aspiration :    
    //  asp.cnt=pause_cnt;
      break;
    case Incubation:
      sk_cnt=pause_cnt;
      //give_event(eventSkOper,0); 
      ev_st=eventSkOper;
      break;
    case Pause:
      ev_st=eventStartSqPr;
      break;
    case Dry:
      dry_cnt=pause_cnt;
      //give_event(eventdryOper,0);
      ev_st=eventdryOper;
      break;
    default:
      break;
    }
    if(ev_st){
     give_event(ev_st,0);
    }else{
      give_event(eventStartSq,0);
    }
     ev_st=0;
    break;
  case hseEngHeatOn:
    beep(0,100,1);
    BSP_ETC_OnOff_Device(FAN,FAN_ON);
    BSP_ETC_OnOff_Device(HEATER,HEATER_ON);
    break;
  case hseEngHeatOff:
    beep(0,100,1);
    BSP_ETC_OnOff_Device(FAN,FAN_OFF);
    BSP_ETC_OnOff_Device(HEATER,HEATER_OFF);
    break;
  case eventEngAutoClean:
    if(!ac_step){
      beep(0,100,1);
      clean_rk_time=10000;
      give_event(eventCleanRbOper,0);
      ac_step++;
      break;
    }else if(ac_step==1){
      clean.cnt=3;
      if(clean.cnt)
        clean.cnt--;
      clean.dsp_time=7*1000;
      clean.asp_time=35*1000;
      give_event(clean.event[0],0);
    }else if(ac_step==2){
      clean_rk_time=16000;
      give_event(eventCleanRbOper,0);
    }else if(ac_step==3){
      ac_step=0;
      //send_buf[0]

      send_buf[1]=0xFF;
      send_buf[3]=0xAA;
     // send_lcd_sq_data(eventEngAcStop,0);
      usb_send_pack(evnetCtAndPt,send_buf);
      beep(0,100,2);
      break;
    }
    ac_step++;
    break;
  case eventEngAcPause:
    give_event(eventEngAutoClean,0);
    break;
    
  case eventTimer500ms: 
    //send_buf[0]=1;
    //usb_send_pack(eventCrrtTotalTime,send_buf);
    switch(state)
    {
    case stStby:
      if(stop_pre_st==stReady){
        stop_pre_st=0;
        switch (sq[oper_step].prNum[oper_pr])
          
          
        {
        case Prime:
          Stmt_stop_pr();
          break;
        case Dispense:
          Stmt_stop_pr();
          break;
        case Aspiration:
          Stmt_stop_pr();
          break;
        case Incubation:
          sk_cnt=1;
          break;
        case Buzzer:
          break;
        case Dry:
          dry_cnt=1;
          break;
        case Pause:
          break;
        default: break;
        }
        init_evnet();
        //   beep(0,100,2);
        pump_all_stop();
        init_evnet();
        tmchan_init();
        mt_init_pos();
        state=stStby;  
        beep(0,100,0); 
      }
      break;
    case stReady:
      stop_pre_st=stReady;
      break;
    }        
    break;
    
  case eventTimer100ms:
    break;
  case eventTimer1s:
      HAL_GPIO_TogglePin(LED_MCU_CHECK_GPIO_Port,LED_MCU_CHECK_Pin);
      
    if(sk_cnt){
    //  sk_cnt--;
      //give_event(shk.event[1],0); 
      if(state==stPause){
      // break;
        step_time_sec=0;
      }
      else{
        sk_cnt--;
        set_timer_(shk.event[1],500,0); 
      }
    }     
    if(dry_cnt)
    {
      dry_cnt--;
    //  give_event(drys.event[1],0);  
    }
 //-----------------
    if(state==stReady){
      switch (sq[oper_step].prNum[oper_pr])
      {
      case Incubation:
      case Dry:
        if(!horor_asp_time){
          give_event(eventAspTime,0);
           horor_asp_time=INCU_Roll_Back;
        }
          horor_asp_time--;   
        break;
      default:
   
        break;
      }
      
    }
      if(fan_oper)
      if(!fan_cnt_time--){
          set_timer_(eventOperFanEnd,fan_oper_time*1000,0); //asp time
          AirFan_On;
          fan_oper=false;
      }
    
    break;
  case eventOperFanEnd:
        AirFan_Off;
        fan_cnt_time=1140;
    break;
  case eventDevReset:
    usb_send_pack(eventDevReset,usb_data_buf);
    HAL_NVIC_SystemReset();
    break;  
  case eventError:
    state=stErr;
    beep(1,700,3);
    break;
  case hseInfFwVer:
   send_buf[0]=*vr_pnt;
   send_buf[1]=*(vr_pnt+2);
   
   send_buf[2]=*(vr_pnt+4);
   send_buf[3]=*(vr_pnt+5);
   //lcd_send_pack(hspInfFwVer,send_buf);
   break;
    
    
    
    
    
  case eventEquiType:
    // usb_send_pack(eventEquiType,equip_test);
    break;

  case eventLedWrite:
    pp_tm.led_vol[0]=usb_data_buf[0];
    pp_tm.led_vol[1]=usb_data_buf[1];
    pp_tm.led_vol[2]=usb_data_buf[2];
    pp_tm.led_vol[3]=usb_data_buf[3];
    led_light_set(usb_data_buf);
    pump_tm_wirte();
    usb_send_pack(eventLedWrite,usb_data_buf);
    break;
  case eventLedRead:
    pump_tm_read();
    send_buf[0]=pp_tm.led_vol[0];
    send_buf[1]=pp_tm.led_vol[1];
    send_buf[2]=pp_tm.led_vol[2];
    send_buf[3]=pp_tm.led_vol[3];
    //led_read();
    usb_send_pack(eventLedRead,send_buf);
   // usb_send_pack(eventLedRead,led_vol);
    //set_timer_(eventSystickTimer,3000,0); 
    break;

  case eventLedOnOff:
    if(usb_data_buf[3]) led_toggl(LEDON);
    else              led_toggl(LEDOFF);
    usb_send_pack(eventLedOnOff,usb_data_buf);
    break;
    case eventSystickTimer:
   //  usb_send_pack(eventLedRead,led_vol);
    break;
  case eventCrrtProTime:
    // usb_send_pack(eventCrrtProTime,test);
    break;
    //     case eventTest:
    //          flag =~flag;
    //         BSP_IndcatorLED_OnOff(1,flag);
    //     break;
  case eventAspTime:
    switch (sq[oper_step].prNum[oper_pr])
    {
    case Incubation:
    case Dry:
      for(byte k=1; k<7; k++)
        Pump_Single_Run(k,PUMP_CCW,5000);
        Pump_Single_Run(PUMP_ASP1,PUMP_CCW,8000);  //pump dir set up
     // Pump_Single_Run(PUMP_ASP2,PUMP_CCW,8000);  //pump dir set up  
      set_timer_(eventAspTimeEnd,8000,0); //asp time
      break;
    default:
      break;
    }
       
    break;
  case eventAspTimeEnd:
    
    switch (sq[oper_step].prNum[oper_pr])
    {
    case Incubation:
    case Dry:     
      Pump_Single_Break(PUMP_ASP1);

      for(byte k=1; k<7; k++)
        Pump_Single_Break(k);
       // Pump_Single_Stop(k);Pump_Single_Stop(k);
      break;
    default:
      break;
    }
    
    break;
  }

  return event;
}
