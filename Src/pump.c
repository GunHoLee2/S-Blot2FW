#include "main.h"
#include "string.h"

struct pump pp;
struct pump_time pp_tm;

/*
//PUMP Setting Mode
          eventPumpOper=0x0401,
          eventReserve20,
          eventReserve21,
          eventPumpAutoCalSet,
          eventReserve22,
          eventPumpUseTimeSet,
          eventReadPumpUseTime,
          eventPumpOffSet,
          eventReserve23,
          eventReserve24,
          
          eventReserve25,
          eventReserve26,
          eventPumpRollBack,
          eventReserve27,
          eventReserve28,
          eventReadBathVol,
          eventBathVolSet,
*/

uint32_t kk =0;
uint16_t pump_oper_timer=0;
byte disp_offset_cnt=3;
byte prime_oper_cnt=0;
bool prime_oper_flg=0;
uint16_t eve_cmd=0;
event pump_control(event event)
{

    switch(event)
    {
    case eventPumpOper:
      if(usb_data_buf[3]){
          if(usb_data_buf[0]==9)
            pump_all_call(usb_data_buf[1],0);
          else
          Pump_Single_Run(usb_data_buf[0],usb_data_buf[1],0);
      }else
        Pump_Single_Break(usb_data_buf[0]);
        send_pp_cfg(eventPumpOper, pp_val_ch(usb_data_buf,4));
    break;
    case eventPumpAutoCalSet:
         usb_send_pack(eventPumpAutoCalSet,0);
         give_event(eventPumpAutoCalSart,0);
    break;
    case eventSopTImeSet:
    case eventPumpUseTimeSet:
        send_pp_cfg(eventSopTImeSet, pp_val_ch(usb_data_buf,4));
        pp_tm.use_time[usb_data_buf[0]-1]=pp_val_ch(usb_data_buf, 2);
        pump_tm_wirte();
    break;
    case eventReadPumpUseTime:
        pump_tm_read();
        send_pp_cfg(eventReadPumpUseTime, pp_tm.use_time[usb_data_buf[0]-1]);          
    break;
    case eventPumpOffSet:
        if(usb_data_buf[1]){
          state=stPcEng;
          usb_send_pack(eventPumpOffSet,usb_data_buf);
          pr_ps.dsp_pp_num=usb_data_buf[0];
          pr_ps.dsp_pp_time=6000;
          pr_ps.asp_pp_time=9000;
          dsp.dsp_num=3;
          give_event(eventPrimeBathPsn,0);
        }else{
         pump_cfg_read();
         usb_data_buf[2]=pp.offset[usb_data_buf[0]-1]>>8;
         usb_data_buf[3]=pp.offset[usb_data_buf[0]-1]&0xff;
         usb_send_pack(eventPumpOffSet,usb_data_buf);
        }
        
    break;
    case eventPumpRollBack:
     asm("NOP");
    break;
    case eventReadBathVol:
      pump_cfg_read();
      send_pp_cfg(eventReadBathVol, pp.bath_vol);     
    break;
    case eventBathVolSet:
      bath_en(1);
      prime_oper_cnt=1;
      prime_oper_flg=true;
      pr_ps.dsp_pp_num=3;
      pr_ps.dsp_pp_time=4000;
      pr_ps.asp_pp_time=7000;
      usb_send_pack(eventBathVolSet, usb_data_buf);
      eve_cmd=eventBathVolSet;
      give_event(eventPrimeBathPsn,0);
    break;
    case eventPumpOffSetOper:
      if(disp_offset_cnt==3)
          {    
            //memcpy(&pp.offset[usb_data_buf[0]-1],&usb_data_buf[2],sizeof(uint16_t));
            pp.offset[usb_data_buf[0]-1]=(usb_data_buf[2]<<8|(usb_data_buf[3]&0xff));
            pump_cfg_wirte();
            mt_init_pos();                               
            Stmt_AbsMove(SHAKE_MT,SK_SPD,mt_ctrl.shake_dsp_pos);
            if(usb_data_buf[0]%2){
              Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos);
            }else
              Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos-mt_ctrl.strip_width);
           /* if((usb_data_buf[0]==4)||(usb_data_buf[0]==6))
            Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos+DSP_CH_WIDTH);
            else if((usb_data_buf[0]==3)||(usb_data_buf[0]==8))
            Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos+(DSP_CH_WIDTH*2));
            else*/
            //Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.xtray_dsp_pos);
          }
          Pump_Single_Run(usb_data_buf[0],PUMP_CW,pp.offset[usb_data_buf[0]-1]);
          set_timer_(eventPumpOffSetEnd,pp.offset[usb_data_buf[0]-1],0);   
    break;
    case eventPumpOffSetEnd:
       Pump_Single_Break(usb_data_buf[0]);
       disp_offset_cnt--;
       if(!disp_offset_cnt){
       mt_init_pos();
       disp_offset_cnt=3;
       state=stStby;
       break;
       }
       Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.strip_width);
       give_event(eventPumpOffSetOper,0);
       //kk=set_ul_fun(600, usb_data_buf[0]);
    break;
    case eventPrimeCntinOper:
        if(prime_oper_cnt--)
           give_event(eventPrimeBathPsn,0);
        else{
           prime_oper_flg=false;
           prime_oper_cnt=1;
           if(eve_cmd==eventBathVolSet){
            Pump_Single_Run(3,PUMP_CW,0); 
           }else if(eve_cmd==eventPumpAutoCalSet)
              give_event(eventPumpAutoCalPumpSel,0);
           
        }
      break;
    case eventPrimeCntinEnd:
      break;
    default:break;
    }
return event;
}


int32_t pp_val_ch(byte *data, byte size)
{
  int32_t temp=0;
  if(size==4){
  temp|=data[0]<<24;
  temp|=data[1]<<16;
  temp|=data[2]<<8;
  temp|=data[3];
  }else{
  temp|=data[2]<<8;
  temp|=data[3];
  }
  return temp;  
}

/*
int32_t pp_tm_cnt[PUMP_CHNNEL]={0,};
int32_t dev_num=0;
void pp_tim_cnt(int32_t devNm)
{
   dev_num=devNm;
   pp_tm_cnt[devNm]=0;
   
  if(pp_tm_cnt[devNum]>1000){
   pp_tm.use_time[devNum]++;
   pp_tm_cnt[devNum]=0;
  }
}*/

uint32_t pp_tim_ms[PUMP_CHNNEL]={0,};
void pp_tim_sum(int32_t devNm, uint time)
{
  pp_tim_ms[devNm-1]+=time;
  
  if(pp_tim_ms[devNm-1]>=1000){
    pp_tm.use_time_s[devNm-1]+=pp_tim_ms[devNm-1]/1000;
    
    if((pp_tim_ms[devNm-1]%1000)>0)
    pp_tim_ms[devNm-1]=pp_tim_ms[devNm-1]%1000;
    else
    pp_tim_ms[devNm-1]=0;
  }
  
  if(pp_tm.use_time_s[devNm-1]>=60){
    pp_tm.use_time_min[devNm-1]++;
    pp_tm.use_time_s[devNm-1]=0;
  }
  if(pp_tm.use_time_min[devNm-1]>=60){
    pp_tm.use_time[devNm-1]++;
    pp_tm.use_time_min[devNm-1]=0;
  }
  
}

void pump_mem_init()
{
  byte k;
   if(pp_tm.use_time_min[7]==0xffffffff)
     memset(&pp_tm,0,sizeof(pp_tm));
   
   if(pp.offset[0]==0xffff){
     for(k=0;k<8;k++){
       pp.offset[k]=1000;
       pp.bath_vol=4500;
     }
     
     pump_cfg_wirte();
   } 
}

uint32_t set_ul_fun(uint32_t set_ul, byte pump_num)
{
  return ((uint32_t)pp.offset[pump_num-1]*set_ul)/STN_UL;
}