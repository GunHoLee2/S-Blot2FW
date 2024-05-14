#include "main.h"
#include "string.h"

#define CHATTER_CYCLES   30//30

enum bath_state bath_st=stBathNone;

void bath_en(uint32_t state)
{
  BSP_ETC_OnOff_Device(4, state);
}

byte e=0;

  uint16_t port_prev=0, chatter_cycles=0;
  bool button_changed=0;//,locked=0;
  uint16_t port=eventBathEmp1;

void bath_polling()
{
 
  
  
  
  if(!BSP_ETC_Input_Check(1)) 
    port=eventBathFull1;
  else port=eventBathEmp1;
  
  
  
 // if(!BSP_ETC_Input_Check(2)) 
 //   port[2]=eventBathFull2;
//  else port[2]=eventBathEmp2;*/
  
   if(port!=port_prev)
      {
        // long repeat or pushed
        port_prev=port;
        chatter_cycles=CHATTER_CYCLES;
        return;
      }
    else if(chatter_cycles)
      {
        chatter_cycles--;
        if(!chatter_cycles)
         button_changed=true;       
        return;
      }
    
     
    if(button_changed)
     {  
        give_event(port,0);
        button_changed=false;
        //sysOnTimer=0;
     }
}

bool bath_full_flg[2]={0,0};
uint32_t bath_full_cnt=0;
byte bath_oper_cnt=0;
uint32_t bath_aver_buf[3]={0,};
byte auto_cal_pump_num=0,pre_auto_cal_pump_num=0;
void pp_cal_control(event event)
{
  switch(event)
  {
  case eventBathFull1:
    /*if(state==stPrepare){
      pr_ps.asp_pp_time=15000;
      if( bath_full_flg[0]==false)
        give_event(eventPrimeAspOut,0);
      set_timer_(eventAllPumpCall,17000,0); 
    }
    if((eve_cmd==eventBathVolSet)||(eve_cmd==eventPumpAutoCalSet)){
      bath_full_cnt=0;
    }
    
    bath_st=stBathFull1;q
    bath_full_flg[0]=true;*/
    error(eventError,errBathFull);
    HAL_NVIC_SystemReset();
    //set_timer_(eventDevReset,2000,0);  
    
      
     
    break;
  case eventBathEmp1:
   /* bath_st=stBathEmpty1;
    if(state==stPrepare){
      tmchan_init();
      set_timer_(eventAllPumpCall,1000,0); 
    }
    bath_full_flg[0]=false;*/
    
    break;
  case eventBathFull2:
    bath_full_flg[1]=true;
    bath_st=stBathFull2;
    if(eve_cmd==eventBathVolSet){ 
      Pump_Single_Break(3);
      // bath_en(0);
      bath_aver_buf[bath_oper_cnt]=bath_full_cnt;//auto_set_ul_fun(bath_full_cnt,3);
      bath_full_cnt=0;
      pr_ps.asp_pp_time=30000;
      bath_oper_cnt++;
      if(bath_oper_cnt>2){  
        eve_cmd=0;
        pp.bath_vol=auto_set_ul_fun(bath_averge(3),3);
        send_pp_cfg(eventReadBathVol, pp.bath_vol);
        pump_cfg_wirte();
        memset(bath_aver_buf,0,sizeof(bath_aver_buf));
        bath_oper_cnt=0;
        beep(0,100,2);     
      }
      bath_en(0);
      give_event(eventPrimeAspOut,0);
    }else if(eve_cmd==eventPumpAutoCalSet){
      Pump_Single_Break(auto_cal_pump_num);
      bath_aver_buf[bath_oper_cnt]=bath_full_cnt;//auto_set_ul_fun(bath_full_cnt,auto_cal_pump_num);
      bath_full_cnt=0;
      pr_ps.asp_pp_time=30000;
      bath_oper_cnt++;
      
      bath_en(0);
      give_event(evnetPumpAutoCalPumpOperEnd,0);
      
    }
    
    break;
  case eventBathEmp2:
   // bath_full_flg[1]=false;
 //   bath_st=stBathEmpty2;
    break;
    
  case eventBathVolCntnOper:
    // bath_en(1);
    if(eve_cmd==eventBathVolSet){
      bath_en(1);
      Pump_Single_Run(3,PUMP_CW,0); 
    }
    break;
  case eventPumpAutoCalSart:
    bath_en(0);
    prime_oper_cnt=1;
    prime_oper_flg=true;
    auto_cal_pump_num=3;
    pre_auto_cal_pump_num=auto_cal_pump_num;
    pr_ps.dsp_pp_num=3;
    pr_ps.dsp_pp_time=4000;
    pr_ps.asp_pp_time=7000;
    eve_cmd=eventPumpAutoCalSet;
    give_event(eventPrimeBathPsn,0);
    break;
  case eventPumpAutoCalPumpSel:
    if(pre_auto_cal_pump_num==auto_cal_pump_num){
      bath_en(1);
      bath_full_cnt=0;
      Pump_Single_Run(auto_cal_pump_num,PUMP_CW,0);
    }else{
      bath_oper_cnt=0;
      if(auto_cal_pump_num>8)
      {
        auto_cal_pump_num=0;
        pre_auto_cal_pump_num=0;
        eve_cmd=0;
        auto_cal_result();
        pump_cfg_wirte();
        beep(0,500,3);
      }else{
        prime_oper_cnt=1;
        bath_full_cnt=0;
        prime_oper_flg=true;
        pre_auto_cal_pump_num=auto_cal_pump_num;
        pr_ps.dsp_pp_num=auto_cal_pump_num;
        pr_ps.dsp_pp_time=4000;
        pr_ps.asp_pp_time=7000;  
        give_event(eventPrimeBathPsn,0);
      }
    }
    break;
  case evnetPumpAutoCalPumpOperEnd:
    if(bath_oper_cnt>2)
    {   
      bath_averge(auto_cal_pump_num);
      auto_cal_pump_num++;
      beep(0,100,1); 
    }
    bath_en(0);
    give_event(eventPrimeAspOut,0);
    
    break;
    
  }
}

uint32_t auto_set_ul_fun(uint32_t ms, byte pump_num)
{
  return (ms*STN_UL)/pp.offset[pump_num-1];
}
uint32_t bath_averge(byte pump_num)
{
  uint32_t aver_time=0;
  byte k=0;
  for(k=0;k<3;k++)
    aver_time+=bath_aver_buf[k];
  pp.bath_time[pump_num-1]=aver_time/3;
  return pp.bath_time[pump_num-1];
}

void auto_cal_result()
{
  byte k=0;
  for(k=2;k<8;k++)
    pp.offset[k]=(uint16_t)((pp.bath_time[k]*STN_UL)/pp.bath_vol);
  
}
/*
int putchar(int ch)

{
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  USART_SendData(USART2,ch);
  return ch;
}
*/