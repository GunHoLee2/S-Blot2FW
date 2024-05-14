
#include "main.h"
#include "string.h"
//#include "stm32f1xx_hal_uart.h"

#define RECEIVE_BUF_LENGTH 100
static byte const NO_RECIVED=0xFF;
byte usb_receive_bufs[RECEIVE_BUF_LENGTH];
byte sub_receive_bufs[RECEIVE_BUF_LENGTH];
byte current_rec_buf_ind=0;
byte usb_recived_rec_buf_ind=0xff; // 0xff
byte *usb_receive_buf;
byte *usb_received_buf;

byte sub_current_rec_buf_ind=0;
byte sub_recived_rec_buf_ind=0xff; // 0xff
byte *sub_receive_buf;
byte *sub_received_buf;


byte *pnt=0, *pnt1=0 ;
uint16_t checksum,checksum_b;
uint16_t sub_checksum,sub_checksum_b;
uint16_t checksums=0xffff;
//byte compar_reult;

//data char k,checksums,checksum_b;

#define RS232_START 0x5B
#define RS232_END 0x5D

#define RS232_START1 0xFB

#define RS232_END1 0xFD

#define LED_ON_OFF           1
#define LED_SET              2
#define LED_READ             3
#define LED_HEAT_SET_1       4
#define LED_HEAT_SET_2       5
#define LED_HEAT_OnOff       6
#define LED_CMD_Temp_Set     7
#define LED_CMD_Temp_Set2     8


/*
void usb_uart_init()
{
 HAL_UART_Receive_IT(&hUSB_COM_Handle, usb_receive_bufs, sizeof(usb_receive_bufs));
}*/

uint8_t ascii2hex(uint8_t code)
{
  uint8_t result = 0;
  
  result = code - 0x30;
  if(result > 9)
    result -= 7;
  return result;
}


uint8_t math_uint(uint8_t lsb, uint8_t hsb )
{
    uint16_t return_vl=0;
    return_vl = ascii2hex(lsb) << 4;
    return_vl |= ascii2hex(hsb);
    return (uint8_t)return_vl;
}


uint jo=0;
byte escap=0, escap1=0;
byte buf_check[1000]={0,};


void led_rx_int(byte usb_chr)
{
  byte chr1=0;
  // byte i=0;
  chr1=usb_chr;
  
  if((chr1==RS232_START1)&&!escap1)
  {
    escap1=1;
    sub_current_rec_buf_ind=0;
    pnt1=sub_receive_buf=&sub_receive_bufs[sub_current_rec_buf_ind];
    sub_checksum_b=0;
    sub_checksum=0;
  }else if(pnt1){  
    

    sub_current_rec_buf_ind++;
    *(pnt1++)=chr1;
    
    if(sub_current_rec_buf_ind>5){
      sub_current_rec_buf_ind=0;
      pnt1=0;
      escap1=0;
      sub_receive_buf=0;
      if(sub_receive_bufs[5]==0xFD){
        exe_led_ctrl(sub_receive_bufs[0]);
        return;
      }
    }
    
    
  }
}
void usb_rx_int(byte usb_chr)
{
  byte chr=0;
  byte i=0;
  chr=usb_chr;
//  buf_check[jo]=chr;
 // jo++;
  //usb_receive_bufs[i]=chr;
  //i++;
  
//-------------------------------------
  if((chr==RS232_START)&&!escap)
  {
    escap=1;
    current_rec_buf_ind=0;
    pnt=usb_receive_buf=&usb_receive_bufs[current_rec_buf_ind];
    checksum_b=0;
    
    
    checksum=0;
     
  }else if(pnt){
    
    current_rec_buf_ind++;
      if(chr==RS232_END){
        checksum_b&=0x00ff;
        checksum=math_uint(usb_receive_bufs[14],usb_receive_bufs[15]);
        if(checksum==checksum_b){
            usb_recived_rec_buf_ind=current_rec_buf_ind;
             //BSP_UART_PutIT_Byte(3,usb_receive_bufs,18);
            for(i=0;i<16;i++)
            usb_receive_bufs[i]=ascii2hex(usb_receive_bufs[i]);
            current_rec_buf_ind=0;
            pnt=0;
            checksum=0;
            checksum_b=0;
            escap=0;
          
            et_cmd_ctrl();
          return;   
        }else{
         current_rec_buf_ind=0;//checke sum error
         pnt=0;
         checksum=0;checksum_b=0;
         escap=0;
         return;
        }
       }
       *(pnt++)=chr;
      if(current_rec_buf_ind<15)
       checksum_b+=chr;
  }//else{
//    //pnt=0;
//    //current_rec_buf_ind=0;
//    checksum=0;
//    checksum_b=0;
    
 // }
}
byte led_vol[4]={0,};
void exe_led_ctrl(byte cmd)
{
  switch(cmd)
  {
  case LED_ON_OFF:
    break;
  case LED_SET:
    break;
  case LED_READ:
    break;
  case LED_CMD_Temp_Set:
    led_vol[2]=sub_receive_bufs[1];
    led_vol[3]=sub_receive_bufs[2];
    if(air_temp_read_flg){
     usb_send_pack(eventAirTempRead,  led_vol);
     air_temp_read_flg=false;
    }else
    usb_send_pack(eventHeatPadTempRead,  led_vol);
    break;
  case LED_CMD_Temp_Set2:
    led_vol[2]=sub_receive_bufs[1];
    led_vol[3]=sub_receive_bufs[2];
    usb_send_pack(eventTrayTempRead,  led_vol);
    break;
  default:
    break;
  }
    USB_IT_Enable();
    memset(sub_receive_bufs,0,RECEIVE_BUF_LENGTH);
}

uint8_t usb_data_buf[4]={0,};
uint16_t event_cmd=0,etc_event_cmd=0;
uint8_t dat[4]={0x05, 0x05, 0 , 0};
uint16_t cmd_code=0;
void execute_usb_rs232()
{
    if(usb_recived_rec_buf_ind!=NO_RECIVED)
      {
        usb_received_buf=(usb_receive_bufs+usb_recived_rec_buf_ind);
        event_cmd=usb_receive_bufs[0]<<12|usb_receive_bufs[1]<<8|usb_receive_bufs[2]<<4|usb_receive_bufs[3];
        usb_data_buf[0]=usb_receive_bufs[6]<<4|usb_receive_bufs[7];
        usb_data_buf[1]=usb_receive_bufs[8]<<4|usb_receive_bufs[9];
        usb_data_buf[2]=usb_receive_bufs[10]<<4|usb_receive_bufs[11];
        usb_data_buf[3]=usb_receive_bufs[12]<<4|usb_receive_bufs[13];
        
        switch(event_cmd)
          {
           case 0:
            // event_cmd=eventEquiType;
           break;
           case eventSpuOn:    // the installation of the global parameters SPU
            // memcpy(&config,&received_buf[2],sizeof(config));
            break;
           case eventSopDsp:
             vSwap_u8bit(&usb_data_buf[0],&usb_data_buf[1]);
             vSwap_u8bit(&usb_data_buf[2],&usb_data_buf[3]);
             memcpy(&dsp.dsp_num,usb_data_buf,sizeof(usb_data_buf));
              //dsp.pum_num=usb_data_buf[4];
              //dsp.dsp_num=usb_data_buf[3];
             totoal_strip=dsp.dsp_num;
             dsp.dsp_vol=set_ul_fun(dsp.dsp_vol, dsp.pum_num);
             break;
           case eventSopAsp:
             totoal_strip=asp.asp_num=usb_data_buf[2];
            // asp.pum_num=usb_data_buf[2];
             asp.asp_time=(uint32_t)(usb_data_buf[3]*1000);
              //dsp.pum_num=usb_data_buf[4];
              //dsp.dsp_num=usb_data_buf[3];
            break;
          case eventFunPrime:
            pr_ps.dsp_pp_num=usb_data_buf[0];
           // pr_ps.asp_pp_num=5000;
            pr_ps.rollbk_pp_num=usb_data_buf[0];
            //if(3>usb_data_buf[3])
             // usb_data_buf[3]=5;
            pr_ps.asp_pp_time=(uint32_t)(usb_data_buf[3]*1000)+3000;
            pr_ps.dsp_pp_time=(uint32_t)(usb_data_buf[3]*1000);
            //lcd_send_pack(eventFunPrime,usb_data_buf);
            break;
          case eventFunShake:
            shk.speed=usb_data_buf[0];
            shk.time=usb_data_buf[2]<<8;
            shk.time|=usb_data_buf[3];
            /// memcpy(&shk.time,&usb_data_buf[3],sizeof(shk.time));
            break;
          case eventFunRollBack:
            bk.pum_num=usb_data_buf[0];
            bk.time=usb_data_buf[3]*1000;
            break;
          case eventBathDsp:
            fc.dsp_tm=(uint32_t)usb_data_buf[3]*1000;
            break;
          case eventBathAsp:
            fc.asp_tm=(uint32_t)usb_data_buf[3]*1000;
            break;
          case evnetPrEndRes:
            cmd_code=evnetPrEndRes;
            break;
          
      
          default:break;  
            
          }
        set_timer_(event_cmd,50,0); 
        usb_recived_rec_buf_ind=NO_RECIVED;
    // for the testing
         //print("recived cmd=%d ", received_buf[1]);
      }
  
}

#define TRANSMIT_BUF_LENGTH 50
byte usb_transmit_bufs[TRANSMIT_BUF_LENGTH];
void usb_send_pack(enum cntrl_event cmd, uint8_t *data)
{
  uint8_t chksum=0;
  uint16_t ev_cmd=cmd;
  int i;
   if(state==stErr)
     ev_cmd|=0x8000;
   
   if(cmd==eventEquiType)
     ev_cmd=0;
   
   usb_transmit_bufs[0]=RS232_START;
   sprintf((char*)&usb_transmit_bufs[1], "%X", (ev_cmd>>12)&0x0f);
   sprintf((char*)&usb_transmit_bufs[2], "%X", (ev_cmd>>8)&0x0f);
   sprintf((char*)&usb_transmit_bufs[3], "%X", (ev_cmd>>4)&0x0f);
   sprintf((char*)&usb_transmit_bufs[4], "%X", ev_cmd&0x0f); 
   usb_transmit_bufs[5]|=0x30;
   usb_transmit_bufs[6]|=0x38;
   for(i = 0; i < 4 + 1; i++)
    sprintf((char*)&usb_transmit_bufs[(i*2)+7], "%02X", *(data+i));
    for(i = 0; i < 14; i++) chksum += usb_transmit_bufs[i + 1];
    sprintf((char*)&usb_transmit_bufs[15], "%02X]", chksum);
   usb_transmit_bufs[17]|=RS232_END;
   BSP_UART_PutIT_Byte(0, usb_transmit_bufs,18);
   memset(usb_transmit_bufs,0,TRANSMIT_BUF_LENGTH);
}

void lcd_send_pack(enum cntrl_event cmd, uint8_t *data)
{
 // uint8_t chksum=0;
 // uint16_t ev_cmd=cmd;
  //int i;
  /*
   if(state==stErr)
     ev_cmd|=0x8000;
   
   if(cmd==eventEquiType)
     ev_cmd=0;
   
   usb_transmit_bufs[0]=RS232_START1;
   sprintf((char*)&usb_transmit_bufs[1], "%X", (ev_cmd>>12)&0x0f);
   sprintf((char*)&usb_transmit_bufs[2], "%X", (ev_cmd>>8)&0x0f);
   sprintf((char*)&usb_transmit_bufs[3], "%X", (ev_cmd>>4)&0x0f);
   sprintf((char*)&usb_transmit_bufs[4], "%X", ev_cmd&0x0f); 
   usb_transmit_bufs[5]|=0x30;
   usb_transmit_bufs[6]|=0x38;
   for(i = 0; i < 4 + 1; i++)
    sprintf((char*)&usb_transmit_bufs[(i*2)+7], "%02X", *(data+i));
    for(i = 0; i < 14; i++) chksum += usb_transmit_bufs[i + 1];
    sprintf((char*)&usb_transmit_bufs[15], "%02X]", chksum);
   usb_transmit_bufs[17]|=RS232_END1;
   BSP_UART_PutIT_Byte(1, usb_transmit_bufs,18);*/
  
  uint8_t index =0;
  usb_transmit_bufs[index++] = RS232_START1;
  usb_transmit_bufs[index++] = cmd;
  usb_transmit_bufs[index++] = data[0];
  usb_transmit_bufs[index++] = data[1];
  usb_transmit_bufs[index++] = data[2];
  usb_transmit_bufs[index++] = data[3];
  usb_transmit_bufs[index++] = RS232_END1;
  
 // HAL_UART_Transmit(&huart2, (uint8_t*)usb_transmit_bufs, 7, 50); // 송신함수
   BSP_UART_PutIT_Byte(1,usb_transmit_bufs, 7);
  //  while ((USART2->SR & 0x40)==0);
 // uart2_tx_flag =0;
   memset(usb_transmit_bufs,0,7);
}

void led_ctrl(byte cmd,uint32_t data)
{
   usb_transmit_bufs[0]=RS232_START1;
   usb_transmit_bufs[1] = cmd;
   usb_transmit_bufs[2]=(data>>24)&0xff;
   usb_transmit_bufs[3]=(data>>16)&0xff;
   usb_transmit_bufs[4]=(data>>8)&0xff;
   usb_transmit_bufs[5]=data&0xff;
   usb_transmit_bufs[6]|=RS232_END1;
   BSP_UART_PutIT_Byte(1, usb_transmit_bufs,7);
   memset(usb_transmit_bufs,0,TRANSMIT_BUF_LENGTH);
}

void temp_ctrl(byte cmd,uint32_t data)
{
   usb_transmit_bufs[0]=RS232_START1;
   usb_transmit_bufs[1] = cmd;
   usb_transmit_bufs[2]=(data>>24)&0xff;
   usb_transmit_bufs[3]=(data>>16)&0xff;
   usb_transmit_bufs[4]=(data>>8)&0xff;
   usb_transmit_bufs[5]=data&0xff;
   usb_transmit_bufs[6]|=RS232_END1;
   BSP_UART_PutIT_Byte(1, usb_transmit_bufs,7);
   memset(usb_transmit_bufs,0,TRANSMIT_BUF_LENGTH);
}

void led_light_set(byte *data)
{
 int buf=0;
 buf=*data;
 buf|=*(data+1)<<8;
 buf|=*(data+2)<<16;
 buf|=*(data+3)<<24;  
 led_ctrl(0x02,buf);
}

void temp_set(byte *data)
{
 int buf=0;
 buf=*data;
 buf|=*(data+1)<<8;
 buf|=*(data+2)<<16;
 buf|=*(data+3)<<24;  
 led_ctrl(0x07,buf);
}

void tray_temp_set(byte *data)
{
 int buf=0;
 buf=*data;
 buf|=*(data+1)<<8;
 buf|=*(data+2)<<16;
 buf|=*(data+3)<<24;  
 led_ctrl(0x08,buf);
}



void led_read()
{
  led_ctrl(0x03,0);
}

void led_toggl(byte set)
{
  if(set)led_ctrl(0x01,0x01010101);
  else led_ctrl(0x01,0x0000);
}

void et_cmd_ctrl()
{
  uint8_t cam_buf[4]={0,};
  uint8_t rec_cam_buf[4]={0,};
  cam_buf[0]=usb_receive_bufs[6]<<4|usb_receive_bufs[7];
  cam_buf[1]=usb_receive_bufs[8]<<4|usb_receive_bufs[9];
  cam_buf[2]=usb_receive_bufs[10]<<4|usb_receive_bufs[11];
  cam_buf[3]=usb_receive_bufs[12]<<4|usb_receive_bufs[13];
  etc_event_cmd=usb_receive_bufs[0]<<12|usb_receive_bufs[1]<<8|usb_receive_bufs[2]<<4|usb_receive_bufs[3];
  
  switch(etc_event_cmd)
  {
  case 0:
    if(state==stStby)
      send_sq_cfg(None,0);
    else if(state==stReady)
      send_sq_cfg(None,2);
    else
      send_sq_cfg(None,1);
    break;
  case eventStopSq:   
    state=stStby;
    timer_init(); 
    sq_strt_chk=false;
#ifdef PLUS //heating
  temp_ctrl(6,0); //heat off
  AirFan_Off;
#endif  
    break;
    // case (etc_event_cmd&0xff00)==0xb100:
    /*  if((etc_event_cmd&0xf000)==0xc000){
    app_data_wirte(etc_event_cmd&0x0fff,usb_receive_bufs);
    pc_send_pack(etc_event_cmd,usb_receive_bufs);
    event_cmd=0;
  }else if((etc_event_cmd&0xf000)==0xd000){
    app_data_read(etc_event_cmd&0x0fff,usb_receive_bufs);
    pc_send_pack(etc_event_cmd,usb_receive_bufs);
    event_cmd=0;
  }*/
    //  app_data_wirte(etc_event_cmd&0x0fff,usb_receive_bufs);
    break;
    
  default:
    if((etc_event_cmd&0xff00)==0xb100){
      //  memset(usb_receive_bufs,0,TRANSMIT_BUF_LENGTH);
      app_data_read(etc_event_cmd&0x0fff,rec_cam_buf);
      // app_data_wirte(etc_event_cmd&0x0fff,usb_receive_bufs);
      usb_send_pack((enum cntrl_event)etc_event_cmd,rec_cam_buf);
    }else if((etc_event_cmd&0xff00)==0xb200){
      //  memset(usb_receive_bufs,0,TRANSMIT_BUF_LENGTH);
      //  app_data_wirte(etc_event_cmd&0x0fff,usb_receive_bufs);
      app_data_read(etc_event_cmd&0x0fff,rec_cam_buf);
      usb_send_pack((enum cntrl_event)etc_event_cmd,rec_cam_buf);
      
    }else if((etc_event_cmd&0xff00)==0xb900)
      usb_send_pack((enum cntrl_event)etc_event_cmd,0);
    
    
    if((etc_event_cmd&0xff00)==0xA100){
      app_data_wirte(etc_event_cmd&0x0fff,cam_buf);
      usb_send_pack((enum cntrl_event)etc_event_cmd,cam_buf);
    }else if((etc_event_cmd&0xff00)==0xA200){
      app_data_wirte(etc_event_cmd&0x0fff,cam_buf);
      usb_send_pack((enum cntrl_event)etc_event_cmd,cam_buf);
    }else if((etc_event_cmd&0xff00)==0xA900)
      usb_send_pack((enum cntrl_event)etc_event_cmd,0);
    
    break;  
    
  }
  
}

volatile void send_mt_cfg(enum cntrl_event cmd, int32_t data)
{
  byte buf[4]={0,};
  buf[0]|=data>>24;
  buf[1]|=data>>16;
  buf[2]|=data>>8;
  buf[3]|=data;
  usb_send_pack(cmd,buf);
}

volatile void send_sq_cfg(enum cntrl_event cmd, int32_t data)
{
  byte buf[4]={0,};
  buf[0]|=data>>24;
  buf[1]|=data>>16;
  buf[2]|=data>>8;
  buf[3]|=data;
  usb_send_pack(cmd,buf);
}


volatile void send_pp_cfg(enum cntrl_event cmd, int32_t data)
{
  byte buf[4]={0,};
  buf[0]|=data>>24;
  buf[1]|=data>>16;
  buf[2]|=data>>8;
  buf[3]|=data;
  usb_send_pack(cmd,buf);
}



void send_qc_dsp()
{
  byte buf[4]={0,};
  uint16_t vol=dsp.dsp_vol;
  buf[1]=(uint8_t)dsp.dsp_num;
  buf[0]=(uint8_t)dsp.pum_num;
  memcpy(&buf[2],&vol ,sizeof(dsp.dsp_vol));
  vSwap_u8bit(&buf[2],&buf[3]);
  usb_send_pack(eventSopDsp,buf);
}

void send_qc_prime()
{
  byte buf[4]={0,};
  buf[0]=pr_ps.asp_pp_num;
  buf[3]=(byte)(pr_ps.asp_pp_time/1000);
  usb_send_pack(eventFunPrime,buf);
}

void send_qc_asp()
{
  byte buf[4]={0,};
  uint16_t tm=asp.asp_time;
  buf[0]=(uint8_t)asp.pum_num;
  buf[3]=(uint8_t)(tm/1000);
  buf[2]=asp.asp_num;;
  usb_send_pack(eventSopAsp,buf);
}

void send_qc_sk()
{
  byte buf[4]={0,};
  buf[0]=(uint8_t)shk.speed;
  buf[3]=(uint8_t)(shk.time);
  usb_send_pack(eventFunShake,buf);
}

void send_qc_etc(enum cntrl_event cmd, uint32_t time)
{
  byte buf[4]={0,};
  buf[3]=(uint8_t)(time/1000);
  usb_send_pack(cmd,buf);
}

void send_qc_rollbk()
{
  byte buf[4]={0,};
  buf[0]=(uint8_t)bk.pum_num;
  buf[3]=(uint8_t)(bk.time/1000);
  usb_send_pack(eventFunRollBack,buf);
}

void send_dry(enum cntrl_event cmd,uint8_t temp, uint8_t tm)
{
   byte buf[4]={0,};
   buf[2]=temp;
   buf[3]=tm;
   usb_send_pack(cmd,buf);
}


// lcd uart trans
volatile void send_lcd_sq_cfg(enum cntrl_event cmd, int32_t data)
{
  byte buf[4]={0,};
  buf[0]|=data>>24;
  buf[1]|=data>>16;
  buf[2]|=data>>8;
  buf[3]|=data;
 // lcd_send_pack(cmd,buf);
}

volatile void send_lcd_sq_data(enum cntrl_event cmd, int32_t data)
{
  byte buf[4]={0,};
  buf[0]|=data>>24;
  buf[1]|=data>>16;
  buf[2]|=data>>8;
  buf[3]|=data;
  //lcd_send_pack(cmd,buf);
   usb_send_pack(cmd,buf);
}


void lcd_send_rc_pack(enum cntrl_event cmd,enum cntrl_event rec_cmd, uint8_t *data)
{
byte send_cnt=0;
cmd_code=0;
sec_delay_set();
//lcd_send_pack(cmd, data);
DEV_CMD_RETRY:

 if(send_cnt<10){
  if(sec_delay(3)){
  //lcd_send_pack(cmd, data);
  
  sec_delay_set();
  send_cnt++;
  }
  execute_usb_rs232();
  if(rec_cmd!=cmd_code){
   goto DEV_CMD_RETRY;
  }
  
 }
 init_evnet();
 if(rec_cmd!=cmd_code)
    error(eventError,errRS232);

}
