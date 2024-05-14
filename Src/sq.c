#include "main.h"
#include "string.h"
enum step st=St_None;
byte step_all_num =0;
byte step_num=0;
byte step_cnt=0;
byte id=0;
extern byte inhalant=0,food=0, etc=0, totoal_strip=0;
struct sq sq[12];
/*
          eventSqInMode=0x0801,
          eventSqTotalStep,
          eventSqSave,
          eventSqId,
          eventReserve29,
          eventReserve30,
          eventReserve31,
          eventReserve32,
          eventSqStepNum,0809
          eventSqPrNum,  080A     
          eventSqPrParamByte,080B
          eventSqPrParamWord,080C
          eventSqPrParamDword,080D
*/


byte lcd_sq_id=0;
byte lcd_step=0;
byte dbug=0;
byte prev_pos=0;
byte *ver_pnt=0;

event sq_control(event event)
{
int step_time=0;
uint16_t pr_num=0;
byte lcd_all_step=0;
byte usb_send_buf[4]={0,};
byte k=0;
    switch(event)
    {
    case eventSqInMode:
      if(usb_data_buf[3]==1){
       memset(&sq,0,sizeof(sq));
       sq_pram_write(0); 
      }   
       send_sq_cfg(eventSqInMode,usb_data_buf[3]);     
    break;
    case eventSqTotalStep:
       step_all_num=usb_data_buf[3];  
       send_sq_cfg(eventSqTotalStep,step_all_num);
    break;
    case eventSqSave:
      sq_pram_write(id);  
      send_sq_cfg(eventSqSave,usb_data_buf[3]);
    break;
    case eventSqId:
      id=usb_data_buf[3];
      send_sq_cfg(eventSqId,usb_data_buf[3]);
    break;
    case eventSqStepNum:
      step_num+=1;
      sq[step_num-1].stNum=usb_data_buf[3];
      send_sq_cfg(eventSqStepNum,sq[step_num-1].stNum);

      
        if(usb_data_buf[3]==0xfe){
        step_all_num=0;
        step_num=0;
        pr_num=0;
        step_cnt=0;
        }
      
    break; 
    case eventSqPrNum:
      pr_num=usb_data_buf[0]<<8;
      pr_num|=usb_data_buf[1];
      sq[step_num-1].prCnt=step_cnt=usb_data_buf[3];
      sq[step_num-1].prCnt+=1;
      sq[step_num-1].prNum[step_cnt]=pr_num;
      send_sq_cfg(eventSqPrNum,sq_val_ch(usb_data_buf,4));
    break;

    case eventSqPrParamDword:
      sq[step_num-1].dword[step_cnt]=sq_val_ch(usb_data_buf,4);
      send_sq_cfg(eventSqPrParamDword,sq[step_num-1].dword[step_cnt]);
    break;
    

    case eventSeqSel:
      usb_data_buf[2]=1;
      usb_send_pack(eventSeqSel,usb_data_buf);
      break;
   
    case eventTotalSN:
      
      for (int st=0; st<12; st++)
      {
        if(sq[st].stNum==0xfe){
          usb_send_buf[3]=st;
          usb_send_pack(eventTotalSN,usb_send_buf);
          break;     
        }
      }
      if(!usb_send_buf[3])
      usb_send_pack(eventTotalSN,usb_send_buf);
   /*   if( sq[lcd_all_step].stNum!=0xfe){
        lcd_all_step++;
        if(lcd_all_step>12){
          usb_send_buf[3]=0;
          //full_step_cnt=0;
          usb_send_pack(eventTotalSN,usb_send_buf);
        }else
          give_event(eventTotalSN,0);
      }else{
        
      
        usb_send_buf[3]=lcd_all_step;
        // full_step_cnt=0;
        usb_send_pack(eventTotalSN,usb_send_buf);
      }
     //  usb_send_buf[3]=11;
     // usb_send_pack(eventTotalSN,usb_send_buf);/*/
      break;
    case eventStepTime:
      for( k=0; k<12; k++){
        if(sq[usb_data_buf[3]-1].prNum[k]==0x05){
          step_time+=sq[usb_data_buf[3]-1].dword[k];
          step_time+=((55+(6*totoal_strip))/60);
          sq[usb_data_buf[3]-1].dword[19]=step_time;
        }else if(sq[usb_data_buf[3]-1].prNum[k]==(enum process)Dry){
          step_time+=sq[usb_data_buf[3]-1].dword[k]&0x000000FF;
          sq[usb_data_buf[3]-1].dword[19]=step_time;     
        }
        
      }
      usb_send_buf[1]=usb_data_buf[3];
      usb_send_buf[2]=step_time>>8;
      usb_send_buf[3]=step_time;
      usb_send_pack(eventStepTime,usb_send_buf);
      break;

    case eventHwVer:
      usb_data_buf[3]=0; 
      usb_send_pack(eventHwVer,usb_data_buf);
      break;
    case eventSetpId:
      usb_send_buf[3]=sq[usb_data_buf[3]-1].stNum;
      usb_send_pack(eventSetpId,usb_send_buf);
      break;
    case eventFwVer:
      ver_pnt=FW_VER;
      usb_send_buf[0]=*ver_pnt;
      usb_send_buf[1]=*(ver_pnt+2);
      usb_send_buf[2]=*(ver_pnt+4);
      usb_send_buf[3]=*(ver_pnt+5);
      usb_send_pack(eventFwVer,usb_send_buf);
      break;
//lcd init rx
    case hseSqId:
       lcd_sq_id=0;//usb_data_buf[3];
       sq_pram_read(lcd_sq_id);
       set_timer_(hspSqId,10,0);  
       // BSP_UART_PutIT_Byte(3, "a",2);
     break;
    case hseSqAllStep:
      set_timer_(hspSqAllStep,10,0);
       //BSP_UART_PutIT_Byte(3, "b",2);
     break;
    case hseSqStepNum:
      set_timer_(hspSqStepNum,10,0);
      //BSP_UART_PutIT_Byte(3, "c",2);
     break;
    case hseSqAllPr:
      set_timer_(hspSqAllPr,10,0);
       // BSP_UART_PutIT_Byte(3, "d",2);
     break;
    case hseSqPrNum:
       set_timer_(hspSqPrNum,10,0);
       // BSP_UART_PutIT_Byte(3, "e",2);
     break;
    case hseSqData:
      set_timer_(hspSqData,10,0);
       // BSP_UART_PutIT_Byte(3, "f",2);
     break;
// lcd init tx
    case hspSqId:
     send_lcd_sq_cfg(hspSqId,lcd_sq_id); 
     break;
    case hspSqAllStep:
      for (int st=0; st<12; st++)
      {
        if(sq[st].stNum==0xfe)
          break;
          lcd_all_step=st;
      }
      send_lcd_sq_cfg(hspSqAllStep,lcd_all_step+1); 
     break;
    case hspSqStepNum:
      lcd_step=usb_data_buf[0];
      send_lcd_sq_cfg(hspSqStepNum, sq[lcd_step].stNum); 
     break;
    case hspSqAllPr:
       send_lcd_sq_cfg(hspSqAllPr, sq[lcd_step].prCnt); 
     break;
    case hspSqPrNum:
       send_lcd_sq_cfg(hspSqPrNum, sq[lcd_step].prNum[usb_data_buf[0]]); 
     break;
    case hspSqData:
       send_lcd_sq_cfg(hspSqData, sq[lcd_step].dword[usb_data_buf[0]]); 
       break;
    case hspStart:
      break;
    case hspStop:
      break;
   // case eventQrBar:
     //  usb_send_pack(eventQrBar,0);
      //break;
    case eventCamMov:
    //  mt_init_pos();
     // if(prev_pos!=usb_data_buf[3]){
    //  Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.cam_strip_pos+mt_ctrl.strip_width*(usb_data_buf[3]-1));
     dSPIN_Go_To( mt_ctrl.cam_strip_pos+mt_ctrl.strip_width*(usb_data_buf[3]-1),X_AIS_MT);
    //  dSPIN_Move(FWD,mt_ctrl.cam_strip_pos+mt_ctrl.strip_width*(usb_data_buf[3]-1),X_AIS_MT);
     // prev_pos=usb_data_buf[3];
     // }
  //    dSPIN_Go_To()
      usb_send_pack(eventCamMov,usb_data_buf);
      break;
      
      
    case eventQrMove:
      //Stmt_AbsMove(X_AIS_MT,X_AIS_SPD, mt_ctrl.qr_pos);
      dSPIN_Go_To( mt_ctrl.qr_pos,X_AIS_MT);
      usb_send_pack(eventQrMove,0);
      break;
    case eventQrMvWrite:
      mt_ctrl.qr_pos=motor_val_ch(usb_data_buf,4);
      send_mt_cfg(eventQrMvWrite, mt_ctrl.qr_pos);
      motor_cfg_wirte();
      mt_init_pos();
      dSPIN_Go_To( mt_ctrl.qr_pos,X_AIS_MT);
   //   Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,mt_ctrl.qr_pos);
       // usb_send_pack(eventQrMvWrite,0);
      break;
    case eventQrMvRead:
      motor_cfg_read();
      send_mt_cfg(eventQrMvRead, mt_ctrl.qr_pos);
      break;
    case eventSplTotalNum:
      inhalant=usb_data_buf[0];
      food=usb_data_buf[1];
      etc=usb_data_buf[2];
      totoal_strip=usb_data_buf[3];
      Sk_Home();
      usb_send_pack(eventSplTotalNum, usb_data_buf);
      break;
    case eventAutoClean:
       give_event(eventEngAutoClean,0);
       usb_send_pack(eventAutoClean, usb_data_buf);
      break;
      
      case eventMutiConEnd:
        usb_send_pack(eventMutiConEnd,0);
        break;  
        
    
    
     default:break;
    }
return event;
}

int32_t sq_val_ch(byte *data, byte size)
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

void sq_init()
{
  if(sq[0].stNum==0xff || sq[0].stNum==0){
    memset(&sq,0,sizeof(sq));
    sq[0].stNum=1;
    sq[0].prCnt=1;
    sq[0].prNum[0]=1;
    sq[0].dword[0]=0x03001F40;
    
    sq[1].stNum=0xfe;
    sq_pram_write(0);
    
    sq_pram_read(0); 
  }
  
}
/*
struct sq sq[12]= 
{
  {
    1, 5,
    {1, 3, 11, 5, 4},
    {0x03001F40, 0x03000190, 0x030003E8, 0x003C0001, 0x000205DC}    
  },
  
  {
    2, 8, 
    {1, 3, 11, 2, 6, 8, 5, 4},
    {0x04001F40, 0x040000FA, 0x040003E8, 0x003C0001, 0x00000005,
    0x00000000, 0x003C0001, 0x000205DC }    
  },
  
  {
    10,1, 
    {7},
    {1}    
  },
  
  {
    0xfe, 
  }
};
*/