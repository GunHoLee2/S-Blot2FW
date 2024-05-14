#include "main.h"

#include <string.h>



struct hspState error_state, prev_state;

bool error(enum cntrl_event err, uint8_t err_st)
{
    if(state!=stErr)  // Not error state or err priority is higer 
    {
      //shutdown(); 
     //beep(0.5,1);
      error_state.error=err_st;
      //print("err %d\r\n",err);
      state=stErr;
    //  send_lcd_sq_data(err, error_state.error);
      beep(1,700,3);
       dwHsSW_Delay_ms(2000);
      //usb_send_pack((enum)eventError,&error_state.error);
       usb_send_pack(eventError, &err_st);
      //head.send(hseState,sizeof(error_state),&error_state);
     
    }
    return 1;
}

uint16_t get_error()
{
 return error_state.error; 
}

void clear_error()

{
 error_state.error=0; 
}

void send_error(uint16_t err)
{
 error_state.error=err;
 //state=stErr;
// head.send(hseState,sizeof(error_state),&error_state);    
}