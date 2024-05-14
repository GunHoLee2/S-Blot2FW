#include "main.h"
#include "string.h"

#define PUMP_NUM 9
#define OP_MODE_NONE 0

uint32_t roll_bk_tm=0; 
/*
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

*/

struct prime pr_ps= 
// Prime #1
   {
    {eventPrimeBathPsn,eventPrimeDspOut,eventPrimeDspEnd,eventPrimeAspOut,eventPrimeAspEnd,eventPrimeAspHome, 
    },
    500, //{7,11,15},  // bath_psn
    500,  // speed
    1,      //dsp pump number    
    3000, //dsp time
    400, //dsp pin length
    1,  // asp pump number
    3000,  // asp time
    400,  // asp length       
    1,   // roll back pump number       
    1000 //roll back time 
  };//,
/*
 struct dispense 
 {
  enum cntrl_event event[PS_MAX_EVENT];
  byte dsp_num;
  uint32_t pum_num;
  uint32_t dsp_vol;
 } ;*/
 struct dispense dsp=
  {
    {eventDspSkAngle,eventDspSkEnd,eventDspStripMove,eventDspOper,eventDspEnd,evnetDspCon 
    },
    10,
    0,
    1000
  };

/*
 struct aspiraiotn asp 
 {
  enum cntrl_event event[PS_MAX_EVENT];
  byte asp_num;
  uint32_t pum_num;
  uint16_t asp_time;
 } ; 
 */ 
  struct aspiraion asp=
  {
    {eventAspSkAngle,eventAspSkEnd,eventAspStripMove,eventAspOut,eventAspOper,eventAspEnd,evnetAspCon 
    },
    10,
    0,
    5,
    1000
  };
 /*
  struct shake  
 {
  enum cntrl_event event[PS_MAX_EVENT];
  int32_t speed;
  uint16_t time;
 } ;
 */
 
  struct shake shk=
  {
    {eventSkOper,eventSkEnd 
    },
    10,
    1000
  };
 /*
  struct roll_bk  
 {
  enum cntrl_event event[PS_MAX_EVENT];
  byte pum_num;
  uint32_t time;
 } ;
 */
  struct roll_bk bk=
  {
    {eventRbOper,eventRbEnd
    },
    0,
    1000,
  };
  /*
 struct etc_fc    
 {
  enum cntrl_event event[PS_MAX_EVENT];
  uint32_t dsp_tm;
  uint32_t asp_tm;
 } ;
*/
 struct etc_fc fc =    
 {
  {eventBathDspOper,eventBathDspEnd,eventBathAspOper,eventBathAspEnd
  },
  1000,
  1000
 };
 /*
  struct dryer  
 {
  enum cntrl_event event[PS_MAX_EVENT];
  uint8_t temp;
  uint8_t time;
 } ;
 */
  struct dryer drys=
  {
    {eventdryOper,eventdryMove,eventdryEnd 
    },
    10,
    10
  };

  // Clean
 struct clean clean=
 {
    {eventCleanBathPsn,eventCleanDspOut,eventCleanDspEnd,eventCleanAspOut,eventCleanAspEnd,eventCleanAspHome, 
    },
    3, //{7,11,15},  // bath_psn
    4,  // speed
    7,      //dsp pump number    
 };//,
 
void pump_all_call(byte dir, uint time)
{
  byte k;
    for(k=0; k<PUMP_NUM; k++)
    Pump_Single_Run(k,dir,time);
}

void dsp_pump_all_call(byte dir, uint time)
{
  byte k;
    for(k=0; k<PUMP_NUM; k++)
    Pump_Single_Run(k,dir,time);
}


void pump_all_stop()
{
   byte k;
    for(k=1; k<PUMP_NUM; k++)
    Pump_Single_Break(k);
}

void dsp_pump_all_stop()
{
   byte k;
    for(k=1; k<PUMP_NUM; k++)
    Pump_Single_Break(k);
}



