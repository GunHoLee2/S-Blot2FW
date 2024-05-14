
#ifndef HEAD_SPU_H
#define HEAD_SPU_H

#define ATTR_PACKED

enum hsEvents
  {
     // for bootloader for self-programming
    hseNone=0,    // 
    hseBoot=1,    // 
    hseBootAns,   // 
    hseReset,     //  

  };



enum ERROR
  {
    errOK,                 // 0   no alarm
    errBathFull,                // bath에 물이 있음
    errTMTimeOut,
    errRS232,             // 통신 불량
    errBoot,              //시퀀
   _errEnd
  };



struct hspState
{
  uint16_t error;
}ATTR_PACKED;


#endif