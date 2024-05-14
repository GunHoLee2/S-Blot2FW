
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
    errBathFull,                // bath�� ���� ����
    errTMTimeOut,
    errRS232,             // ��� �ҷ�
    errBoot,              //����
   _errEnd
  };



struct hspState
{
  uint16_t error;
}ATTR_PACKED;


#endif