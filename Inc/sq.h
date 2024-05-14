#ifndef SQ_H
#define SQ_H

enum step
{
  St_None,
  Wetting ,
  Specimen_dsp,
  Washing1,
  Antibody,
  Washing2,
  Enzyme,
  WashingDW,
  Substrate_Sol,
  dry,
  analy
}; 

enum process
{
  Pr_None,
  Prime,
  RollBack,
  Dispense,
  Aspiration,
  Incubation,
  Buzzer,
  Dry,
  Pause,
  Smp_Dispense,
  Auto_Clean,
  Prime_D,
  Clean_RollBack,
  Dispense_D,
};


#pragma pack(push,1)
 __packed  struct sq
 {
  uint8_t  stNum; //step number
  uint8_t  prCnt; //step number
  uint8_t  prNum[20]; //process number
  uint32_t dword[20];
 };
#pragma pack(pop)

extern struct sq sq[12];

event sq_control(event event);
int32_t sq_val_ch(byte *data, byte size);
void sq_init();
extern byte inhalant,food, etc, totoal_strip;
#endif