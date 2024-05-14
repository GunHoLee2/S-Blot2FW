#ifndef CNTRL_H
#define CNTRL_H

enum state
  {
    stStby,
    stPrepare,
    stShutdown,
    stReady,
    stEmit,
    stErr,
    stErrSafe,
    stPowerUp,
    stPause,
    stPauseLcd,
    stPcEng,
    stBoot,
  };

enum kbl_ft_state
  {
    stNone,
    stEngDsp,
    stEngPrime,
  };


extern enum state state;
extern enum kbl_ft_state ft_state;
event execute_control(event);

extern byte stop_pre_st;
extern uint16_t ev_st;
extern bool sq_strt_chk;
extern uint16_t step_time_min;
extern uint16_t step_time_sec,step_time_min;
extern uint16_t ev_st; 

#define INCU_Roll_Back 280
#endif