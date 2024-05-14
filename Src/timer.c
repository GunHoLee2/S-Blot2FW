#include "main.h"
#include "string.h"


#define TIEMR0_TIK 500
#define TIMER1_TIK 100
#define TIMER0_1S_TIK 1
ulong timer=TIEMR0_TIK;//timer_freq;
ulong timer1=TIMER1_TIK;
byte ad_t_cnt = TIMER0_1S_TIK;
int sec_delay_cnt =0;

ulong beep_cnt=0;
ulong beepoff_cnt=0;
int get_tick_1s=0;

uint16_t timer_ticks=0;
static const byte channels=5;
struct tmchan
      {
        uint16_t event;
        uint delay;
        void(*event_func)(uint16_t evnt);
      };//channel[2];//[channels];
struct tmchan channel[10];

void tmchan_init()
{
  memset(channel, 0, sizeof(channel));
}

void  timer_tick_init()
{
  timer_ticks=0;

}

void timer_init()
{
  timer=TIEMR0_TIK;
}

void timer_mk_int()
{

/*100ms timer irpt*/
    if(timer1) 
      --timer1;
    else
      {
        //give_event(eventTimer100ms,0);      
        timer1=TIMER1_TIK; 
      }
    


bath_full_cnt++;
  

}


void timer_eve()
{
   if(timer){ 
      --timer;
  }else
      {
        give_event(eventTimer500ms,0);
        //timer=2000;//timer_freq; 
        timer=TIEMR0_TIK;//timer_freq;

		if(ad_t_cnt)
			--ad_t_cnt;
		else
                {
			give_event(eventTimer1s,0);
			ad_t_cnt = TIMER0_1S_TIK;
                        get_tick_1s++;
                        sec_delay_cnt++;
                        
                        step_time_sec++;
                        if(step_time_sec>60){
                          step_time_min++;
                          step_time_sec=0;
                        }
		} 
     }
timer_ticks++;	
}

int get_timer_1s()
{
  return get_tick_1s;
}

void set_timer_1s()
{
  get_tick_1s=0;
}

void tm_execute()
  {
  byte i=0;
    if(timer_ticks)
      {
	  
        for(i=0; i<channels; i++)
          if(channel[i].event)
            {
              if(channel[i].delay<=timer_ticks) // the time
                {
                  if(channel[i].event_func)  // if have func
                    (*channel[i].event_func)(channel[i].event);
                  else  // if not have func
                    give_event(channel[i].event,0);
                   channel[i].event=0;
                }
              else
                channel[i].delay-=timer_ticks;
            }
		
        timer_ticks=0;
      }
  }


void tm_add(uint16_t evnt, uint time, void(*event_func)(uint16_t evnt))
  {
  	uint16_t i=0;
    for(i=0; i<channels; i++)
      if(!channel[i].event)
        {
          channel[i].delay=time;
          channel[i].event_func=event_func=0;
          channel[i].event=evnt;
          return;
        }
    //error(errTimerFull);
  }


void execute_timer()
  {
    tm_execute();
  }

void set_timer_(uint16_t evnt, uint time, void(*event_func)(uint16_t evnt))
  {
    timer_ticks=0;
    if(!(state==stPauseLcd))
    tm_add(evnt, time, event_func);
    else{
        if((evnt==eventPrimeDspEnd)||(evnt==eventPrimeAspEnd)||(evnt==eventDspEnd)||(evnt==eventAspEnd))
         tm_add(evnt, time, event_func);
        else{
         ev_st=evnt;
         init_evnet();
         //tmchan_init();
        }
    }
  }

#define ARAM 10
uint32_t beep_pin =0;
uint32_t aram_cnt =0;
uint32_t beepon_cnt=0;
byte reapet=0;
byte fts =0;


void beep(byte ft, ulong duraiton, byte repeat)
{
  fts=ft;
  if(fts)
    aram_cnt=ARAM;
  else{
  beepoff_cnt=duraiton;
  if(repeat>0)
  reapet=repeat-1;
  }
  beepon_cnt=beep_cnt=duraiton;
}

void 
beep_cnt_polling()
{
  if(beep_cnt){
    beep_cnt--;
    if(fts==1){
      if(!aram_cnt--){
#ifdef BEEP
        beep_pin=~beep_pin;
        BSP_ETC_OnOff_Device(0,beep_pin);
#endif
      aram_cnt=ARAM;
        }
    }else{
#ifdef BEEP
      BSP_ETC_OnOff_Device(0,1);
#endif       
    }
  }else{
     if(beepoff_cnt){
     beepoff_cnt--;
#ifdef BEEP
     BSP_ETC_OnOff_Device(0,0);
#endif 
     }else{
       if(reapet&&!beepoff_cnt&&!beep_cnt){
        beepoff_cnt=beep_cnt=beepon_cnt;
        reapet--;
      }  
      BSP_ETC_OnOff_Device(0,0);
     }
  }
  
}


void sec_delay_set()
{
  sec_delay_cnt=0;
}

int sec_delay(int num)
{
  if(sec_delay_cnt>num)
  return 1;
  else 
  return 0;
}

