#include "main.h"
#include "string.h"


#define TIEMR0_TIK 500
#define TIMER1_TIK 100
#define TIMER0_1S_TIK 1
ulong timer=TIEMR0_TIK;//timer_freq;
ulong timer1=TIMER1_TIK;
byte ad_t_cnt = TIMER0_1S_TIK;

uint16_t timer_ticks=0;
static const byte channels=5;
struct tmchan
      {
        uint16_t event;
        uint delay;
        void(*event_func)(uint16_t evnt);
      };//channel[2];//[channels];
struct tmchan channel[5];

void tmchan_init()
{
  memset(channel, 0, sizeof(channel));
}


void timer_mk_int()
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
		} 
     }
    
/*100ms timer irpt*/
    if(timer1) 
      --timer1;
    else
      {
        give_event(eventTimer100ms,0);      
        timer1=TIMER1_TIK; 
      }
    
timer_ticks++;	
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
    tm_add(evnt, time, event_func);
  }


