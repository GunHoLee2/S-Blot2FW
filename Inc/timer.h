#ifndef USB_UART_H
#define USB_UART_H

void timer_mk_int();
void tm_execute();
void tmchan_init();
void timer_eve();
//void tm_execute();
//void tm_add(byte evnt, uint time, void(*event_func)(byte evnt));
void set_timer_(uint16_t evnt, uint time, void(*event_func)(uint16_t evnt));

#define set_timer(event, time, func) set_timer_(event, (time)*2,func);//*timer_freq/1000., func)  // ms
#define set_etimer(event, time) set_timer_(event, (time)*2);//*timer_freq/1000.)


void beep_cnt_polling();
//void beep(byte ft, ulong duraiton);
void beep(byte ft, ulong duraiton, byte repeat);
void timer_tick_init();
void timer_init();
int get_timer_1s();
void set_timer_1s();
void sec_delay_set();
int sec_delay(int num);
#endif 
