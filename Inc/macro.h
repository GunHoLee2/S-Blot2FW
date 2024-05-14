#ifndef MACROS_H
#define MACROS_H

#define BEEP
#define FW_VER "0.0.02"

#define MEM_ADD 0xA0
#define MEM_PAGE 64
#define MEM_START_ADD 0

#define MT_MEM_START_ADD 0//100

//#define SQ_MEM_START_ADD0 MT_MEM_START_ADD+200
#define PUMP_MEM_START_ADD MT_MEM_START_ADD+1024//SQ_MEM_START_ADD0+128//100
#define PP_MEM_START_ADD   PUMP_MEM_START_ADD+512
#define SQ_MEM_START_ADD0   PP_MEM_START_ADD+512
//192//100
#define SQ_MEM_START_ADD1 SQ_MEM_START_ADD0+1280



#define APP_MEM_START_ADD  6400
#define APP_MEM_SIZE_ADD   APP_MEM_START_ADD+25600
#define TEM_MEM_START_ADD   APP_MEM_SIZE_ADD



#define ASP_NUM 1
#define ASP_NUM1 8

#define FAN 1
#define FAN_OFF 1
#define FAN_ON 0

#define HEATER 3

#define HEATER_OFF 0
#define HEATER_ON 1

#define STRIP_MAX_NUM 12
#define LEDON  1
#define LEDOFF 0

#define DBG

#define PLUS

typedef unsigned char byte;
typedef signed char sbyte;
typedef unsigned int uint;
typedef unsigned long ulong;

#define MONITOR(arg) { byte si=__save_interrupt(); __disable_interrupt(); arg; __restore_interrupt(si); }


#endif
