#ifndef EVENT_H
#define EVENT_H

typedef uint16_t event;
uint16_t get_event();
void give_event(uint16_t event, byte nomore);
unsigned char del_event(uint16_t event);
void init_evnet();



#define EVENT_NUM_MAX	(8)

typedef enum EVENT_TYPE
{
	EVENT_NONE  = 0,
	EVENT_ENTRY,
	EVENT_EXIT,
	EVENT_DONE,
	EVENT_CUSTOM_BASE
} EVENT_TYPE;

//!-------------------------------------------------------------------
//! EVENT
//!-------------------------------------------------------------------
struct EVENT;
typedef void(*event_handler)(struct EVENT e);
typedef byte EVENT_PARAM;
typedef struct EVENT
{
	event_handler handler;
	EVENT_PARAM param;
} EVENT;

typedef struct  event_list_item_type
{	
	EVENT e;
    struct event_list_item_type* next;
    struct event_list_item_type* prev;
} event_list_item_type;

//!-------------------------------------------------------------------                 
//! EVENT Timer
//!-------------------------------------------------------------------

#define EVENT_TIMER_LIST_ITEM_MAX			(EVENT_NUM_MAX - 1) 	// up to 32
#define EVENT_TIMER_STATE_STOPPED			(0)
#define EVENT_TIMER_STATE_RUNNING			(1)

typedef void(*event_timer_list_item_pf_type) (EVENT e);
typedef struct  event_timer_list_item_type
{	                
    byte state;
    uint ms;
    EVENT e;
    struct event_timer_list_item_type* next;
    struct event_timer_list_item_type* prev;
} event_timer_list_item_type;

//!-------------------------------------------------------------------
//! Fuction Definitions
//!-------------------------------------------------------------------


extern volatile uint RFID_SYSTEM_TICK;
extern bool event_idle_sleep_enable;
extern void SysTick_Handler_oper(void);

void event_init(void);
void event_send(EVENT e);
void event_post(EVENT e);
void event_post_delayed(EVENT e, uint32_t ms);
void event_dispatch(void);
void event_flush(event_handler h);
void event_timer_flush(event_handler h);


enum 
cntrl_event
{
  //status read
         // eventEquiType=0,      //Erp 기준 장비 분류
          None,
          eventEquiSN=1,          //장비 시리얼 넘버
          eventFwVer,           //fw 버전
          eventReserve0, 
          eventHwVer,           //hw 버전       
          eventSetpId,   
          eventTotalSqNum,         //장비에 저장 되어 있는 시퀀스 개수
          eventStSqId,            //시퀀스의 ID 확인
          eventSeqSelR,         //현재 로딩 되어있는 시퀀스 메모리 블록 Number
          eventTotalSN,         //전체 스텝 개수
          
          eventTotalTime,       //각 스텝의 소요 시간
          eventCrrtTotalTime,   // 진행 중인 step과 step 진행 시간
          eventCrrtProTime,     // 현재 프로세스 진행 시간
          eventStepTime,        //step의 시간을 확인
          evnetCtAndPt,         //현재 실행 중인 step과 proces를 알려준다.
          ///evnetSeqSel=0x10, 
         //현재 실행 중인 step과 proces를 알려준다.
    
 //Control Write
          eventSeqSel=0x0010,   // 선택 시퀀스 Loading
          eventCntinRun,        //선택한 스텝부터 시퀀스 끝까지 실행
          eventStepSigRun,      //선택한 스텝만 실행
          eventStPsSigRun,      //선택한 스탭의 process만 실행
          eventSeqStop,         //진행 중인 Process 를 완료 하고 Stand-by 상태가 됨
          eventPuse,            //일시정지
          eventDevReset,        //장바 sw 리셋
          eventCamMov,
          eventAutoClean,       //AutoClean
          eventDisifection,     //Disinfection
          eventReserve2,
          eventReserve3,
          eventSplTotalNum,     //검사할 샘플 중 개수
          eventWriteSN,         //장비 시리얼 번호 저장
          eventQrMove=0x002E,
//       
          eventLedWrite=0x0105,
          eventLedRead,
          eventLedOnOff,
          
          
//Motor Setting Mode
          eventAspHome=0x0301,  //Asp Pin Home
          eventAspJogRun,       //Asp Jog Run
          eventBathAspHiSet,    //Bath Asp pin 높이 Set
          eventReadBathAspHiSet,//Bath Asp pin 높이 확인
          eventTrayAspHiSet,    
          eventReadTrayAspHiSet,
          eventReserve4,
          eventReserve5,
          eventReserve6,
          eventShakeMaxPos, 
          
          eventReadShakeMaxPos,
          eventShakeHomePos,
          eventReadShakeHomePos,
          eventShakeHome,
          eventShakeJogRun,//0f
          eventShakeSpdSet,     //shake speed
          eventReadShakeSpd,
          eventShakeAspPos,
          eventReadShakeAsp,
          eventShakeDspPos,
          
          eventReadShakeDsp,
          eventShakeAnalyPos,
          eventReadShakeAnaly,
          eventShakeDryPos,
          eventReadShakeDry,
          eventShakeSplDspPos,
          eventReadShakeSplDsp,
          eventShakeSpdPid,     //shake 스피트 간격
          eventReserve9,
          eventReserve10,
          
          
          //1f
          eventReadXaxiSpd,
          eventXaxiAcclSet,        // X 가감속 설정
          eventReadXaxiAccl,
          eventXaxiHome=0x0323,
          eventXaxiMove,
          eventStripWidth,
          eventReadStripWidth,
          eventReserve11,
          eventTrayDispXStrt,
          
          eventReadTrayDispXStrt,
          eventReserve12,
          eventReserve13,
          eventTrayAspXStrt,
          eventReadTrayAspXStrt,
          eventReserve14, 
          eventReserve15,//2f
          eventReserve16,
          eventReserve17,
          eventReserve18,
          
          eventReserve19,
          eventBathXPos,
          eventReadBathXPos,//5
          eventQrMvWrite=0x0338,
          eventQrMvRead, // 
          eventCamPosWirte,
          eventCamPosRead,
          eventCamWidthWrite,
          evnetCamWidthRead,
          
          eventXaxiBathSave= 0x034D,
          eventXaxiAspSave,
          eventXaxiDspSave,
          eventShakeAspSave,
          eventShakeDspSave,
          eventShakeDrySave,
          eventShakeAnalysisSave,
          eventShakeSmpDspSave,
          eventShakeHomeSave,
          eventAspBathSave,
          eventAspTraySave,
          eventMutiConSave,
 
          
//PUMP Setting Mode
          eventPumpOper=0x0401,
          eventReserve20,
          eventReserve21,
          eventPumpAutoCalSet,
          eventReserve22,
          eventPumpUseTimeSet,
          eventReadPumpUseTime,
          //
          eventPumpOffSet,
          eventReserve23,
          eventReserve24,
          
          eventReserve25,
          eventReserve26,
          eventPumpRollBack,
          eventReserve27,
          eventReserve28,//0f
          eventReadBathVol,
          eventBathVolSet,
          eventReadAll=0x0418,

//Sq Generator mode
          eventSqInMode=0x0801,
          eventSqTotalStep,
          eventSqSave,
          eventSqId,
          eventReserve29,
          eventReserve30,
          eventReserve31,
          eventReserve32,
          eventReserve33,
          eventSqStepNum,
          eventSqPrNum,       
          eventSqPrParamByte,
          eventSqPrParamWord,
          eventSqPrParamDword,
          
//QC Mode
          eventSopDsp=0x0901,
          eventSopAsp,
          eventReserve34,
          eventReserve35,
          eventReserve36,
          eventSopTImeSet,
          eventFunPrime,
          eventPumpOffset,
          eventFunShake,     
          eventFunRollBack,
          eventReserve38,
          eventFunDry,
          
          eventRunStop,
          eventBathDsp,
          eventBathAsp,
          
          
          
          eventEquiType,
          
#ifdef PLUS
          eventTrayHeatingPADch1Write=0x0913,
          eventTrayHeatingPADch1Read,
          eventTrayHeatingPADch2Write,
          eventTrayHeatingPADch2Read,
          eventHeatPadOnOff,
          eventHeatPadTempRead,
          eventHeatPadTempSet,
          eventIrData,
          eventTempSensPosSet,
          eventTempSensPosRead,
          eventShakerStop,
          eventTrayTempRead,
          eventAirTempRead,
    
#endif     
          
          
          
          
         
          
          eventTest=0x013A,
          motorStop1,
          motorStop2,
          
        //  eventAlyGo=0A0A,
          eventAlyGo=0x0A0A,
          
          eventAppStartRe=0x2000, 
          eventAppStart,
          reserve0,
          reserve1,
          
          reserve2,
          reserve3,
          reserve4,
          eventStartSq,
          eventStopSq,
          evnetPrEnd,
          
          eventSqOperCnt,
          eventhesPause,
          eventhspPause,
          eventhspPauseLcd,
          eventhesPauseReStart,
          eventStartBtnRes,
          evnetPrEndRes,
          eventStartSqPr,
          eventEngAutoClean,
          eventEngAcPause,
          eventEngAcStop,
          
          hseSqId=0x3000,
          hseSqAllStep,
          hseSqStepNum,
          hseSqAllPr,
          hseSqPrNum,
          hseSqData,
          hseEngHeatOn,
          hseEngHeatOff,
          hseInfFwVer,
          
         
         
          hspSqId=0x4000,
          hspSqAllStep,
          hspSqStepNum,
          hspSqAllPr,
          hspSqPrNum,
          hspSqData,
          hspReserve0,
          hspStart,
          hspStop,
          hspInfFwVer,
          
          
          eventError=0x8000,
          
          
          
          
          
          
          //prime
          eventPrimeBathPsn=0xA000,
          eventPrimeDspOut,
          eventPrimeDspEnd,
          eventPrimeAspOut,
          eventPrimeAspEnd,
          eventPrimeAspHome,
          //dispense
          eventDspSkAngle,
          eventDspSkEnd,
          eventDspStripMove,
          eventDspOper,
          eventDspEnd,
          evnetDspCon,
          //aspiraion
          eventAspSkAngle,
          eventAspSkEnd,
          eventAspStripMove,
          eventAspOut,
          eventAspOper,
          eventAspEnd,
          evnetAspCon,
          //incubation
          eventSkOper,
          eventSkEnd,
          //incubation C
          eventSkOperC,
          eventSkEndC,
           //RollBack
          eventRbOper,
          eventRbEnd,
          //Clean RollBack
          eventCleanRbOper,
          eventCleanRbEnd,
          //dry
          eventdryOper,
          eventdryMove,
          eventdryEnd,
          //prime
          eventCleanBathPsn,
          eventCleanDspOut,
          eventCleanDspEnd,
          eventCleanAspOut,
          eventCleanAspEnd,
          eventCleanAspHome,
          
          
          //etc function
          eventBathDspOper,
          eventBathDspEnd,
          eventBathAspOper,
          eventBathAspEnd,
          
          eventSpuOn,
          eventTimer500ms,
          eventTimer100ms,
          eventTimer1s,
          eventAllstHome,
          eventAllPumpCall,
          eventAllPumpStop,
          eventSystickTimer,
          eventBathFull1,
          eventBathEmp1,
          eventBathFull2,
          eventBathEmp2,
          eventPumpOffSetOper,
          eventPumpOffSetEnd,
          eventPrimeCntinOper,
          eventPrimeCntinEnd,
          eventBathVolCntnOper,
          eventPumpAutoCalSart,
          eventPumpAutoCalPumpSel,
          evnetPumpAutoCalPumpOperEnd,
          eventAspTime,
          eventAspTimeEnd,
          eventSysState,
          eventOperFanEnd,

          
          
          eventAspTimeOut,
          eventMutiConEnd=0xB800,
	 _eventEnd
};






#endif