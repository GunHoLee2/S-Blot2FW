#include "main.h"
#include "string.h"

hsAT24xx_ConfigType mtCfg;
hsAT24xx_ConfigType ppCfg;
hsAT24xx_ConfigType pp2Cfg;
hsAT24xx_ConfigType appCfg;
hsAT24xx_ConfigType sqCfg[10];
hsAT24xx_ConfigType temCfg;

void eeprom_init()
{
     hsAT24xx_Config(&mtCfg,MEM_PAGE,MEM_ADD, MT_MEM_START_ADD,4,1024);
     hsAT24xx_Config(&ppCfg,MEM_PAGE,MEM_ADD, PUMP_MEM_START_ADD,4,512);
     hsAT24xx_Config(&pp2Cfg,MEM_PAGE,MEM_ADD, PP_MEM_START_ADD,4,512);
     //sAT24xx_Config(hsAT24xx_ConfigType *p_tMemConfig, uint16_t wPageSize, uint16_t wDev_Address, uint16_t wMemStart_Adrress, uint16_t wMemAddress_Size, uint16_t wMemMaxSize)
     hsAT24xx_Config(&sqCfg[0],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD0,4,SQ_MEM_START_ADD0+1280);

     hsAT24xx_Config(&sqCfg[1],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD1,4,SQ_MEM_START_ADD1+1280);
     /*hsAT24xx_Config(&sqCfg[2],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD2,4,SQ_MEM_START_ADD2+1280);
     hsAT24xx_Config(&sqCfg[3],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD3,4,SQ_MEM_START_ADD3+1280);
     hsAT24xx_Config(&sqCfg[4],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD4,4,SQ_MEM_START_ADD4+1280);
     hsAT24xx_Config(&sqCfg[5],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD5,4,SQ_MEM_START_ADD5+1280);
     hsAT24xx_Config(&sqCfg[6],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD6,4,SQ_MEM_START_ADD6+1280);
     hsAT24xx_Config(&sqCfg[7],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD7,4,SQ_MEM_START_ADD7+1280);
     hsAT24xx_Config(&sqCfg[8],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD8,4,SQ_MEM_START_ADD8+1280);
     hsAT24xx_Config(&sqCfg[9],MEM_PAGE,MEM_ADD, SQ_MEM_START_ADD9,4,SQ_MEM_START_ADD9+1280);
     */
     hsAT24xx_Config(&appCfg,MEM_PAGE,MEM_ADD, APP_MEM_START_ADD,4,APP_MEM_SIZE_ADD);
     hsAT24xx_Config(&temCfg,MEM_PAGE,MEM_ADD, TEM_MEM_START_ADD,4,512);
     
    
     
     motor_param_init();
     pump_tm_read();
     
     
     
     pump_cfg_read();
     sq_pram_read(0);
     sq_init();
     temp_param_init();
       
    // for(byte i=0;i>21)
     //  app_data_wirte(0x0fff,0);
}
void motor_cfg_read()
{hsAT24xx_Read(&mtCfg, (uint8_t*)&mt_ctrl, 0, sizeof(mt_ctrl));}
void motor_cfg_wirte()
{hsAT24xx_Write(&mtCfg, (uint8_t*)&mt_ctrl, 0,  sizeof(mt_ctrl));}

void pump_cfg_read()
{hsAT24xx_Read(&ppCfg, (uint8_t*)&pp, 0, sizeof(pp));}
void pump_cfg_wirte()
{hsAT24xx_Write(&ppCfg, (uint8_t*)&pp, 0,  sizeof(pp));}
     
void pump_tm_read()
{hsAT24xx_Read(&pp2Cfg, (uint8_t*)&pp_tm, 0, sizeof(pp_tm));}
void pump_tm_wirte()
{hsAT24xx_Write(&pp2Cfg, (uint8_t*)&pp_tm, 0, sizeof(pp_tm));}


void app_data_read(uint16_t addr, uint8_t* data){
 //  hsAT24xx_Read(&appCfg, data,(addr*4, 4);
  hsAT24xx_Read(&appCfg, data,addr*4, 4);
}


void app_data_wirte(uint16_t addr, uint8_t* data ){
 // hsAT24xx_Write(&appCfg, data,(addr*4),  4);
  hsAT24xx_Write(&appCfg, data, addr*4,  4);
}
void heat_pram_read()
{hsAT24xx_Read(&temCfg, (uint8_t*)&heat_tmp, 0, sizeof(heat_tmp));}
void heat_pram_wirte()
{hsAT24xx_Write(&temCfg, (uint8_t*)&heat_tmp, 0, sizeof(heat_tmp));}
//void pump_cfg_read()
//{hsAT24xx_Read(&pp2Cfg, (uint8_t*)&pp, PP_MEM_START_ADD, sizeof(pp));}
//void pump_cfg_wirte()
//{hsAT24xx_Write(&pp2Cfg, (uint8_t*)&pp, PP_MEM_START_ADD, sizeof(pp));}
void sq_pram_read(byte step)
{
  byte k;
  uint16_t ST_ADD;
  k=step;
 // for(k=0; k<step; k++){
  switch(k)
  {
  case 0:
  ST_ADD=SQ_MEM_START_ADD0;
  break;
  case 1:
  ST_ADD=SQ_MEM_START_ADD1;  
  break;
  case 2:
  break;
 /* ST_ADD=SQ_MEM_START_ADD2;  
  break;
  case 3:
  ST_ADD=SQ_MEM_START_ADD3;
  break;
  case 4:
  ST_ADD=SQ_MEM_START_ADD4;  
  break;
  case 5:
  ST_ADD=SQ_MEM_START_ADD5;  
  break;
  case 6:
  ST_ADD=SQ_MEM_START_ADD6;  
  break;
  case 7:
  ST_ADD=SQ_MEM_START_ADD7;
  break;
  case 8:
  ST_ADD=SQ_MEM_START_ADD8;  
  break;
  case 9:
  ST_ADD=SQ_MEM_START_ADD9;  
  break;*/

  }
 // for(i=0; i<10; i++)
  hsAT24xx_Read(&sqCfg[k], (uint8_t*)&sq, ST_ADD, sizeof(sq));
  //}

}
void sq_pram_write(byte step)
{
  byte k;
  uint16_t ST_ADD;
  k=step;
 // for(k=0; k<step; k++){
  switch(k)
  {
  case 0:
  ST_ADD=SQ_MEM_START_ADD0;
  break;
  case 1:
  ST_ADD=SQ_MEM_START_ADD1;  
  break;
 /* case 2:
  ST_ADD=SQ_MEM_START_ADD2;  
  break;
  case 3:
  ST_ADD=SQ_MEM_START_ADD3;
  break;
  case 4:
  ST_ADD=SQ_MEM_START_ADD4;  
  break;
  case 5:
  ST_ADD=SQ_MEM_START_ADD5;  
  break;
  case 6:
  ST_ADD=SQ_MEM_START_ADD6;  
  break;
  case 7:
  ST_ADD=SQ_MEM_START_ADD7;
  break;
  case 8:
  ST_ADD=SQ_MEM_START_ADD8;  
  break;
  case 9:
  ST_ADD=SQ_MEM_START_ADD9;  
  break;*/

  }
  //for(i=0; i<10; i++)
  hsAT24xx_Write(&sqCfg[k], (uint8_t*)&sq, ST_ADD, sizeof(sq)); 
  //}  
}

