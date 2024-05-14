#ifndef EEPROM_H
#define EEPROM_H


void eeprom_init();
void motor_cfg_wirte();
void motor_cfg_read();

void pump_cfg_read();
void pump_cfg_wirte();
void pump_tm_read();
void pump_tm_wirte();
void heat_pram_read();
void heat_pram_wirte();


void sq_pram_read(byte step);
void sq_pram_write(byte step);

void app_data_wirte(uint16_t addr, uint8_t* data );
void app_data_read(uint16_t addr, uint8_t* data);



#endif
