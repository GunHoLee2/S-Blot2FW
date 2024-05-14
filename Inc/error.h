#ifndef ERROR_H
#define ERROR_H

#define CMD_ERROR               1
#define DATA_ERROR              2
#define BUSY_ERROR              3
#define BUSY_INT_ERROR          4

bool error(enum cntrl_event err, uint8_t err_st);
uint16_t get_error();
void clear_error();
void send_error(uint16_t err);



#endif

