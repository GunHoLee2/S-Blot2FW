#ifndef __UART_H
#define __UART_H




void usb_uart_init();
uint8_t ascii2hex(uint8_t code);
uint8_t math_uint(uint8_t lsb, uint8_t hsb );

volatile void send_sq_cfg(enum cntrl_event cmd, int32_t data);



#endif

void execute_usb_rs232();
void usb_send_pack(enum cntrl_event cmd, uint8_t *data);
extern void usb_rx_int(byte usb_chr);
extern uint8_t usb_data_buf[4];
void et_cmd_ctrl();

void lcd_send_pack(enum cntrl_event cmd, uint8_t *data);

volatile void send_mt_cfg(enum cntrl_event cmd, int32_t data);
volatile void send_sq_cfg(enum cntrl_event cmd, int32_t data);
volatile void send_pp_cfg(enum cntrl_event cmd, int32_t data);
void send_qc_dsp();
void send_qc_prime();
void send_qc_asp();
void send_qc_sk();
void send_qc_etc(enum cntrl_event cmd, uint32_t time);
void send_qc_rollbk();
void send_dry(enum cntrl_event cmd,uint8_t temp, uint8_t tm);


volatile void send_lcd_sq_cfg(enum cntrl_event cmd, int32_t data);
volatile void send_lcd_sq_data(enum cntrl_event cmd, int32_t data);
void lcd_send_rc_pack(enum cntrl_event cmd,enum cntrl_event rec_cmd, uint8_t *data);


void led_ctrl(byte cmd,uint32_t data);
void temp_ctrl(byte cmd,uint32_t data);
void led_read();
void led_rx_int(byte usb_chr);
volatile void send_mt_cfg(enum cntrl_event cmd, int32_t data);
void exe_led_ctrl(byte cmd);
void led_toggl(byte set);
void led_light_set(byte *data);
void temp_set(byte *data);
extern byte led_vol[4];
void tray_temp_set(byte *data);
