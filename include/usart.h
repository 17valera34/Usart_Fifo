#ifndef __USART_H
#define __USART_H

#include "avr/io.h"
#define CMD_BUF_SIZE 64 // Buffer size for parsed command string
#define RX_BUF_SIZE 64  // Circular RX buffer size

extern volatile unsigned char cmd_buf[CMD_BUF_SIZE];
extern volatile uint8_t cmd_ready;

void gpio_init(void);
void usart_init(void);
void usart_send_char(uint8_t data);
void usart_send_string(const char *str);
int usart_str_cmp(const char *s1, const char *s2);
void clear_buffer(void);
int usart_available(void);
uint8_t usart_read_byte(void);
void handle_usart_message(uint8_t byte);
void process_usart_command(const char *cmd);


#endif
