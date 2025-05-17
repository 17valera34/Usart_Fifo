#include "usart.h"
#include <avr/interrupt.h>

#define BAUD 9600
#define BAUD_RATE (F_CPU / (16UL * BAUD) - 1)

// Circular buffer for raw received data
static volatile unsigned char rx_buf[RX_BUF_SIZE];
static volatile uint8_t usart_rx_head = 0, usart_rx_tail = 0;

// Internal state flags
static volatile uint8_t usart_rx_ready, in_message = 0;

// Buffer to store the current command being assembled
volatile unsigned char cmd_buf[CMD_BUF_SIZE];
volatile uint8_t cmd_index = 0, cmd_ready = 0;

// Configures GPIO (sets PB5 as output for LED control)
void gpio_init(void)
{
    DDRB |= (1 << DDB5);
    PORTB &= ~(1 << PORTB5);
}

void usart_init(void)
{
    UBRR0H = (uint8_t)(BAUD_RATE >> 8);
    UBRR0L = (uint8_t)BAUD_RATE;

    UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable TX, RX and RX complete interrupt
    UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);   // 8-bit data, 1 stop bit, no parity
}

// Sends a single byte over USART
void usart_send_char(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0)))
    {
        /* code */
    }
    UDR0 = data;
}

// Sends a null-terminated string over USART
void usart_send_string(const char *str)
{
    while (*str)
    {
        usart_send_char(*str++);
    }
}

// Clears the RX buffer and resets head/tail indices
void clear_buffer(void)
{
    for (uint8_t i = 0; i < RX_BUF_SIZE; i++)
    {
        rx_buf[i] = '\0';
    }
    usart_rx_head = 0;
    usart_rx_tail = 0;
}

// Compares two strings
int usart_str_cmp(const char *s1, const char *s2)
{
    while (*s1 && (*s1 == *s2))
    {
        s1++;
        s2++;
    }
    return *(const unsigned char *)s1 - *(const unsigned char *)s2;
}

// Returns number of available bytes in RX buffer
int usart_available(void)
{
    return (usart_rx_head - usart_rx_tail) % RX_BUF_SIZE;
}

// Reads one byte from RX buffer
uint8_t usart_read_byte(void)
{
    if (usart_rx_head == usart_rx_tail)
    {
        return 0;
    }
    else
    {
        uint8_t byte = rx_buf[usart_rx_tail];
        usart_rx_tail = (usart_rx_tail + 1) % RX_BUF_SIZE;
        return byte;
    }
}

// Handles incoming character stream, looking for messages starting with '#' and ending with '\r'
void handle_usart_message(uint8_t byte)
{
    if (byte == '#')
    {
        cmd_index = 0;
        in_message = 1;
    }
    else if (in_message != 0)
    {
        if (byte == '\r')
        {
            cmd_buf[cmd_index] = '\0';
            in_message = 0;
            cmd_ready = 1;
        }
        else if (cmd_index < CMD_BUF_SIZE - 1)
        {
            cmd_buf[cmd_index++] = byte;
        }
    }
}

// Processes received command
void process_usart_command(const char *cmd)
{
    if (usart_str_cmp(cmd, "ON") == 0)
    {
        PORTB |= (1 << PORTB5);
        usart_send_string("LED ON\r\n");
    }
    else if (usart_str_cmp(cmd, "OFF") == 0)
    {
        PORTB &= ~(1 << PORTB5);
        usart_send_string("LED OFF\r\n");
    }
    else
    {
        usart_send_string("UNKNOWN CMD\r\n");
    }
}

// USART RX interrupt handler
ISR(USART_RX_vect)
{
    uint8_t next = (usart_rx_head + 1) % RX_BUF_SIZE;

    if (next != usart_rx_tail)
    {
        rx_buf[usart_rx_head] = UDR0;
        usart_rx_head = next;
    }
}