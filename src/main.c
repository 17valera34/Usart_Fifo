#include <avr/io.h>
#include "usart.h"
#include <avr/interrupt.h>

// Polls for a ready command and processes it
void usart_task(void)
{
  if (cmd_ready)
  {
    process_usart_command((char *)cmd_buf);
    cmd_ready = 0;
  }
}

int main(void)
{
  gpio_init();  // Initialize GPIO (LED)
  clear_buffer(); // Clear RX buffer
  usart_init();   // Initialize USART
  usart_send_string("Send String...\r\n");
  sei();  // Enable global interrupts

  while (1)
  {
    // Handle received characters
    while (usart_available())
    {
      uint8_t c = usart_read_byte();
      handle_usart_message(c);
    }

    // Execute command if available
    usart_task();
  }
}
