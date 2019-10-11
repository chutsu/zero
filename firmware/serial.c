#include "serial.h"

void serial_configure(serial_t *s,
                      const uint32_t speed,
                      const uint8_t parity) {
  s->connfd = USART1;
  s->speed = speed;
  s->parity = parity;

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

  /* Setup GPIO pin for transmit and receive. */
	gpio_set_mode(GPIOA,
	              GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	              GPIO_USART1_TX);
  gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN,
                GPIO_USART1_RX);
  usart_set_mode(USART1, USART_MODE_TX_RX);

  usart_set_baudrate(USART1, speed);
  usart_set_databits(USART1, 8);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
}

void serial_connect(const serial_t *s) {
  usart_enable(s->connfd);
}

void serial_disconnect(const serial_t *s) {
  usart_disable(s->connfd);
}

void serial_write_byte(const serial_t *s, const uint8_t data) {
  usart_send_blocking(s->connfd, data);
}

void serial_write(const serial_t *s, const uint8_t *data, const size_t size) {
  for (size_t i = 0; i < size; i++) {
    usart_send_blocking(s->connfd, data[i]);
  }
}

void serial_write_string(const serial_t *s, const char *str) {
  for (size_t i = 0; i < strlen(str); i++) {
    serial_write_byte(s, str[i]);
  }
}

void serial_read_byte(const serial_t *s, uint8_t *data) {
  *data = usart_recv_blocking(s->connfd);
}

void serial_read(const serial_t *s, uint8_t *data, const size_t size) {
  for (size_t i = 0; i < size; i++) {
    data[i] = usart_recv_blocking(s->connfd);
  }
}
