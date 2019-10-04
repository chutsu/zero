#include "serial.h"

void serial_configure(serial_t *s,
                      const uint32_t conn,
                      const uint32_t speed,
                      const uint8_t parity) {
  s->connfd = conn;
  s->speed = speed;
  s->parity = parity;

	rcc_periph_clock_enable(RCC_USART1);
	gpio_set_mode(GPIOA,
	              GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	              GPIO_USART1_TX);
  usart_set_baudrate(USART1, s->speed);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
}

void serial_connect(const serial_t *s) {
  usart_enable(s->connfd);
}

void serial_disconnect(const serial_t *s) {
  usart_disable(s->connfd);
}

void serial_write(const serial_t *s, const uint8_t *data, const size_t size) {
  for (size_t i = 0; i < size; i++) {
    usart_send(s->connfd, data[i]);
  }
}

void serial_read(const serial_t *s, uint8_t *data, const size_t size) {
  for (size_t i = 0; i < size; i++) {
    data[i] = usart_recv(s->connfd);
  }
}
