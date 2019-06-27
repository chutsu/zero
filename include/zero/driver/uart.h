#ifndef ZERO_DRIVER_UART_H
#define ZERO_DRIVER_UART_H

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "zero/log.h"

/**
 * UART
 */
typedef struct uart_t {
  int connected;
  int connection;
  char *port;

  int speed;
  int parity;
  int blocking;
} uart_t;

/**
 * Connect
 *
 * @param[in,out] uart UART
 * @returns 0 or -1 for success or failure
 */
int uart_connect(uart_t *uart);

/**
 * Disconnect
 *
 * @param[in,out] uart UART
 * @returns 0 or -1 for success or failure
 */
int uart_disconnect(uart_t *uart);

/**
 * Set interface attributes
 *
 * @param[in,out] uart UART
 * @param[in] speed UART speed
 * @param[in] parity UART parity
 * @returns 0 or -1 for success or failure
 */
int uart_configure(const uart_t *uart, const int speed, const int parity);

/**
 * Set blocking
 *
 * @param[in,out] uart UART
 * @param[in] blocking Blocking
 * @returns 0 or -1 for success or failure
 */
int uart_set_blocking(const uart_t *uart, const int blocking);

#endif /* ZERO_DRIVER_UART_H */
