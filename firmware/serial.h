#ifndef SERIAL_H
#define SERIAL_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <stdint.h>
#include <stdlib.h>

#include "core.h"

/**
 * Serial
 */
typedef struct serial_t {
  uint32_t connfd;
  uint32_t speed;
  uint8_t parity;
  uint8_t blocking;
} serial_t;

/**
 * Set interface attributes
 *
 * @param[in,out] s Serial
 * @param[in] conn Connection descriptor
 * @param[in] speed Serial speed
 * @param[in] parity Serial parity
 */
void serial_configure(serial_t *s,
                      const uint32_t conn,
                      const uint32_t speed,
                      const uint8_t parity);

/**
 * Connect
 *
 * @param[in,out] s Serial
 */
void serial_connect(const serial_t *s);

/**
 * Disconnect
 *
 * @param[in,out] s Serial
 */
void serial_disconnect(const serial_t *s);

/**
 * Write to serial.
 *
 * @param[in] s Serial
 * @param[in] data Data to transmit
 * @param[in] size Size of data to transmit
 */
void serial_write(const serial_t *s,
                  const uint8_t *data,
                  const size_t size);

/**
 * Read from serial.
 *
 * @param[in] s Serial.
 * @param[in,out] data Data to transmit.
 * @param[in,out] size Size of data to read.
 */
void serial_read(const serial_t *s, uint8_t *data, const size_t size);

#endif /* SERIAL_H */
