#ifndef SERIAL_H
#define SERIAL_H

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
 * @param[in] speed Serial speed
 * @param[in] parity Serial parity
 */
void serial_configure(serial_t *s,
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
 * Write a single byte to serial.
 *
 * @param[in] s Serial
 * @param[in] data Data to transmit
 */
void serial_write_byte(const serial_t *s, const uint8_t data);

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
 * Read a single byte to serial.
 *
 * @param[in] s Serial
 * @param[out] data Data to transmit
 */
void serial_read_byte(const serial_t *s, uint8_t *data);

/**
 * Read from serial.
 *
 * @param[in] s Serial.
 * @param[in,out] data Data to transmit.
 * @param[in] size Size of data to read.
 */
void serial_read(const serial_t *s, uint8_t *data, const size_t size);

#endif /* SERIAL_H */
