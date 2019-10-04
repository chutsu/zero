#ifndef CORE_H
#define CORE_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

float deg2rad(const float d);
float rad2deg(const float r);
int8_t int8(const uint8_t *data, const size_t offset);
uint8_t uint8(const uint8_t *data, const size_t offset);
int16_t int16(const uint8_t *data, const size_t offset);
uint16_t uint16(const uint8_t *data, const size_t offset);
int32_t int32(const uint8_t *data, const size_t offset);
uint32_t uint32(const uint8_t *data, const size_t offset);

#endif  /* CORE_H */
