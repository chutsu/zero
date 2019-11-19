#ifndef CORE_HPP
#define CORE_HPP

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

inline float deg2rad(const float d) { return d * (M_PI / 180.0); }

inline float rad2deg(const float r) { return r * (180.0 / M_PI); }

inline int8_t int8(const uint8_t *data, const size_t offset) {
  return (int8_t)(data[offset]);
}

inline uint8_t uint8(const uint8_t *data, const size_t offset) {
  return (uint8_t)(data[offset]);
}

inline int16_t int16(const uint8_t *data, const size_t offset) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline uint16_t uint16(const uint8_t *data, const size_t offset) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline int32_t int32(const uint8_t *data, const size_t offset) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
      (data[offset + 1] << 8) | (data[offset]));
}

inline uint32_t uint32(const uint8_t *data, const size_t offset) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
      (data[offset + 1] << 8) | (data[offset]));
}

inline void tf_trans(const float T[16], float r[3]) {
  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

inline void tf_rot(const float T[16], float C[9]) {
  C[0] = T[0]; C[1] = T[1]; C[2] = T[2];
  C[3] = T[3]; C[4] = T[4]; C[5] = T[5];
  C[6] = T[6]; C[7] = T[7]; C[8] = T[8];
}

#endif  // CORE_HPP
