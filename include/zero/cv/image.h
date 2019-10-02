#ifndef IMAGE_H
#define IMAGE_H

#include "zero/core.h"

typedef struct image_t {
  uint8_t *data;
  int width;
  int height;
} image_t;

void image_init(image_t *img, uint8_t *data, int width, int height);

#endif // IMAGE_H
