#include "zero/cv/image.h"

void image_init(image_t *img, uint8_t *data, int width, int height) {
  img->data = data;
  img->width = width;
  img->height = height;
}
