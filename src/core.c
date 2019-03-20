#include "zero/core.h"

void vec_new(vec_t *v, const int length) {
  v->data = malloc(sizeof(real_t) * length);
  v->length = length;
}

void vec_free(vec_t *v) {
  free(v->data);
}

void mat_new(mat_t *m, const int rows, const int cols) {
  m->data = malloc(sizeof(real_t) * rows * cols);
  m->rows = rows;
  m->cols = cols;
}

void mat_free(mat_t *m) {
  free(m->data);
}

inline real_t deg2rad(const real_t d) {
  return d * (M_PI / 180.0);
}

inline real_t rad2deg(const real_t r) {
  return r * (180.0 / M_PI);
}
