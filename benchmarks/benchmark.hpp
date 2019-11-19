#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

static float randf(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

static void mat_set(double *A,
                    const size_t stride,
                    const size_t i,
                    const size_t j,
                    const double val) {
  assert(A != NULL);
  assert(stride != 0);

  A[(i * stride) + j] = val;
}

double *create_random_sq_matrix(const size_t m) {
  double *A = (double *) malloc(sizeof(double) * m * m);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < m; j++) {
      mat_set(A, m, i, j, randf(-1.0, 1.0));
    }
  }

  return A;
}
