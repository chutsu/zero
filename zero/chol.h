#ifndef CHOL_H
#define CHOL_H

#include <math.h>
#include <stdlib.h>
#include <assert.h>

double *cholesky(const double *A, const size_t n) {
  assert(A != NULL);
  assert(n > 0);
  double *L = calloc(n * n, sizeof(double));

  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < (i + 1); j++) {

      if (i == j) {
        double s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[j * n + k] * L[j * n + k];
        }
        L[i * n + j] = sqrt(A[i * n + i] - s);

      } else {
        double s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[i * n + k] * L[j * n + k];
        }
        L[i * n + j] = (1.0 / L[j * n + j] * (A[i * n + j] - s));
      }
    }
  }

  return L;
}

#endif /* CHOL_H */
