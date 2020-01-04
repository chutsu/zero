#ifndef CHOL_H
#define CHOL_H

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include "zero/core.h"

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

void chol_lls_solve(const double *A,
                    const double *b,
                    double *x,
                    const size_t n) {
  /* Allocate memory */
  double *Lt = calloc(n * n, sizeof(double));
  double *y = calloc(n, sizeof(double));

  /* Cholesky decomposition */
  double *L = cholesky(A, n);
  mat_transpose(L, n, n, Lt);

  /* Forward substitution */
  /* Ax = b -> LLt x = b. */
  /* Let y = Lt x, L y = b (Solve for y) */
  for (int i = 0; i < (int) n; i++) {
    double alpha = b[i];
    for (int j = 0; j < i; j++) {
      alpha -= L[i * n + j] * y[j];
    }
    y[i] = alpha / L[i * n + i];
  }

  /* Backward substitution */
  /* Now we have y, we can go back to (Lt x = y) and solve for x */
  for (int i = n - 1; i >= 0; i--) {
    double alpha = y[i];
    for (int j = i; j < (int) n; j++) {
      alpha -= Lt[i * n + j] * x[j];
    }
    x[i] = alpha / Lt[i * n + i];
  }

  /* Clean up */
  free(L);
  free(Lt);
}

#endif /* CHOL_H */
