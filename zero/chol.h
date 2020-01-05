#ifndef CHOL_H
#define CHOL_H

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <lapacke.h>

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

    if (fltcmp(L[i * n + i], 0.0) == 0) {
      y[i] = 0.0;

    } else {
      for (int j = 0; j < i; j++) {
        alpha -= L[i * n + j] * y[j];
      }
      y[i] = alpha / L[i * n + i];
    }
  }

  /* Backward substitution */
  /* Now we have y, we can go back to (Lt x = y) and solve for x */
  for (int i = n - 1; i >= 0; i--) {
    double alpha = y[i];

    if (fltcmp(Lt[i * n + i], 0.0) == 0) {
      x[i] = 0.0;

    } else {
      for (int j = i; j < (int) n; j++) {
        alpha -= Lt[i * n + j] * x[j];
      }
      x[i] = alpha / Lt[i * n + i];
    }
  }

  /* Clean up */
  free(L);
  free(Lt);
}

void chol_lls_solve2(const double *A,
                     const double *b,
                     double *x,
                     const size_t n) {
  /* Cholesky Decomposition */
  const char uplo = 'L';
  double *a = mat_new(n, n);
  mat_copy(A, n, n, a);
  int retval = LAPACKE_dpotrf(LAPACK_ROW_MAJOR, uplo, n, a, n);
  if (retval != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }
  print_matrix("A", A, n, n);
  print_matrix("a", a, n, n);

  /* #<{(| Solve Ax = b using Cholesky decomposed A from above |)}># */
  /* double *b_copy = mat_new(n, 1); */
  /* mat_copy(b, n, 1, b_copy); */
  /* print_matrix("a", a, n, n); */
  /* print_matrix("b_copy", b_copy, n, 1); */
  /* retval = LAPACKE_dpotrs(LAPACK_ROW_MAJOR, uplo, n, 1, a, n, b_copy, n); */
  /* if (retval != 0) { */
  /*   fprintf(stderr, "Failed to solve Ax = b!\n"); */
  /* } */

  /* free(a); */
  /* free(b_copy); */
}

#endif /* CHOL_H */
