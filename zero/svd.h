#ifndef SVD_H
#define SVD_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <lapacke.h>

int svd(double *A, int m, int n, double *U, double *s, double *V_t) {
  const int lda = n;
  const int ldu = m;
  const int ldvt = n;
  double superb[n - 1];
  char jobu = 'A';
  char jobvt = 'A';

  lapack_int retval = LAPACKE_dgesvd(LAPACK_ROW_MAJOR,
                                     jobu,
                                     jobvt,
                                     m,
                                     n,
                                     A,
                                     lda,
                                     s,
                                     U,
                                     ldu,
                                     V_t,
                                     ldvt,
                                     superb);
  if (retval > 0) {
    return -1;
  }

  return 0;
}

/* int pinv(double *A, const int m, const int n) { */
/*   #<{(| Pseudo invert A with SVD |)}># */
/*  */
/*   #<{(| -- Decompose H with SVD |)}># */
/*   double *w = malloc(sizeof(double) * n); */
/*   double *V = malloc(sizeof(double) * n * n); */
/*   if (svd(A, n, n, w, V) != 0) { */
/*     return -1; */
/*   } */
/*  */
/*   #<{(| -- Form reciprocal singular matrix S_inv from singular vector w |)}>#
 */
/*   double *S_inv = malloc(sizeof(double) * n * n); */
/*   zeros(S_inv, n, n); */
/*   int index = 0; */
/*   for (int i = 0; i < n; i++) { */
/*     for (int j = 0; j < n; j++) { */
/*       if (i == j) { */
/*         S_inv[index] = 1.0 / w[index]; */
/*       } */
/*     } */
/*   } */
/*  */
/*   #<{(| -- pinv(H) = U S^-1 V' |)}># */
/*   double *US = malloc(sizeof(double) * n * n); */
/*   dot(A, n, n, S_inv, n, n, US); */
/*   dot(US, n, n, V, n, n, A); */
/*  */
/*   #<{(| -- Clean up |)}># */
/*   free(w); */
/*   free(V); */
/*   free(S_inv); */
/*   free(US); */
/*  */
/*   return 0; */
/* } */

#endif /* SVD_H */
