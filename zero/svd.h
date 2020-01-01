#ifndef SVD_H
#define SVD_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <lapacke.h>

#include "zero/core.h"

int svd(double *A, int m, int n, double *U, double *s, double *V_t) {
  const int lda = n;
  const int ldu = m;
  const int ldvt = n;
  const char jobu = 'A';
  const char jobvt = 'A';
  const int superb_size = (m < n) ? m : n;
  double *superb = malloc(sizeof(double) * (superb_size - 1));
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

  /* Clean up */
  free(superb);

  return 0;
}

/* Computes (a2 + b2)1/2 without destructive underflow or overflow. */
static float pythag(double a, double b) {
  double at = fabs(a), bt = fabs(b), ct, result;
  if (at > bt) {
    ct = bt / at;
    result = at * sqrt(1.0 + ct * ct);
  } else if (bt > 0.0) {
    ct = at / bt;
    result = bt * sqrt(1.0 + ct * ct);
  } else
    result = 0.0;
  return (result);
}

void svdcmp(float **a, int m, int n, float w[], float **v) {
  int flag, i, its, j, jj, k, l, nm;
  float anorm, c, f, g, h, s, scale, x, y, z, *rv1;
  /* rv1 = vector(1, n); */
  rv1 = malloc(sizeof(double) * n);
  g = scale = anorm = 0.0;

  /* Householder reduction to bidiagonal form. */
  for (i = 1; i <= n; i++) {
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i <= m) {
      for (k = i; k <= m; k++)
        scale += fabs(a[k][i]);
      if (scale) {
        for (k = i; k <= m; k++) {
          a[k][i] /= scale;
          s += a[k][i] * a[k][i];
        }
        f = a[i][i];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        a[i][i] = f - g;
        for (j = l; j <= n; j++) {
          for (s = 0.0, k = i; k <= m; k++)
            s += a[k][i] * a[k][j];
          f = s / h;
          for (k = i; k <= m; k++)
            a[k][j] += f * a[k][i];
        }
        for (k = i; k <= m; k++)
          a[k][i] *= scale;
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if (i <= m && i != n) {
      for (k = l; k <= n; k++)
        scale += fabs(a[i][k]);
      if (scale) {
        for (k = l; k <= n; k++) {
          a[i][k] /= scale;
          s += a[i][k] * a[i][k];
        }
        f = a[i][l];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        a[i][l] = f - g;
        for (k = l; k <= n; k++)
          rv1[k] = a[i][k] / h;
        for (j = l; j <= m; j++) {
          for (s = 0.0, k = l; k <= n; k++)
            s += a[j][k] * a[i][k];
          for (k = l; k <= n; k++)
            a[j][k] += s * rv1[k];
        }
        for (k = l; k <= n; k++)
          a[i][k] *= scale;
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  /* Accumulation of right-hand transformations. */
  for (i = n; i >= 1; i--) {
    if (i < n) {
      if (g) {
        /* Double division to avoid possible underflow. */
        for (j = l; j <= n; j++) {
          v[j][i] = (a[i][j] / a[i][l]) / g;
        }
        for (j = l; j <= n; j++) {
          for (s = 0.0, k = l; k <= n; k++)
            s += a[i][k] * v[k][j];
          for (k = l; k <= n; k++)
            v[k][j] += s * v[k][i];
        }
      }
      for (j = l; j <= n; j++) {
        v[i][j] = v[j][i] = 0.0;
      }
    }
    v[i][i] = 1.0;
    g = rv1[i];
    l = i;
  }

  /* Accumulation of left-hand transformations. */
  for (i = MIN(m, n); i >= 1; i--) {
    l = i + 1;
    g = w[i];
    for (j = l; j <= n; j++)
      a[i][j] = 0.0;
    if (g) {
      g = 1.0 / g;
      for (j = l; j <= n; j++) {
        for (s = 0.0, k = l; k <= m; k++)
          s += a[k][i] * a[k][j];
        f = (s / a[i][i]) * g;
        for (k = i; k <= m; k++)
          a[k][j] += f * a[k][i];
      }
      for (j = i; j <= m; j++)
        a[j][i] *= g;
    } else
      for (j = i; j <= m; j++)
        a[j][i] = 0.0;
    ++a[i][i];
  }

  /* Diagonalization of the bidiagonal form: Loop over */
  for (k = n; k >= 1; k--) {
    /* Singular values, and over allowed iterations. */
    for (its = 1; its <= 30; its++) {
      flag = 1;
      for (l = k; l >= 1; l--) { /*Test for splitting. */
        nm = l - 1;              /* Note that rv1[1] is always zero. */
        if ((float) (fabs(rv1[l]) + anorm) == anorm) {
          flag = 0;
          break;
        }
        if ((float) (fabs(w[nm]) + anorm) == anorm)
          break;
      }
      if (flag) {
        c = 0.0; /* Cancellation of rv1[l], if l > 1. */
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          rv1[i] = c * rv1[i];
          if ((float) (fabs(f) + anorm) == anorm)
            break;
          g = w[i];
          h = pythag(f, g);
          w[i] = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          for (j = 1; j <= m; j++) {
            y = a[j][nm];
            z = a[j][i];
            a[j][nm] = y * c + z * s;
            a[j][i] = z * c - y * s;
          }
        }
      }
      z = w[k];

      if (l == k) {
        /* Convergence. */
        if (z < 0.0) {
          /* Singular value is made nonnegative. */
          w[k] = -z;
          for (j = 1; j <= n; j++)
            v[j][k] = -v[j][k];
        }
        break;
      }
      if (its == 30) {
        printf("no convergence in 30 svdcmp iterations");
      }
      x = w[l];

      /* Shift from bottom 2 - by - 2 minor. */
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
      c = s = 1.0;

      /* Next QR transformation :  */
      for (j = l; j <= nm; j++) {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        for (jj = 1; jj <= n; jj++) {
          x = v[jj][j];
          z = v[jj][i];
          v[jj][j] = x * c + z * s;
          v[jj][i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z;

        /* Rotation can be arbitrary if z = 0.  */
        if (z) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = c * g + s * y;
        x = c * y - s * g;
        for (jj = 1; jj <= m; jj++) {
          y = a[jj][j];
          z = a[jj][i];
          a[jj][j] = y * c + z * s;
          a[jj][i] = z * c - y * s;
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }
  /* free_vector(rv1, 1, n); */
  free(rv1);
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
