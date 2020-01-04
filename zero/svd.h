#ifndef SVD_H
#define SVD_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define USE_LAPACKE
#ifdef USE_LAPACKE
#include <lapacke.h>
#endif

#include "zero/core.h"

int svd(double *A, int m, int n, double *U, double *s, double *V_t) {
#ifdef USE_LAPACKE
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
#else
  fprintf(stderr, "Not Supported!");
  exit(-1);
#endif

  return 0;
}

static double pythag(double a, double b) {
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

/**
 * svdcomp - SVD decomposition routine.
 * Takes an mxn matrix a and decomposes it into udv, where u,v are
 * left and right orthogonal transformation matrices, and d is a
 * diagonal matrix of singular values.
 *
 * This routine is adapted from svdecomp.c in XLISP-STAT 2.1 which is
 * code from Numerical Recipes adapted by Luke Tierney and David Betz.
 *
 * Input to dsvd is as follows:
 *   A = mxn matrix to be decomposed, gets overwritten with U
 *   m = row dimension of a
 *   n = column dimension of a
 *   w = returns the vector of singular values of a
 *   V = returns the right orthogonal transformation matrix
 */
int svdcomp(double *A, int m, int n, double *w, double *V) {
  assert(m < n);
  int flag, i, its, j, jj, k, l, nm;
  double c, f, h, s, x, y, z;
  double anorm = 0.0, g = 0.0, scale = 0.0;

  /* Householder reduction to bidiagonal form */
  double *rv1 = malloc(sizeof(double) * n);
  for (i = 0; i < n; i++) {
    /* left-hand reduction */
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m) {
      for (k = i; k < m; k++)
        scale += fabs(A[k * n + i]);
      if (scale) {
        for (k = i; k < m; k++) {
          A[k * n + i] = (A[k * n + i] / scale);
          s += (A[k * n + i] * A[k * n + i]);
        }
        f = A[i * n + i];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        A[i * n + i] = (f - g);
        if (i != n - 1) {
          for (j = l; j < n; j++) {
            for (s = 0.0, k = i; k < m; k++)
              s += (A[k * n + i] * A[k * n + j]);
            f = s / h;
            for (k = i; k < m; k++)
              A[k * n + j] += (f * A[k * n + i]);
          }
        }
        for (k = i; k < m; k++)
          A[k * n + i] = (A[k * n + i] * scale);
      }
    }
    w[i] = (scale * g);

    /* right-hand reduction */
    g = s = scale = 0.0;
    if (i < m && i != n - 1) {
      for (k = l; k < n; k++)
        scale += fabs(A[i * n + k]);
      if (scale) {
        for (k = l; k < n; k++) {
          A[i * n + k] = (A[i * n + k] / scale);
          s += (A[i * n + k] * A[i * n + k]);
        }
        f = A[i * n + l];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        A[i * n + l] = (f - g);
        for (k = l; k < n; k++)
          rv1[k] = A[i * n + k] / h;
        if (i != m - 1) {
          for (j = l; j < m; j++) {
            for (s = 0.0, k = l; k < n; k++)
              s += (A[j * n + k] * A[i * n + k]);
            for (k = l; k < n; k++)
              A[j * n + k] += (s * rv1[k]);
          }
        }
        for (k = l; k < n; k++)
          A[i * n + k] = (A[i * n + k] * scale);
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  /* accumulate the right-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    if (i < n - 1) {
      if (g) {
        for (j = l; j < n; j++)
          V[j * n + i] = ((A[i * n + j] / A[i * n + l]) / g);
        /* double division to avoid underflow */
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++)
            s += (A[i * n + k] * V[k * n + j]);
          for (k = l; k < n; k++)
            V[k * n + j] += (s * V[k * n + i]);
        }
      }
      for (j = l; j < n; j++)
        V[i * n + j] = V[j * n + i] = 0.0;
    }
    V[i * n + i] = 1.0;
    g = rv1[i];
    l = i;
  }

  /* accumulate the left-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    l = i + 1;
    g = w[i];
    if (i < n - 1)
      for (j = l; j < n; j++)
        A[i * n + j] = 0.0;
    if (g) {
      g = 1.0 / g;
      if (i != n - 1) {
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < m; k++)
            s += (A[k * n + i] * A[k * n + j]);
          f = (s / A[i * n + i]) * g;
          for (k = i; k < m; k++)
            A[k * n + j] += (f * A[k * n + i]);
        }
      }
      for (j = i; j < m; j++)
        A[j * n + i] = (A[j * n + i] * g);
    } else {
      for (j = i; j < m; j++)
        A[j * n + i] = 0.0;
    }
    ++A[i * n + i];
  }

  /* diagonalize the bidiagonal form */
  for (k = n - 1; k >= 0; k--) {     /* loop over singular values */
    for (its = 0; its < 30; its++) { /* loop over allowed iterations */
      flag = 1;
      for (l = k; l >= 0; l--) { /* test for splitting */
        nm = l - 1;
        if (fabs(rv1[l]) + anorm == anorm) {
          flag = 0;
          break;
        }
        if (fabs(w[nm]) + anorm == anorm)
          break;
      }
      if (flag) {
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          if (fabs(f) + anorm != anorm) {
            g = w[i];
            h = pythag(f, g);
            w[i] = h;
            h = 1.0 / h;
            c = g * h;
            s = (-f * h);
            for (j = 0; j < m; j++) {
              y = A[j * n + nm];
              z = A[j * n + i];
              A[j * n + nm] = y * c + z * s;
              A[j * n + i] = z * c - y * s;
            }
          }
        }
      }
      z = w[k];
      if (l == k) {    /* convergence */
        if (z < 0.0) { /* make singular value nonnegative */
          w[k] = (-z);
          for (j = 0; j < n; j++)
            V[j * n + k] = (-V[j * n + k]);
        }
        break;
      }
      if (its >= 30) {
        free((void *) rv1);
        fprintf(stderr, "No convergence after 30,000! iterations \n");
        return (0);
      }

      /* shift from bottom 2 x 2 minor */
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;

      /* next QR transformation */
      c = s = 1.0;
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
        y = y * c;
        for (jj = 0; jj < n; jj++) {
          x = V[(jj * n) + j];
          z = V[(jj * n) + i];
          V[jj * n + j] = x * c + z * s;
          V[jj * n + i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z;
        if (z) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = (c * g) + (s * y);
        x = (c * y) - (s * g);
        for (jj = 0; jj < m; jj++) {
          y = A[jj * n + j];
          z = A[jj * n + i];
          A[jj * n + j] = (y * c + z * s);
          A[jj * n + i] = (z * c - y * s);
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }

  free(rv1);
  return 0;
}

int pinv(double *A, const int m, const int n, double *A_inv) {
  /* Decompose A with SVD */
  double *U = malloc(sizeof(double) * m * n);
  double *d = malloc(sizeof(double) * n);
  double *V_t = malloc(sizeof(double) * n * n);
  if (svd(A, m, n, U, d, V_t) != 0) {
    return -1;
  }

  /* Form reciprocal singular matrix S_inv from singular vector d */
  double *S_inv = malloc(sizeof(double) * n * n);
  zeros(S_inv, n, n);
  int mat_index = 0;
  int vec_index = 0;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        S_inv[mat_index] = 1.0 / d[vec_index];
        vec_index++;
      }
      mat_index++;
    }
  }

  /* pinv(H) = V S^-1 U' */
  double *V = malloc(sizeof(double) * n * n);
  mat_transpose(V_t, n, n, V);

  double *U_t = malloc(sizeof(double) * n * n);
  mat_transpose(U, n, n, U_t);

  double *VSi = malloc(sizeof(double) * n * n);
  dot(V, n, n, S_inv, n, n, VSi);
  dot(VSi, n, n, U_t, n, n, A_inv);

  /* Clean up */
  free(U);
  free(U_t);
  free(d);
  free(S_inv);
  free(V);
  free(V_t);
  free(VSi);

  return 0;
}

#endif /* SVD_H */
