#include "zero/math.h"

#include <cblas.h>

void dot_cblas(const double * restrict A, const size_t A_m, const size_t A_n,
							 const double * restrict B, const size_t B_m, const size_t B_n,
							 double * restrict C) {
	UNUSED(B_m);
  assert(A != NULL && B != NULL && C != NULL);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  cblas_dgemm(
    CblasRowMajor, /* Matrix data arrangement */
    CblasNoTrans,  /* Transpose A */
    CblasNoTrans,  /* Transpose B */
    A_m,           /* Number of rows in A and C */
    B_n,           /* Number of cols in B and C */
    A_n,           /* Number of cols in A */
    1.0,           /* Scaling factor for the product of A and B */
    A,             /* Matrix A */
    A_n,           /* First dimension of A */
    B,             /* Matrix B */
    B_n,           /* First dimension of B */
    1.0,           /* Scale factor for C */
    C,             /* Output */
    A_m            /* First dimension of C */
  );
}

void dot2(double **A, const size_t A_m, const size_t A_n,
          double **B, const size_t B_m, const size_t B_n,
          double **C) {
  assert(A != NULL && B != NULL && C != NULL);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  size_t m = A_m;
  size_t n = B_n;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      double sum = 0.0;
      for (size_t k = 0; k < A_n; k++) {
        sum += A[i][k] * B[k][j];
      }
      C[i][j] = sum;
    }
  }
}

int main() {
	/* #<{(| Array |)}># */
	/* for (size_t k = 1; k < 300; k++) { */
	/* 	size_t m = k; */
	/* 	double *A = malloc(sizeof(double) * m * m); */
	/* 	double *B = malloc(sizeof(double) * m * m); */
	/* 	double *C = malloc(sizeof(double) * m * m); */
  /*  */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		for (size_t j = 0; j < m; j++) { */
	/* 			mat_set(A, m, i, j, randf(-1.0, 1.0)); */
	/* 			mat_set(B, m, i, j, randf(-1.0, 1.0)); */
	/* 		} */
	/* 	} */
  /*  */
	/* 	struct timespec t = tic(); */
	/* 	dot(A, m, m, B, m, m, C); */
  /*   printf("Matrix size [%ld] Finnished in %f seconds. \n", m, toc(&t)); */
  /*  */
	/* 	free(A); */
	/* 	free(B); */
	/* 	free(C); */
	/* } */

	/* Array - Blas */
	for (size_t k = 1; k < 1000; k++) {
		size_t m = k;
		double *A = malloc(sizeof(double) * m * m);
		double *B = malloc(sizeof(double) * m * m);
		double *C = malloc(sizeof(double) * m * m);

		for (size_t i = 0; i < m; i++) {
			for (size_t j = 0; j < m; j++) {
				mat_set(A, m, i, j, randf(-1.0, 1.0));
				mat_set(B, m, i, j, randf(-1.0, 1.0));
			}
		}

		struct timespec t = tic();
		dot_cblas(A, m, m, B, m, m, C);
    printf("Matrix size [%ld] Finnished in %f seconds. \n", m, toc(&t));

		free(A);
		free(B);
		free(C);
	}

	/* #<{(| 2D array |)}># */
	/* for (size_t k = 1; k < 300; k++) { */
	/* 	size_t m = k; */
	/* 	double **A = malloc(sizeof(double *) * m); */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		A[i] = malloc(sizeof(double) * m); */
	/* 	} */
  /*  */
	/* 	double **B = malloc(sizeof(double *) * m); */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		B[i] = malloc(sizeof(double) * m); */
	/* 	} */
  /*  */
	/* 	double **C = malloc(sizeof(double *) * m); */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		C[i] = malloc(sizeof(double) * m); */
	/* 	} */
  /*  */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		for (size_t j = 0; j < m; j++) { */
	/* 			A[i][j] = randf(-1.0, 1.0); */
	/* 			B[i][j] = randf(-1.0, 1.0); */
	/* 		} */
	/* 	} */
  /*  */
	/* 	struct timespec t = tic(); */
	/* 	dot2(A, m, m, B, m, m, C); */
  /*   printf("Matrix size [%ld] Finnished in %f seconds. \n", m, toc(&t)); */
  /*  */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		free(A[i]); */
	/* 	} */
	/* 	free(A); */
  /*  */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		free(B[i]); */
	/* 	} */
	/* 	free(B); */
  /*  */
	/* 	for (size_t i = 0; i < m; i++) { */
	/* 		free(C[i]); */
	/* 	} */
	/* 	free(C); */
	/* } */

  return 0;
}
