#include "zero/munit.h"
#include "zero/math.h"

int test_eye() {
  double A[25] = {0.0};
  eye(A, 5, 5);

  /* print_matrix("I", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      double expected = (i == j) ? 1.0 : 0.0;
      MU_CHECK(fltcmp(A[idx], expected) == 0);
      idx++;
    }
  }

  return 0;
}

int test_ones() {
  double A[25] = {0.0};
  ones(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_CHECK((fabs(A[idx] - 1.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_zeros() {
  double A[25] = {0.0};
  zeros(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_CHECK((fabs(A[idx] - 0.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_mat_set() {
  double A[9] = {0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0};

  mat_set(A, 3, 0, 0, 1.0);
  mat_set(A, 3, 1, 1, 1.0);
  mat_set(A, 3, 2, 2, 1.0);

  /* print_matrix("A", A, 3, 3); */
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 1), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 2, 2), 1.0) == 0);

  return 0;
}

int test_mat_val() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};

  /* print_matrix("A", A, 3, 3); */
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 1), 2.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 2), 3.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 0), 4.0) == 0);

  return 0;
}

int test_mat_block() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double B[4] = {0.0};
  mat_block(A, 3, 1, 1, 2, 2, B);

  /* print_matrix("B", B, 2, 2); */
  MU_CHECK(fltcmp(mat_val(B, 2, 0, 0), 5.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 0, 1), 6.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 1, 0), 8.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 1, 1), 9.0) == 0);

  return 0;
}

int test_mat_transpose() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double C[9] = {0.0};
  mat_transpose(A, 3, 3, C);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_add() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double B[9] = {9.0, 8.0, 7.0,
                 6.0, 5.0, 4.0,
                 3.0, 2.0, 1.0};
  double C[9] = {0.0};
  mat_add(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_sub() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double B[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double C[9] = {0.0};
  mat_sub(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_vec_add() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double B[9] = {9.0, 8.0, 7.0,
                 6.0, 5.0, 4.0,
                 3.0, 2.0, 1.0};
  double C[9] = {0.0};
  vec_add(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

int test_vec_sub() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double B[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double C[9] = {0.0};
  vec_sub(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

/* void mat_dot(const double *A, const size_t A_m, const size_t A_n, */
/*              const double *B, const size_t B_m, const size_t B_n, */
/*              double *C) { */
/*   assert(A_n == B_m); */
/*  */
/*   cblas_dgemm( */
/*     CblasRowMajor, #<{(| Matrix data arrangement |)}># */
/*     CblasNoTrans,  #<{(| Transpose A |)}># */
/*     CblasNoTrans,  #<{(| Transpose B |)}># */
/*     A_m,           #<{(| Number of rows in A and C |)}># */
/*     B_n,           #<{(| Number of cols in B and C |)}># */
/*     A_n,           #<{(| Number of cols in A |)}># */
/*     1.0,           #<{(| Scaling factor for the product of A and B |)}># */
/*     A,             #<{(| Matrix A |)}># */
/*     A_n,           #<{(| First dimension of A |)}># */
/*     B,             #<{(| Matrix B |)}># */
/*     B_n,           #<{(| First dimension of B |)}># */
/*     1.0,           #<{(| Scale factor for C |)}># */
/*     C,             #<{(| Output |)}># */
/*     A_m            #<{(| First dimension of C |)}># */
/*   ); */
/* } */

int test_dot() {
  double A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  double B[3] = {1.0, 2.0, 3.0};
  double C[9] = {0.0};

  /* Multiply matrix A and B */
  dot(A, 3, 3, B, 3, 1, C);
  print_vector("C", C, 3);

  return 0;
}

int test_tf_set_rot() {
	double C[9];
	for (int i = 0; i < 9; i++) {
		C[i] = 1.0;
	}

  double T[16] = {0.0};
	tf_set_rot(T, C);
  /* print_matrix("T", T, 4, 4); */

	MU_CHECK(fltcmp(T[0], 1.0) == 0);
	MU_CHECK(fltcmp(T[1], 1.0) == 0);
	MU_CHECK(fltcmp(T[2], 1.0) == 0);
	MU_CHECK(fltcmp(T[3], 0.0) == 0);

	MU_CHECK(fltcmp(T[4], 1.0) == 0);
	MU_CHECK(fltcmp(T[5], 1.0) == 0);
	MU_CHECK(fltcmp(T[6], 1.0) == 0);
	MU_CHECK(fltcmp(T[7], 0.0) == 0);

	MU_CHECK(fltcmp(T[8], 1.0) == 0);
	MU_CHECK(fltcmp(T[9], 1.0) == 0);
	MU_CHECK(fltcmp(T[10], 1.0) == 0);
	MU_CHECK(fltcmp(T[11], 0.0) == 0);

	MU_CHECK(fltcmp(T[12], 0.0) == 0);
	MU_CHECK(fltcmp(T[13], 0.0) == 0);
	MU_CHECK(fltcmp(T[14], 0.0) == 0);
	MU_CHECK(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_set_trans() {
	double r[3] = {1.0, 2.0, 3.0};

  double T[16] = {0.0};
	tf_set_trans(T, r);
  /* print_matrix("T", T, 4, 4); */

	MU_CHECK(fltcmp(T[0], 0.0) == 0);
	MU_CHECK(fltcmp(T[1], 0.0) == 0);
	MU_CHECK(fltcmp(T[2], 0.0) == 0);
	MU_CHECK(fltcmp(T[3], 1.0) == 0);

	MU_CHECK(fltcmp(T[4], 0.0) == 0);
	MU_CHECK(fltcmp(T[5], 0.0) == 0);
	MU_CHECK(fltcmp(T[6], 0.0) == 0);
	MU_CHECK(fltcmp(T[7], 2.0) == 0);

	MU_CHECK(fltcmp(T[8], 0.0) == 0);
	MU_CHECK(fltcmp(T[9], 0.0) == 0);
	MU_CHECK(fltcmp(T[10], 0.0) == 0);
	MU_CHECK(fltcmp(T[11], 3.0) == 0);

	MU_CHECK(fltcmp(T[12], 0.0) == 0);
	MU_CHECK(fltcmp(T[13], 0.0) == 0);
	MU_CHECK(fltcmp(T[14], 0.0) == 0);
	MU_CHECK(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_trans() {
  double T[16] = {1.0, 2.0, 3.0, 4.0,
								  5.0, 6.0, 7.0, 8.0,
								  9.0, 10.0, 11.0, 12.0,
								  13.0, 14.0, 15.0, 16.0};
  print_matrix("T", T, 4, 4);

	/* Get translation vector */
	double r[3];
	tf_trans(T, r);
  print_vector("r", r, 3);

	MU_CHECK(fltcmp(r[0], 4.0) == 0);
	MU_CHECK(fltcmp(r[1], 8.0) == 0);
	MU_CHECK(fltcmp(r[2], 12.0) == 0);

  return 0;
}

int test_tf_rot() {
  /* Transform */
  double T[16] = {1.0, 2.0, 3.0, 4.0,
								  5.0, 6.0, 7.0, 8.0,
								  9.0, 10.0, 11.0, 12.0,
								  13.0, 14.0, 15.0, 16.0};
  print_matrix("T", T, 4, 4);

	/* Get rotation matrix */
	double C[9];
	tf_rot(T, C);
  print_matrix("C", C, 3, 3);

	for (size_t i = 0; i < 9; i++) {
		MU_CHECK(fltcmp(C[i], i + 1) == 0);
	}

  return 0;
}

int test_tf_quat() {
  /* Transform */
  double T[16] = {1.0, 0.0, 0.0, 0.0,
								  0.0, 1.0, 0.0, 0.0,
								  0.0, 0.0, 1.0, 0.0,
								  0.0, 0.0, 0.0, 1.0};

	/* Create rotation matrix */
	const double euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
	double C[9] = {0};
	euler321(euler, C);
	tf_set_rot(T, C);

	/* Extract quaternion from transform */
	double q[4] = {0};
	tf_quat(T, q);

	/* Convert quaternion back to euler angles */
	double rpy[3] = {0};
	quat2euler(q, rpy);

	MU_CHECK(fltcmp(rad2deg(rpy[0]), 10.0) == 0);
	MU_CHECK(fltcmp(rad2deg(rpy[1]), 20.0) == 0);
	MU_CHECK(fltcmp(rad2deg(rpy[2]), 30.0) == 0);

	return 0;
}

int test_tf_inv() {
	/* Create Transform */
  double T[16] = {1.0, 0.0, 0.0, 0.0,
								  0.0, 1.0, 0.0, 0.0,
								  0.0, 0.0, 1.0, 0.0,
								  0.0, 0.0, 0.0, 1.0};
	/* -- Set rotation component */
	const double euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
	double C[9] = {0};
	euler321(euler, C);
	tf_set_rot(T, C);
	/* -- Set translation component */
	double r[3] = {1.0, 2.0, 3.0};
	tf_set_trans(T, r);
	print_matrix("T", T, 4, 4);
	printf("\n");

	/* Invert transform */
  double T_inv[16] = {0};
	tf_inv(T, T_inv);
	print_matrix("T_inv", T_inv, 4, 4);
	printf("\n");

	/* Double Invert transform */
  double T_inv_inv[16] = {0};
	tf_inv(T_inv, T_inv_inv);
	print_matrix("T_inv_inv", T_inv_inv, 4, 4);

	/* Assert */
	int idx = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			MU_CHECK(fltcmp(T_inv_inv[idx], T[idx]) == 0);
		}
	}

	return 0;
}

int test_tf_point() {
  /* Transform */
  const double T[16] = {1.0, 0.0, 0.0, 1.0,
								   			0.0, 1.0, 0.0, 2.0,
								   			0.0, 0.0, 1.0, 3.0,
								   			0.0, 0.0, 0.0, 1.0};
  print_matrix("T", T, 4, 4);

  /* Point */
  double p[3] = {1.0, 2.0, 3.0};
  print_vector("p", p, 3);

  /* Transform point */
  double result[3] = {0};
	tf_point(T, p, result);
  print_vector("result", result, 3);

  return 0;
}

int test_tf_hpoint() {
  /* Transform */
  const double T[16] = {1.0, 0.0, 0.0, 1.0,
								   			0.0, 1.0, 0.0, 2.0,
								   			0.0, 0.0, 1.0, 3.0,
								   			0.0, 0.0, 0.0, 1.0};
  print_matrix("T", T, 4, 4);

  /* Homogeneous point */
  double hp[4] = {1.0, 2.0, 3.0, 1.0};
  print_vector("hp", hp, 4);

  /* Transform homogeneous point */
  double result[4] = {0};
	tf_hpoint(T, hp, result);
  print_vector("result", result, 4);

  return 0;
}

int test_euler321() {
	/* Euler to rotation matrix */
	const double euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
	double C[9] = {0};
	euler321(euler, C);

	/* Rotation matrix to quaternion */
	double q[4] = {0};
	rot2quat(C, q);

	/* Quaternion to Euler angles*/
	double euler2[3] = {0};
	quat2euler(q, euler2);

  print_vector("euler", euler, 3);
  print_vector("euler2", euler2, 3);

	return 0;
}

int test_rot2quat() {
	/* Rotation matrix to quaternion */
  const double C[9] = {1.0, 0.0, 0.0,
								  		 0.0, 1.0, 0.0,
								  		 0.0, 0.0, 1.0};
	double q[4] = {0.0};
	rot2quat(C, q);
  print_vector("q", q, 4);

	return 0;
}

int test_quat2euler() {
  const double C[9] = {1.0, 0.0, 0.0,
											 0.0, 1.0, 0.0,
											 0.0, 0.0, 1.0};

	/* Rotation matrix to quaternion */
	double q[4] = {0.0};
	rot2quat(C, q);
  print_vector("q", q, 4);

	/* Quaternion to Euler angles */
	double rpy[3] = {0.0};
	quat2euler(q, rpy);
  print_vector("euler", rpy, 3);

	return 0;
}

int test_quat2rot() {
	/* Euler to rotation matrix */
	const double euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
	double C[9] = {0};
	euler321(euler, C);

	/* Rotation matrix to quaternion */
	double q[4] = {0.0};
	rot2quat(C, q);
  /* print_vector("q", q, 4); */

	/* Quaternion to rotation matrix */
	double rot[9] = {0.0};
	quat2rot(q, rot);

	for (int i = 0; i < 9; i++) {
		MU_CHECK(fltcmp(C[i], rot[i]) == 0);
	}

	return 0;
}

void test_suite() {
	/* Linear Algebra */
  MU_ADD_TEST(test_eye);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_mat_set);
  MU_ADD_TEST(test_mat_val);
  MU_ADD_TEST(test_mat_block);
  MU_ADD_TEST(test_mat_transpose);
  MU_ADD_TEST(test_mat_add);
  MU_ADD_TEST(test_mat_sub);
  MU_ADD_TEST(test_vec_add);
  MU_ADD_TEST(test_vec_sub);
  MU_ADD_TEST(test_dot);

	/* Transforms */
  MU_ADD_TEST(test_tf_set_rot);
  MU_ADD_TEST(test_tf_set_trans);
  MU_ADD_TEST(test_tf_trans);
  MU_ADD_TEST(test_tf_rot);
  MU_ADD_TEST(test_tf_quat);
  MU_ADD_TEST(test_tf_inv);
  MU_ADD_TEST(test_tf_point);
  MU_ADD_TEST(test_tf_hpoint);
  MU_ADD_TEST(test_euler321);
  MU_ADD_TEST(test_rot2quat);
  MU_ADD_TEST(test_quat2euler);
  MU_ADD_TEST(test_quat2rot);
}

MU_RUN_TESTS(test_suite);
