#include "munit.h"
#include "zero/gui.h"

/*******************************************************************************
 *                                  UTILS
 ******************************************************************************/

int test_gl_zeros() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0};

  gl_zeros(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  MU_CHECK(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_ones() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0};

  gl_ones(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  MU_CHECK(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_eye() {
  GLfloat A[3*4] = {0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0};
  GLfloat expected[3*4] = {1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0,
                           0.0, 0.0, 0.0};

  gl_eye(A, 3, 4);
  gl_print_matrix("A", A, 3, 4);
  MU_CHECK(gl_equals(A, expected, 3, 4, 1e-8));

  return 0;
}

int test_gl_equals() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat C[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 10.0};

  /* Assert */
  MU_CHECK(gl_equals(A, B, 3, 3, 1e-8) == 1);
  MU_CHECK(gl_equals(A, C, 3, 3, 1e-8) == 0);

  return 0;
}

int test_gl_vec3_cross() {
  const GLfloat u[3] = {1.0f, 2.0f, 3.0f};
  const GLfloat v[3] = {4.0f, 5.0f, 6.0f};
  GLfloat z[3] = {0};
  gl_vec3f_cross(u, v, z);

  /* Assert */
  GLfloat expected[3] = {-3.0f, 6.0f, -3.0f};
  gl_print_vector("z", z, 3);
  gl_print_vector("expected", z, 3);
  MU_CHECK(gl_equals(z, expected, 3, 1, 1e-8));

  return 0;
}

int test_gl_dot() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat C[3*3] = {0.0};
  gl_dot(A, 3, 3, B, 3, 3, C);

  /* Assert */
  GLfloat expected[3*3] = {30.0f, 66.0f, 102.0f,
                           36.0f, 81.0f, 126.0f,
                           42.0f, 96.0f, 150.0f};
  gl_print_matrix("C", C, 3, 3);
  gl_print_matrix("expected", expected, 3, 3);
  MU_CHECK(gl_equals(C, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_norm() {
  const GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  const GLfloat n = gl_norm(x, 3);

  /* Assert */
  const GLfloat expected = 3.741657f;
  MU_CHECK(fabs(n - expected) < 1e-6);

  return 0;
}

int test_gl_normalize() {
  GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  gl_normalize(x, 3);

  /* Assert */
  const GLfloat expected[3] = {0.26726f, 0.53452f, 0.80178f};
  MU_CHECK(gl_equals(x, expected, 3, 1, 1e-5));

  return 0;
}

/*******************************************************************************
 *                                  SHADER
 ******************************************************************************/

int test_shader_compile() {

  return 0;
}

int test_shader_link() {

  return 0;
}

/*******************************************************************************
 *                                GL PROGRAM
 ******************************************************************************/

int test_glprog_setup() {

  return 0;
}

/*******************************************************************************
 *                                   GUI
 ******************************************************************************/

int test_gui_setup() {
  int argc = 0;
  char **argv = NULL;
  gui_setup(argc, argv);
  return 0;
}

void test_suite() {
  /* UTILS */
  MU_ADD_TEST(test_gl_zeros);
  MU_ADD_TEST(test_gl_ones);
  MU_ADD_TEST(test_gl_eye);
  MU_ADD_TEST(test_gl_equals);
  MU_ADD_TEST(test_gl_vec3_cross);
  MU_ADD_TEST(test_gl_dot);
  MU_ADD_TEST(test_gl_norm);
  MU_ADD_TEST(test_gl_normalize);

  /* SHADER */
  MU_ADD_TEST(test_shader_compile);
  MU_ADD_TEST(test_shader_link);

  /* GLPROGRAM */
  MU_ADD_TEST(test_glprog_setup);

  /* GUI */
  MU_ADD_TEST(test_gui_setup);
}

MU_RUN_TESTS(test_suite)
