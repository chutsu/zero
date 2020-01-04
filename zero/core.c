#include "zero/core.h"

/******************************************************************************
 *                                   DATA
 ******************************************************************************/

char *malloc_string(const char *s) {
  char *retval = malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
}

int csv_rows(const char *fp) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return -1;
  }

  /* Loop through lines */
  int nb_rows = 0;
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      nb_rows++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return nb_rows;
}

int csv_cols(const char *fp) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return -1;
  }

  /* Get line that isn't the header */
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  /* Parse line to obtain number of elements */
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < len_max; i++) {
    if (line[i] == ',') {
      found_separator = 1;
      nb_elements++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

char **csv_fields(const char *fp, int *nb_fields) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Get last header line */
  char field_line[1024] = {0};
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      strcpy(field_line, line);
    }
  }

  /* Parse fields */
  *nb_fields = csv_cols(fp);
  char **fields = malloc(sizeof(char *) * *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    /* Ignore # and ' ' */
    if (c == '#' || c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      /* Add field name to fields */
      fields[field_idx] = malloc_string(field_name);
      memset(field_name, '\0', sizeof(char) * 100);
      field_idx++;
    } else {
      /* Append field name */
      field_name[strlen(field_name)] = c;
    }
  }

  /* Cleanup */
  fclose(infile);

  return fields;
}

double **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  /* Obtain number of rows and columns in csv data */
  *nb_rows = csv_rows(fp);
  *nb_cols = csv_cols(fp);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for csv data */
  double **data = malloc(sizeof(double *) * *nb_rows);
  for (int i = 0; i < *nb_cols; i++) {
    data[i] = malloc(sizeof(double) * *nb_cols);
  }

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[1024] = {0};
  size_t len_max = 1024;
  int row_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[row_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Cleanup */
  fclose(infile);

  return data;
}

static int *parse_iarray_line(char *line) {
  char entry[1024] = {0};
  int index = 0;
  int *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = calloc(array_size + 1, sizeof(int));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

int **load_iarrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = csv_rows(csv_path);
  int **array = calloc(*nb_arrays, sizeof(int *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_iarray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

static double *parse_darray_line(char *line) {
  char entry[1024] = {0};
  int index = 0;
  double *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = calloc(array_size, sizeof(double));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

double **load_darrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = csv_rows(csv_path);
  double **array = calloc(*nb_arrays, sizeof(double *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_darray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

/******************************************************************************
 *                                 GENERAL
 ******************************************************************************/

float randf(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

float toc(struct timespec *tic) {
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

double deg2rad(const double d) { return d * (M_PI / 180.0); }

double rad2deg(const double r) { return r * (180.0 / M_PI); }

int fltcmp(const double x, const double y) {
  if (fabs(x - y) < 1e-6) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

double lerpd(const double a, const double b, const double t) {
  return a * (1.0 - t) + b * t;
}

float lerpf(const float a, const float b, const float t) {
  return a * (1.0 - t) + b * t;
}

double sinc(const double x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    static const double c_2 = 1.0 / 6.0;
    static const double c_4 = 1.0 / 120.0;
    static const double c_6 = 1.0 / 5040.0;
    const double x_2 = x * x;
    const double x_4 = x_2 * x_2;
    const double x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

/******************************************************************************
 *                              LINEAR ALGEBRA
 ******************************************************************************/

void print_matrix(const char *prefix,
                  const double *data,
                  const size_t m,
                  const size_t n) {
  assert(prefix != NULL);
  assert(data != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0;
  printf("%s:\n", prefix);
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      printf("%f\t", data[idx]);
      idx++;
    }
    printf("\n");
  }
}

void print_vector(const char *prefix, const double *data, const size_t length) {
  assert(prefix != NULL);
  assert(data != NULL);
  assert(length != 0);

  size_t idx = 0;
  printf("%s: ", prefix);
  for (size_t i = 0; i < length; i++) {
    printf("%f\t", data[idx]);
    idx++;
  }
  printf("\n");
}

void eye(double *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = (i == j) ? 1.0 : 0.0;
      idx++;
    }
  }
}

void ones(double *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 1.0;
      idx++;
    }
  }
}

void zeros(double *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 0.0;
      idx++;
    }
  }
}

double *mat_new(const int m, const int n) {
  return calloc(m * n, sizeof(double));
}

int mat_save(const char *save_path, const double *A, const int m, const int n) {
  FILE *csv_file = fopen(save_path, "w");
  if (csv_file == NULL) {
    return -1;
  }

  int idx = 0;
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      fprintf(csv_file, "%f", A[idx]);
      idx++;
      if ((j + 1) != n) {
        fprintf(csv_file, ",");
      }
    }
    fprintf(csv_file, "\n");
  }
  fclose(csv_file);

  return 0;
}

void mat_set(double *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const double val) {
  assert(A != NULL);
  assert(stride != 0);

  A[(i * stride) + j] = val;
}

double
mat_val(const double *A, const size_t stride, const size_t i, const size_t j) {
  assert(A != NULL);
  assert(stride != 0);
  return A[(i * stride) + j];
}

void mat_copy(const double *src, const int m, const int n, double *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

void mat_block_get(const double *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   double *block) {
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      block[idx] = mat_val(A, stride, i, j);
      idx++;
    }
  }
}

void mat_block_set(double *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   const double *block) {
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      mat_set(A, stride, i, j, block[idx]);
      idx++;
    }
  }
}

void mat_diag_set(double *A, const int m, const int n, const double *d) {
  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        A[mat_index] = d[vec_index];
        vec_index++;
      } else {
        A[mat_index] = 0.0;
      }
      mat_index++;
    }
  }
}

void mat_diag_get(const double *A, const int m, const int n, double *d) {
  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        d[vec_index] = A[mat_index];
        vec_index++;
      }
      mat_index++;
    }
  }
}

void mat_transpose(const double *A, size_t m, size_t n, double *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A_t, m, j, i, mat_val(A, n, i, j));
    }
  }
}

int mat_equals(const double *A, const double *B, const int m, const size_t n) {
  for (size_t i = 0; i < (m * n); i++) {
    if (fltcmp(A[i], B[i]) != 0) {
      return -1;
    }
  }

  return 0;
}

void mat_add(const double *A, const double *B, double *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(C, n, i, j, mat_val(A, n, i, j) + mat_val(B, n, i, j));
    }
  }
}

void mat_sub(const double *A, const double *B, double *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(C, n, i, j, mat_val(A, n, i, j) - mat_val(B, n, i, j));
    }
  }
}

void mat_scale(double *A, const size_t m, const size_t n, const double scale) {
  assert(A != NULL);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A, n, i, j, mat_val(A, n, i, j) * scale);
    }
  }
}

double *vec_new(const size_t length) { return calloc(length, sizeof(double)); }

int vec_equals(const double *a, const double *b, const size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (fltcmp(a[i], b[i]) != 0) {
      return -1;
    }
  }
  return 0;
}

void vec_add(const double *x, const double *y, double *z, size_t length) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(length > 0);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] + y[i];
  }
}

void vec_sub(const double *x, const double *y, double *z, size_t length) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(length > 0);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] + y[i];
  }
}

void vec_scale(double *x, const size_t length, const double scale) {
  for (size_t i = 0; i < length; i++) {
    x[i] = x[i] * scale;
  }
}

double vec_norm(const double *x, const size_t length) {
  double sum = 0.0;
  for (size_t i = 0; i < length; i++) {
    sum += x[i] * x[i];
  }
  return sqrt(sum);
}

void dot(const double *A,
         const size_t A_m,
         const size_t A_n,
         const double *B,
         const size_t B_m,
         const size_t B_n,
         double *C) {
  assert(A != NULL && B != NULL && A != C && B != C);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  size_t m = A_m;
  size_t n = B_n;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      double sum = 0.0;
      for (size_t k = 0; k < A_n; k++) {
        sum += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
      C[(i * n) + j] = sum;
    }
  }
}

void skew(const double x[3], double A[3 * 3]) {
  A[0] = 0.0;
  A[1] = -x[2];
  A[2] = x[1];
  A[3] = x[2];
  A[4] = 0.0;
  A[5] = -x[0];
  A[6] = -x[1];
  A[7] = x[0];
  A[8] = 0.0;
}

void fwdsubs(const double *L, const double *b, double *y, const size_t n) {
  for (size_t i = 0; i < n; i++) {
    double alpha = b[i];
    for (size_t j = 0; j < i; j++) {
      alpha -= L[i * n + j] * y[j];
    }
    y[i] = alpha / L[i * n + i];
  }
}

void bwdsubs(const double *U, const double *y, double *x, const size_t n) {
  for (int i = n - 1; i >= 0; i--) {
    double alpha = y[i];
    for (int j = i; j < (int) n; j++) {
      alpha -= U[i * n + j] * x[j];
    }
    x[i] = alpha / U[i * n + i];
  }
}

/******************************************************************************
 *                              TRANSFORMS
 ******************************************************************************/

void tf_set_rot(double T[4 * 4], const double C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];
  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];
  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
}

void tf_set_trans(double T[4 * 4], const double r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  T[3] = r[0];
  T[7] = r[1];
  T[11] = r[2];
}

void tf_trans(const double T[4 * 4], double r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

void tf_rot(const double T[4 * 4], double C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];
  C[3] = T[4];
  C[4] = T[5];
  C[5] = T[6];
  C[6] = T[8];
  C[7] = T[9];
  C[8] = T[10];
}

void tf_quat(const double T[4 * 4], double q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  double C[3 * 3] = {0};
  tf_rot(T, C);
  rot2quat(C, q);
}

void tf_inv(const double T[4 * 4], double T_inv[4 * 4]) {
  assert(T != NULL);
  assert(T_inv != NULL);
  assert(T != T_inv);

  /* Get original rotation and translation component */
  double C[3 * 3] = {0};
  double r[3] = {0};
  tf_rot(T, C);
  tf_trans(T, r);

  /* Invert rotation component */
  double C_inv[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_inv);

  /* Set rotation component */
  tf_set_rot(T_inv, C_inv);

  /* Set translation component */
  double r_inv[3] = {0};
  mat_scale(C_inv, 3, 3, -1.0);
  dot(C_inv, 3, 3, r, 3, 1, r_inv);
  tf_set_trans(T_inv, r_inv);

  /* Make sure the last element is 1 */
  T_inv[15] = 1.0;
}

void tf_point(const double T[4 * 4], const double p[3], double retval[3]) {
  assert(T != NULL);
  assert(p != NULL);
  assert(retval != NULL);
  assert(p != retval);

  const double hp_a[4] = {p[0], p[1], p[2], 1.0};
  double hp_b[4] = {0.0, 0.0, 0.0, 0.0};
  dot(T, 4, 4, hp_a, 4, 1, hp_b);

  retval[0] = hp_b[0];
  retval[1] = hp_b[1];
  retval[2] = hp_b[2];
}

void tf_hpoint(const double T[4 * 4], const double hp[4], double retval[4]) {
  assert(T != NULL);
  assert(hp != retval);
  dot(T, 4, 4, hp, 4, 1, retval);
}

void euler321(const double euler[3], double C[3 * 3]) {
  assert(euler != NULL);
  assert(C != NULL);

  const float phi = euler[0];
  const float theta = euler[1];
  const float psi = euler[2];

  /* 1st row */
  C[0] = cos(psi) * cos(theta);
  C[1] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  C[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  /* 2nd row */
  C[3] = sin(psi) * cos(theta);
  C[4] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  C[5] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  /* 3rd row */
  C[6] = -sin(theta);
  C[7] = cos(theta) * sin(phi);
  C[8] = cos(theta) * cos(phi);
}

void rot2quat(const double C[3 * 3], double q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const double C00 = C[0];
  const double C01 = C[1];
  const double C02 = C[2];
  const double C10 = C[3];
  const double C11 = C[4];
  const double C12 = C[5];
  const double C20 = C[6];
  const double C21 = C[7];
  const double C22 = C[8];

  const double tr = C00 + C11 + C22;
  double S = 0.0f;
  double qw = 0.0f;
  double qx = 0.0f;
  double qy = 0.0f;
  double qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

void quat2euler(const double q[4], double euler[3]) {
  assert(q != NULL);
  assert(euler != NULL);

  const float qw = q[0];
  const float qx = q[1];
  const float qy = q[2];
  const float qz = q[3];

  const float qw2 = qw * qw;
  const float qx2 = qx * qx;
  const float qy2 = qy * qy;
  const float qz2 = qz * qz;

  const float t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const float t2 = asin(2 * (qy * qw - qx * qz));
  const float t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  euler[0] = t1;
  euler[1] = t2;
  euler[2] = t3;
}

void quat2rot(const double q[4], double C[3 * 3]) {
  assert(q != NULL);
  assert(C != NULL);

  const double qw = q[0];
  const double qx = q[1];
  const double qy = q[2];
  const double qz = q[3];

  const double qx2 = qx * qx;
  const double qy2 = qy * qy;
  const double qz2 = qz * qz;
  const double qw2 = qw * qw;

  /* Homogeneous form */
  /* -- 1st row */
  C[0] = qw2 + qx2 - qy2 - qz2;
  C[1] = 2 * (qx * qy - qw * qz);
  C[2] = 2 * (qx * qz + qw * qy);
  /* -- 2nd row */
  C[3] = 2 * (qx * qy + qw * qz);
  C[4] = qw2 - qx2 + qy2 - qz2;
  C[5] = 2 * (qy * qz - qw * qx);
  /* -- 3rd row */
  C[6] = 2 * (qx * qz - qw * qy);
  C[7] = 2 * (qy * qz + qw * qx);
  C[8] = qw2 - qx2 - qy2 + qz2;
}

void quatlmul(const double p[4], const double q[4], double r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);

  const double pw = p[0];
  const double px = p[1];
  const double py = p[2];
  const double pz = p[3];

  /* clang-format off */
  const double lprod[4*4] = {
    pw, -px, -py, -pz,
    px, pw, -pz, py,
    py, pz, pw, -px,
    pz, -py, px, pw
  };
  /* clang-format on */

  dot(lprod, 4, 4, q, 4, 1, r);
}

void quatrmul(const double p[4], const double q[4], double r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);

  const double qw = q[0];
  const double qx = q[1];
  const double qy = q[2];
  const double qz = q[3];

  /* clang-format off */
  const double rprod[4*4] = {
    qw, -qx, -qy, -qz,
    qx, qw, qz, -qy,
    qy, -qz, qw, qx,
    qz, qy, -qx, qw
  };
  /* clang-format on */

  dot(rprod, 4, 4, p, 4, 1, r);
}

void quatmul(const double p[4], const double q[4], double r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);
  quatlmul(p, q, r);
}

void quatdelta(const double dalpha[3], double dq[4]) {
  const double half_norm = 0.5 * vec_norm(dalpha, 3);
  const double k = sinc(half_norm) * 0.5;
  const double vector[3] = {k * dalpha[0], k * dalpha[0], k * dalpha[0]};
  double scalar = cos(half_norm);

  dq[0] = scalar;
  dq[1] = vector[0];
  dq[2] = vector[1];
  dq[3] = vector[2];
}

/******************************************************************************
 *                                   POSE
 ******************************************************************************/

void pose_set_quat(pose_t *pose, const double q[4]) {
  assert(pose != NULL);
  assert(q != NULL);

  pose->q[0] = q[0];
  pose->q[1] = q[1];
  pose->q[2] = q[2];
  pose->q[3] = q[3];
}

void pose_set_trans(pose_t *pose, const double r[3]) {
  assert(pose != NULL);
  assert(r != NULL);

  pose->r[0] = r[0];
  pose->r[1] = r[1];
  pose->r[2] = r[2];
}

void pose_print(const char *prefix, const pose_t *pose) {
  assert(prefix != NULL);
  assert(pose != NULL);

  if (prefix) {
    printf("[%s] ", prefix);
  }
  printf("q: (%f, %f, %f, %f)", pose->q[0], pose->q[1], pose->q[2], pose->q[3]);
  printf("\t");
  printf("r: (%f, %f, %f)\n", pose->r[0], pose->r[1], pose->r[2]);
}

void pose2tf(const pose_t *pose, double T[4 * 4]) {
  assert(pose != NULL);
  assert(T != NULL);

  double C[3 * 3] = {0};
  quat2rot(pose->q, C);

  eye(T, 4, 4);
  tf_set_rot(T, C);
  tf_set_trans(T, pose->r);
}

pose_t *load_poses(const char *csv_path, int *nb_poses) {
  assert(csv_path != NULL);
  assert(nb_poses != NULL);

  FILE *csv_file = fopen(csv_path, "r");
  char line[1024] = {0};
  *nb_poses = csv_rows(csv_path);
  pose_t *poses = malloc(sizeof(pose_t) * *nb_poses);

  int pose_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[1024] = {0};
    double data[7] = {0};
    int index = 0;
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[index] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        index++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    poses[pose_idx].q[0] = data[0];
    poses[pose_idx].q[1] = data[1];
    poses[pose_idx].q[2] = data[2];
    poses[pose_idx].q[3] = data[3];

    poses[pose_idx].r[0] = data[4];
    poses[pose_idx].r[1] = data[5];
    poses[pose_idx].r[2] = data[6];

    pose_idx++;
  }
  fclose(csv_file);

  return poses;
}
