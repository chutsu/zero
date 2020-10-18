#include "zero/zero.h"

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
    return -1;
  }

  /* Loop through lines */
  int nb_rows = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
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
    return -1;
  }

  /* Get line that isn't the header */
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  /* Parse line to obtain number of elements */
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < MAX_LINE_LENGTH; i++) {
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
  char field_line[MAX_LINE_LENGTH] = {0};
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
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
  assert(fp != NULL);

  /* Obtain number of rows and columns in csv data */
  *nb_rows = csv_rows(fp);
  *nb_cols = csv_cols(fp);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for csv data */
  double **data = malloc(sizeof(double *) * *nb_rows);
  for (int i = 0; i < *nb_rows; i++) {
    data[i] = malloc(sizeof(double) * *nb_cols);
  }

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
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

  /* Clean up */
  fclose(infile);

  return data;
}

static int *parse_iarray_line(char *line) {
  char entry[MAX_LINE_LENGTH] = {0};
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

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
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
  char entry[MAX_LINE_LENGTH] = {0};
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

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
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

double pythag(const double a, const double b) {
  double at = fabs(a);
  double bt = fabs(b);
  double ct = 0.0;
  double result = 0.0;

  if (at > bt) {
    ct = bt / at;
    result = at * sqrt(1.0 + ct * ct);
  } else if (bt > 0.0) {
    ct = at / bt;
    result = bt * sqrt(1.0 + ct * ct);
  } else {
    result = 0.0;
  }

  return result;
}

double lerpd(const double a, const double b, const double t) {
  return a * (1.0 - t) + b * t;
}

void lerp3d(const double *a, const double *b, const double t, double *x) {
  x[0] = lerpf(a[0], b[0], t);
  x[1] = lerpf(a[1], b[1], t);
  x[2] = lerpf(a[2], b[2], t);
}

float lerpf(const float a, const float b, const float t) {
  return a * (1.0 - t) + b * t;
}

void lerp3f(const float *a, const float *b, const float t, float *x) {
  x[0] = lerpf(a[0], b[0], t);
  x[1] = lerpf(a[1], b[1], t);
  x[2] = lerpf(a[2], b[2], t);
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
      printf("%.4f\t", data[idx]);
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
    printf("%.4f\t", data[idx]);
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

double *mat_new(const size_t m, const size_t n) {
  return calloc(m * n, sizeof(double));
}

int mat_cmp(const double *A, const double *B, const size_t m, const size_t n) {
  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      int retval = fltcmp(A[index], B[index]);
      if (retval != 0) {
        printf("Failed at index[%zu]\n", index);
        return retval;
      }
      index++;
    }
  }

  return 0;
}

int mat_equals(const double *A, const double *B,
               const size_t m, const size_t n,
               const double thresh) {
  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(A[index] - B[index]) > thresh) {
        printf("Failed at index[%zu]\n", index);
        return -1;
      }
      index++;
    }
  }

  return 0;
}

int mat_save(const char *save_path, const double *A, const int m, const int n) {
  FILE *csv_file = fopen(save_path, "w");
  if (csv_file == NULL) {
    return -1;
  }

  int idx = 0;
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      fprintf(csv_file, "%e", A[idx]);
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

double *mat_load(const char *mat_path, int *nb_rows, int *nb_cols) {
  /* Obtain number of rows and columns in csv data */
  *nb_rows = csv_rows(mat_path);
  *nb_cols = csv_cols(mat_path);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for csv data */
  double *A = malloc(sizeof(double) * *nb_rows * *nb_cols);

  /* Load file */
  FILE *infile = fopen(mat_path, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;
  int idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        A[idx] = strtod(entry, NULL);
        idx++;

        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Clean up */
  fclose(infile);

  return A;
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

void mat_triu(const double *A, const size_t n, double *U) {
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      U[i * n + j] = (j >= i) ? A[i * n + j] : 0.0;
    }
  }
}

void mat_tril(const double *A, const size_t n, double *L) {
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      L[i * n + j] = (j <= i) ? A[i * n + j] : 0.0;
    }
  }
}

double mat_trace(const double *A, const size_t m, const size_t n) {
  double tr = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      tr += (i == j) ? A[i * n + j] : 0.0;
    }
  }
  return tr;
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

void vec_copy(const double *src, const size_t length, double *dest) {
  for (size_t i = 0; i < length; i++) {
    dest[i] = src[i];
  }
}

int vec_equals(const double *x, const double *y, const size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (fltcmp(x[i], y[i]) != 0) {
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
    z[i] = x[i] - y[i];
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
      for (size_t k = 0; k < A_n; k++) {
        C[(i * n) + j] += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
    }
  }
}

void skew(const double x[3], double A[3 * 3]) {
  /* First row */
  A[0] = 0.0;
  A[1] = -x[2];
  A[2] = x[1];

  /* Second row */
  A[3] = x[2];
  A[4] = 0.0;
  A[5] = -x[0];

  /* Third row */
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

int check_jacobian(const char *jac_name,
                   const double *fdiff,
                   const double *jac,
									 const size_t m,
									 const size_t n,
                   const double threshold,
									 const int print) {
  int retval = 0;
  int ok = 1;
	double *delta = mat_new(m, n);
	mat_sub(fdiff, jac, delta, m, n);

  // Check if any of the values are beyond the threshold
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(mat_val(delta, n, i, j)) >= threshold) {
        ok = 0;
      }
    }
  }

  // Print result
  if (ok == 0) {
    if (print) {
      LOG_ERROR("Bad jacobian [%s]!\n", jac_name);
			print_matrix("analytical jac", jac, m, n);
			printf("\n");
			print_matrix("num diff jac", fdiff, m, n);
			printf("\n");
			print_matrix("difference matrix", delta, m, n);
			printf("\n");
    }
    retval = -1;
  } else {
    if (print) {
      printf("Check [%s] ok!\n", jac_name);
    }
    retval = 0;
  }

  return retval;
}

/******************************************************************************
 *                                  SVD
 ******************************************************************************/

/* int svd(double *A, int m, int n, double *U, double *s, double *V_t) { */
/*   const int lda = n; */
/*   const int ldu = m; */
/*   const int ldvt = n; */
/*   const char jobu = 'A'; */
/*   const char jobvt = 'A'; */
/*   const int superb_size = (m < n) ? m : n; */
/*   double *superb = malloc(sizeof(double) * (superb_size - 1)); */
/*   int retval = LAPACKE_dgesvd(LAPACK_ROW_MAJOR, */
/*                               jobu, */
/*                               jobvt, */
/*                               m, */
/*                               n, */
/*                               A, */
/*                               lda, */
/*                               s, */
/*                               U, */
/*                               ldu, */
/*                               V_t, */
/*                               ldvt, */
/*                               superb); */
/*   if (retval > 0) { */
/*     return -1; */
/*   } */
/*  */
/*   #<{(| Clean up |)}># */
/*   free(superb); */
/*  */
/*   return 0; */
/* } */

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
  /* assert(m < n); */
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
      for (k = i; k < m; k++) {
        scale += fabs(A[k * n + i]);
      }

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
            for (s = 0.0, k = i; k < m; k++) {
              s += (A[k * n + i] * A[k * n + j]);
            }
            f = s / h;
            for (k = i; k < m; k++) {
              A[k * n + j] += (f * A[k * n + i]);
            }
          }
        }

        for (k = i; k < m; k++) {
          A[k * n + i] = (A[k * n + i] * scale);
        }
      }
    }
    w[i] = (scale * g);

    /* right-hand reduction */
    g = s = scale = 0.0;
    if (i < m && i != n - 1) {
      for (k = l; k < n; k++) {
        scale += fabs(A[i * n + k]);
      }

      if (scale) {
        for (k = l; k < n; k++) {
          A[i * n + k] = (A[i * n + k] / scale);
          s += (A[i * n + k] * A[i * n + k]);
        }

        f = A[i * n + l];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        A[i * n + l] = (f - g);

        for (k = l; k < n; k++) {
          rv1[k] = A[i * n + k] / h;
        }

        if (i != m - 1) {
          for (j = l; j < m; j++) {
            for (s = 0.0, k = l; k < n; k++) {
              s += (A[j * n + k] * A[i * n + k]);
            }
            for (k = l; k < n; k++) {
              A[j * n + k] += (s * rv1[k]);
            }
          }
        }
        for (k = l; k < n; k++)
          A[i * n + k] = (A[i * n + k] * scale);
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  /* Accumulate the right-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    if (i < n - 1) {
      if (g) {
        for (j = l; j < n; j++) {
          V[j * n + i] = ((A[i * n + j] / A[i * n + l]) / g);
        }
        /* double division to avoid underflow */
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++) {
            s += (A[i * n + k] * V[k * n + j]);
          }
          for (k = l; k < n; k++) {
            V[k * n + j] += (s * V[k * n + i]);
          }
        }
      }
      for (j = l; j < n; j++) {
        V[i * n + j] = V[j * n + i] = 0.0;
      }
    }
    V[i * n + i] = 1.0;
    g = rv1[i];
    l = i;
  }

  /* accumulate the left-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    l = i + 1;
    g = w[i];
    if (i < n - 1) {
      for (j = l; j < n; j++) {
        A[i * n + j] = 0.0;
      }
    }
    if (g) {
      g = 1.0 / g;
      if (i != n - 1) {
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < m; k++) {
            s += (A[k * n + i] * A[k * n + j]);
          }
          f = (s / A[i * n + i]) * g;

          for (k = i; k < m; k++) {
            A[k * n + j] += (f * A[k * n + i]);
          }
        }
      }
      for (j = i; j < m; j++) {
        A[j * n + i] = (A[j * n + i] * g);
      }
    } else {
      for (j = i; j < m; j++) {
        A[j * n + i] = 0.0;
      }
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

      /* Shift from bottom 2 x 2 minor */
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

/* int pinv(double *A, const int m, const int n, double *A_inv) { */
/*   #<{(| Decompose A with SVD |)}># */
/*   double *U = malloc(sizeof(double) * m * n); */
/*   double *d = malloc(sizeof(double) * n); */
/*   double *V_t = malloc(sizeof(double) * n * n); */
/*   if (svd(A, m, n, U, d, V_t) != 0) { */
/*     return -1; */
/*   } */
/*  */
/*   #<{(| Form reciprocal singular matrix S_inv from singular vector d |)}># */
/*   double *S_inv = malloc(sizeof(double) * n * n); */
/*   zeros(S_inv, n, n); */
/*   int mat_index = 0; */
/*   int vec_index = 0; */
/*   for (int i = 0; i < n; i++) { */
/*     for (int j = 0; j < n; j++) { */
/*       if (i == j) { */
/*         S_inv[mat_index] = 1.0 / d[vec_index]; */
/*         vec_index++; */
/*       } */
/*       mat_index++; */
/*     } */
/*   } */
/*  */
/*   #<{(| pinv(H) = V S^-1 U' |)}># */
/*   double *V = malloc(sizeof(double) * n * n); */
/*   mat_transpose(V_t, n, n, V); */
/*  */
/*   double *U_t = malloc(sizeof(double) * n * n); */
/*   mat_transpose(U, n, n, U_t); */
/*  */
/*   double *VSi = malloc(sizeof(double) * n * n); */
/*   dot(V, n, n, S_inv, n, n, VSi); */
/*   dot(VSi, n, n, U_t, n, n, A_inv); */
/*  */
/*   #<{(| Clean up |)}># */
/*   free(U); */
/*   free(U_t); */
/*   free(d); */
/*   free(S_inv); */
/*   free(V); */
/*   free(V_t); */
/*   free(VSi); */
/*  */
/*   return 0; */
/* } */

/******************************************************************************
 *                                  CHOL
 ******************************************************************************/

double *chol(const double *A, const size_t n) {
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
  double *L = chol(A, n);
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

#ifdef USE_LAPACK
void chol_lls_solve2(const double *A,
                     const double *b,
                     double *x,
                     const size_t m) {
  /* Cholesky Decomposition */
  int info = 0;
  int lda = m;
  int n = m;
  char uplo = 'L';
  double *a = mat_new(m, m);
  mat_copy(A, m, m, a);
  dpotrf_(&uplo, &n, a, &lda, &info);
  if (info != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }

  /* Solve Ax = b using Cholesky decomposed A from above */
  vec_copy(b, m, x);
  int nhrs = 1;
  int ldb = m;
  dpotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
  if (info != 0) {
    fprintf(stderr, "Failed to solve Ax = b!\n");
  }

  free(a);
}
#endif

/******************************************************************************
 *                               TRANSFORMS
 ******************************************************************************/

void tf(const double C[3 * 3], const double r[3], double T[4 * 4]) {
  assert(C != NULL);
  assert(r != NULL);
  assert(T != NULL);

  T[0] = C[0]; T[1] = C[1]; T[2] = C[2];  T[3] = r[0];
  T[4] = C[3]; T[5] = C[4]; T[6] = C[5];  T[7] = r[1];
  T[8] = C[6]; T[9] = C[7]; T[10] = C[8]; T[11] = r[2];
  T[12] = 0.0; T[13] = 0.0; T[14] = 0.0;  T[15] = 1.0;
}

void tf_rot_set(double T[4 * 4], const double C[3 * 3]) {
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

void tf_trans_set(double T[4 * 4], const double r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  T[3] = r[0];
  T[7] = r[1];
  T[11] = r[2];
}

void tf_trans_get(const double T[4 * 4], double r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

void tf_rot_get(const double T[4 * 4], double C[3 * 3]) {
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

void tf_quat_get(const double T[4 * 4], double q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  double C[3 * 3] = {0};
  tf_rot_get(T, C);
  rot2quat(C, q);
}

void tf_inv(const double T[4 * 4], double T_inv[4 * 4]) {
  assert(T != NULL);
  assert(T_inv != NULL);
  assert(T != T_inv);

  /* Get original rotation and translation component */
  double C[3 * 3] = {0};
  double r[3] = {0};
  tf_rot_get(T, C);
  tf_trans_get(T, r);

  /* Invert rotation component */
  double C_inv[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_inv);

  /* Set rotation component */
  tf_rot_set(T_inv, C_inv);

  /* Set translation component */
  double r_inv[3] = {0};
  mat_scale(C_inv, 3, 3, -1.0);
  dot(C_inv, 3, 3, r, 3, 1, r_inv);
  tf_trans_set(T_inv, r_inv);

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

void tf_perturb_rot(double T[4 * 4], const double step_size, const int i) {
	/* Build perturb drvec */
	double drvec[3] = {0};
	drvec[i] = step_size;

	/* Decompose transform to rotation and translation */
	double C[3 * 3] = {0};
	tf_rot_get(T, C);

	/* Perturb rotation */
	double C_rvec[3 * 3] = {0};
	double C_diff[3 * 3] = {0};
	rvec2rot(drvec, 1e-8, C_rvec);
	dot(C_rvec, 3, 3, C, 3, 3, C_diff);
  tf_rot_set(T, C_diff);
}

void tf_perturb_trans(double T[4 * 4], const double step_size, const int i) {
	/* Build perturb dr */
	double dr[3] = {0};
	dr[i] = step_size;

	/* Decompose transform get translation */
	double r[3] = {0};
	tf_trans_get(T, r);

	/* Perturb translation */
  const double r_diff[3] = {r[0] + dr[0], r[1] + dr[1], r[2] + dr[2]};
	tf_trans_set(T, r_diff);
}

void rvec2rot(const double *rvec, const double eps, double *R) {
  /* Magnitude of rvec */
  const double theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1]);
  // ^ basically norm(rvec), but faster

  /* Check if rotation is too small */
  if (theta < eps) {
		R[0] = 1.0;
		R[1] = -rvec[2];
		R[2] = rvec[1];

    R[3] = rvec[2];
		R[4] = 1.0;
		R[5] = -rvec[0];

    R[6] = -rvec[1];
		R[7] = rvec[0],
		R[8] = 1.0;
		return;
  }

  /* Convert rvec to rotation matrix */
  double rvec_normed[3] = {rvec[0], rvec[1], rvec[2]};
	vec_scale(rvec_normed, 3, 1 / theta);
  const double x = rvec_normed[0];
  const double y = rvec_normed[1];
  const double z = rvec_normed[2];

  const double c = cos(theta);
  const double s = sin(theta);
  const double C = 1 - c;

  const double xs = x * s;
  const double ys = y * s;
  const double zs = z * s;

  const double xC = x * C;
  const double yC = y * C;
  const double zC = z * C;

  const double xyC = x * yC;
  const double yzC = y * zC;
  const double zxC = z * xC;

  R[0] = x * xC + c;
	R[1] = xyC - zs;
	R[2] =  zxC + ys;

  R[3] = xyC + zs;
	R[4] = y * yC + c;
	R[5] = yzC - xs;

	R[6] = zxC - ys;
	R[7] = yzC + xs;
	R[8] = z * zC + c;
}

void euler321(const double euler[3], double C[3 * 3]) {
  assert(euler != NULL);
  assert(C != NULL);

  const double phi = euler[0];
  const double theta = euler[1];
  const double psi = euler[2];

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

void pose_init(pose_t *pose,
               const timestamp_t ts,
               const double q[4],
               const double r[3]) {
  assert(pose != NULL);
  assert(q != NULL);
  assert(r != NULL);

  pose->ts = ts;
  pose_set_quat(pose, q);
  pose_set_trans(pose, r);
}

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

void pose_get_quat(const pose_t *pose, double q[4]) {
  assert(pose != NULL);
  assert(q != NULL);

  q[0] = pose->q[0];
  q[1] = pose->q[1];
  q[2] = pose->q[2];
  q[3] = pose->q[3];
}

void pose_get_trans(const pose_t *pose, double r[3]) {
  assert(pose != NULL);
  assert(r != NULL);

  r[0] = pose->r[0];
  r[1] = pose->r[1];
  r[2] = pose->r[2];
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
  tf_rot_set(T, C);
  tf_trans_set(T, pose->r);
}

pose_t *load_poses(const char *csv_path, int *nb_poses) {
  assert(csv_path != NULL);
  assert(nb_poses != NULL);

  FILE *csv_file = fopen(csv_path, "r");
  char line[MAX_LINE_LENGTH] = {0};
  *nb_poses = csv_rows(csv_path);
  pose_t *poses = malloc(sizeof(pose_t) * *nb_poses);

  int pose_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[MAX_LINE_LENGTH] = {0};
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

/*****************************************************************************
 *                                  IMAGE
 *****************************************************************************/

void image_init(image_t *img, uint8_t *data, int width, int height) {
  img->data = data;
  img->width = width;
  img->height = height;
}

/*****************************************************************************
 *                                  PINHOLE
 *****************************************************************************/

void pinhole_K(const double fx,
               const double fy,
               const double cx,
               const double cy,
               double K[9]) {
  K[0] = fx;
  K[1] = 0.0;
  K[2] = cx;
  K[3] = 0.0;
  K[4] = fy;
  K[5] = cy;
  K[6] = 0.0;
  K[7] = 0.0;
  K[8] = 1.0;
}

double pinhole_focal_length(const int image_width, const double fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

int pinhole_project(const double K[9], const double p_C[3], double x[2]) {
  const double fx = K[0];
  const double fy = K[4];
  const double cx = K[2];
  const double cy = K[5];

  const double px = p_C[0] / p_C[2];
  const double py = p_C[1] / p_C[2];

  x[0] = px * fx + cx;
  x[1] = py * fy + cy;

  return 0;
}

void pinhole_calc_K(const double image_width,
                    const double image_height,
                    const double lens_hfov,
                    const double lens_vfov,
                    double K[9]) {
  const double fx = pinhole_focal_length(image_width, lens_hfov);
  const double fy = pinhole_focal_length(image_height, lens_vfov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  return pinhole_K(fx, fy, cx, cy, K);
}

/*****************************************************************************
 *                                 RADTAN
 *****************************************************************************/

void radtan4_distort(const double k1,
                     const double k2,
                     const double p1,
                     const double p2,
                     const double p[2],
                     double p_d[2]) {
  /* Point */
  const double x = p[0];
  const double y = p[1];

  /* Apply radial distortion */
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;
  const double radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const double x_dash = x * radial_factor;
  const double y_dash = y * radial_factor;

  /* Apply tangential distortion */
  const double xy = x * y;
  const double x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const double y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

  /* Distorted point */
  p_d[0] = x_ddash;
  p_d[1] = y_ddash;
}

void radtan4_point_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_point[2 * 2]) {
  /* Point */
  const double x = p[0];
  const double y = p[1];

  /* Apply radial distortion */
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

  /* Point Jacobian is 2x2 */
  /* clang-format off */
  J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x +
               x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point[2] = J_point[1];
  J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x +
               y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
  /* clang-format on */
}

void radtan4_param_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_param[2 * 4]) {
  UNUSED(k1);
  UNUSED(k2);
  UNUSED(p1);
  UNUSED(p2);

  /* Point */
  const double x = p[0];
  const double y = p[1];

  /* Setup */
  const double x2 = x * x;
  const double y2 = y * y;
  const double xy = x * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * r2;
  J_param[1] = x * r4;
  J_param[2] = 2 * xy;
  J_param[3] = 3 * x2 + y2;

  J_param[4] = y * r2;
  J_param[5] = y * r4;
  J_param[6] = x2 + 3 * y2;
  J_param[7] = 2 * xy;
}

/*****************************************************************************
 *                                  EQUI
 *****************************************************************************/

void equi4_distort(const double k1,
                   const double k2,
                   const double k3,
                   const double k4,
                   const double p[2],
                   double p_d[2]) {
  const double x = p[0];
  const double y = p[1];
  const double r = sqrt(x * x + y * y);

  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double s = thd / r;

  const double x_dash = s * x;
  const double y_dash = s * y;

  p_d[0] = x_dash;
  p_d[1] = y_dash;
}

void equi4_point_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_point[2 * 2]) {
  const double x = p[0];
  const double y = p[1];
  const double r = sqrt(x * x + y * y);

  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const double th_r = 1.0 / (r * r + 1.0);
  const double thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const double s = thd / r;
  const double s_r = thd_th * th_r / r - thd / (r * r);
  const double r_x = 1.0 / r * x;
  const double r_y = 1.0 / r * y;

  /* Point Jacobian is 2x2 */
  J_point[0] = s + x * s_r * r_x;
  J_point[1] = x * s_r * r_y;
  J_point[2] = y * s_r * r_x;
  J_point[3] = s + y * s_r * r_y;
}

void equi4_param_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_param[2 * 4]) {
  UNUSED(k1);
  UNUSED(k2);
  UNUSED(k3);
  UNUSED(k4);

  const double x = p[0];
  const double y = p[1];
  const double r = sqrt(x * x + y * y);

  const double th = atan(r);
  const double th2 = th * th;
  const double th3 = th2 * th;
  const double th5 = th3 * th2;
  const double th7 = th5 * th2;
  const double th9 = th7 * th2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * th3 / r;
  J_param[1] = x * th5 / r;
  J_param[2] = x * th7 / r;
  J_param[3] = x * th9 / r;

  J_param[4] = y * th3 / r;
  J_param[5] = y * th5 / r;
  J_param[6] = y * th7 / r;
  J_param[7] = y * th9 / r;
}
