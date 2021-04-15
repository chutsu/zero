#include "zero/zero.h"

/******************************************************************************
 *                               FILE SYSTEM
 ******************************************************************************/

/**
 * List files in directory.
 *
 * @param[in] path Path to directory
 * @param[out] nb_files Number of files in directory
 * @returns List of files in directory
 */
char **list_files(const char *path, int *nb_files) {
  struct dirent **namelist;
  int N = scandir(path, &namelist, 0, alphasort);
  if (N < 0) {
    return NULL;
  }

  /* The first two are '.' and '..' */
  free(namelist[0]);
  free(namelist[1]);

  /* Allocate memory for list of files */
  char **files = malloc(sizeof(char *) * (N - 2));
  *nb_files = 0;

  /* Create list of files */
  for (int i = 2; i < N; i++) {
    char fp[9046] = {0};
    strcat(fp, path);
    strcat(fp, (fp[strlen(fp) - 1] == '/') ? "" : "/");
    strcat(fp, namelist[i]->d_name);

    files[*nb_files] = malloc(sizeof(char) * (strlen(fp) + 1));
    strcpy(files[*nb_files], fp);
    (*nb_files)++;

    free(namelist[i]);
  }
  free(namelist);

  return files;
}

/**
 * Free list of files.
 *
 * @param[in] data List of files
 * @param[in] n Number of files
 */
void list_files_free(char **data, const int n) {
  for (int i = 0; i < n; i++) {
    free(data[i]);
  }
  free(data);
}

/**
 * Read file contents.
 *
 * @param[in] fp Path to file
 * @returns
 * - Success: File contents
 * - Failure: NULL
 */
char *file_read(const char *fp) {
  FILE *f = fopen(fp, "rb");
  if (f == NULL) {
    return NULL;
  }

  fseek(f, 0, SEEK_END);
  long int length = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *buffer = malloc(length);
  if (buffer) {
    fread(buffer, 1, length, f);
  }
  fclose(f);

  return buffer;
}

/**
 * Skip line in file.
 *
 * @param[in,out] fp Pointer to file
 */
void skip_line(FILE *fp) {
  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!");
  }
}

/**
 * Get number of rows in file.
 *
 * @returns Number of rows in file else -1 for failure.
 */
int file_rows(const char *fp) {
  FILE *file = fopen(fp, "rb");
  if (file == NULL) {
    fclose(file);
    return -1;
  }

  /* Obtain number of lines */
  int nb_rows = 0;
  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, file) != -1) {
    nb_rows++;
  }

  return nb_rows;
}

/**
 * Copy file from path src to path dest.
 *
 * @param[in] src Source file
 * @param[in] dest Destination file
 *
 * @returns
 * - 0 for success
 * - -1 if src file could not be opend
 * - -2 if dest file could not be opened
 */
int file_copy(const char *src, const char *dest) {
  FILE *src_file = fopen(src, "rb");
  if (src_file == NULL) {
    fclose(src_file);
    return -1;
  }

  FILE *dest_file = fopen(dest, "wb");
  if (dest_file == NULL) {
    fclose(src_file);
    fclose(dest_file);
    return -2;
  }

  /* BUFSIZE default is 8192 bytes */
  /* BUFSIZE of 1 means one chareter at time */
  char buf[BUFSIZ];
  size_t read = 0;
  while ((read = fread(buf, 1, BUFSIZ, src_file)) == 0) {
    fwrite(buf, 1, read, dest_file);
  }

  /* Clean up */
  fclose(src_file);
  fclose(dest_file);

  return 0;
}

/******************************************************************************
 *                                   DATA
 ******************************************************************************/

/**
 * Allocate heap memory for string s
 *
 * @param[in] s String to copy from
 * @returns A heap memory allocated string
 */
char *malloc_string(const char *s) {
  char *retval = malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
}

/**
 * Get number of rows in a delimited file
 *
 * @param[in] fp Path to file
 * @returns
 * - Number of rows
 * - -1 for failure
 */
int dsv_rows(const char *fp) {
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

/**
 * Get number of columns in a delimited file fp.
 *
 * @param[in] fp Path to file
 * @param[in] delim Delimiter
 *
 * @returns
 * - Number of columns
 * - -1 for failure
 */
int dsv_cols(const char *fp, const char delim) {
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
    if (line[i] == delim) {
      found_separator = 1;
      nb_elements++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

/**
 * Get the fields of the delimited file.
 *
 * @param[in] fp Path to file
 * @param[in] delim Delimiter
 * @param[out] nb_fields Number of fields
 *
 * @returns
 * - List of field strings
 * - NULL for failure
 */
char **dsv_fields(const char *fp, const char delim, int *nb_fields) {
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
  *nb_fields = dsv_cols(fp, delim);
  char **fields = malloc(sizeof(char *) * *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    /* Ignore # and ' ' */
    if (c == '#' || c == delim) {
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

/**
 * Load delimited separated value data as a matrix.
 *
 * @param[in] fp Path to file
 * @param[in] delim Delimiter
 * @param[out] nb_rows Number of rows
 * @param[out] nb_cols Number of cols
 *
 * @returns
 * - Matrix of DSV data
 * - NULL for failure
 */
real_t **dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols) {
  assert(fp != NULL);

  /* Obtain number of rows and columns in dsv data */
  *nb_rows = dsv_rows(fp);
  *nb_cols = dsv_cols(fp, delim);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for dsv data */
  real_t **data = malloc(sizeof(real_t *) * *nb_rows);
  for (int i = 0; i < *nb_rows; i++) {
    data[i] = malloc(sizeof(real_t) * *nb_cols);
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

/**
 * Load delimited separated value data as a matrix.
 *
 * @param[in] fp Path to file
 * @param[out] nb_rows Number of rows
 * @param[out] nb_cols Number of cols
 *
 * @returns
 * - Matrix of CSV data
 * - NULL for failure
 */
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  return dsv_data(fp, ',', nb_rows, nb_cols);
}

/**
 * Parse integer array line.
 *
 * @param[in] line Line to parse
 *
 * @returns
 * - 1D vector of integers
 * - NULL for failure
 */
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

/**
 * Parse 2D integer arrays from csv file.
 *
 * @param[in] csv_path Path to csv file
 * @param[out] nb_arrays Number of integer arrays
 *
 * @returns
 * - List of 1D vector of integers
 * - NULL for failure
 */
int **load_iarrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
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

/**
 * Parse real array line.
 *
 * @param[in] line Line to parse
 *
 * @returns
 * - 1D vector of real
 * - NULL for failure
 */
static real_t *parse_darray_line(char *line) {
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  real_t *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = calloc(array_size, sizeof(real_t));
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

/**
 * Parse 2D real arrays from csv file.
 *
 * @param[in] csv_path Path to csv file
 * @param[out] nb_arrays Number of real arrays
 *
 * @returns
 * - List of 1D vector of reals
 * - NULL for failure
 */
real_t **load_darrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
  real_t **array = calloc(*nb_arrays, sizeof(real_t *));

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

/**
 * Load real vector from file
 *
 * @param[in] file_path Path to file
 * @returns
 * - Real vector
 * - Null for failure
 */
real_t *load_vector(const char *file_path) {
  /* Open file */
  FILE *csv = fopen(file_path, "r");
  if (csv == NULL) {
    return NULL;
  }

  /* Get number of lines in file */
  size_t nb_lines = 0;
  char buf[MAX_LINE_LENGTH] = {0};
  while (fgets(buf, MAX_LINE_LENGTH, csv)) {
    nb_lines++;
  }
  rewind(csv);

  /* Load vector */
  real_t *v = malloc(sizeof(real_t) * nb_lines);
  for (size_t i = 0; i < nb_lines; i++) {
#if PRECISION == 1
    int retval = fscanf(csv, "%f", &v[i]);
#elif PRECISION == 2
    int retval = fscanf(csv, "%le", &v[i]);
#endif
    if (retval != 1) {
      return NULL;
    }
  }
  fclose(csv);

  return v;
}

/******************************************************************************
 *                                   TIME
 ******************************************************************************/

/**
 * Tic, start timer.
 *
 * @returns A timespec struct encapsulating the time instance when tic() is
 * called
 */
struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

/**
 * Toc, stop timer.
 *
 * @param[in] tic Time at start
 * @returns Time elapsed in seconds
 */
float toc(struct timespec *tic) {
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

/**
 * Toc, stop timer.
 *
 * @param[in] tic Time at start
 * @returns Time elapsed in milli-seconds
 */
float mtoc(struct timespec *tic) { return toc(tic) * 1000.0; }

/**
 * Get time now in seconds since epoch.
 * @return Time now in seconds since epoch
 */
float time_now() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((float) t.tv_sec + ((float) t.tv_usec) / 1000000.0);
}


/******************************************************************************
 *                                 MATHS
 ******************************************************************************/

/**
 * Generate random number between a and b from a uniform distribution.
 *
 * @param[in] a Lower bound
 * @param[in] b Upper bound
 * @returns Random number
 */
float randf(const float a, const float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

/**
 * Degrees to radians.
 * @param[in] d Degrees
 * @returns Radians
 */
real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

/**
 * Radians to degrees.
 * @param[in] r Radians
 * @returns Degrees
 */
real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

/**
 * Compare reals.
 *
 * @param[in] x First number
 * @param[in] y Second number
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int fltcmp(const real_t x, const real_t y) {
  if (fabs(x - y) < 1e-6) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

/**
 * Pythagoras
 *
 *   c = sqrt(a^2 + b^2)
 *
 * @param[in] a
 * @param[in] b
 * @returns Hypotenuse of a and b
 */
real_t pythag(const real_t a, const real_t b) {
  real_t at = fabs(a);
  real_t bt = fabs(b);
  real_t ct = 0.0;
  real_t result = 0.0;

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

/**
 * 1D Linear interpolation.
 *
 * @param[in] a First value
 * @param[in] b Second value
 * @param[in] t Interpolation parameter
 * @returns Linear interpolated value between a and b
 */
real_t lerp(const real_t a, const real_t b, const real_t t) {
  return a * (1.0 - t) + b * t;
}

/**
 * 3D Linear interpolation.
 *
 * @param[in] a First vector
 * @param[in] b Second vector
 * @param[in] t Interpolation parameter
 * @param[out] x 3D linear interpolated vector between a and b
 */
void lerp3(const real_t *a, const real_t *b, const real_t t, real_t *x) {
  x[0] = lerp(a[0], b[0], t);
  x[1] = lerp(a[1], b[1], t);
  x[2] = lerp(a[2], b[2], t);
}

/**
 * Sinc.
 * @param[in] x
 * @return Result of sinc
 */
real_t sinc(const real_t x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    const real_t c2 = 1.0 / 6.0;
    const real_t c4 = 1.0 / 120.0;
    const real_t c6 = 1.0 / 5040.0;
    const real_t x2 = x * x;
    const real_t x4 = x2 * x2;
    const real_t x6 = x2 * x2 * x2;
    return 1.0 - c2 * x2 + c4 * x4 - c6 * x6;
  }
}

/**
 * Calculate mean from vector x of length length.
 *
 * @param[in] x 1D vector
 * @param[in] length Length of x
 * @returns Mean of x
 */
real_t mean(const real_t* x, const size_t length) {
  real_t sum = 0.0;
  for (size_t i = 0; i < length; i++) {
    sum += x[i];
  }
  real_t N = length;
  return sum / N;
}

/**
 * Calculate median from vector x of length length.
 *
 * @param[in] x 1D vector
 * @param[in] length Length of x
 * @returns Median of x
 */
real_t median(const real_t* x, const size_t length) {
  /* qsort(x, length, sizeof(real_t), fltcmp); */
  return 0;
}

/**
 * Calculate variance from vector x of length length.
 *
 * @param[in] x 1D vector
 * @param[in] length Length of x
 * @returns Variance of x
 */
real_t var(const real_t *x, const size_t length) {
  real_t mu = mean(x, length);

  real_t sse = 0.0;
  for (size_t i = 0; i < length; i++) {
    sse += (x[i] - mu) * (x[i] - mu);
  }

  return length;
}

/**
 * Calculate standard deviation from vector x of length length.
 *
 * @param[in] x 1D vector
 * @param[in] length Length of x
 * @returns Standard deviation of x
 */
real_t stddev(const real_t *x, const size_t length) {
  return sqrt(var(x, length));
}


/******************************************************************************
 *                              LINEAR ALGEBRA
 ******************************************************************************/

/**
 * Print matrix A of size m x n.
 *
 * @param[in] prefix Name for the matrix
 * @param[in] A Matrix
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 */
void print_matrix(const char *prefix,
                  const real_t *A,
                  const size_t m,
                  const size_t n) {
  assert(prefix != NULL);
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0;
  printf("%s:\n", prefix);
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      printf("%.4f\t", A[idx]);
      idx++;
    }
    printf("\n");
  }
  printf("\n");
}

/**
 * Print vector v of length n
 *
 * @param[in] prefix Name for the matrix
 * @param[in] v Vector
 * @param[in] n Length of vector
 */
void print_vector(const char *prefix, const real_t *v, const size_t n) {
  assert(prefix != NULL);
  assert(v != NULL);
  assert(n != 0);

  size_t idx = 0;
  printf("%s: ", prefix);
  for (size_t i = 0; i < n; i++) {
    printf("%.4f\t", v[idx]);
    idx++;
  }
  printf("\n");
}

/**
 * Form identity matrix.
 *
 * @param[out] A Matrix
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 */
void eye(real_t *A, const size_t m, const size_t n) {
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

/**
 * Form ones matrix.
 *
 * @param[out] A Matrix
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 */
void ones(real_t *A, const size_t m, const size_t n) {
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

/**
 * Form zeros matrix.
 *
 * @param[out] A Matrix
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 */
void zeros(real_t *A, const size_t m, const size_t n) {
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

/**
 * Malloc matrix.
 *
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 * @returns Heap allocated matrix
 */
real_t *mat_new(const size_t m, const size_t n) {
  return calloc(m * n, sizeof(real_t));
}

/**
 * Compare two matrices A and B of size m x n.
 *
 * @param[in] A First matrix
 * @param[in] B Second matrix
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 *
 * @returns
 * - 0 if A == B
 * - 1 if A > B
 * - -1 if A < B
 */
int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n) {
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

/**
 * Check to see if two matrices A and B of size m x n are equal.
 *
 * @param[in] A First matrix
 * @param[in] B Second matrix
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 * @param[in] tol Tolerance
 *
 * @returns
 * - 0 if A == B
 * - -1 if A != B
 */
int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol) {
  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(A[index] - B[index]) > tol) {
        printf("Failed at index[%zu]\n", index);
        return -1;
      }
      index++;
    }
  }

  return 0;
}

/**
 * Save matrix A of size m x n to save_path.
 *
 * @param[in] save_path Path to save matrix
 * @param[in] A First matrix
 * @param[in] m Number of rows
 * @param[in] n Number of cols
 *
 * @returns
 * - 0 Success
 * - -1 Failure
 */
int mat_save(const char *save_path, const real_t *A, const int m, const int n) {
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

/**
 * Load matrix from file in mat_path.
 *
 * @param[in] mat_path Path to matrix file
 * @param[out] nb_rows Number of rows
 * @param[out] nb_cols Number of cols
 * @returns Heap allocated matrix
 */
real_t *mat_load(const char *mat_path, int *nb_rows, int *nb_cols) {
  /* Obtain number of rows and columns in csv data */
  *nb_rows = dsv_rows(mat_path);
  *nb_cols = dsv_cols(mat_path, ',');
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for csv data */
  real_t *A = malloc(sizeof(real_t) * *nb_rows * *nb_cols);

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

/**
 * Set matrix value at (i, j).
 *
 * @param[in,out] A Matrix
 * @param[in] stride
 * @param[in] i
 * @param[in] j
 * @param[in] val
 */
void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val) {
  assert(A != NULL);
  assert(stride != 0);
  A[(i * stride) + j] = val;
}

/**
 * Get matrix value at (i, j).
 *
 * @param[in] A Matrix
 * @param[in] stride
 * @param[in] i
 * @param[in] j
 *
 * @returns Matrix value at (i, j)
 */
real_t
mat_val(const real_t *A, const size_t stride, const size_t i, const size_t j) {
  assert(A != NULL);
  assert(stride != 0);
  return A[(i * stride) + j];
}

/**
 * Copy matrix src of size m x n to dest.
 *
 * @param[in] src Source matrix
 * @param[in] m Row dimension of src
 * @param[in] n Column dimension of src
 * @param[in] dest Destination matrix
 */
void mat_copy(const real_t *src, const int m, const int n, real_t *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

/**
 * Get matrix sub-block in A.
 *
 * @param[in] A Input matrix
 * @param[in] stride Stride
 * @param[in] rs Row start
 * @param[in] cs Column start
 * @param[in] re Row end
 * @param[in] ce Column end
 * @param[out] block Matrix sub-block
 */
void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   real_t *block) {
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

/**
 * Set matrix sub-block in A.
 *
 * @param[in,out] A Target matrix
 * @param[in] stride Stride
 * @param[in] rs Row start
 * @param[in] cs Column start
 * @param[in] re Row end
 * @param[in] ce Column end
 * @param[in] block Matrix sub-block
 */
void mat_block_set(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   const real_t *block) {
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

/**
 * Get diagonal vector from matrix A.
 *
 * @param[in] A Target matrix
 * @param[in] m Row dimension of matrix A
 * @param[in] n Column dimension of matrix A
 * @param[out] d Return diagonal vector of A
 */
void mat_diag_get(const real_t *A, const int m, const int n, real_t *d) {
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

/**
 * Set the diagonal of matrix A.
 *
 * @param[in,out] A Target matrix
 * @param[in] m Row dimension of matrix A
 * @param[in] n Column dimension of matrix A
 * @param[in] d Diagonal vector
 */
void mat_diag_set(real_t *A, const int m, const int n, const real_t *d) {
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

/**
 * Get upper triangular matrix of A.
 *
 * @param[in] A Target matrix
 * @param[in] n Column dimension of matrix A
 * @param[out] U Upper triangular matrix of A
 */
void mat_triu(const real_t *A, const size_t n, real_t *U) {
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      U[i * n + j] = (j >= i) ? A[i * n + j] : 0.0;
    }
  }
}

/**
 * Get lower triangular matrix of A.
 *
 * @param[in] A Target matrix
 * @param[in] n Column dimension of matrix A
 * @param[out] L Lower triangular matrix of A
 */
void mat_tril(const real_t *A, const size_t n, real_t *L) {
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      L[i * n + j] = (j <= i) ? A[i * n + j] : 0.0;
    }
  }
}

/**
 * Get the trace matrix of A.
 *
 * @param[in] A Target matrix
 * @param[in] m Row dimension of matrix A
 * @param[in] n Column dimension of matrix A
 *
 * @returns Trace of matrix A
 */
real_t mat_trace(const real_t *A, const size_t m, const size_t n) {
  real_t tr = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      tr += (i == j) ? A[i * n + j] : 0.0;
    }
  }
  return tr;
}

/**
 * Transpose of matrix A.
 *
 * @param[in] A Target matrix
 * @param[in] m Row dimension of matrix A
 * @param[in] n Column dimension of matrix A
 * @param[out] A_t Transpose of matrix A
 */
void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A_t, m, j, i, mat_val(A, n, i, j));
    }
  }
}

/**
 * Sum two matrices A and B.
 *
 *    C = A + B
 *
 * @param[in] A First matrix
 * @param[in] B Second matrix
 * @param[out] C Result matrix
 * @param[in] m Row dimension of matrix A
 * @param[in] n Column dimension of matrix A
 */
void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    C[i] = A[i] + B[i];
  }
}

/**
 * Subtract two matrices A and B.
 *
 *    C = A - B
 *
 * @param[in] A First matrix
 * @param[in] B Second matrix
 * @param[out] C Result matrix
 * @param[in] m Row dimension of matrix A
 * @param[in] n Column dimension of matrix A
 */
void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    C[i] = A[i] - B[i];
  }
}

/**
 * Scale matrix A inplace with a scale factor.
 *
 *     A_new = scale * A
 *
 * @param[in,out] A Target matrix
 * @param[in] m Row dimension of matrix A
 * @param[in] n Column dimension of matrix A
 * @param[in] scale Scale scalar
 */
void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale) {
  assert(A != NULL);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    A[i] = A[i] * scale;
  }
}

/**
 * Create new vector in heap memory.
 *
 * @param[in] length Length of vector
 * @returns Heap allocated vector
 */
real_t *vec_new(const size_t length) { return calloc(length, sizeof(real_t)); }

/**
 * Copy vector.
 *
 * @param[in] src Source vector
 * @param[in] length Length of source vector
 * @param[out] dest Destination vector
 */
void vec_copy(const real_t *src, const size_t length, real_t *dest) {
  for (size_t i = 0; i < length; i++) {
    dest[i] = src[i];
  }
}

/**
 * Check if vectors x and y are equal.
 *
 * @param[in] x First vector
 * @param[in] y Second vector
 * @param[in] length Length of source vector
 *
 * @returns
 * - 1 for x == 1
 * - 0 for x != 1
 */
int vec_equals(const real_t *x, const real_t *y, const size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (fltcmp(x[i], y[i]) != 0) {
      return 0;
    }
  }

  return 1;
}

/**
 * Sum two vectors.
 *
 * @param[in] x First vector
 * @param[in] y Second vector
 * @param[out] z Result vector
 * @param[in] length Length of vector x and y
 */
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t length) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(length > 0);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] + y[i];
  }
}

/**
 * Subtract two vectors.
 *
 * @param[in] x First vector
 * @param[in] y Second vector
 * @param[out] z Result vector
 * @param[in] length Length of vector x and y
 */
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t length) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(length > 0);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] - y[i];
  }
}

/**
 * Scale a vector in-place.
 *
 * @param[in,out] x Vector
 * @param[in] length Length of vector x
 * @param[in] scale Scale factor
 */
void vec_scale(real_t *x, const size_t length, const real_t scale) {
  for (size_t i = 0; i < length; i++) {
    x[i] = x[i] * scale;
  }
}

/**
 * Vector norm.
 *
 * @param[in] x Vector
 * @param[in] length Length of vector x
 * @returns Norm of vector x
 */
real_t vec_norm(const real_t *x, const size_t length) {
  real_t sum = 0.0;
  for (size_t i = 0; i < length; i++) {
    sum += x[i] * x[i];
  }
  return sqrt(sum);
}

/**
 * Dot product of A and B
 *
 * @param[in] A Matrix / vector
 * @param[in] A_m Row dimension of A
 * @param[in] A_n Column dimension of A
 * @param[in] B Matrix / vector
 * @param[in] B_m Row dimension of B
 * @param[in] B_n Column dimension of B
 * @param[out] C Result
 */
void dot(const real_t *A,
         const size_t A_m,
         const size_t A_n,
         const real_t *B,
         const size_t B_m,
         const size_t B_n,
         real_t *C) {
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

/**
 * Create skew-symmetric matrix.
 *
 * @param[in] x Vector of size 3
 * @param[out] A 3x3 Skew symmetric matrix
 */
void skew(const real_t x[3], real_t A[3 * 3]) {
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

/**
 * Forward substitution.
 *
 * @param[in] L Lower triangular matrix
 * @param[in] b Vector b
 * @param[out] y Vector y
 * @param[in] n Number of columns in L
 */
void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n) {
  for (size_t i = 0; i < n; i++) {
    real_t alpha = b[i];
    for (size_t j = 0; j < i; j++) {
      alpha -= L[i * n + j] * y[j];
    }
    y[i] = alpha / L[i * n + i];
  }
}

/**
 * Backward substitution.
 *
 * @param[in] U Upper triangular matrix
 * @param[in] y Vector y
 * @param[out] x Vector x
 * @param[in] n Number of columns in U
 */
void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n) {
  for (int i = n - 1; i >= 0; i--) {
    real_t alpha = y[i];
    for (int j = i; j < (int) n; j++) {
      alpha -= U[i * n + j] * x[j];
    }
    x[i] = alpha / U[i * n + i];
  }
}

/**
 * Check jacobian.
 *
 * @param[in] jac_name Jacobian name
 * @param[in] fdiff Finite difference
 * @param[in] jac Jacobian
 * @param[in] m Row dimension of Jacobian
 * @param[in] n Column dimension of Jacobian
 * @param[in] tol Tolerance
 * @param[in] verbose Verbose mode
 */
int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int verbose) {
  int retval = 0;
  int ok = 1;
  real_t *delta = mat_new(m, n);
  mat_sub(fdiff, jac, delta, m, n);

  /* Check if any of the values are beyond the tol */
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(mat_val(delta, n, i, j)) >= tol) {
        ok = 0;
      }
    }
  }

  /* Print result */
  if (ok == 0) {
    if (verbose) {
      LOG_ERROR("Bad jacobian [%s]!\n", jac_name);
      print_matrix("analytical jac", jac, m, n);
      print_matrix("num diff jac", fdiff, m, n);
      print_matrix("difference matrix", delta, m, n);
    }
    retval = -1;
  } else {
    if (verbose) {
      printf("Check [%s] ok!\n", jac_name);
    }
    retval = 0;
  }

  return retval;
}

#ifdef USE_CBLAS
/**
 * Dot product of A and B using CBLAS
 *
 * @param[in] A Matrix / vector
 * @param[in] A_m Row dimension of A
 * @param[in] A_n Column dimension of A
 * @param[in] B Matrix / vector
 * @param[in] B_m Row dimension of B
 * @param[in] B_n Column dimension of B
 * @param[out] C Result
 */
void cblas_dot(const real_t *A,
               const size_t A_m,
               const size_t A_n,
               const real_t *B,
               const size_t B_m,
               const size_t B_n,
               real_t *C) {
  UNUSED(B_m);
  assert(A != NULL && B != NULL && C != NULL);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

#if PRECISION == 1
  cblas_sgemm(CblasRowMajor, /* Matrix data arrangement */
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
              0.0,           /* Scale factor for C */
              C,             /* Output */
              B_n);          /* First dimension of C */
#elif PRECISION == 2
  cblas_dgemm(CblasRowMajor, /* Matrix data arrangement */
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
              0.0,           /* Scale factor for C */
              C,             /* Output */
              B_n);          /* First dimension of C */
#endif
}
#endif


/******************************************************************************
 *                                  SVD
 ******************************************************************************/

/* int svd(real_t *A, int m, int n, real_t *U, real_t *s, real_t *V_t) { */
/*   const int lda = n; */
/*   const int ldu = m; */
/*   const int ldvt = n; */
/*   const char jobu = 'A'; */
/*   const char jobvt = 'A'; */
/*   const int superb_size = (m < n) ? m : n; */
/*   real_t *superb = malloc(sizeof(real_t) * (superb_size - 1)); */
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
 * SVD decomposition

 * Takes an m x n matrix a and decomposes it into UDV, where U, V are left and
 * right orthogonal transformation matrices, and D is a diagonal matrix of
 * singular values.
 *
 * This routine is adapted from svdecomp.c in XLISP-STAT 2.1 which is
 * code from Numerical Recipes adapted by Luke Tierney and David Betz.
 *
 * @param[in,out] A An mxn matrix to be decomposed, gets overwritten with U
 * @param[in] m Row dimension of A
 * @param[in] n Column dimension of A
 * @param[out] w Returns the vector of singular values of a
 * @param[out] V Returns the right orthogonal transformation matrix
 *
 * @returns
 * - 0 for success
 * - -1 for failure
 */
int svdcomp(real_t *A, int m, int n, real_t *w, real_t *V) {
  /* assert(m < n); */
  int flag, i, its, j, jj, k, l, nm;
  real_t c, f, h, s, x, y, z;
  real_t anorm = 0.0, g = 0.0, scale = 0.0;

  /* Householder reduction to bidiagonal form */
  real_t *rv1 = malloc(sizeof(real_t) * n);
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
        /* real_t division to avoid underflow */
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

/* int pinv(real_t *A, const int m, const int n, real_t *A_inv) { */
/*   #<{(| Decompose A with SVD |)}># */
/*   real_t *U = malloc(sizeof(real_t) * m * n); */
/*   real_t *d = malloc(sizeof(real_t) * n); */
/*   real_t *V_t = malloc(sizeof(real_t) * n * n); */
/*   if (svd(A, m, n, U, d, V_t) != 0) { */
/*     return -1; */
/*   } */
/*  */
/*   #<{(| Form reciprocal singular matrix S_inv from singular vector d |)}># */
/*   real_t *S_inv = malloc(sizeof(real_t) * n * n); */
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
/*   real_t *V = malloc(sizeof(real_t) * n * n); */
/*   mat_transpose(V_t, n, n, V); */
/*  */
/*   real_t *U_t = malloc(sizeof(real_t) * n * n); */
/*   mat_transpose(U, n, n, U_t); */
/*  */
/*   real_t *VSi = malloc(sizeof(real_t) * n * n); */
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

/**
 * Cholesky decomposition.
 *
 * @param[in] A Square matrix of size n x n
 * @param[in] n Column dimension of A
 * @param[out] L Returns lower trianguloar matrix of A
 */
void chol(const real_t *A, const size_t n, real_t *L) {
  assert(A != NULL);
  assert(n > 0);
  /* real_t *L = calloc(n * n, sizeof(real_t)); */

  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < (i + 1); j++) {

      if (i == j) {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[j * n + k] * L[j * n + k];
        }
        L[i * n + j] = sqrt(A[i * n + i] - s);

      } else {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[i * n + k] * L[j * n + k];
        }
        L[i * n + j] = (1.0 / L[j * n + j] * (A[i * n + j] - s));
      }
    }
  }
}

/**
 * Solve Ax = b using Cholesky decomposition.
 *
 * @param[in] A Square matrix of size n x n
 * @param[in] b Vector of length n
 * @param[out] x Solution vector of length n
 * @param[in] n Column dimension of A
 */
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n) {
  /* Allocate memory */
  real_t *L = calloc(n * n, sizeof(real_t));
  real_t *Lt = calloc(n * n, sizeof(real_t));
  real_t *y = calloc(n, sizeof(real_t));

  /* Cholesky decomposition */
  chol(A, n, L);
  mat_transpose(L, n, n, Lt);

  /* Forward substitution */
  /* Ax = b -> LLt x = b. */
  /* Let y = Lt x, L y = b (Solve for y) */
  for (int i = 0; i < (int) n; i++) {
    real_t alpha = b[i];

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
    real_t alpha = y[i];

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
  free(y);
  free(L);
  free(Lt);
}

#ifdef USE_LAPACK
/**
 * Solve Ax = b using Cholesky decomposition.
 *
 * @param[in] A Square matrix of size n x n
 * @param[in] b Vector of length n
 * @param[out] x Solution vector of length n
 * @param[in] n Column dimension of A
 */
void lapack_chol_solve(const real_t *A,
                       const real_t *b,
                       real_t *x,
                       const size_t m) {
  /* Cholesky Decomposition */
  int info = 0;
  int lda = m;
  int n = m;
  char uplo = 'L';
  real_t *a = mat_new(m, m);
  mat_copy(A, m, m, a);
#if PRECISION == 1
  spotrf_(&uplo, &n, a, &lda, &info);
#elif PRECISION == 2
  dpotrf_(&uplo, &n, a, &lda, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }

  /* Solve Ax = b using Cholesky decomposed A from above */
  vec_copy(b, m, x);
  int nhrs = 1;
  int ldb = m;
#if PRECISION == 1
  spotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#elif PRECISION == 2
  dpotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to solve Ax = b!\n");
  }

  free(a);
}
#endif

/******************************************************************************
 *                               TRANSFORMS
 ******************************************************************************/

/**
 * Form 4x4 homogeneous transformation matrix.
 *
 * @param[in] params Pose parameters ([qw, qx, qy, qz] + [rx, ry, rz])
 * @param[out] T 4x4 Transformation matrix
 */
void tf(const real_t params[7], real_t T[4 * 4]) {
  assert(params != NULL);
  assert(T != NULL);

  const real_t q[4] = {params[0], params[1], params[2], params[3]};
  const real_t r[3] = {params[4], params[5], params[6]};

  real_t C[3 * 3] = {0};
  quat2rot(q, C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];
  T[3] = r[0];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];
  T[7] = r[1];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
  T[11] = r[2];

  T[12] = 0.0;
  T[13] = 0.0;
  T[14] = 0.0;
  T[15] = 1.0;
}

/**
 * Form pose parameter vector from 4x4 homogeneous transformation matrix.
 *
 * @param[in] T 4x4 Transformation matrix
 * @param[out] params Pose parameters ([qw, qx, qy, qz] + [rx, ry, rz])
 */
void tf_params(const real_t T[4 * 4], real_t params[7]) {
  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  real_t r[3] = {0};
  tf_trans_get(T, r);

  real_t q[4] = {0};
  rot2quat(C, q);

  params[0] = q[0];
  params[1] = q[1];
  params[2] = q[2];
  params[3] = q[3];

  params[4] = r[0];
  params[5] = r[1];
  params[6] = r[2];
}

/**
 * Set the rotational component in the 4x4 transformation matrix.
 *
 * @param[in,out] T 4x4 Transformation matrix
 * @param[in] C 3x3 Rotation matrix
 */
void tf_rot_set(real_t T[4 * 4], const real_t C[3 * 3]) {
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

/**
 * Set the translational component in the 4x4 transformation matrix.
 *
 * @param[in,out] T 4x4 Transformation matrix
 * @param[in] r 3x1 Translation vector
 */
void tf_trans_set(real_t T[4 * 4], const real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  T[3] = r[0];
  T[7] = r[1];
  T[11] = r[2];
}

/**
 * Get the translational component in the 4x4 transformation matrix.
 *
 * @param[in] T 4x4 Transformation matrix
 * @param[out] r 3x1 Translation vector
 */
void tf_trans_get(const real_t T[4 * 4], real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

/**
 * Get the rotational component in the 4x4 transformation matrix.
 *
 * @param[in] T 4x4 Transformation matrix
 * @param[out] C 3x3 Rotation matrix
 */
void tf_rot_get(const real_t T[4 * 4], real_t C[3 * 3]) {
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

/**
 * Get the quaternion from the 4x4 transformation matrix.
 *
 * @param[in] T 4x4 Transformation matrix
 * @param[out] q Quaternion
 */
void tf_quat_get(const real_t T[4 * 4], real_t q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);
  rot2quat(C, q);
}

/**
 * Invert the 4x4 homogeneous transformation matrix.
 *
 * @param[in] T 4x4 Transformation matrix
 * @param[out] T_inv Inverted 4x4 Transformation matrix
 */
void tf_inv(const real_t T[4 * 4], real_t T_inv[4 * 4]) {
  assert(T != NULL);
  assert(T_inv != NULL);
  assert(T != T_inv);

  /* Get original rotation and translation component */
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T, C);
  tf_trans_get(T, r);

  /* Invert rotation component */
  real_t C_inv[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_inv);

  /* Set rotation component */
  tf_rot_set(T_inv, C_inv);

  /* Set translation component */
  real_t r_inv[3] = {0};
  mat_scale(C_inv, 3, 3, -1.0);
  dot(C_inv, 3, 3, r, 3, 1, r_inv);
  tf_trans_set(T_inv, r_inv);

  /* Make sure the last element is 1 */
  T_inv[15] = 1.0;
}

/**
 * Transform point using 4x4 homogeneous transformation matrix.
 *
 * @param[in] T 4x4 Transformation matrix
 * @param[in] p 3x1 Point vector
 * @param[out] retval Transformed point
 */
void tf_point(const real_t T[4 * 4], const real_t p[3], real_t retval[3]) {
  assert(T != NULL);
  assert(p != NULL);
  assert(retval != NULL);
  assert(p != retval);

  const real_t hp_a[4] = {p[0], p[1], p[2], 1.0};
  real_t hp_b[4] = {0.0, 0.0, 0.0, 0.0};
  dot(T, 4, 4, hp_a, 4, 1, hp_b);

  retval[0] = hp_b[0];
  retval[1] = hp_b[1];
  retval[2] = hp_b[2];
}

/**
 * Transform homogeneous point using 4x4 homogeneous transformation matrix.
 *
 * @param[in] T 4x4 Transformation matrix
 * @param[in] hp 4x1 Homogeneous point vector
 * @param[out] retval Transformed homogeneous point
 */
void tf_hpoint(const real_t T[4 * 4], const real_t hp[4], real_t retval[4]) {
  assert(T != NULL);
  assert(hp != retval);
  dot(T, 4, 4, hp, 4, 1, retval);
}

/**
 * Perturb the rotational component of a 4x4 homogeneous transformation matrix.
 *
 * @param[in,out] T 4x4 transformation matrix
 * @param[in] step_size Step size
 * @param[in] i i-th parameter
 */
void tf_perturb_rot(real_t T[4 * 4], const real_t step_size, const int i) {
  /* Build perturb drvec */
  real_t drvec[3] = {0};
  drvec[i] = step_size;

  /* Decompose transform to rotation and translation */
  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  /* Perturb rotation */
  real_t C_rvec[3 * 3] = {0};
  real_t C_diff[3 * 3] = {0};
  rvec2rot(drvec, 1e-8, C_rvec);
  dot(C_rvec, 3, 3, C, 3, 3, C_diff);
  tf_rot_set(T, C_diff);
}

/**
 * Perturb the translation component of a 4x4 homogeneous transformation matrix.
 *
 * @param[in,out] T 4x4 transformation matrix
 * @param[in] step_size Step size
 * @param[in] i i-th parameter
 */
void tf_perturb_trans(real_t T[4 * 4], const real_t step_size, const int i) {
  /* Build perturb dr */
  real_t dr[3] = {0};
  dr[i] = step_size;

  /* Decompose transform get translation */
  real_t r[3] = {0};
  tf_trans_get(T, r);

  /* Perturb translation */
  const real_t r_diff[3] = {r[0] + dr[0], r[1] + dr[1], r[2] + dr[2]};
  tf_trans_set(T, r_diff);
}

/**
 * Convert rotation vector to 3x3 rotation matrix.
 *
 * @param[in] rvec Rotation vector
 * @param[in] eps Epsilon
 * @param[out] R 3x3 Rotation matrix
 */
void rvec2rot(const real_t *rvec, const real_t eps, real_t *R) {
  /* Magnitude of rvec */
  const real_t theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1]);
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
    R[7] = rvec[0], R[8] = 1.0;
    return;
  }

  /* Convert rvec to rotation matrix */
  real_t rvec_normed[3] = {rvec[0], rvec[1], rvec[2]};
  vec_scale(rvec_normed, 3, 1 / theta);
  const real_t x = rvec_normed[0];
  const real_t y = rvec_normed[1];
  const real_t z = rvec_normed[2];

  const real_t c = cos(theta);
  const real_t s = sin(theta);
  const real_t C = 1 - c;

  const real_t xs = x * s;
  const real_t ys = y * s;
  const real_t zs = z * s;

  const real_t xC = x * C;
  const real_t yC = y * C;
  const real_t zC = z * C;

  const real_t xyC = x * yC;
  const real_t yzC = y * zC;
  const real_t zxC = z * xC;

  R[0] = x * xC + c;
  R[1] = xyC - zs;
  R[2] = zxC + ys;

  R[3] = xyC + zs;
  R[4] = y * yC + c;
  R[5] = yzC - xs;

  R[6] = zxC - ys;
  R[7] = yzC + xs;
  R[8] = z * zC + c;
}

/**
 * Convert Euler angles to 3x3 rotation matrix.
 *
 * @param[in] euler Euler angles in radians
 * @param[out] C 3x3 Rotation matrix
 */
void euler321(const real_t euler[3], real_t C[3 * 3]) {
  assert(euler != NULL);
  assert(C != NULL);

  const real_t phi = euler[0];
  const real_t theta = euler[1];
  const real_t psi = euler[2];

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

/**
 * Convert 3x3 rotation matrix to Quaternion.
 *
 * @param[in] C 3x3 Rotation matrix
 * @param[out] q Quaternion
 */
void rot2quat(const real_t C[3 * 3], real_t q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const real_t C00 = C[0];
  const real_t C01 = C[1];
  const real_t C02 = C[2];
  const real_t C10 = C[3];
  const real_t C11 = C[4];
  const real_t C12 = C[5];
  const real_t C20 = C[6];
  const real_t C21 = C[7];
  const real_t C22 = C[8];

  const real_t tr = C00 + C11 + C22;
  real_t S = 0.0f;
  real_t qw = 0.0f;
  real_t qx = 0.0f;
  real_t qy = 0.0f;
  real_t qz = 0.0f;

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

/**
 * Convert Quaternion to Euler angles.
 *
 * @param[in] q Quaternion
 * @param[out] euler Euler angles in radians
 */
void quat2euler(const real_t q[4], real_t euler[3]) {
  assert(q != NULL);
  assert(euler != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qw2 = qw * qw;
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;

  const real_t t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const real_t t2 = asin(2 * (qy * qw - qx * qz));
  const real_t t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  euler[0] = t1;
  euler[1] = t2;
  euler[2] = t3;
}

/**
 * Convert Quaternion to 3x3 rotation matrix.
 *
 * @param[in] q Quaternion
 * @param[out] C 3x3 Rotation matrix.
 */
void quat2rot(const real_t q[4], real_t C[3 * 3]) {
  assert(q != NULL);
  assert(C != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

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

/**
 * Quaternion left-multiply.
 *
 * @param[in] p First quaternion
 * @param[in] q Second quaternion
 * @param[out] r Result
 */
void quat_lmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);

  const real_t pw = p[0];
  const real_t px = p[1];
  const real_t py = p[2];
  const real_t pz = p[3];

  /* clang-format off */
  const real_t lprod[4*4] = {
    pw, -px, -py, -pz,
    px, pw, -pz, py,
    py, pz, pw, -px,
    pz, -py, px, pw
  };
  /* clang-format on */

  cblas_dot(lprod, 4, 4, q, 4, 1, r);
}

/**
 * Quaternion right-multiply.
 *
 * @param[in] p First quaternion
 * @param[in] q Second quaternion
 * @param[out] r Result
 */
void quat_rmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  /* clang-format off */
  const real_t rprod[4*4] = {
    qw, -qx, -qy, -qz,
    qx, qw, qz, -qy,
    qy, -qz, qw, qx,
    qz, qy, -qx, qw
  };
  /* clang-format on */

  dot(rprod, 4, 4, p, 4, 1, r);
}

/**
 * Quaternion multiply.
 *
 * @param[in] p First quaternion
 * @param[in] q Second quaternion
 * @param[out] r Result
 */
void quat_mul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);
  quat_lmul(p, q, r);
}

/**
 * Form delta quaternion from a small rotation vector.
 *
 * @param[in] dalpha Delta alpha
 * @param[out] dq Delta Quaternion
 */
void quat_delta(const real_t dalpha[3], real_t dq[4]) {
  const real_t half_norm = 0.5 * vec_norm(dalpha, 3);
  const real_t k = sinc(half_norm) * 0.5;
  const real_t vector[3] = {k * dalpha[0], k * dalpha[1], k * dalpha[2]};
  real_t scalar = cos(half_norm);

  dq[0] = scalar;
  dq[1] = vector[0];
  dq[2] = vector[1];
  dq[3] = vector[2];
}


/*****************************************************************************
 *                                  IMAGE
 *****************************************************************************/

/**
 * Setup image.
 *
 * @param[out] img Image
 * @param[in] width Image width
 * @param[in] height Image height
 * @param[in] data Image data
 */
void image_setup(image_t *img,
                 const int width,
                 const int height,
                 uint8_t *data) {
  assert(img != NULL);
  img->data = data;
  img->width = width;
  img->height = height;
}

/**
 * Load image.
 *
 * @param[in] file_path Path to image file
 * @returns Image
 */
image_t *image_load(const char *file_path) {
  int img_w = 0;
  int img_h = 0;
  int img_c = 0;
  stbi_set_flip_vertically_on_load(1);
  unsigned char *data = stbi_load(file_path, &img_w, &img_h, &img_c, 0);
  if (!data) {
    FATAL("Failed to load image file: [%s]", file_path);
  }

  image_t *img = malloc(sizeof(image_t));
  img->width = img_w;
  img->height = img_h;
  img->channels = img_c;
  img->data = data;
  return img;
}

/**
 * Print image properties.
 *
 * @param[in] img Image
 */
void image_print_properties(const image_t *img) {
  assert(img != NULL);
  printf("img.width: %d\n", img->width);
  printf("img.height: %d\n", img->height);
  printf("img.channels: %d\n", img->channels);
}

/**
 * Free image.
 *
 * @param[in] img Image
 */
void image_free(image_t *img) {
  free(img->data);
  free(img);
}

/*****************************************************************************
 *                                  CV
 *****************************************************************************/

/********************************* RADTAN ************************************/

/**
 * Distort point using Radial-Tangential distortion.
 *
 * @param[in] params Distortion parameters (k1, k2, p1, p2)
 * @param[in] p Image point to be distorted
 * @param[out] p_d Distorted image point
 */
void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  /* Distortion parameters */
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  /* Apply tangential distortion */
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

  /* Distorted point */
  p_d[0] = x_ddash;
  p_d[1] = y_ddash;
}

/**
 * Radial-Tangential point jacobian.
 *
 * @param[in] params Distortion parameters (k1, k2, p1, p2)
 * @param[in] p Image point
 * @param[out] J_point Point jacobian
 */
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]) {
  /* Distortion parameters */
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  /* Point Jacobian is 2x2 */
  J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x;
  J_point[0] += x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point[2] = J_point[1];
  J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x;
  J_point[3] += y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
}

/**
 * Radial-Tangential parameters jacobian.
 *
 * @param[in] params Distortion parameters (k1, k2, p1, p2)
 * @param[in] p Image point
 * @param[out] J_param Parameters jacobian
 */
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]) {
  UNUSED(params);

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Setup */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t xy = x * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

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

/********************************** EQUI *************************************/

/**
 * Distort point using Equi-Distant distortion.
 *
 * @param[in] params Distortion parameters (k1, k2, k3, k4)
 * @param[in] p Image point to be distorted
 * @param[out] p_d Distorted image point
 */
void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;

  const real_t x_dash = s * x;
  const real_t y_dash = s * y;

  p_d[0] = x_dash;
  p_d[1] = y_dash;
}

/**
 * Equi-Distant point jacobian.
 *
 * @param[in] params Distortion parameters (k1, k2, k3, k4)
 * @param[in] p Image point
 * @param[out] J_point 2x2 Point jacobian
 */
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]) {
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const real_t th_r = 1.0 / (r * r + 1.0);
  const real_t thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const real_t s = thd / r;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;

  /* Point Jacobian is 2x2 */
  J_point[0] = s + x * s_r * r_x;
  J_point[1] = x * s_r * r_y;
  J_point[2] = y * s_r * r_x;
  J_point[3] = s + y * s_r * r_y;
}

/**
 * Equi-distant parameters jacobian.
 *
 * @param[in] params Distortion parameters (k1, k2, k3, k4)
 * @param[in] p Image point
 * @param[out] J_param 2x4 Parameter jacobian
 */
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]) {
  UNUSED(params);

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th3 = th2 * th;
  const real_t th5 = th3 * th2;
  const real_t th7 = th5 * th2;
  const real_t th9 = th7 * th2;

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

/******************************** PINHOLE ************************************/

/**
 * Estimate pinhole focal length.
 *
 * The focal length is estimated using:
 * - Image resolution
 * - Field of view of the lens / camera.
 *
 * @param[in] image_width Image width [pixels]
 * @param[in] fov Field of view [rads]
 *
 * @returns Focal length in pixels
 */
real_t pinhole_focal(const int image_width, const real_t fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

/**
 * Project 3D point observed from the camera to the image plane.
 *
 * @param[in] params Pinhole parameters (fx, fy, cx, cy)
 * @param[in] p_C 3x1 Point vector in camera frame
 * @param[in] x 2x1 Image point in image plane
 */
void pinhole_project(const real_t params[4], const real_t p_C[3], real_t x[2]) {
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  const real_t px = p_C[0] / p_C[2];
  const real_t py = p_C[1] / p_C[2];

  x[0] = px * fx + cx;
  x[1] = py * fy + cy;
}

/**
 * Pinhole point jacobian.
 *
 * @param[in] params Pinhole parameters (fx, fy, cx, cy)
 * @param[out] J 2x3 Point Jacobian
 */
void pinhole_point_jacobian(const real_t params[4], real_t J[2 * 3]) {
  J[0] = params[0];
  J[1] = 0.0;
  J[2] = 0.0;
  J[3] = params[1];
}

/**
 * Pinhole parameter jacobian.
 *
 * @param[in] params Pinhole parameters (fx, fy, cx, cy)
 * @param[in] x 2x1 Image point
 * @param[out] J 2x4 Parameter Jacobian
 */
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]) {
  UNUSED(params);

  J[0] = x[0];
  J[1] = 0.0;
  J[2] = 1.0;
  J[3] = 0.0;

  J[4] = 0.0;
  J[5] = x[1];
  J[6] = 0.0;
  J[7] = 1.0;
}

/***************************** PINHOLE-RADTAN4 ********************************/

/**
 * Projection of 3D point to image plane using Pinhole + Radial-Tangential.
 *
 * @param[in] params Intrinsics parameters (fx, fy, cx, cy, k1, k2, p1, p2)
 * @param[in] p_C 3x1 Point vector observed in camera frame
 * @param[out] x 2x1 Projected image point
 */
void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]) {
  /* Project */
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  /* Distort */
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  /* Scale and center */
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p[0] * fx + cx;
  x[1] = p[1] * fy + cy;
}

/**
 * Projection Jacobian of Pinhole + Radial-Tangential.
 *
 * @param[in] params Intrinsics parameters (fx, fy, cx, cy, k1, k2, p1, p2)
 * @param[in] p_C 3x1 Point vector observed in camera frame
 * @param[out] J 2x3 Projection Jacobian
 */
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]) {
  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[0];
  const real_t z = p_C[0];
  const real_t p[2] = {x / z, y / z};

  /* Projection Jacobian */
  real_t J_proj[2 * 3] = {0};
  J_proj[0] = 1.0 / z;
  J_proj[1] = 0.0;
  J_proj[2] = -x / (z * z);
  J_proj[3] = 0.0;
  J_proj[4] = 1.0 / z;
  J_proj[5] = -y / (z * z);

  /* Distortion Point Jacobian */
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};
  real_t J_dist_point[2 * 2] = {0};
  radtan4_point_jacobian(d, p, J_dist_point);

  /* Project Point Jacobian */
  real_t J_proj_point[2 * 3] = {0};
  pinhole_point_jacobian(params, J_proj_point);

  /* J = J_proj * J_dist_point * J_proj_point; */
  real_t J_dist_proj[2 * 3] = {0};
  dot(J_dist_point, 2, 2, J_proj, 2, 3, J_dist_proj);
  dot(J_proj, 2, 2, J_dist_proj, 2, 3, J);
}

/**
 * Parameter Jacobian of Pinhole + Radial-Tangential.
 *
 * @param[in] params Intrinsics parameters (fx, fy, cx, cy, k1, k2, p1, p2)
 * @param[in] p_C 3x1 Point vector observed in camera frame
 * @param[out] J 2x8 Parameter Jacobian
 */
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]) {
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[0];
  const real_t z = p_C[0];
  const real_t p[2] = {x / z, y / z};

  /* Distort */
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  /* Project params Jacobian: J_proj_params */
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  /* Project point Jacobian: J_proj_point */
  real_t J_proj_point[2 * 3] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  /* Distortion point Jacobian: J_dist_params */
  real_t J_dist_params[2 * 2] = {0};
  radtan4_params_jacobian(d, p, J_dist_params);

  /* J = [J_proj_params, J_proj_point * J_dist_params] */
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];


}

/****************************** PINHOLE-EQUI4 *********************************/

/**
 * Projection of 3D point to image plane using Pinhole + Equi-Distant.
 *
 * @param[in] params Intrinsics parameters (fx, fy, cx, cy, k1, k2, k3, k4)
 * @param[in] p_C 3x1 Point vector observed in camera frame
 * @param[out] x 2x1 Projected image point
 */
void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t x[2]) {
  /* Project */
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  /* Distort */
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  /* Scale and center */
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p[0] * fx + cx;
  x[1] = p[1] * fy + cy;
}

/**
 * Projection Jacobian of Pinhole + Equi-Distant.
 *
 * @param[in] params Intrinsics parameters (fx, fy, cx, cy, k1, k2, k3, k4)
 * @param[in] p_C 3x1 Point vector observed in camera frame
 * @param[out] J 2x3 Projection Jacobian
 */
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]) {
  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[0];
  const real_t z = p_C[0];
  const real_t p[2] = {x / z, y / z};

  /* Projection Jacobian */
  real_t J_proj[2 * 3] = {0};
  J_proj[0] = 1.0 / z;
  J_proj[1] = 0.0;
  J_proj[2] = -x / (z * z);
  J_proj[3] = 0.0;
  J_proj[4] = 1.0 / z;
  J_proj[5] = -y / (z * z);

  /* Distortion Point Jacobian */
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};
  real_t J_dist_point[2 * 2] = {0};
  equi4_point_jacobian(d, p, J_dist_point);

  /* Project Point Jacobian */
  real_t J_proj_point[2 * 3] = {0};
  pinhole_point_jacobian(params, J_proj_point);

  /* J = J_proj * J_dist_point * J_proj_point; */
  real_t J_dist_proj[2 * 3] = {0};
  dot(J_dist_point, 2, 2, J_proj, 2, 3, J_dist_proj);
  dot(J_proj, 2, 2, J_dist_proj, 2, 3, J);
}

/******************************************************************************
 *                              SENSOR FUSION
 ******************************************************************************/

/* POSE --------------------------------------------------------------------- */

void pose_setup(pose_t *pose,
                uint64_t *param_id,
                const timestamp_t ts,
                const real_t *data) {
  pose->param_id = *param_id;
  pose->ts = ts;
  *param_id += 1;

  /* Quaternion */
  pose->data[0] = data[0];
  pose->data[1] = data[1];
  pose->data[2] = data[2];
  pose->data[3] = data[3];

  /* Translation */
  pose->data[4] = data[4];
  pose->data[5] = data[5];
  pose->data[6] = data[6];
}

/* SPEED AND BIASES --------------------------------------------------------- */

void speed_biases_setup(speed_biases_t *sb,
                        uint64_t *param_id,
                        const timestamp_t ts,
                        const real_t *data) {
  sb->param_id = *param_id;
  sb->ts = ts;
  *param_id += 1;

  /* Velocity */
  sb->data[0] = data[0];
  sb->data[1] = data[1];
  sb->data[2] = data[2];

  /* Accel biases */
  sb->data[3] = data[3];
  sb->data[4] = data[4];
  sb->data[5] = data[5];

  /* Gyro biases */
  sb->data[6] = data[6];
  sb->data[7] = data[7];
  sb->data[8] = data[8];
}

/* FEATURE ------------------------------------------------------------------ */

void feature_setup(feature_t *p,
                   uint64_t *param_id,
                   const real_t *data) {
  p->param_id = *param_id;
  *param_id += 1;

  p->data[0] = data[0];
  p->data[1] = data[1];
  p->data[2] = data[2];
}

/* EXTRINSICS --------------------------------------------------------------- */

void extrinsics_setup(extrinsics_t *extrinsics,
                      uint64_t *param_id,
                      const real_t *data) {
  extrinsics->param_id = *param_id;
  *param_id += 1;

  /* Quaternion */
  extrinsics->data[0] = data[0];
  extrinsics->data[1] = data[1];
  extrinsics->data[2] = data[2];
  extrinsics->data[3] = data[3];

  /* Translation */
  extrinsics->data[4] = data[4];
  extrinsics->data[5] = data[5];
  extrinsics->data[6] = data[6];
}

/* CAMERA ------------------------------------------------------------------- */

void camera_setup(camera_t *camera,
                  uint64_t *param_id,
                  const int cam_idx,
                  const int cam_res[2],
                  const char *proj_model,
                  const char *dist_model,
                  const real_t data[8]) {
  camera->param_id = *param_id;
  camera->cam_idx = cam_idx;
  camera->resolution[0] = cam_res[0];
  camera->resolution[1] = cam_res[1];

  strcpy(camera->proj_model, proj_model);
  strcpy(camera->dist_model, dist_model);

  camera->data[0] = data[0];
  camera->data[1] = data[1];
  camera->data[2] = data[2];
  camera->data[3] = data[3];
  camera->data[4] = data[4];
  camera->data[5] = data[5];
  camera->data[6] = data[6];
  camera->data[7] = data[7];
}

void camera_print(const camera_t *camera) {
  printf("cam_idx: %d\n", camera->cam_idx);
  printf("cam_res: [%d, %d]\n", camera->resolution[0], camera->resolution[1]);
  printf("proj_model: %s\n", camera->proj_model);
  printf("dist_model: %s\n", camera->dist_model);
  printf("data: [");
  for (int i = 0; i < 8; i++) {
    if ((i + 1) < 8) {
      printf("%f, ", camera->data[i]);
    } else {
      printf("%f", camera->data[i]);
    }
  }
  printf("]\n");
}

/* POSE FACTOR -------------------------------------------------------------- */

void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]) {
  zeros(factor->pose_meas, 7, 1);
  factor->pose_est = pose;

  zeros(factor->covar, 6, 6);
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[7] = 1.0 / (var[1] * var[1]);
  factor->covar[14] = 1.0 / (var[2] * var[2]);
  factor->covar[21] = 1.0 / (var[3] * var[3]);
  factor->covar[28] = 1.0 / (var[4] * var[4]);
  factor->covar[35] = 1.0 / (var[5] * var[5]);

  zeros(factor->r, 6, 1);
  factor->r_size = 6;

  zeros(factor->J0, 6, 6);
  factor->jacs[0] = factor->J0;
  factor->nb_params = 1;
}

void pose_factor_reset(pose_factor_t *factor) {
  zeros(factor->r, 6, 1);
  zeros(factor->J0, 6, 6);
}

int pose_factor_eval(pose_factor_t *factor) {
  assert(factor != NULL);

  /* Map params */
  real_t pose_est[4 * 4] = {0};
  real_t pose_meas[4 * 4] = {0};
  tf(factor->pose_est->data, pose_est);
  tf(factor->pose_meas, pose_meas);

  /* Invert estimated pose */
  real_t pose_est_inv[4 * 4] = {0};
  tf_inv(pose_est, pose_est_inv);

  /* Calculate delta pose */
  real_t dpose[4 * 4] = {0};
  dot(pose_est, 4, 4, pose_meas, 4, 4, dpose);

  /* Calculate pose error */
  real_t *r = factor->r;
  const real_t dqw = dpose[0];
  const real_t dqx = dpose[1];
  const real_t dqy = dpose[2];
  const real_t dqz = dpose[3];
  const real_t drx = dpose[4];
  const real_t dry = dpose[5];
  const real_t drz = dpose[6];
  /* -- dtheta */
  r[0] = 2.0 * dqx;
  r[1] = 2.0 * dqy;
  r[2] = 2.0 * dqz;
  /* -- dr */
  r[3] = drx;
  r[4] = dry;
  r[5] = drz;

  /* Calculate Jacobians */
  /* clang-format off */
  real_t *J = factor->J0;
  J[0]  = -dqw; J[1]  =  dqz; J[2]  = -dqy; J[3]  =  0.0; J[4]  =  0.0; J[5]  =  0.0;
  J[6]  = -dqz; J[7]  = -dqw; J[8]  =  dqx; J[9]  =  0.0; J[10] =  0.0; J[11] =  0.0;
  J[12] =  dqy; J[13] = -dqx; J[14] = -dqw; J[15] =  0.0; J[16] =  0.0; J[17] =  0.0;
  J[18] =  0.0; J[19] =  0.0; J[20] =  0.0; J[21] = -1.0; J[22] =  0.0; J[23] =  0.0;
  J[24] =  0.0; J[25] =  0.0; J[26] =  0.0; J[27] =  0.0; J[28] = -1.0; J[29] =  0.0;
  J[30] =  0.0; J[31] =  0.0; J[32] =  0.0; J[33] =  0.0; J[34] =  0.0; J[35] = -1.0;
  factor->jacs[0] = J;
  /* clang-format on */

  return 0;
}

/* CAMERA FACTOR ------------------------------------------------------------ */

void cam_factor_setup(cam_factor_t *factor,
                      pose_t *pose,
                      extrinsics_t *extrinsics,
                      camera_t *camera,
                      const real_t var[2]) {
  factor->pose = pose;
  factor->extrinsics = extrinsics;
  factor->camera = camera;

  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = 1.0 / (var[1] * var[1]);

  zeros(factor->r, 2, 1);

  zeros(factor->J0, 2, 6);
  zeros(factor->J1, 2, 6);
  zeros(factor->J2, 2, 8);
  zeros(factor->J3, 2, 3);
  factor->jacs[0] = factor->J0;
  factor->jacs[1] = factor->J1;
  factor->jacs[2] = factor->J2;
  factor->jacs[3] = factor->J3;
  factor->nb_params = 4;
}

void cam_factor_reset(cam_factor_t *factor) {
  zeros(factor->r, 2, 1);
  zeros(factor->J0, 2, 6);
  zeros(factor->J1, 2, 6);
  zeros(factor->J2, 2, 8);
  zeros(factor->J3, 2, 3);
}

int cam_factor_eval(cam_factor_t *factor) {
  assert(factor != NULL);
  assert(factor->pose);
  assert(factor->extrinsics);
  assert(factor->feature);
  assert(factor->camera);

  /* Map params */
  /* -- Sensor pose */
  real_t T_WS[4 * 4] = {0};
  tf(factor->pose->data, T_WS);
  /* -- Sensor-Camera extrinsics */
  real_t T_SC[4 * 4] = {0};
  tf(factor->extrinsics->data, T_SC);
  /* -- Camera pose */
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  dot(T_WS, 4, 4, T_SC, 4, 4, T_WC);
  tf_inv(T_WC, T_CW);
  /* -- Landmark */
  real_t *p_W = factor->feature->data;
  real_t p_C[3 * 1] = {0};
  tf_point(T_CW, p_W, p_C);
  /* -- Project point from world to image plane */
  real_t z_hat[2];
  real_t *cam_params = factor->camera->data;
  pinhole_radtan4_project(cam_params, p_C, z_hat);

  /* Calculate residuals */
  /* -- Residual */
  real_t err[2] = {0};
  err[0] = factor->z[0] - z_hat[0];
  err[1] = factor->z[1] - z_hat[1];
  /* -- Weighted residual */
  real_t sqrt_info[2 * 2] = {0};
  sqrt_info[0] = 1.0 / factor->covar[0];
  sqrt_info[1] = 0.0;
  sqrt_info[2] = 0.0;
  sqrt_info[3] = 1.0 / factor->covar[1];
  dot(sqrt_info, 2, 2, err, 2, 1, factor->r);

  /* Calculate jacobians */
  /* -- Camera params jacobian */
  /* -- Extrinsics jacobian */
  /* -- Pose jacobian */
  /* -- Landmark jacobian */

  return 0;
}

/* IMU FACTOR --------------------------------------------------------------- */

void imu_buf_setup(imu_buf_t *imu_buf) {
  for (int k = 0; k < MAX_IMU_BUF_SIZE; k++) {
    imu_buf->ts[k] = 0.0;

    imu_buf->acc[k][0] = 0.0;
    imu_buf->acc[k][1] = 0.0;
    imu_buf->acc[k][2] = 0.0;

    imu_buf->gyr[k][0] = 0.0;
    imu_buf->gyr[k][1] = 0.0;
    imu_buf->gyr[k][2] = 0.0;
  }

  imu_buf->size = 0;
}

void imu_buf_print(const imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    const real_t *acc = imu_buf->acc[k];
    const real_t *gyr = imu_buf->gyr[k];

    printf("ts: %ld ", imu_buf->ts[k]);
    printf("acc: [%.2f, %.2f, %.2f] ", acc[0], acc[1], acc[2]);
    printf("gyr: [%.2f, %.2f, %.2f] ", gyr[0], gyr[1], gyr[2]);
    printf("\n");
  }
}

void imu_buf_add(imu_buf_t *imu_buf,
                 timestamp_t ts,
                 real_t acc[3],
                 real_t gyr[3]) {
  int k = imu_buf->size;
  imu_buf->ts[k] = ts;
  imu_buf->acc[k][0] = acc[0];
  imu_buf->acc[k][1] = acc[1];
  imu_buf->acc[k][2] = acc[2];
  imu_buf->gyr[k][0] = gyr[0];
  imu_buf->gyr[k][1] = gyr[1];
  imu_buf->gyr[k][2] = gyr[2];
  imu_buf->size++;
}

void imu_buf_clear(imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    timestamp_t *ts = &imu_buf->ts[k];
    real_t *acc = imu_buf->acc[k];
    real_t *gyr = imu_buf->gyr[k];

    *ts = 0;
    acc[0] = 0.0;
    acc[1] = 0.0;
    acc[2] = 0.0;
    gyr[0] = 0.0;
    gyr[1] = 0.0;
    gyr[2] = 0.0;
  }
  imu_buf->size = 0;
}

void imu_buf_copy(const imu_buf_t *from, imu_buf_t *to) {
  to->size = 0;
  for (int k = 0; k < from->size; k++) {
    to->ts[k] = from->ts[k];

    to->acc[k][0] = from->acc[k][0];
    to->acc[k][1] = from->acc[k][1];
    to->acc[k][2] = from->acc[k][2];

    to->gyr[k][0] = from->gyr[k][0];
    to->gyr[k][1] = from->gyr[k][1];
    to->gyr[k][2] = from->gyr[k][2];
  }
  to->size = from->size;
}

void imu_factor_setup(imu_factor_t *factor,
                      imu_params_t *imu_params,
                      imu_buf_t *imu_buf,
                      pose_t *pose_i,
                      speed_biases_t *sb_i,
                      pose_t *pose_j,
                      speed_biases_t *sb_j) {
  factor->imu_params = imu_params;
  imu_buf_copy(imu_buf, &factor->imu_buf);
  factor->pose_i = pose_i;
  factor->sb_i = sb_i;
  factor->pose_j = pose_j;
  factor->sb_j = sb_j;

  zeros(factor->covar, 15, 15);
  zeros(factor->r, 15, 1);
  factor->r_size = 15;

  factor->jacs[0] = factor->J0;
  factor->jacs[1] = factor->J1;
  factor->jacs[2] = factor->J2;
  factor->jacs[3] = factor->J3;
  factor->nb_params = 4;
}

void imu_factor_reset(imu_factor_t *factor) {
  zeros(factor->r, 15, 1);
  zeros(factor->J0, 2, 6);
  zeros(factor->J1, 2, 9);
  zeros(factor->J2, 2, 6);
  zeros(factor->J3, 2, 9);
}

int imu_factor_eval(imu_factor_t *factor) {
  assert(factor != NULL);

  /* factor->jacs[0] */


  return 0;
}

/* SOLVER ------------------------------------------------------------------- */

void solver_setup(solver_t *solver) {
  assert(solver);

  solver->nb_cam_factors = 0;
  solver->nb_imu_factors = 0;

  solver->nb_poses = 0;
  solver->nb_cams = 0;
  solver->nb_extrinsics = 0;
  solver->nb_features = 0;

  solver->x_size = 0;
  solver->r_size = 0;
}

void solver_print(solver_t *solver) {
  printf("solver:\n");
  printf("r_size: %d\n", solver->r_size);
  printf("x_size: %d\n", solver->x_size);
  printf("nb_cam_factors: %d\n", solver->nb_cam_factors);
  printf("nb_imu_factors: %d\n", solver->nb_imu_factors);
  printf("nb_poses: %d\n", solver->nb_poses);
}

static void solver_evaluator(solver_t *solver,
                             int **param_orders,
                             int *param_sizes,
                             int nb_params,
                             real_t *r,
                             int r_size,
                             real_t **jacs) {
  real_t *H = solver->H;
  int H_size = solver->x_size;
  real_t *g = solver->g;

  for (int i = 0; i < nb_params; i++) {
    int *idx_i = param_orders[i];
    int size_i = param_sizes[i];
    const real_t *J_i = jacs[i];

    real_t J_i_trans[MAX_H_SIZE] = {0};
    mat_transpose(J_i, r_size, size_i, J_i_trans);

    for (int j = i; j < nb_params; j++) {
      int *idx_j = param_orders[j];
      int size_j = param_sizes[i];
      const real_t *J_j = jacs[j];

      real_t H_ij[MAX_H_SIZE] = {0};
      dot(J_i_trans, size_i, r_size, J_j, r_size, size_j, H_ij);

      /* Fill Hessian H */
      /* H_ij = J_i' * J_j */
      /* H_ji = H_ij' */
      int stride = H_size;
      int rs = *idx_i;
      int cs = *idx_j;
      int re = rs + size_i;
      int ce = cs + size_j;
      if (i == j) {
        mat_block_set(H, stride, rs, cs, re, ce, H_ij);
      } else {
        real_t H_ji[MAX_H_SIZE] = {0};
        mat_transpose(H_ij, size_i, size_j, H_ji);
        mat_block_set(H, stride, rs, cs, re, ce, H_ij);
        mat_block_set(H, stride, cs, rs, ce, re, H_ij);
      }

      /* Fill in the R.H.S of H dx = g */
      /* g = -J_i * r */
      mat_scale(J_i_trans, H_size, r_size, -1);
      dot(J_i_trans, H_size, r_size, r, r_size, 1, g);
    }
  }

  /* Update parameter order */
  for (int i = 0; i < nb_params; i++) {
    param_orders[i] = param_orders[i] + param_sizes[i];
  }
}

int solver_eval(solver_t *solver) {
  assert(solver != NULL);

  int pose_idx = 0;
  int lmks_idx = solver->nb_poses * 6;
  int exts_idx = lmks_idx + solver->nb_features * 3;
  int cams_idx = exts_idx + solver->nb_extrinsics * 6;

  /* Evaluate camera factors */
  for (int i = 0; i < solver->nb_cam_factors; i++) {
    cam_factor_t *factor = &solver->cam_factors[i];
    cam_factor_eval(factor);

    int *param_orders[4] = {&pose_idx, &exts_idx, &cams_idx, &lmks_idx};
    int param_sizes[4] = {6, 6, 8, 3};
    int nb_params = 4;

    solver_evaluator(solver,
                     param_orders,
                     param_sizes,
                     nb_params,
                     factor->r,
                     factor->r_size,
                     factor->jacs);
  }

  return 0;
}

/* int solver_optimize(solver_t *solver) { */
/*   struct timespec solve_tic = tic(); */
/*   real_t lambda_k = 1e-4; */
/*  */
/*   int iter = 0; */
/*   int max_iter = 10; */
/*   int verbose = 1; */
/*  */
/*   for (iter = 0; iter < max_iter; iter++) { */
/*     #<{(| Cost k |)}># */
/*     #<{(| x = solver_get_state(solver); |)}># */
/*     #<{(| solver_eval(solver, H, g, &marg_size, &remain_size); |)}># */
/*     #<{(| const matx_t H_diag = (H.diagonal().asDiagonal()); |)}># */
/*     #<{(| H = H + lambda_k * H_diag; |)}># */
/*     #<{(| dx = H.ldlt().solve(g); |)}># */
/*     #<{(| e = solver_residuals(solver); |)}># */
/*     #<{(| cost = 0.5 * e.transpose() * e; |)}># */
/*  */
/*     #<{(| Cost k+1 |)}># */
/*     #<{(| solver_update(solver, dx); |)}># */
/*     #<{(| e = solver_residuals(solver); |)}># */
/*     const real_t cost_k = 0.5 * e.transpose() * e; */
/*  */
/*     const real_t cost_delta = cost_k - cost; */
/*     const real_t solve_time = toc(&solve_tic); */
/*     const real_t iter_time = (iter == 0) ? 0 : (solve_time / iter); */
/*  */
/*     if (verbose) { */
/*       printf("iter[%d] ", iter); */
/*       printf("cost[%.2e] ", cost); */
/*       printf("cost_k[%.2e] ", cost_k); */
/*       printf("cost_delta[%.2e] ", cost_delta); */
/*       printf("lambda[%.2e] ", lambda_k); */
/*       printf("iter_time[%.4f] ", iter_time); */
/*       printf("solve_time[%.4f]  ", solve_time); */
/*       printf("\n"); */
/*  */
/*       // // Calculate reprojection error */
/*       // size_t nb_keypoints = e.size() / 2.0; */
/*       // real_t sse = 0.0; */
/*       // for (size_t i = 0; i < nb_keypoints; i++) { */
/*       //   sse += pow(e.segment(i * 2, 2).norm(), 2); */
/*       // } */
/*       // const real_t rmse = sqrt(sse / nb_keypoints); */
/*       // printf("rmse reproj error: %.2f\n", rmse); */
/*     } */
/*  */
/*     #<{(| Determine whether to accept update |)}># */
/*     if (cost_k < cost) { */
/*       #<{(| Accept update |)}># */
/*       lambda_k /= update_factor; */
/*       cost = cost_k; */
/*     } else { */
/*       #<{(| Reject update |)}># */
/*       #<{(| solver_set_state(solver, x); // Restore state |)}># */
/*       lambda_k *= update_factor; */
/*     } */
/*  */
/*     #<{(| Termination criterias |)}># */
/*     if (fabs(cost_delta) < cost_change_threshold) { */
/*       break; */
/*     } else if ((solve_time + iter_time) > time_limit) { */
/*       break; */
/*     } */
/*   } */
/*  */
/*   #<{(| solve_time = toc(&solve_tic); |)}># */
/*   #<{(| if (verbose) { |)}># */
/*   #<{(|   printf("cost: %.2e\t", cost); |)}># */
/*   #<{(|   printf("solver took: %.4fs\n", solve_time); |)}># */
/*   #<{(| } |)}># */
/* } */
