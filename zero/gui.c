#include "gui.h"

/*******************************************************************************
 *                                  UTILS
 ******************************************************************************/

GLfloat gl_deg2rad(const GLfloat d) {
  return d * M_PI / 180.0f;
}

GLfloat gl_rad2deg(const GLfloat r) {
  return r * 180.0f / M_PI;
}

void gl_print_vector(const char *prefix, const GLfloat *x, const int length) {
  printf("%s: [", prefix);
  for (int i = 0; i < length; i++) {
    printf("%f", x[i]);
    if ((i + 1) != length) {
      printf(", ");
    }
  }
  printf("]\n");
}

void gl_print_matrix(const char *prefix,
                      const GLfloat *A,
                      const int nb_rows,
                      const int nb_cols) {
  printf("%s:\n", prefix);
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      printf("%f", A[i + (j * 3)]);
      if ((j + 1) != nb_cols) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void gl_zeros(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 0.0f;
  }
}

void gl_ones(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 1.0f;
  }
}

void gl_eye(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      A[i + (j * nb_rows)] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void gl_vec2f(GLfloat *v, const GLfloat x, const GLfloat y) {
  v[0] = x;
  v[1] = y;
}

void gl_vec3f(GLfloat *v, const GLfloat x, const GLfloat y, const GLfloat z) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

void gl_vec4f(GLfloat *v,
              const GLfloat x,
              const GLfloat y,
              const GLfloat z,
              const GLfloat w) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
  v[3] = w;
}

int gl_equals(const GLfloat *A,
              const GLfloat *B,
              const int nb_rows,
              const int nb_cols,
              const GLfloat tol) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    if (fabs(A[i] - B[i]) > tol) {
      return 0;
    }
  }

  return 1;
}

void gl_vec3f_cross(const GLfloat u[3], const GLfloat v[3], GLfloat n[3]) {
  assert(u);
  assert(v);
  assert(n);

  n[0] = u[1] * v[2] - u[2] * v[1];
  n[1] = u[2] * v[0] - u[0] * v[2];
  n[2] = u[0] * v[1] - u[1] * v[0];
}

void gl_add(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] + B[i];
  }
}

void gl_sub(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] - B[i];
  }
}

void gl_dot(const GLfloat *A, const int A_m, const int A_n,
            const GLfloat *B, const int B_m, const int B_n,
            GLfloat *C) {
  assert(A_n == B_m);

  int m = A_m;
  int n = B_n;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      for (int k = 0; k < A_n; k++) {
        C[i + (j * n)] += A[i + (k * A_n)] * B[k + (j * B_n)];
      }
    }
  }
}

void gl_scale(GLfloat factor, GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] *= factor;
  }
}

GLfloat gl_norm(const GLfloat *x, const int size) {
  GLfloat sum_sq = 0.0f;
  for (int i = 0; i < size; i++) {
    sum_sq += x[i] * x[i];
  }

  return sqrt(sum_sq);
}

void gl_normalize(GLfloat *x, const int size) {
  const GLfloat n = gl_norm(x, size);
  for (int i = 0; i < size; i++) {
    x[i] /= n;
  }
}

void gl_perspective(const GLfloat fov,
			 	 		 	 	    const GLfloat near,
				 	 	 	      const GLfloat far,
									  GLfloat P[4*4]) {
	const GLfloat scale = 1.0f / tan(fov * 0.5f * M_PI / 180.0f);
	const GLfloat a = -far / (far - near);
	const GLfloat b = -far * near / (far - near);

	/* clang-format off */
	P[0] = scale; P[4] = 0.0f;  P[8] = 0.0f; P[12] = 0.0f;
	P[1] = 0.0f;  P[5] = scale; P[9] = 0.0f; P[13] = 0.0f;
	P[2] = 0.0f;  P[6] = 0.0f;  P[10] = -a;  P[14] = -1.0f;
	P[3] = 0.0f;  P[7] = 0.0f;  P[11] = -b;  P[15] = 0.0f;
	/* clang-format on */
}

void gl_frustrum(const GLfloat fov,
                 const GLfloat ratio,
                 const GLfloat near,
                 const GLfloat far,
                 GLfloat P[4*4]) {
  const GLfloat scale = tan(fov * 0.5f * M_PI / 180.0f) * near;
  const GLfloat right = ratio * scale;
  const GLfloat left = -1.0f * right;
  const GLfloat top = scale;
  const GLfloat bottom = -1.0f * top;

  P[0] = 2.0f * near / (right - left);
  P[1] = 0.0f;
  P[2] = 0.0f;
  P[3] = 0.0f;

  P[4] = 0.0f;
  P[5] = 2.0f * near / (top - bottom);
  P[6] = 0.0f;
  P[7] = 0.0f;

  P[8] = (right + left) / (right - left);
  P[9] = (top + bottom) / (top - bottom);
  P[10] = -(far + near) / (far - near);
  P[11] = -1;

  P[12] = 0.0f;
  P[13] = 0.0f;
  P[14] = -2.0f * far * near / (far - near);
  P[15] = 0.0f;
}

void gl_lookat(const GLfloat eye[3],
               const GLfloat at[3],
               const GLfloat up[3],
               GLfloat V[4*4]) {
  /* Z-axis: Camera forward */
  GLfloat z[3] = {0};
  gl_sub(at, eye, 3, 1, z);
  gl_normalize(z, 3);

  /* X-axis: Camera right */
  GLfloat x[3] = {0};
  gl_vec3f_cross(z, up, x);
  gl_normalize(x, 3);

  /* Y-axis: Camera up */
  GLfloat y[3] = {0};
  gl_vec3f_cross(x, z, y);

  /* View matrix - column order */
	/* clang-format off */
  /* V[0] = x[0]; V[4] = x[1]; V[8] = x[2];  V[12] = -a; */
  /* V[1] = y[0]; V[5] = y[1]; V[9] = y[2];  V[13] = -b; */
  /* V[2] = z[0]; V[6] = z[1]; V[10] = z[2]; V[14] = -c; */
  /* V[3] = 0.0f; V[7] = 0.0f; V[11] = 0.0f; V[15] = 1.0f; */
	/* clang-format on */
}

/*******************************************************************************
 *                                  SHADER
 ******************************************************************************/

GLuint shader_compile(const char *shader_src, const int type) {
  assert(shader_src != NULL);

  GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &shader_src, NULL);
  glCompileShader(shader);

  GLint success = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char log[512];
    glGetShaderInfoLog(shader, 512, NULL, log);
    printf("Failed to compile fragment shader:\n%s\n", log);
    return -1;
  }

  return shader;
}

GLuint shaders_link(const int vertex_shader,
                 	 	const int fragment_shader,
                 	 	const int geometry_shader) {
  assert(vertex_shader != -1);
  assert(fragment_shader != -1);

  // Attach shaders to link
  GLuint program = glCreateProgram();
  glAttachShader(program, vertex_shader);
  glAttachShader(program, fragment_shader);
  if (geometry_shader != -1) {
    glAttachShader(program, geometry_shader);
  }
  glLinkProgram(program);

  // Link program
  GLint success = 0;
  char log[1024];
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(program, 1024, NULL, log);
    printf("Failed to link shaders:\nReason: %s\n", log);
    exit(-1);
  }

  // Delete shaders
  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  if (geometry_shader == -1) {
    glDeleteShader(geometry_shader);
  }

  return program;
}

/*******************************************************************************
 *                                GL PROGRAM
 ******************************************************************************/

int gl_prog_setup(const char *vs_src, const char *fs_src) {
  const GLuint vs = shader_compile(vs_src, GL_VERTEX_SHADER);
  const GLuint fs = shader_compile(fs_src, GL_FRAGMENT_SHADER);
  const GLuint program_id = shaders_link(vs, fs, -1);
  return program_id;
}

int gl_prog_set_int(const GLint id, const char *k, const GLint v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1i(location, v);
  return 0;
}

int gl_prog_set_vec2i(const GLint id, const char *k, const GLint v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2i(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3i(const GLint id, const char *k, const GLint v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3i(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4i(const GLint id, const char *k, const GLint v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4i(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_float(const GLint id, const char *k, const GLfloat v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1f(location, v);
  return 0;
}

int gl_prog_set_vec2f(const GLint id, const char *k, const GLfloat v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2f(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3f(const GLint id, const char *k, const GLfloat v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3f(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4f(const GLint id, const char *k, const GLfloat v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4f(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_mat2f(const GLint id, const char *k, const GLfloat v[2*2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix2fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat3f(const GLint id, const char *k, const GLfloat v[3*3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix3fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat4f(const GLint id, const char *k, const GLfloat v[4*4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix4fv(location, 1, GL_FALSE, v);
  return 0;
}

/*******************************************************************************
 *                                 GL-CAMERA
 ******************************************************************************/

void gl_camera_setup(gl_camera_t *camera) {
  gl_zeros(camera->focal, 3, 1);
  gl_vec3f(camera->world_up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->position, 0.0f, 0.0f, 0.0f);
  gl_vec3f(camera->right, -1.0f, 0.0f, 0.0f);
  gl_vec3f(camera->up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->front, 0.0f, 0.0f, -1.0f);
  camera->yaw = gl_deg2rad(0.0f);
  camera->pitch = gl_deg2rad(0.0f);

  camera->movement_speed = 50.0f;
  camera->mouse_sensitivity = 0.02f;
  camera->fov = gl_deg2rad(45.0f);
  camera->near = 0.1f;
  camera->far = 100.0f;
}

void gl_camera_update(gl_camera_t *camera) {
	camera->front[0] = sin(camera->yaw) * cos(camera->pitch);
	camera->front[1] = sin(camera->pitch);
	camera->front[2] = cos(camera->yaw) * cos(camera->pitch);
	gl_normalize(camera->front, 3);

	gl_vec3f_cross(camera->front, camera->world_up, camera->right);
	gl_normalize(camera->right, 3);

	gl_vec3f_cross(camera->right, camera->front, camera->up);
	gl_normalize(camera->up, 3);
}

void gl_camera_projection_matrix(gl_camera_t *camera, GLfloat P[4*4]) {
	gl_perspective(camera->fov, camera->near, camera->far, P);
}

void gl_camera_view_matrix(gl_camera_t *camera, GLfloat V[4*4]) {
	const GLfloat radius = 10.0f;
	GLfloat eye[3] = {0};
	eye[0] = camera->focal[0] + radius * sin(camera->yaw);
	eye[1] = camera->focal[1] - radius * sin(camera->pitch);
	eye[2] = camera->focal[2] + radius * cos(camera->yaw);
	gl_lookat(eye, camera->focal, camera->world_up, V);
}

/*******************************************************************************
 *                                    GUI
 ******************************************************************************/

double step = 10;
double scale = 1.2;
double win_w, win_h;
double pan_x = 0.0;
double pan_y = 0.0;
double zoom_k = 200.0;

/* static void error(const char *msg) { */
/*   fprintf(stderr, "%s ", msg); */
/*   exit(1); */
/* } */

static void reset(void) {
  pan_x = 0.0;
  pan_y = 0.0;
  zoom_k = 200.0;
}

static void zoom(double factor) {
  zoom_k *= factor;
}

static void pan(double dx, double dy) {
  pan_x += dx / zoom_k;
  pan_y += dy / zoom_k;
}

static void zoom_at(double x, double y, double factor) {
  pan(-x, -y);
  zoom(factor);
  pan(x, y);
}

static void keyboard_callback(unsigned char key, int x, int y) {
  UNUSED(x);
  UNUSED(y);

  if (key == '\033' || key == 'q') {
    exit(0);
  }
}

static void special_callback(int key, int x, int y) {
  UNUSED(x);
  UNUSED(y);

  switch (key) {
    case GLUT_KEY_HOME: reset(); break;
    case GLUT_KEY_LEFT: pan(-step, 0); break;
    case GLUT_KEY_RIGHT: pan(step, 0); break;
    case GLUT_KEY_DOWN: pan(0, -step); break;
    case GLUT_KEY_UP: pan(0, step); break;
  }

  glutPostRedisplay();
}

static void mouse(int b, int s, int x, int y) {
  if (s != GLUT_DOWN)
    return;
  y = win_h - 1 - y;

  switch (b) {
  case GLUT_LEFT_BUTTON: zoom_at(x, y, scale); break;
  case GLUT_RIGHT_BUTTON: zoom_at(x, y, 1 / scale); break;
  }

  glutPostRedisplay();
}

static void draw_callback(void) {
  glMatrixMode(GL_MODELVIEW);

  glClearColor(0.5, 0.5, 1.0, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();
  glScalef(1.0, 1.0, 1.0);
  glTranslatef(0.0, 0.0, -1.0f);

  glutSwapBuffers();
}

static void resize_callback(int width, int height) {
  /* Avoid division by zero */
  if (height == 0) {
    height = 1;
  }
  win_w = width;
  win_h = height;

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  /* gluOrtho2D(0, width, 0, height); */
}

void gui_setup(int argc, char **argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

  int screen_width = glutGet(GLUT_SCREEN_WIDTH);
  int screen_height = glutGet(GLUT_SCREEN_HEIGHT);
  char *window_title = "Test";
  int window_width = 1024;
  int window_height = 768;
  int window_x = (screen_width - window_width) / 2.0;
  int window_y = (screen_height - window_height) / 2.0;

  glutInitWindowSize(window_width, window_height);
  glutCreateWindow(window_title);
  glutPositionWindow(window_x, window_y);
  glutDisplayFunc(draw_callback);
  glutReshapeFunc(resize_callback);
  glutKeyboardFunc(keyboard_callback);
  glutSpecialFunc(special_callback);
  glutMouseFunc(mouse);

  glFrontFace(GL_CCW);
  glEnable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);

  reset();
  glutMainLoop();
}

void gui_add_cube(gl_entity_t *entity) {
  /* Vertex shader source */
  const char *glcube_vs = "                                                    \
#version 330 core                                                               \
layout (location = 0) in vec3 in_pos;                                           \
layout (location = 1) in vec3 in_color;                                        \
                                                                               \
out vec3 color;                                                                 \
                                                                               \
uniform mat4 model;                                                             \
uniform mat4 view;                                                             \
uniform mat4 projection;                                                       \
                                                                               \
void main() {                                                                  \
  gl_Position = projection * view * model * vec4(in_pos, 1.0);                 \
  color = in_color;                                                             \
}";

  /* Fragment shader source */
  const char *glcube_fs = "                                                    \
#version 150 core                                                             \
in vec3 color;                                                                \
out vec4 frag_color;                                                          \
                                                                              \
void main() {                                                                 \
  frag_color = vec4(color, 1.0f);                                             \
}";

  /* Shader program */
  entity->program_id = gl_prog_setup(glcube_vs, glcube_fs);

  // Vertices
  // clang-format off
  const float color[3] = {0.9, 0.4, 0.2};
  const float cube_size = 0.5;
  const float r = color[0];
  const float g = color[1];
  const float b = color[2];
  const GLfloat vertices[] = {
    // Triangle 1
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 2
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 3
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 4
    cube_size, cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 5
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 6
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 7
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 8
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 9
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 10
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 11
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 12
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b
    // Triangle 12 : end
  };
  const size_t nb_triangles = 12;
  const size_t vertices_per_triangle = 3;
  const size_t nb_vertices = vertices_per_triangle * nb_triangles;
  const size_t vertex_buffer_size = sizeof(float) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gui_remove_cube(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gui_draw_cube(const gl_entity_t *entity) {
  glUseProgram(entity->program_id);
  /* gl_prog_set_mat4f("projection", camera.projection()); */
  /* gl_prog_set_mat4f("view", camera.view()); */
  /* gl_prog_set_mat4f("model", T_SM_); */

  // 12 x 3 indices starting at 0 -> 12 triangles -> 6 squares
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0); // Unbind VAO
}
