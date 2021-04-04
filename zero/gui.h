#ifndef ZERO_GUI_H
#define ZERO_GUI_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

/* #include <glad/glad.h> */

#include "zero/zero.h"

/*******************************************************************************
 *                                  UTILS
 ******************************************************************************/

GLfloat gl_deg2rad(const GLfloat d);
GLfloat gl_rad2deg(const GLfloat r);
void gl_print_vector(const char *prefix, const GLfloat *x, const int length);
void gl_print_matrix(const char *prefix,
                     const GLfloat *A,
                     const int nb_rows,
                     const int nb_cols);
int gl_equals(const GLfloat *A,
              const GLfloat *B,
              const int nb_rows,
              const int nb_cols,
              const GLfloat tol);
void gl_zeros(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_ones(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_eye(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_vec2f(GLfloat *v, const GLfloat x, const GLfloat y);
void gl_vec3f(GLfloat *v, const GLfloat x, const GLfloat y, const GLfloat z);
void gl_vec4f(GLfloat *v,
              const GLfloat x,
              const GLfloat y,
              const GLfloat z,
              const GLfloat w);
void gl_vec3f_cross(const GLfloat u[3], const GLfloat v[3], GLfloat n[3]);

void gl_add(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C);
void gl_sub(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C);
void gl_dot(const GLfloat *A, const int A_m, const int A_n,
            const GLfloat *B, const int B_m, const int B_n,
            GLfloat *C);
void gl_scale(GLfloat factor, GLfloat *A, const int nb_rows, const int nb_cols);
GLfloat gl_norm(const GLfloat *x, const int size);
void gl_normalize(GLfloat *x, const int size);


void gl_perspective(const GLfloat fov,
			 	 		 	 	    const GLfloat near,
				 	 	 	      const GLfloat far,
									  GLfloat P[4*4]);
void gl_frustrum(const GLfloat fov,
								 const GLfloat ratio,
                 const GLfloat near,
                 const GLfloat far,
                 GLfloat P[4*4]);
void gl_lookat(const GLfloat eye[3],
               const GLfloat at[3],
               const GLfloat up[3],
               GLfloat V[4*4]);

/*******************************************************************************
 *                                  SHADER
 ******************************************************************************/

GLuint shader_compile(const char *shader_src, const int type);
GLuint shaders_link(const int vertex_shader,
                 	 	const int fragment_shader,
                 	 	const int geometry_shader);

/*******************************************************************************
 *                                GL PROGRAM
 ******************************************************************************/

typedef struct gl_entity_t {
  GLint program_id;
  GLuint vao;
  GLuint vbo;
  GLuint ebo;
} gl_entity_t;

int gl_prog_setup(const char *vs_src, const char *fs_src);

int gl_prog_set_int(const GLint id, const char *k, const GLint v);
int gl_prog_set_vec2i(const GLint id, const char *k, const GLint v[2]);
int gl_prog_set_vec3i(const GLint id, const char *k, const GLint v[3]);
int gl_prog_set_vec4i(const GLint id, const char *k, const GLint v[4]);

int gl_prog_set_float(const GLint id, const char *k, const GLfloat v);
int gl_prog_set_vec2f(const GLint id, const char *k, const GLfloat v[2]);
int gl_prog_set_vec3f(const GLint id, const char *k, const GLfloat v[3]);
int gl_prog_set_vec4f(const GLint id, const char *k, const GLfloat v[4]);
int gl_prog_set_mat2f(const GLint id, const char *k, const GLfloat v[2*2]);
int gl_prog_set_mat3f(const GLint id, const char *k, const GLfloat v[3*3]);
int gl_prog_set_mat4f(const GLint id, const char *k, const GLfloat v[4*4]);

/*******************************************************************************
 *                                 GL-CAMERA
 ******************************************************************************/

typedef struct gl_camera_t {
  GLfloat focal[3];
  GLfloat world_up[3];
  GLfloat position[3];
  GLfloat right[3];
  GLfloat up[3];
  GLfloat front[3];
  GLfloat yaw;
  GLfloat pitch;

  GLfloat movement_speed;
  GLfloat mouse_sensitivity;
  GLfloat fov;
  GLfloat near;
  GLfloat far;
} gl_camera_t;

void gl_camera_setup(gl_camera_t *camera);
void gl_camera_update(gl_camera_t *camera);
void gl_camera_projection_matrix(gl_camera_t *camera, GLfloat P[4*4]);
void gl_camera_view_matrix(gl_camera_t *camera, GLfloat V[4*4]);

/*******************************************************************************
 *                                    GUI
 ******************************************************************************/

void gui_setup(int argc, char **argv);
void gui_add_cube(gl_entity_t *entity);
void gui_remove_cube(const gl_entity_t *entity);

#endif /* ZERO_GUI_H */
