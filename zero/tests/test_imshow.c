#include <unistd.h>

#define STB_IMAGE_IMPLEMENTATION
#include "zero/stb_image.h"

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#define ESCAPE 27
int window;
unsigned int texture_id;


void resize_window(int width, int height) {
	/* Prevent division by zero */
  if (height == 0) {
    height = 1;
  }

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, (GLfloat) width / (GLfloat) height, 0.1f, 100.0f);
  glMatrixMode(GL_MODELVIEW);
}

void draw_window() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

	glBindTexture(GL_TEXTURE_2D, texture_id);
	glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(-1.0, -1.0, 0.0);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(1.0, -1.0, 0.0);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(1.0, 1.0, 0.0);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(-1.0, 1.0, 0.0);
	glEnd();
	glFlush();

  glutSwapBuffers();
}

void keyboard_callback(unsigned char key, int x, int y) {
	usleep(100);  /* avoid thrashing this procedure */

	if (key == ESCAPE) {
		glutDestroyWindow(window);
		exit(0);
	}
}

void load_image(int img_w, int img_h, int img_c, const unsigned char *data) {
  glGenTextures(1, &texture_id);
  glBindTexture(GL_TEXTURE_2D, texture_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  GLenum format;
  switch (img_c) {
  case 1: format = GL_RED; break;
  case 3: format = GL_RGB; break;
  case 4: format = GL_RGBA; break;
  }
  glTexImage2D(GL_TEXTURE_2D,
               0,
               format,
               img_w,
               img_h,
               0,
               format,
               GL_UNSIGNED_BYTE,
               data);
	glEnable(GL_TEXTURE_2D);
}

int main(int argc, char **argv) {
  /* Load image */
  const char *img_path = "test_data/imaes/awesomeface.png";
  /* const char *img_path = "test_data/images/google.jpeg"; */

  int img_w = 0;
  int img_h = 0;
  int img_c = 0;
	stbi_set_flip_vertically_on_load(1);
  unsigned char *data = stbi_load(img_path, &img_w, &img_h, &img_c, 0);
  if (!data) {
    perror("Failed to load image file");
		return -1;
  }

	/* Setup glut */
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowSize(img_w, img_h);
  glutInitWindowPosition(0, 0);

	/* Setup window */
  window = glutCreateWindow("Image");
  load_image(img_w, img_h, img_c, data);
  glutDisplayFunc(&draw_window);
  glutKeyboardFunc(&keyboard_callback);
  glutMainLoop();

  /* Clean up */
  stbi_image_free(data);

  return 0;
}
