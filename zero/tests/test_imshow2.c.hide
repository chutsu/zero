#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "zero/zero.h"


double step = 10;
double scale = 1.2;

double win_w, win_h;

double pan_x = 0.0;
double pan_y = 0.0;
double zoom_k = 200.0;

void error(const char *msg) {
  fprintf(stderr, "%s ", msg);
  exit(1);
}

void reset(void) {
  pan_x = 0.0;
  pan_y = 0.0;
  zoom_k = 200.0;
}

void zoom(double factor) {
  zoom_k *= factor;
}

void pan(double dx, double dy) {
  pan_x += dx / zoom_k;
  pan_y += dy / zoom_k;
}

void zoom_at(double x, double y, double factor) {
  pan(-x, -y);
  zoom(factor);
  pan(x, y);
}

void keyboard_callback(unsigned char key, int x, int y) {
  if (key == '\033' || key == 'q') {
    exit(0);
  }
}

void special_callback(int k, int x, int y) {
  switch (k) {
    case GLUT_KEY_HOME: reset(); break;
    case GLUT_KEY_LEFT: pan(-step, 0); break;
    case GLUT_KEY_RIGHT: pan(step, 0); break;
    case GLUT_KEY_DOWN: pan(0, -step); break;
    case GLUT_KEY_UP: pan(0, step); break;
  }

  glutPostRedisplay();
}

void mouse(int b, int s, int x, int y) {
  if (s != GLUT_DOWN)
    return;
  y = win_h - 1 - y;

  switch (b) {
  case GLUT_LEFT_BUTTON: zoom_at(x, y, scale); break;
  case GLUT_RIGHT_BUTTON: zoom_at(x, y, 1 / scale); break;
  }

  glutPostRedisplay();
}

void draw_callback(void) {
  glMatrixMode(GL_MODELVIEW);

  glClearColor(0.5, 0.5, 1.0, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* glBegin(GL_QUADS); */
  /*   glColor3f(0.0f, 0.0f, 0.0f); */
  /*   glVertex2f(-1.0, -1.0); */
  /*   glColor3f(0.0f, 0.0f, 0.0f); */
  /*   glVertex2f(1.0, -1.0); */
  /*   glColor3f(0.0f, 0.0f, 0.0f); */
  /*   glVertex2f(1.0, 1.0); */
  /*   glColor3f(0.0f, 0.0f, 0.0f); */
  /*   glVertex2f(-1.0, 1.0); */
  /* glEnd(); */

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

  glLoadIdentity();
  glScalef(1.0, 1.0, 1.0);
  glTranslatef(0.0, 0.0, -1.0f);
  /* glScalef(zoom_k, zoom_k, 1); */
  /* glTranslatef(pan_x, pan_y, -1.0f); */

  /* printf("pan_x: %f, pan_y: %f, zoom_k: %f\n", pan_x, pan_y, zoom_k); */

  glutSwapBuffers();
}

void resize_callback(int width, int height) {
  /* Avoid division by zero */
  if (height == 0) {
    height = 1;
  }

  win_w = width;
  win_h = height;

  /* glViewport(0, 0, width, height); */
  /* glViewport(-width / 2.0, -height / 2.0, width, height); */
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  /* gluOrtho2D(0, width, 0, height); */
}

int main(int argc, char **argv) {
  image_t *img = image_load("./test_data/images/awesomeface.png");
  unsigned int texture_id;

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(640, 512);

  glutCreateWindow("Test");
  glutDisplayFunc(draw_callback);
  glutReshapeFunc(resize_callback);
  /* glutKeyboardFunc(keyboard_callback); */
  /* glutSpecialFunc(special_callback); */
  /* glutMouseFunc(mouse); */

  glFrontFace(GL_CCW);
  glEnable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);

  /* Load image data */
  glGenTextures(1, &texture_id);
  glBindTexture(GL_TEXTURE_2D, texture_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  GLenum format = 0;
  switch (img->channels) {
  case 1: format = GL_LUMINANCE; break;
  case 3: format = GL_RGB; break;
  case 4: format = GL_RGBA; break;
  }

  glTexImage2D(GL_TEXTURE_2D,
               0,
               format,
               img->width,
               img->height,
               0,
               format,
               GL_UNSIGNED_BYTE,
               img->data);
  glEnable(GL_TEXTURE_2D);

  reset();
  glutMainLoop();
  image_free(img);

  return 0;
}
