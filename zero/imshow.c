#include "zero/imshow.h"

/* void imshow_resize_window(const int width, const int height) { */
/*   glViewport(0, 0, width, height); */
/*   glMatrixMode(GL_PROJECTION); */
/*   glLoadIdentity(); */
/*   gluPerspective(45.0f, (GLfloat) width / (GLfloat) height, 0.1f, 100.0f); */
/* } */
/*  */
/* void imshow_draw_window() { */
/*   glMatrixMode(GL_MODELVIEW); */
/*   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); */
/*   glLoadIdentity(); */
/*  */
/*   glBindTexture(GL_TEXTURE_2D, imshow_texture_id); */
/*   glBegin(GL_QUADS); */
/*     glTexCoord2f(0.0f, 0.0f); */
/*     glVertex3f(-1.0, -1.0, 0.0); */
/*     glTexCoord2f(1.0f, 0.0f); */
/*     glVertex3f(1.0, -1.0, 0.0); */
/*     glTexCoord2f(1.0f, 1.0f); */
/*     glVertex3f(1.0, 1.0, 0.0); */
/*     glTexCoord2f(0.0f, 1.0f); */
/*     glVertex3f(-1.0, 1.0, 0.0); */
/*   glEnd(); */
/*   glFlush(); */
/*  */
/*   glutSwapBuffers(); */
/* } */
/*  */
/* void imshow_keyboard_callback(unsigned char key, int x, int y) { */
/*   UNUSED(x); */
/*   UNUSED(y); */
/*   usleep(100);  #<{(| avoid thrashing this procedure |)}># */
/*  */
/*   const int esc_key = 27; */
/*   if (key == esc_key) { */
/*     glutDestroyWindow(imshow_window); */
/*     exit(0); */
/*   } */
/* } */
/*  */
/* void imshow_load_image(const int img_w, */
/*                        const int img_h, */
/*                        const int img_c, */
/*                        const unsigned char *data) { */
/*   glGenTextures(1, &imshow_texture_id); */
/*   glBindTexture(GL_TEXTURE_2D, imshow_texture_id); */
/*   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); */
/*   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); */
/*  */
/*   GLenum format = 0; */
/*   switch (img_c) { */
/*   case 1: format = GL_RED; break; */
/*   case 3: format = GL_RGB; break; */
/*   case 4: format = GL_RGBA; break; */
/*   } */
/*   glTexImage2D(GL_TEXTURE_2D, */
/*                0, */
/*                format, */
/*                img_w, */
/*                img_h, */
/*                0, */
/*                format, */
/*                GL_UNSIGNED_BYTE, */
/*                data); */
/*   glEnable(GL_TEXTURE_2D); */
/* } */
/*  */
/* void imshow_setup() { */
/*   #<{(| Setup glut |)}># */
/*   int argc = 0; */
/*   char *argv[1] = {""}; */
/*   glutInit(&argc, argv); */
/*   glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH); */
/* } */
/*  */
/* void imshow(const char *title, const char *image_path) { */
/*   #<{(| Load image |)}># */
/*   image_t image; */
/*   image_load(&image, image_path); */
/*  */
/*   #<{(| Setup window |)}># */
/*   const float screen_width = glutGet(GLUT_SCREEN_WIDTH); */
/*   const float screen_height = glutGet(GLUT_SCREEN_HEIGHT); */
/*   const float pos_x = (screen_width - image.width) / 2.0; */
/*   const float pos_y = (screen_height - image.height) / 2.0; */
/*   glutInitWindowPosition(pos_x, pos_y); */
/*   imshow_window = glutCreateWindow(title); */
/*   glutReshapeWindow(image.width, image.height); */
/*   imshow_load_image(image.width, image.height, image.channels, image.data); */
/*  */
/*   #<{(| Register callbacks |)}># */
/*   glutDisplayFunc(&imshow_draw_window); */
/*   glutKeyboardFunc(&imshow_keyboard_callback); */
/*   glutMainLoop(); */
/*  */
/*   #<{(| Clean up |)}># */
/*   image_free(&image); */
/* } */
