#include <unistd.h>
#include <pthread.h>

#include "zero.h"
#include "gui.h"

/* #include <GL/glut.h> */
/* #include <GL/gl.h> */
/* #include <GL/glu.h> */

int imshow_window;
unsigned int imshow_texture_id;

void imshow_resize_window(const int width, const int height);
void imshow_draw_window();
void imshow_keyboard_callback(unsigned char key, int x, int y);
void imshow_load_image(const int img_w,
                       const int img_h,
                       const int img_c,
                       const unsigned char *data);

void imshow_setup();
void imshow(const char *title, const char *image_path);


void *imshow_thread(void *data) {
  UNUSED(data);
  xapp_t app;
  xapp_setup(&app, "Test", 0, 0, 800, 600, 1);
  xapp_loop(&app);
  return NULL;
}
