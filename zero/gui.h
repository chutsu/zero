#ifndef ZERO_GUI_H
#define ZERO_GUI_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>

#include "zero/zero.h"

#define APP_DRAW_CIRCLE_SEGMENTS 1000

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

struct xapp_t {
  Display *disp;
  int screen;
  Window root;
  Window win;

  int x;
  int y;
  int width;
  int height;
  int border_width;

  XVisualInfo *vi;
  Colormap cmap;
  XWindowAttributes gwa;
  XSetWindowAttributes swa;

  GLXContext glc;

} typedef xapp_t;

void xapp_draw_quad(const xapp_t *app,
                    const int x,
                    const int y,
                    const int width,
                    const int height);
void xapp_draw_circle(const xapp_t *app,
                      const float cx,
                      const float cy,
                      const float r);
void xapp_setup(xapp_t *app,
                const char *title,
                const int x,
                const int y,
                const int win_width,
                const int win_height,
                const int border_width);
void xapp_clear(xapp_t *app);
void xapp_loop(xapp_t *app);

#endif /* ZERO_GUI_H */
