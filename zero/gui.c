#include "gui.h"

void xapp_draw_quad(const xapp_t *app,
                    const int x,
                    const int y,
                    const int width,
                    const int height) {
  UNUSED(app);
  UNUSED(x);
  UNUSED(y);
  UNUSED(width);
  UNUSED(height);

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
}

void xapp_draw_circle(const xapp_t *app,
                      const float cx,
                      const float cy,
                      const float r) {
  UNUSED(app);

  glBegin(GL_LINE_LOOP);
  for (int ii = 0; ii < APP_DRAW_CIRCLE_SEGMENTS; ii++) {
    double theta =
        2.0f * M_PI * (double) ii / (double) APP_DRAW_CIRCLE_SEGMENTS;
    double x = r * cos(theta) + cx;
    double y = r * sin(theta) + cy;
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f(x, y);
  }
  glEnd();
}

void xapp_setup(xapp_t *app,
                const char *title,
                const int x,
                const int y,
                const int win_width,
                const int win_height,
                const int border_width) {
  /* Setup display, screen and default root window */
  app->disp = XOpenDisplay(NULL);
  if (app->disp == NULL) {
    fprintf(stderr, "Cannot open display\n");
    exit(1);
  }
  app->screen = DefaultScreen(app->disp);
  app->root = DefaultRootWindow(app->disp);

  /* Setup OpenGL context */
  GLint att[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
  app->vi = glXChooseVisual(app->disp, 0, att);
  if (app->vi == NULL) {
    printf("Failed to setup GL context!");
    exit(0);
  }
  /* -- Colormap */
  app->cmap = XCreateColormap(app->disp, app->root, app->vi->visual, AllocNone);
  /* -- Window attributes */
  app->swa.colormap = app->cmap;
  app->swa.event_mask = ExposureMask | KeyPressMask;

  /* Setup window */
  /* app->x = APP_X; */
  /* app->y = APP_Y; */
  /* app->width = APP_WIDTH; */
  /* app->height = APP_HEIGHT; */
  /* app->border_width = APP_BORDER_WIDTH; */
  app->x = x;
  app->y = y;
  app->width = win_width;
  app->height = win_height;
  app->border_width = border_width;
  app->win = XCreateWindow(app->disp,
                           app->root,
                           app->x,
                           app->y,
                           app->width,
                           app->height,
                           app->border_width,
                           app->vi->depth,
                           InputOutput,
                           app->vi->visual,
                           CWColormap | CWEventMask,
                           &app->swa);

  XSelectInput(app->disp, app->win, ExposureMask | KeyPressMask);
  XMapWindow(app->disp, app->win);
  XStoreName(app->disp, app->win, title);

  app->glc = glXCreateContext(app->disp, app->vi, NULL, GL_TRUE);
  glXMakeCurrent(app->disp, app->win, app->glc);
  glEnable(GL_DEPTH_TEST);
}

void xapp_clear(xapp_t *app) {
  XGetWindowAttributes(app->disp, app->win, &app->gwa);
  glViewport(0, 0, app->gwa.width, app->gwa.height);
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  double fov = 90.0;
  double aspect = 1.0;
  double znear = 0.1;
  double zfar = 100.0;
  gluPerspective(fov, aspect, znear, zfar);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0, 0, -1);
}

void xapp_loop(xapp_t *app) {
  while (1) {
    XEvent ev;
    XNextEvent(app->disp, &ev);
    if (ev.type == Expose) {
      xapp_clear(app);

      /* xapp_draw_circle(app, 0, 0, 0.1); */
      xapp_draw_quad(app, 0, 0, 10, 10);

      glXSwapBuffers(app->disp, app->win);
    }

    /* Stop app */
    if (ev.type == KeyPress) {
      const int keycode = ev.xkey.keycode;
      int pressed_esc = (keycode == 0x09);
      int pressed_q = (keycode == XKeysymToKeycode(app->disp, (KeySym) 'q'));
      if (pressed_esc || pressed_q) {
        break;
      }
    }
  }

  glXMakeCurrent(app->disp, None, NULL);
  glXDestroyContext(app->disp, app->glc);
  XDestroyWindow(app->disp, app->win);
  XCloseDisplay(app->disp);
}
