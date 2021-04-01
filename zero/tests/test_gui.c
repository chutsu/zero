#include "zero/gui.h"
#include "zero/imshow.h"


int main(void) {
  pthread_t th;
  pthread_create(&th, NULL, imshow_thread, NULL);
  sleep(1);

  pthread_t th2;
  pthread_create(&th2, NULL, imshow_thread, NULL);

  pthread_join(th, NULL);
  pthread_join(th2, NULL);

  return 0;
}
