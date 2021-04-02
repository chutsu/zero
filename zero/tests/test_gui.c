#include "zero/gui.h"
#include "zero/imshow.h"

int main(void) {
  const char *title = "Test";
  const char *image_path = "./test_data/images/flower.jpg";

  imshow_t im;
  imshow_load(&im, title, image_path);
  while (1) {
    if (XK_q == imshow_wait(&im)) {
      break;
    }
  }
	imshow_free(&im);

  return 0;
}
