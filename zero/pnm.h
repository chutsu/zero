#ifndef PNM_H
#define PNM_H

#include <stdio.h>
#include <stdlib.h>

#define PNM_P1 1
#define PNM_P2 2
#define PNM_P3 3

struct pnm_t {
  int type;
  int image_width;
  int image_height;
  int *buffer;
};

struct pnm_t *pnm_malloc(const int type,
                         const int image_width,
                         const int image_height) {
  struct pnm_t *pnm = malloc(sizeof(struct pnm_t));
  pnm->type = type;
  pnm->image_width = image_width;
  pnm->image_height = image_height;
  pnm->buffer = malloc(sizeof(int) * image_width * image_height);
  return pnm;
}

void pnm_free(struct pnm_t *pnm) {
  free(pnm->buffer);
  free(pnm);
  pnm = NULL;
}

#endif /* ZERO_PNM_H */
