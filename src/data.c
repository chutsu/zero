#include "zero/data.h"

void int_list_init(int_list_t *list) {
  for (size_t i = 0; i < INT_LIST_MAX_SIZE; i++) {
    list->data[i] = 0;
  }
  list->length = 0;
}

void int_list_reset(int_list_t *list) {
  int_list_init(list);
}

void int_list_print(int_list_t *list) {
  for (size_t i = 0; i < list->length; i++) {
    printf("%d\n", list->data[i]);
  }
}

int int_list_push_front(int_list_t *list, const int val) {
  /* Pre-Check */
  if (list->length >= INT_LIST_MAX_SIZE) {
    return -1;
  }

  if (list->length) {
    for (size_t i = (list->length - 1); i > 0; i--) {
      list->data[i + 1] = list->data[i];
    }
    list->data[1] = list->data[0];
  }

  list->data[0] = val;
  list->length++;

  return 0;
}

int int_list_push_back(int_list_t *list, const int val) {
  /* Pre-Check */
  if (list->length >= INT_LIST_MAX_SIZE) {
    return -1;
  }

  list->data[list->length] = val;
  list->length++;
  return 0;
}

int int_list_pop_front(int_list_t *list, int *val) {
  /* Pre-Check */
  if (list->length == 0) {
    return -1;
  }

  /* Get value */
  *val = list->data[0];

  /* Shift data */
  for (size_t i = 1; i <= (list->length - 1); i++) {
    list->data[i - 1] = list->data[i];
  }
  list->data[list->length - 1] = 0;
  list->length--;

  return 0;
}

int int_list_pop_back(int_list_t *list, int *val) {
  /* Pre-Check */
  if (list->length == 0) {
    return -1;
  }

  /* Pop back */
  *val = list->data[list->length - 1];
  list->data[list->length - 1] = 0;
  list->length--;

  return 0;
}

int int_list_pop(int_list_t *list, const size_t index, int *val) {
  /* Pre-Check */
  if (list->length == 0) {
    return -1;
  } else if (index < 0 || index > (list->length - 1)) {
    return -1;
  }

  /* Get value */
  *val = list->data[index];

  /* Shift the list */
  size_t end = list->length - 1;
  for (size_t i = end; i > index; i--) {
    list->data[i - 1] = list->data[i];
  }
  list->data[list->length - 1] = 0;
  list->length--;

  return 0;
}

int int_list_get(int_list_t *list, const size_t index, int *val) {
  /* Pre-Check */
  if (list->length == 0) {
    return -1;
  } else if (index < 0 || index > (list->length - 1)) {
    return -1;
  }

  /* Get value */
  *val = list->data[index];
  return 0;
}
