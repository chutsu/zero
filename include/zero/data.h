#ifndef DATA_H
#define DATA_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define INT_LIST_MAX_SIZE 100
#define FLOAT_LIST_MAX_SIZE 100

typedef struct int_list_t {
  int data[INT_LIST_MAX_SIZE];
  size_t length;
} int_list_t;

void int_list_init(int_list_t *list);
void int_list_reset(int_list_t *list);
void int_list_print(int_list_t *list);
int int_list_push_front(int_list_t *list, const int val);
int int_list_push_back(int_list_t *list, const int val);
int int_list_pop_front(int_list_t *list, int *val);
int int_list_pop_back(int_list_t *list, int *val);
int int_list_pop(int_list_t *list, const size_t index, int *val);
int int_list_get(int_list_t *list, const size_t index, int *val);

#endif // DATA_H
