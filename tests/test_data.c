#include "zero/munit.h"
#include "zero/data.h"

int test_int_list_init() {
  int_list_t list;

  int_list_init(&list);
  MU_CHECK(list.data[0] == 0);
  MU_CHECK(list.length == 0);

  return 0;
}

int test_int_list_reset() {
  int_list_t list;

  list.length = 1;
  int_list_reset(&list);
  MU_CHECK(list.data[0] == 0);
  MU_CHECK(list.length == 0);

  return 0;
}

int test_int_list_push_front() {
  int_list_t list;

  int_list_init(&list);
  int_list_push_front(&list, 0);
  int_list_push_front(&list, 1);
  int_list_push_front(&list, 2);
  int_list_push_front(&list, 3);

  MU_CHECK(list.data[0] == 3);
  MU_CHECK(list.data[1] == 2);
  MU_CHECK(list.data[2] == 1);
  MU_CHECK(list.data[3] == 0);
  MU_CHECK(list.length == 4);

  return 0;
}

int test_int_list_push_back() {
  int_list_t list;

  int_list_init(&list);
  int_list_push_back(&list, 0);
  int_list_push_back(&list, 1);
  int_list_push_back(&list, 2);
  int_list_push_back(&list, 3);

  MU_CHECK(list.data[0] == 0);
  MU_CHECK(list.data[1] == 1);
  MU_CHECK(list.data[2] == 2);
  MU_CHECK(list.data[3] == 3);
  MU_CHECK(list.length == 4);

  return 0;
}

int test_int_list_pop_front() {
  int_list_t list;

  // Test pop front when list is empty
  int val = 0;
  int retval = 0;
  int_list_init(&list);
  retval = int_list_pop_front(&list, &val);
  MU_CHECK(retval == -1);
  MU_CHECK(val == 0);

  // Test pop front when list is not empty
  int_list_init(&list);
  int_list_push_back(&list, 0);
  int_list_push_back(&list, 1);
  int_list_push_back(&list, 2);
  int_list_push_back(&list, 3);

  retval = int_list_pop_front(&list, &val);
  MU_CHECK(retval == 0);
  MU_CHECK(val == 0);
  MU_CHECK(list.data[0] == 1);
  MU_CHECK(list.data[1] == 2);
  MU_CHECK(list.data[2] == 3);
  MU_CHECK(list.length == 3);

  return 0;
}

int test_int_list_pop_back() {
  int_list_t list;

  // Test pop back when list is empty
  int val = 0;
  int retval = 0;
  int_list_init(&list);
  retval = int_list_pop_back(&list, &val);
  MU_CHECK(retval == -1);
  MU_CHECK(val == 0);

  // Test pop back when list is not empty
  int_list_init(&list);
  int_list_push_back(&list, 0);
  int_list_push_back(&list, 1);
  int_list_push_back(&list, 2);
  int_list_push_back(&list, 3);

  retval = int_list_pop_back(&list, &val);
  MU_CHECK(retval == 0);
  MU_CHECK(val == 3);
  MU_CHECK(list.data[0] == 0);
  MU_CHECK(list.data[1] == 1);
  MU_CHECK(list.data[2] == 2);
  MU_CHECK(list.length == 3);

  return 0;
}

int test_int_list_pop() {
  int_list_t list;

  // Test pop when list is empty
  int val = 0;
  int retval = 0;
  int_list_init(&list);
  retval = int_list_pop(&list, 2, &val);
  MU_CHECK(retval == -1);
  MU_CHECK(val == 0);

  // Test pop when list is not empty
  int_list_push_back(&list, 0);
  int_list_push_back(&list, 1);
  int_list_push_back(&list, 2);
  int_list_push_back(&list, 3);

  retval = int_list_pop(&list, 2, &val);
  MU_CHECK(retval == 0);
  MU_CHECK(val == 2);
  MU_CHECK(list.data[0] == 0);
  MU_CHECK(list.data[1] == 1);
  MU_CHECK(list.data[2] == 3);
  MU_CHECK(list.length == 3);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_int_list_init);
  MU_ADD_TEST(test_int_list_reset);
  MU_ADD_TEST(test_int_list_push_front);
  MU_ADD_TEST(test_int_list_push_back);
  MU_ADD_TEST(test_int_list_pop_front);
  MU_ADD_TEST(test_int_list_pop_back);
  MU_ADD_TEST(test_int_list_pop);
}

MU_RUN_TESTS(test_suite);
