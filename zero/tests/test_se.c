#include "munit.h"

int test_load() { return 0; }

void test_suite() { MU_ADD_TEST(test_load); }

MU_RUN_TESTS(test_suite)
