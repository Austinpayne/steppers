#include "include/queue.h"
#include "unity/src/unity.h"

#define INIT tuple_queue_t steps; init(&steps)
#define INIT_DF done_func df = donef
#define DF_RET 26

static int donef(void) {
    return DF_RET;
}

void test_queue_init(void) {
    INIT;
    TEST_ASSERT_EQUAL_INT(steps.queue, steps.head);
    TEST_ASSERT_EQUAL_INT(steps.queue, steps.end);
    int i;
    for (i = 0; i < SIZE; i++) {
        TEST_ASSERT_EQUAL_INT(0, steps.queue[i].x);
        TEST_ASSERT_EQUAL_INT(0, steps.queue[i].y);
        TEST_ASSERT_NULL(steps.queue[i].done);
    }
}

void test_queue_add(void) {
    INIT; INIT_DF;
    add(&steps, 3, 5, df);
    TEST_ASSERT_EQUAL_INT(3, steps.queue[0].x);
    TEST_ASSERT_EQUAL_INT(5, steps.queue[0].y);
    TEST_ASSERT_NOT_NULL(steps.queue[0].done);
    TEST_ASSERT_EQUAL_INT(DF_RET, steps.queue[0].done());
    TEST_ASSERT_EQUAL_INT(steps.queue, steps.head);
    TEST_ASSERT_EQUAL_INT(&steps.queue[1], steps.end);
}

void test_queue_rm(void) {
    INIT; INIT_DF;
    add(&steps, 3, 5, df);
    tuple_t removed = rm(&steps);
    TEST_ASSERT_EQUAL_INT(3, removed.x);
    TEST_ASSERT_EQUAL_INT(5, removed.y);
    TEST_ASSERT_NOT_NULL(removed.done);
    TEST_ASSERT_EQUAL_INT(DF_RET, removed.done());
    TEST_ASSERT_EQUAL_INT(&steps.queue[1], steps.head);
    TEST_ASSERT_EQUAL_INT(&steps.queue[1], steps.end);
}

void test_queue_empty(void) {
    INIT;
    TEST_ASSERT_TRUE(is_empty(&steps));
}

void test_queue_not_full(void) {
    INIT;
    TEST_ASSERT_FALSE(is_full(&steps));
}

void test_queue_full(void) {
    INIT;
    int i;
    for (i = 0; i < SIZE; i++) {
        add(&steps, i+1, i+1, NULL);
    }
    TEST_ASSERT_TRUE(is_full(&steps));
    TEST_ASSERT_FALSE(is_empty(&steps));
    TEST_ASSERT_EQUAL_INT(steps.queue, steps.head);
    TEST_ASSERT_EQUAL_INT(steps.queue, steps.end);
}

void test_queue_add_full_fail(void) {
    INIT;
    int i;
    for (i = 0; i < SIZE; i++) {
        TEST_ASSERT_TRUE_MESSAGE(add(&steps, i+1, i+1, NULL), "failed to add");
    }
    TEST_ASSERT_TRUE(is_full(&steps));
    TEST_ASSERT_EQUAL_INT(steps.queue, steps.head);
    TEST_ASSERT_EQUAL_INT(steps.queue, steps.end);
    TEST_ASSERT_FALSE(add(&steps, 12, 12, NULL));
}

void test_queue_fill_empty(void) {
    INIT;
    int i,j;
    int fill_empty_times = 10; // fill/empty x times
    for (i = 0; i < fill_empty_times; i++) {
        for (j = 0; j < SIZE; j++) {
            TEST_ASSERT_TRUE(add(&steps, j+1, j+1, NULL));
            TEST_ASSERT_EQUAL_INT(steps.queue, steps.head);
            if (j < SIZE-1)
                TEST_ASSERT_EQUAL_INT(&(steps.queue[j+1]), steps.end);
            else // wrap
                TEST_ASSERT_EQUAL_INT(steps.queue, steps.end);
        }
        TEST_ASSERT_TRUE(is_full(&steps));
        for (j = 0; j < SIZE; j++) {
            rm(&steps);
            TEST_ASSERT_EQUAL_INT(steps.queue, steps.end);
            if (j < SIZE-1)
                TEST_ASSERT_EQUAL_INT(&(steps.queue[j+1]), steps.head);
            else // wrap
                TEST_ASSERT_EQUAL_INT(steps.queue, steps.head);
        }
        TEST_ASSERT_TRUE(is_empty(&steps));
    }
}

void test_queue_lock_step_fill_empty(void) {
    INIT;
    int i,j;
    int fill_empty_times = 10; // fill/empty x times
    for (i = 0; i < fill_empty_times; i++) {
        for (j = 0; j < SIZE; j++) {
            // printf("i=%d, j=%d\n", i, j);
            TEST_ASSERT_TRUE_MESSAGE(add(&steps, j+1, j+1, NULL), "add failed");
            tuple_t removed = rm(&steps);
            TEST_ASSERT_EQUAL_INT_MESSAGE(j+1, removed.x, "unexpected x removed");
            TEST_ASSERT_EQUAL_INT_MESSAGE(j+1, removed.y, "unexpected y removed");
            TEST_ASSERT_NULL_MESSAGE(removed.done, "unexpected done removed");
            if (j < SIZE-1) {
                TEST_ASSERT_EQUAL_INT_MESSAGE(&(steps.queue[j+1]), steps.head, "unexpected head index");
                TEST_ASSERT_EQUAL_INT_MESSAGE(&(steps.queue[j+1]), steps.end, "unexpected end index");
            } else { // wrap
                TEST_ASSERT_EQUAL_INT_MESSAGE(steps.queue, steps.head, "unexpected head index after wrap");
                TEST_ASSERT_EQUAL_INT_MESSAGE(steps.queue, steps.end, "unexpected end index after wrap");
            }
        }
    }
}

void test_queue_empty_rm(void) {
    INIT;
    tuple_t removed = rm(&steps);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, removed.x, "unexpected x removed");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, removed.y, "unexpected y removed");
    TEST_ASSERT_NULL(removed.done);
    TEST_ASSERT_EQUAL_INT_MESSAGE(steps.queue, steps.head, "unexpected head index");
    TEST_ASSERT_EQUAL_INT_MESSAGE(steps.queue, steps.end, "unexpected end index");
}

void test_queue_invalid_add(void) {
    INIT;
    TEST_ASSERT_FALSE(add(&steps, 0, 0, NULL));
}

void test_queue_valid_adds(void) {
    INIT;
    TEST_ASSERT_TRUE_MESSAGE(add(&steps, 0, 1, NULL), "expected to be able to add (0,1)");
    TEST_ASSERT_TRUE_MESSAGE(add(&steps, 1, 0, NULL), "expected to be able to add (1,0)");
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_queue_init);
    RUN_TEST(test_queue_add);
    RUN_TEST(test_queue_rm);
    RUN_TEST(test_queue_empty);
    RUN_TEST(test_queue_not_full);
    RUN_TEST(test_queue_full);
    RUN_TEST(test_queue_add_full_fail);
    RUN_TEST(test_queue_fill_empty);
    RUN_TEST(test_queue_lock_step_fill_empty);
    RUN_TEST(test_queue_empty_rm);
    RUN_TEST(test_queue_invalid_add);
    RUN_TEST(test_queue_valid_adds);
    return UNITY_END();
}
