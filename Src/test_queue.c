#include "include/queue.h"
#include "unity/src/unity.h"

#define INIT step_queue_t steps; init(&steps)

void test_queue_init(void) {
    INIT;
    TEST_ASSERT_EQUAL_INT(0, steps.head);
    TEST_ASSERT_EQUAL_INT(0, steps.end);
    int i;
    for (i = 0; i < SIZE; i++) {
        TEST_ASSERT_EQUAL_INT(0, steps.queue[i].x_steps);
        TEST_ASSERT_EQUAL_INT(0, steps.queue[i].y_steps);
    }
}

void test_queue_add(void) {
    INIT;
    add(&steps, 3, 5);
    TEST_ASSERT_EQUAL_INT(3, steps.queue[0].x_steps);
    TEST_ASSERT_EQUAL_INT(5, steps.queue[0].y_steps);
    TEST_ASSERT_EQUAL_INT(0, steps.head);
    TEST_ASSERT_EQUAL_INT(1, steps.end);
}

void test_queue_rm(void) {
    INIT;
    add(&steps, 3, 5);
    steps_t removed = rm(&steps);
    TEST_ASSERT_EQUAL_INT(3, removed.x_steps);
    TEST_ASSERT_EQUAL_INT(5, removed.y_steps);
    TEST_ASSERT_EQUAL_INT(1, steps.head);
    TEST_ASSERT_EQUAL_INT(1, steps.end);
}

void test_queue_empty(void) {
    INIT;
    TEST_ASSERT_TRUE(empty(&steps));
}

void test_queue_not_full(void) {
    INIT;
    TEST_ASSERT_FALSE(full(&steps));
}

void test_queue_full(void) {
    INIT;
    int i;
    for (i = 0; i < SIZE; i++) {
        add(&steps, i+1, i+1);
    }
    TEST_ASSERT_TRUE(full(&steps));
    TEST_ASSERT_FALSE(empty(&steps));
    TEST_ASSERT_EQUAL_INT(0, steps.head);
    TEST_ASSERT_EQUAL_INT(0, steps.end);
}

void test_queue_add_full_fail(void) {
    INIT;
    int i;
    for (i = 0; i < SIZE; i++) {
        TEST_ASSERT_TRUE(add(&steps, i+1, i+1));
    }
    TEST_ASSERT_TRUE(full(&steps));
    TEST_ASSERT_EQUAL_INT(0, steps.head);
    TEST_ASSERT_EQUAL_INT(0, steps.end);
    TEST_ASSERT_FALSE(add(&steps, 12, 12));
}

void test_queue_fill_empty(void) {
    INIT;
    int i;
    for (i = 0; i < SIZE; i++) {
        TEST_ASSERT_TRUE(add(&steps, i+1, i+1));
    }
    TEST_ASSERT_TRUE(full(&steps));
    while(!empty(&steps)) {
        rm(&steps);
    }
    TEST_ASSERT_TRUE(empty(&steps));
    TEST_ASSERT_EQUAL_INT(0, steps.head);
    TEST_ASSERT_EQUAL_INT(0, steps.end);
}

void test_queue_lock_step_fill_empty(void) {
    INIT;
    int i;
    for (i = 0; i < SIZE; i++) {
        TEST_ASSERT_TRUE_MESSAGE(add(&steps, i+1, i+1), "add failed");
        steps_t removed = rm(&steps);
        TEST_ASSERT_EQUAL_INT_MESSAGE(i+1, removed.x_steps, "unexpected x removed");
        TEST_ASSERT_EQUAL_INT_MESSAGE(i+1, removed.y_steps, "unexpected y removed");
        if (i != SIZE-1) {
            TEST_ASSERT_EQUAL_INT_MESSAGE(i+1, steps.head, "unexpected head index");
            TEST_ASSERT_EQUAL_INT_MESSAGE(i+1, steps.end, "unexpected end index");
        } else { // wrap
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, steps.head, "unexpected head index");
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, steps.end, "unexpected end index");
        }
    }
}

void test_queue_empty_rm(void) {
    INIT;
    steps_t removed = rm(&steps);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, removed.x_steps, "unexpected x removed");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, removed.y_steps, "unexpected y removed");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, steps.head, "unexpected head index");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, steps.end, "unexpected end index");
}

void test_queue_invalid_add(void) {
    INIT;
    TEST_ASSERT_FALSE(add(&steps, 0, 0));
}

void test_queue_valid_adds(void) {
    INIT;
    TEST_ASSERT_TRUE_MESSAGE(add(&steps, 0, 1), "expected to be able to add (0,1)");
    TEST_ASSERT_TRUE_MESSAGE(add(&steps, 1, 0), "expected to be able to add (1,0)");
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
