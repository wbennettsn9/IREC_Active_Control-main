#include <unity.h>
#include "Vmath.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_vector_addition() {
    Vec3 a(1, 2, 3);
    Vec3 b(4, 5, 6);

    Vec3 result = a + b;
    TEST_ASSERT_EQUAL_FLOAT(5, result.x);
    TEST_ASSERT_EQUAL_FLOAT(7, result.y);
    TEST_ASSERT_EQUAL_FLOAT(9, result.z);
}

void test_vector_addition_zeros() {
    Vec3 a(0, 0, 0);
    Vec3 b(1, 2, 3);
    Vec3 result = a + b;

    TEST_ASSERT_EQUAL_FLOAT(1, result.x);
    TEST_ASSERT_EQUAL_FLOAT(2, result.y);
    TEST_ASSERT_EQUAL_FLOAT(3, result.z);
}

void test_vector_addition_negative() {
    Vec3 a(5, 5, 5);
    Vec3 b(-2, -3, -1);
    Vec3 result = a + b;

    TEST_ASSERT_EQUAL_FLOAT(3, result.x);
    TEST_ASSERT_EQUAL_FLOAT(2, result.y);
    TEST_ASSERT_EQUAL_FLOAT(4, result.z);
}

void test_vector_addition_commutative() {
    Vec3 a(1, 2, 3);
    Vec3 b(4, 5, 6);

    Vec3 result1 = a + b;
    Vec3 result2 = b + a;

    TEST_ASSERT_EQUAL_FLOAT(result1.x, result2.x);
    TEST_ASSERT_EQUAL_FLOAT(result1.y, result2.y);
    TEST_ASSERT_EQUAL_FLOAT(result1.z, result2.z);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_vector_addition);
    RUN_TEST(test_vector_addition_zeros);
    RUN_TEST(test_vector_addition_negative);
    RUN_TEST(test_vector_addition_commutative);
    UNITY_END();
}
