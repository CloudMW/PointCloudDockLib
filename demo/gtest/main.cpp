//
// Created by Mengfanyong on 2025/12/18.
//
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

/*// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) {
    // Expect two strings not to be equal.
    EXPECT_STRNE("hello", "world");
    // Expect equality.
    EXPECT_EQ(7 * 6, 42);
}*/


// A simple function to demonstrate testing
int Factorial(int n)
{
    return n*2;
}

// Test case for the Factorial function
TEST(FactorialTest, HandlesPositiveInput) {
    EXPECT_EQ(Factorial(1), 2);
    EXPECT_EQ(Factorial(2), 4);
    EXPECT_EQ(Factorial(3), 6);
    EXPECT_EQ(Factorial(4), 8);
    EXPECT_EQ(Factorial(5), 10);
}
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    spdlog::info("Starting tests...");
    return RUN_ALL_TESTS();
}