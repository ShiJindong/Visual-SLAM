#include <iostream>
#include <gtest/gtest.h>

int sum(int a, int b) {
	return a+b;
}

TEST(sum, testSum) {
	EXPECT_EQ(5, sum(2, 3));	// 求合2+3=5
	EXPECT_NE(3, sum(3, 4));	// 求合3+4 != 3
}

int main(int argc, char **argv)
{
 testing::InitGoogleTest(&argc, argv);
 return RUN_ALL_TESTS();
}
