#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <iomanip>

int main(int argc, char* argv[])
{
	std::cout << std::setprecision(4) << std::fixed;
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}