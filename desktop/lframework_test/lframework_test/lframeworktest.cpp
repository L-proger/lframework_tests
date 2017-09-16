#include "lframework/UnitTest/UnitTest.h"
#include <iostream>

using namespace LFramework;

TEST_IMPORT(BitField_8bit)

int main()
{
	auto result = TEST_RUN(BitField_8bit);
	std::cin.get();
}

