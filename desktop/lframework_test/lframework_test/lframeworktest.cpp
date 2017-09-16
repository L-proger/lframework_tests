#include "lframework/UnitTest/UnitTest.h"
#include "lframework/Thread/Thread.h"
#include <iostream>

using namespace LFramework;

TEST_IMPORT(BitField_8bit)
TEST_IMPORT(Thread_Create)

int main()
{
	auto result = TEST_RUN(BitField_8bit);
	result = TEST_RUN(Thread_Create);
	std::cin.get();
}

