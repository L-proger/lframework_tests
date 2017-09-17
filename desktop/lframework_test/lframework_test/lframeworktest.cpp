#include "lframework/UnitTest/UnitTest.h"
#include "lframework/Thread/Thread.h"
#include <iostream>
#include "lframework/MCU/USBDevice/USBTypes.h"

using namespace LFramework;

TEST_IMPORT(BitField_8bit)
TEST_IMPORT(Thread_Create)

int main(){
	Testing::runAllTests();
	std::cin.get();
}

