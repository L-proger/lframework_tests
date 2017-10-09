/*
 * main.cpp
 *
 *  Created on: 18 July. 2017 ã.
 *      Author: l-pro
 */

#include "lframework/Thread/Thread.h"
#include "lframework/Thread/Semaphore.h"
#include "lframework/UnitTest/UnitTest.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f7xx_hal.h"
#include <cassert>
#include <cstdlib>
#include "ili9325/ili9325.h"
#include "ili9325/ili9325_fsmc.h"
#include "lmath/lmath/lmath.h"
#include "lframework/MCU/USBDevice/USBDevice.h"
#include "lframework/MCU/USBDevice/UsbInternalFsDeviceDriver.h"
#include "lframework/UnitTest/UnitTest.h"

SRAM_HandleTypeDef hsram1;

using namespace Stm32PlusPlus::Display;

using namespace LFramework::USB;
using namespace LFramework;

typedef Ili9325<DisplayFsmcInterface> Lcd;

using namespace lm;

extern "C" void _exit(int code){
	for(;;){}
}

LFramework::Thread ledThread;

void ledTask(){
	__HAL_RCC_GPIOE_CLK_ENABLE();

	GPIO_InitTypeDef init;
	init.Mode = GPIO_MODE_OUTPUT_PP;
	init.Pin = GPIO_PIN_0;
	init.Pull = GPIO_NOPULL;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &init);

	for(;;){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
		vTaskDelay(1000);
	}
}

struct Color24{
	uint8_t r;
	uint8_t g;
	uint8_t b;

	void fromColor16(uint16_t value){
		b = (value & 0x1f) << 3;
		g = (value & 0x7e0) >> 3;
		r = (value & 0xf800) >> 8;
	}

	uint16_t toColor16(){
		uint16_t result = b >> 3;
		result |= ((uint16_t)g >> 2) << 5;
		result |= ((uint16_t)r >> 3) << 11;
		return result;
	}

	static Color24 lerp(Color24 left, Color24 right, uint16_t val, uint16_t maxVal){
		Color24 result;
		result.r = (int32_t)left.r + ((int32_t)(right.r - left.r) * (int32_t)val) / (int32_t)maxVal;
		result.g = (int32_t)left.g + ((int32_t)(right.g - left.g) * (int32_t)val) / (int32_t)maxVal;
		result.b = (int32_t)left.b + ((int32_t)(right.b - left.b) * (int32_t)val) / (int32_t)maxVal;

		return result;
	}
};

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

UsbInterface<int, float> itf0;


template<typename ... Interfaces>
struct UsbFunction{

};

typedef UsbInterface<int, float> CdcControlInterface;
typedef UsbInterface<int, float> CdcDataInterface;

struct UsbCdcFunction : public UsbFunction<CdcControlInterface, CdcDataInterface>{

};

typedef UsbConfiguration<UsbCdcFunction> C0;
typedef UsbConfiguration<UsbCdcFunction> C1;


C0 config0;
C1 config1;

UsbDevice<Color24, C0, C1> usb(&hpcd_USB_OTG_FS, &config0, &config1);


TEST_IMPORT(BitField_8bit)


void mainThread(){

	if(!TEST_RUN(BitField_8bit)){
		for(;;);
	}

	float2 f2;
	float3 f3;
	float3 right = f3.unitX();
	float4 f4;

	LFramework::ThisThread::setPriority(LFramework::ThreadPriority::Normal);

	ledThread = std::move(LFramework::Thread("led", 256, LFramework::ThreadPriority::Normal, ledTask));

	DisplayFsmcInterface::init();

	Lcd::init();

	Color24 color;
	color.r = 0;
	color.g = 0;
	color.b = 0xff;

	Lcd::clear(color.toColor16());

	usb.descriptor.classDescription = UsbClass::Device::VendorSpecific(0,0);
	usb.descriptor.bMaxPacketSize0 = 64;
	usb.descriptor.bNumConfigurations = 1;
	usb.descriptor.bcdDevice = UsbVersion(1);
	usb.descriptor.bcdUSB = UsbVersion(2);
	usb.descriptor.idProduct = 0;
	usb.descriptor.idVendor = 0x0302;
	//usb.descriptor.

	usb.initPHY();
	usb.start();

	while(true){
		vTaskDelay(1000);
	}
}

extern "C" {
	void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
		usb.onDataOut(epnum);
	}
	void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
		usb.onDataIn(epnum);
	}
	void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
		usb.onSetup();
	}
	void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd){
		usb.onSOF();
	}
	void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
		usb.onReset();
	}
	void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
		usb.onSuspend();
	}
	void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
		usb.onResume();
	}
	void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
		usb.onIsochronousOutIncomplete(epnum);
	}
	void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
		usb.onIsochronousInIncomplete(epnum);
	}
	void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd){
		usb.onConnect();
	}
	void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd){
		usb.onDisconnect();
	}
}



extern "C" void __libc_init_array (void);
extern "C" void mainThreadLauncher(){
	//Manually call global C++ objects constructors (don't forget to remove __libc_init_array call from standard startup code)
	__libc_init_array();
	mainThread();
}
