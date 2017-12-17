#include "lframework/UnitTest/UnitTest.h"
#include "lframework/Thread/Thread.h"
#include <iostream>
#include "lframework/MCU/USBDevice/USBTypes.h"
#include "lframework/MCU/USBDevice/USBDevice.h"
#include "lframework/MCU/USBDevice/PHY/Stm32FsPhy.h"
#include "lframework/MCU/USBDevice/PHY/UsbDeviceSoftPhy.h"
#include <Windows.h>
#include <thread>
#include <array>

#include "lframework/Detect/DetectOS.h"
#include "lframework/Detect/DetectEndianness.h"
#include "lframework/Detect/DetectCPU.h"

using namespace LFramework;
using namespace LFramework::USB;

TEST_IMPORT(BitField_8bit)
TEST_IMPORT(Thread_Create)

template<typename Interface>
class UsbFunction {
public:
	static constexpr size_t interfacesCount = 1;
};

template<typename...Interfaces>
class UsbFunction<InterfaceAssociation<Interfaces...>> {
public:
	static constexpr size_t interfacesCount = sizeof...(Interfaces);
};

template<typename Derived>
struct CSInterfaceDescriptor : public USB::USBDescriptorInit<0x24, Derived>{};

namespace CDC {
	
	namespace FunctionalDescriptors {


		template<uint8_t DescriptorSubtype, typename Derived>
		struct FunctionalDescriptor : public CSInterfaceDescriptor<Derived> {
			uint8_t subtype = DescriptorSubtype;
		};


		struct Header : public FunctionalDescriptor<0, Header> {
			Header(UsbVersion cdcVersion = UsbVersion(1, 1)) {}
			UsbVersion cdcVersion = UsbVersion(1, 1);
		};

		struct CallManagement : public FunctionalDescriptor<1, CallManagement> {
			uint8_t capabilities;
			uint8_t dataInterface;
		};

		struct AbstractControlManagement : public FunctionalDescriptor<2, AbstractControlManagement> {
			uint8_t capabilities;
		};

		template<size_t SubordinateInterfaceCount>
		struct Union : public FunctionalDescriptor<3, Union<SubordinateInterfaceCount>> {
			uint8_t masterInterface;
			std::array<uint8_t, SubordinateInterfaceCount> subordinateInterfaces;
		};
		
	}
}

using CdcInterface0 = UsbInterface<>;
using CdcInterface1 = UsbInterface<>;


class UsbCdcUartFunction : public UsbFunction<InterfaceAssociation<CdcInterface0, CdcInterface1>> {

};

class UsbHidFunction : public UsbFunction<UsbInterface<>> {

};

using CdcAndHidConfig = UsbConfiguration<UsbCdcUartFunction, UsbHidFunction>;


namespace UsbPackets {
	static constexpr uint8_t makePID(uint8_t fourBits) {
		return static_cast<uint8_t>(((fourBits & 0xf) | (~(fourBits << 4) & 0xf0)) & 0xff);
	}

	static_assert(makePID(0b1001) == 0b01101001, "WTF");

	enum class USBSpeed {
		Low,
		Full,
		High
	};

	enum class PID : uint8_t{
		OUTToken = makePID(0b0001),
		INToken = makePID(0b1001),
		SOFToken = makePID(0b0101),
		SETUPToken = makePID(0b1101),

		DATA0 = makePID(0b0011),
		DATA1 = makePID(0b1011),
		DATA2 = makePID(0b0111),
		MDATA = makePID(0b1111),

		ACKHandshake = makePID(0b0010),
		NAKHandshake = makePID(0b1010),
		STALLHandshake = makePID(0b1110),
		NYET = makePID(0b0110),

		PREamble = makePID(0b1100), //TODO: check suspicious values
		ERR = makePID(0b1100),
		Split = makePID(0b1000),
		Ping = makePID(0b0100)
	};

#pragma pack(push, 1)
	template<USBSpeed Speed>
	struct PacketHeader {
		std::conditional_t<Speed == USBSpeed::High, uint32_t, uint8_t> sync;
		PID pid;
	};

	template<typename Header>
	struct TokenPacketData : public Header{
		constexpr TokenPacketData() : _value{ 0 } {}
		constexpr TokenPacketData(const uint16_t value) : _value{ value } {}
		union {
			uint16_t _value;
			BitFieldMember<uint16_t, 0, 7> address;
			BitFieldMember<uint16_t, 7, 4> endpoint;
			BitFieldMember<uint16_t, 11, 5> crc5;
		};
	};

	template<USBSpeed Speed>
	struct TokenPacket : public TokenPacketData<PacketHeader<Speed>> {};

	template<USBSpeed Speed>
	struct HandshakePacket : public PacketHeader<Speed> {};

	template<USBSpeed Speed>
	struct SOF : public PacketHeader<Speed> {
		SOF():_value{0}{}
		union {
			uint16_t _value;
			BitFieldMember<uint16_t, 0, 11> frameNumber;
			BitFieldMember<uint16_t, 11, 5> crc5;
		};
	};

	template<USBSpeed Speed, size_t MaxPacketSize>
	struct DataPacket : public PacketHeader<Speed> {
		static constexpr size_t getMaxPacketSize(USBSpeed speed) {
			return Speed == USBSpeed::Low ? 8 : (Speed == USBSpeed::Full ? 1023 : 1024);
		}

		static_assert(MaxPacketSize <= getMaxPacketSize(Speed), "Max packet size too big");

		std::array<uint8_t, MaxPacketSize> payload;
		uint16_t crc16;
	};
#pragma pack(pop)

	static_assert(sizeof(TokenPacket<USBSpeed::Low>) == 4, "WTF");
	static_assert(sizeof(PacketHeader<USBSpeed::Low>) == 2, "WTF");
	static_assert(sizeof(PacketHeader<USBSpeed::High>) == 5, "WTF");

	static_assert(sizeof(PacketHeader<USBSpeed::Low>::sync) == 1, "WTF");
	static_assert(sizeof(PacketHeader<USBSpeed::Full>::sync) == 1, "WTF");
	static_assert(sizeof(PacketHeader<USBSpeed::High>::sync) == 4, "WTF");
}

int main(){

	auto os = LFramework::Detect::operatingSystem();

	auto endianness = LFramework::Detect::endianness();
	auto endiannessRuntime = LFramework::Detect::endiannessRuntime();
	auto cpuArch = LFramework::Detect::cpuArchitecture();

	Testing::runAllTests();


	UsbPackets::DataPacket<UsbPackets::USBSpeed::High, 1024> p1;

	static constexpr auto v = UsbVersionInit<7,2,3>{};

	uint16_t major = v.major;
	uint16_t minor = v.minor;
	uint16_t subminor = v.subMinor;


	CDC::FunctionalDescriptors::Header header;
	header.cdcVersion = UsbVersion(1, 1);

	CDC::FunctionalDescriptors::CallManagement callManagement;
	callManagement.capabilities = 0;
	callManagement.dataInterface = 1;

	CDC::FunctionalDescriptors::AbstractControlManagement acm;
	acm.capabilities = 2;

	CDC::FunctionalDescriptors::Union<1> unionDesc;
	unionDesc.masterInterface = 0;
	unionDesc.subordinateInterfaces[0] = 1;


	


	UsbCdcUartFunction cdcFunc;
	UsbHidFunction hidFunc;

	CdcAndHidConfig config(&cdcFunc, &hidFunc);

	UsbDevice<UsbDeviceSoftPhy, CdcAndHidConfig> usbDevice(&config);

	usbDevice.start();
	usbDevice.connect();

	while (true) {
		Sleep(1000);
	}

	std::cin.get();
}

