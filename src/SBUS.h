#pragma once
#include <cstdint>
#include <string>
#include <functional>
#include <thread>
#include <memory>

#if !defined(SBUS_STATIC)
#    if defined(BUILD_SBUS)
#        define SBUS_EXPORT __declspec(dllexport)
#    else
#        define SBUS_EXPORT __declspec(dllimport)
#    endif
#else
#    define SBUS_EXPORT
#endif // SBUS_STATIC

namespace serial
{
	class Serial;
}

// Save parsed SBUS data.
struct SBUS_EXPORT SBUS_Value
{
	uint16_t channels[16];
	bool digital1;	// digital channel1 (ch17)
	bool digital2;	// digital channel2 (ch18)
	bool frameLost;
	bool failSafe;
	SBUS_Value():channels{}, digital1(false), digital2(false), frameLost(false), failSafe(false)
	{}
};

constexpr uint16_t SBUS_Buffer_Max = 256;
constexpr uint8_t SBUS_Frame_Size = 25;
constexpr uint8_t SBUS_Header = 0x0F;
constexpr uint16_t SBUS_Channel_Mask = 0x07FF;
constexpr uint8_t SBUS_Digital1_Mask = 0x01;
constexpr uint8_t SBUS_Digital2_Mask = 0x02;
constexpr uint8_t SBUS_Frame_Lost_Mask = 0x04;
constexpr uint8_t SBUS_Fail_Safe_Mask = 0x08;
constexpr uint8_t SBUS_End_Byte0 = 0x00;
// Seems some models of futaba will use the bytes bellow as endbyte.
constexpr uint8_t SBUS_End_Byte1 = 0x04;
constexpr uint8_t SBUS_End_Byte2 = 0x14;
constexpr uint8_t SBUS_End_Byte3 = 0x24;
constexpr uint8_t SBUS_End_Byte4 = 0x34;

typedef std::function<void(SBUS_Value)> SBUSCallback;

class SBUS_EXPORT SBUS
{
public:
	SBUS();
	~SBUS();

	/**
	 * \brief Connect to SBUS serial interface.
	 * \param port Defines which serial port to connect.
	 * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
	 * \param buadrate Default 100000 with fixed 8E2.
	 * \return Whether opening the serial port is successful.
	 */
	bool connect(const std::string& port, int buadrate = 100000);

	/**
	 * \brief Disconnect to serial interface.
	 */
	void disconnect();

	/**
	 * \brief Start reading serial port data.
	 */
	void startReading();

	/**
	 * \brief Stop reading serial port data.
	 */
	void stopReading();

	bool isConnected() const;

	/**
	 * \brief Set the callback function.
	 * \param func Called when a SBUS frame come in.
	 */
	void setCallback(const SBUSCallback& func) { callback_ = func; }

	uint64_t getFrameLostNumber() const { return cntFrameLost_; }

	/**
	 * \brief Write a S.BUS frame by SBUS_Value
	 * \param value S.BUS frame
	 */
	void writeSBUS(const SBUS_Value& value);

private:
	void readSerialPort();
	void incomingData();
	
private:
	serial::Serial* serialPort_;
	uint8_t buffer_[SBUS_Buffer_Max];
	size_t bufferFront_,bufferEnd_;
	SBUSCallback callback_;
	std::unique_ptr<std::thread> readThreadPtr_;
	bool reading_;
	uint64_t cntFrameLost_;
};

