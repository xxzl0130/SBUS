#include "SBUS.h"
#include <iostream>
#include <serial/serial.h>
#include <Windows.h>
using namespace std;

SBUS::SBUS():
	serialPort_(nullptr),
	buffer_{},
	bufferFront_(0),
	bufferEnd_(0),
	reading_(false),
	cntFrameLost_(0)
{
}

SBUS::~SBUS()
= default;

bool SBUS::connect(const std::string& port, int buadrate)
{
	try 
	{
		if (!this->serialPort_)
		{
			this->serialPort_ = new serial::Serial(port, buadrate, serial::Timeout(),
				serial::eightbits, serial::parity_even, serial::stopbits_two);
		}
		if(!this->serialPort_->isOpen())
		{
			delete this->serialPort_;
			this->serialPort_ = nullptr;
			return false;
		}
		return true;
	}
	catch (const std::exception& e)
	{
		cerr << "Open serial port error!\n" << e.what();
		if(serialPort_ != nullptr)
		{
			delete serialPort_;
			serialPort_ = nullptr;
		}
		return false;
	}
}

void SBUS::disconnect()
{
	reading_ = false;
	try
	{
		if(this->serialPort_ != nullptr && this->serialPort_->isOpen())
		{
			this->serialPort_->close();
			delete this->serialPort_;
			this->serialPort_ = nullptr;
		}
	}
	catch (std::exception& e)
	{
		cerr << "Error during disconnect: " << e.what();
	}
}

void SBUS::startReading()
{
	if (this->serialPort_ && this->serialPort_->isOpen())
	{
		reading_ = true;
		readThreadPtr_.reset(new std::thread(std::bind(&SBUS::readSerialPort, this)));
		readThreadPtr_->detach();
	}
}

void SBUS::stopReading()
{
	reading_ = false;
}

bool SBUS::isConnected() const
{
	return this->serialPort_ != nullptr && this->serialPort_->isOpen();
}

void SBUS::writeSBUS(const SBUS_Value& value)
{
	static uint8_t data[SBUS_Frame_Size] = {SBUS_Header};
	if (serialPort_ == nullptr || !serialPort_->isOpen())
		return;
	data[1] = (uint8_t)((value.channels[0] & SBUS_Channel_Mask));
	data[2] = (uint8_t)((value.channels[0] & SBUS_Channel_Mask) >> 8 | (value.channels[1] & SBUS_Channel_Mask) << 3);
	data[3] = (uint8_t)((value.channels[1] & SBUS_Channel_Mask) >> 5 | (value.channels[2] & SBUS_Channel_Mask) << 6);
	data[4] = (uint8_t)((value.channels[2] & SBUS_Channel_Mask) >> 2);
	data[5] = (uint8_t)((value.channels[2] & SBUS_Channel_Mask) >> 10 | (value.channels[3] & SBUS_Channel_Mask) << 1);
	data[6] = (uint8_t)((value.channels[3] & SBUS_Channel_Mask) >> 7 | (value.channels[4] & SBUS_Channel_Mask) << 4);
	data[7] = (uint8_t)((value.channels[4] & SBUS_Channel_Mask) >> 4 | (value.channels[5] & SBUS_Channel_Mask) << 7);
	data[8] = (uint8_t)((value.channels[5] & SBUS_Channel_Mask) >> 1);
	data[9] = (uint8_t)((value.channels[5] & SBUS_Channel_Mask) >> 9 | (value.channels[6] & SBUS_Channel_Mask) << 2);
	data[10] = (uint8_t)((value.channels[6] & SBUS_Channel_Mask) >> 6 | (value.channels[7] & SBUS_Channel_Mask) << 5);
	data[11] = (uint8_t)((value.channels[7] & SBUS_Channel_Mask) >> 3);
	data[12] = (uint8_t)((value.channels[8] & SBUS_Channel_Mask));
	data[13] = (uint8_t)((value.channels[8] & SBUS_Channel_Mask) >> 8 | (value.channels[9] & SBUS_Channel_Mask) << 3);
	data[14] = (uint8_t)((value.channels[9] & SBUS_Channel_Mask) >> 5 | (value.channels[10] & SBUS_Channel_Mask) << 6);
	data[15] = (uint8_t)((value.channels[10] & SBUS_Channel_Mask) >> 2);
	data[16] = (uint8_t)((value.channels[10] & SBUS_Channel_Mask) >> 10 | (value.channels[11] & SBUS_Channel_Mask) << 1);
	data[17] = (uint8_t)((value.channels[11] & SBUS_Channel_Mask) >> 7 | (value.channels[12] & SBUS_Channel_Mask) << 4);
	data[18] = (uint8_t)((value.channels[12] & SBUS_Channel_Mask) >> 4 | (value.channels[13] & SBUS_Channel_Mask) << 7);
	data[19] = (uint8_t)((value.channels[13] & SBUS_Channel_Mask) >> 1);
	data[20] = (uint8_t)((value.channels[13] & SBUS_Channel_Mask) >> 9 | (value.channels[14] & SBUS_Channel_Mask) << 2);
	data[21] = (uint8_t)((value.channels[14] & SBUS_Channel_Mask) >> 6 | (value.channels[15] & SBUS_Channel_Mask) << 5);
	data[22] = (uint8_t)((value.channels[15] & SBUS_Channel_Mask) >> 3);
	data[23] = 0x00;
	data[24] = SBUS_End_Byte0;
	try 
	{
		serialPort_->write(data, SBUS_Frame_Size);
		serialPort_->flush();
	}
	catch (const std::exception& e)
	{
		cerr << "Error during write serial." << e.what();
	}
}

void SBUS::readSerialPort()
{
	while(reading_)
	{
		// One SBUS frame is 25 bytes
		if(this->bufferEnd_ + SBUS_Frame_Size >= SBUS_Buffer_Max)
		{
			memset(this->buffer_, 0, sizeof this->buffer_);
			this->bufferEnd_ = 0;
			this->bufferFront_ = 0;
		}
		try 
		{
			if (!this->serialPort_)
			{
				Sleep(1);
				continue;
			}
			auto len = this->serialPort_->read(this->buffer_ + this->bufferFront_, SBUS_Frame_Size);
			if (len)
			{
				this->bufferEnd_ += len;
				this->incomingData();
			}
			Sleep(1);
		}
		catch (const std::exception& e)
		{
			cerr << "Error during reading.\n" << e.what();
		}
	}
}

bool checkEndByte(uint8_t byte)
{
	return byte == SBUS_End_Byte0 || byte == SBUS_End_Byte1 || byte == SBUS_End_Byte2 || byte == SBUS_End_Byte4
		|| byte== SBUS_End_Byte3;
}

void SBUS::incomingData()
{
	for(auto i = this->bufferFront_;i <= this->bufferEnd_ - SBUS_Frame_Size;++i)
	{
		if(this->buffer_[i] != SBUS_Header)
			continue;
		if(!checkEndByte(this->buffer_[i + SBUS_Frame_Size - 1]))
			continue;
		SBUS_Value value;
		auto* pData = this->buffer_ + i;
		value.channels[0] = ((pData[1] | pData[2] << 8) & SBUS_Channel_Mask);
		value.channels[1] = ((pData[2] >> 3 | pData[3] << 5) & SBUS_Channel_Mask);
		value.channels[2] = ((pData[3] >> 6 | pData[4] << 2 | pData[5] << 10) & SBUS_Channel_Mask);
		value.channels[3] = ((pData[5] >> 1 | pData[6] << 7) & SBUS_Channel_Mask);
		value.channels[4] = ((pData[6] >> 4 | pData[7] << 4) & SBUS_Channel_Mask);
		value.channels[5] = ((pData[7] >> 7 | pData[8] << 1 | pData[9] << 9) & SBUS_Channel_Mask);
		value.channels[6] = ((pData[9] >> 2 | pData[10] << 6) & SBUS_Channel_Mask);
		value.channels[7] = ((pData[10] >> 5 | pData[11] << 3) & SBUS_Channel_Mask);
		value.channels[8] = ((pData[12] | pData[13] << 8) & SBUS_Channel_Mask);
		value.channels[9] = ((pData[13] >> 3 | pData[14] << 5) & SBUS_Channel_Mask);
		value.channels[10] = ((pData[14] >> 6 | pData[15] << 2 | pData[16] << 10) & SBUS_Channel_Mask);
		value.channels[11] = ((pData[16] >> 1 | pData[17] << 7) & SBUS_Channel_Mask);
		value.channels[12] = ((pData[17] >> 4 | pData[18] << 4) & SBUS_Channel_Mask);
		value.channels[13] = ((pData[18] >> 7 | pData[19] << 1 | pData[20] << 9) & SBUS_Channel_Mask);
		value.channels[14] = ((pData[20] >> 2 | pData[21] << 6) & SBUS_Channel_Mask);
		value.channels[15] = ((pData[21] >> 5 | pData[22] << 3) & SBUS_Channel_Mask);
		value.digital1 = pData[23] & SBUS_Digital1_Mask;
		value.digital2 = pData[23] & SBUS_Digital2_Mask;
		value.frameLost = pData[23] & SBUS_Frame_Lost_Mask;
		value.failSafe = pData[23] & SBUS_Fail_Safe_Mask;
		if(value.frameLost)
		{
			++this->cntFrameLost_;
		}
		if(this->callback_)
		{
			this->callback_(value);
		}
		this->bufferFront_ += SBUS_Frame_Size;
		i += SBUS_Frame_Size - 1;
	}
	if(SBUS_Buffer_Max - this->bufferFront_ < SBUS_Frame_Size)
	{
		memcpy(this->buffer_, this->buffer_ + this->bufferFront_, this->bufferEnd_ - this->bufferFront_);
		this->bufferEnd_ -= this->bufferFront_;
		this->bufferFront_ = 0;
	}
	if(this->bufferFront_ == this->bufferEnd_)
	{
		this->bufferFront_ = this->bufferEnd_ = 0;
	}
}
