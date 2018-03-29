#include "sensors.h"

void ADCsensor::init(mcp3008Spi* adc, int channel)
{
	adc_ = adc;
	channel_ = channel;
}

unsigned int ADCsensor::getValue()
{
	unsigned char data[3];
	unsigned int adcVal;
	
	data[0] = 1;  //  first byte transmitted -> start bit
	data[1] = 0b10000000 |( ((channel_ & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
	data[2] = 0; // third byte transmitted....don't care

	adc_->spiWriteRead(data, sizeof(data) );

	adcVal = 0;
	adcVal = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
	adcVal |=  (data[2] & 0xff);
	
	return adcVal;
}

void Encoder::init(int pi, int address)
{
	fd_ = i2c_open(pi, 1, address, 0);
	pi_ = pi;
}

int Encoder::getCount()
{
	char data[4];
	i2c_read_i2c_block_data(pi_, fd_, 0, data, 4);
	
	int result = 0;
	result |= (data[0] << 24) & 0xFFFFFFFF;
	result |= (data[1] << 16) & 0xFFFFFFFF;
	result |= (data[2] << 8) & 0xFFFFFFFF;
	result |= data[3] & 0xFFFFFFFF;
		
	return result;
}