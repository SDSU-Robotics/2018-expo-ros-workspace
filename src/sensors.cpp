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