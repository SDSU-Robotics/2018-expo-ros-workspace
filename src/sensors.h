#pragma once

#include <pigpiod_if2.h>
#include "mcp3008Spi.h"

class ADCsensor
{
public:
	ADCsensor(mcp3008Spi* adc, int channel);
	double getValue();
	
private:
	mcp3008Spi* adc_;
	int channel_;
};

/*class USsensor
{
public:
	USsensor(int pi, int pin);
	double getDistance();
	
private:
	int pi_;
	int pin_;
};*/