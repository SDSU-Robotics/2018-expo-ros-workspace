#include "sensors.h"

ADCsensor::ADCsensor(mcp3008Spi* adc, int channel)
{
	ADCsensor::adc_ = adc;
	ADCsensor::channel_ = channel;
}

double ADCsensor::getValue()
{
	unsigned char data[3];
	int adcVal;
	
	data[0] = 1;  //  first byte transmitted -> start bit
	data[1] = 0b10000000 |( ((channel_ & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
	data[2] = 0; // third byte transmitted....don't care

	adc_->spiWriteRead(data, sizeof(data) );

	adcVal = 0;
	adcVal = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
	adcVal |=  (data[2] & 0xff);
	
	return adcVal;
}

/*USsensor::USsensor(int pi, int pin)
{
	pin_ = pin;
	pi_ = pi;
}


double USsensor::getDistance()
{
	int readStart, pulseStart, pulseEnd, pulseDuration, error;
	double meters;
	
	// set pin to output
	error = set_mode(pi_, pin_, PI_OUTPUT);
	if (error < 0)
		std::cout << "Error: " << pigpio_error(error) << std::endl;
	
	// send trigger pulse
	error = gpio_trigger(pi_, pin_, 10, 1);
	if (error < 0)
		std::cout << "Error: " << pigpio_error(error) << std::endl;
	
	// set pin to input
	error = set_mode(pi_, pin_, PI_INPUT);
	if (error < 0)
		std::cout << "Error: " << pigpio_error(error) << std::endl;
	
	readStart = get_current_tick(pi_);
	
	// get pulse start
	while(gpio_read(pi_, pin_) == 0)
	{
		pulseStart = get_current_tick(pi_);
		if (pulseStart - readStart > 5800)
			return -1;
	}
	
	// get pulse end
	while(gpio_read(pi_, pin_) == 1)
		pulseEnd = get_current_tick(pi_);
	
	pulseDuration = pulseEnd - pulseStart;
	
	meters = pulseDuration / 1000000.0 * 17150;
	
	return meters;
}*/