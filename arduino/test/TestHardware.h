#ifndef TEST_HARDWARE_H
#define TEST_HARDWARE_H

#include <ArduinoHardware.h>

class TestHardware : public ArduinoHardware {
public:
	TestHardware(SERIAL_CLASS* io , long baud = 57600)
		: ArduinoHardware(io, baud), injected_len(0)
	{
	}

	TestHardware() : ArduinoHardware(), injected_len(0)
	{
	}

	void inject(uint8_t *data, size_t len)
	{
		for (int i = 0 ; i < len ; ++i)
			injected_data[injected_len++] = data[i];
	}

	int read(void)
	{
		if (injected_len > 0) {
			int ret = injected_data[read_index++];
			injected_len--;
			if (injected_len == 0)
				read_index = 0;
			return ret;
		} else {
			return ArduinoHardware::read();
		}
	}
private:
	uint8_t injected_data[100];
	size_t injected_len;
	size_t read_index;
};

#endif
