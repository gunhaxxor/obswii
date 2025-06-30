#ifndef KNOB_H
#define KNOB_H

#include <Arduino.h>

// Will conflict with attachIntterupt if it's used elsewhere in the codebase. So let's not use this for now.
// https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize
// #define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "helpers.h"

#define DEBUG
#include "debug.h"
#include "MAPPINGS.h"

class Knob_Class
{
private:
	Encoder enc;
	int encValue, newEncValue;

public:
	Knob_Class(int pin1, int pin2);

	int16_t value;
	// int CC;
	void init();
	bool updated();
	void write(int16_t value);
};

#endif
