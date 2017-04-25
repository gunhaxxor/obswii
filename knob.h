#ifndef KNOB_H
#define KNOB_H

#include <Arduino.h>
#include <Encoder.h>
#include "helpers.h"

#define DEBUG
#include "debug.h"
#include "MAPPINGS.h"


class Knob_Class{
	private:
		Encoder enc;
		int encValue, newEncValue;

	public:
		Knob_Class(int pin1, int pin2);

		uint8_t value;
		// int CC;
		void init();
		bool updated();
		void write(byte value);

};


#endif
