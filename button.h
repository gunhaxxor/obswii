#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include <Bounce.h>
#include "MAPPINGS.H"
#define DEBUG
#include "debug.h"
// #include "midiwrapper.h"

#define BOUNCEDURATION 5

// extern volatile bool IMUactivated;

class Button_Class{
	private:
		const int _pin;
		// volatile bool _updated;
		// volatile bool _active;
		void (*_interrupt)();
		volatile bool _pressedDown;

	public:
		bool toggle;
		volatile uint8_t value;

		Button_Class(int pin , int duration, bool toggle, void(*interrupt)());
		Bounce bounce;

		// volatile bool ignoreNextRise;// Used for not triggering normal switching when entering special modes.
		// volatile bool pressedDownAlreadyChecked;// Hehe. Nice name on this variable! Selfexplanatory i guess

		void init();
		void interrupt();
		void updatePressedState();
		bool isActive();
		bool isPressedDown();

		// bool isUpdated();
		// void clearUpdateFlag();

};


#endif
