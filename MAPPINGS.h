#ifndef MAPPINGS_H
#define MAPPINGS_H

#define MIDICHANNEL 1

//button is toggle?
#define BUTTON1_TOGGLE false
#define BUTTON2_TOGGLE false
#define BUTTON3_TOGGLE false
#define BUTTON4_TOGGLE false
#define BUTTON5_TOGGLE true

#define ENCBUTTON_TOGGLE false
#define SHAKESENSOR_TOGGLE false

//Both when CC and when noteOn.
#define BUTTON1_MIDIVALUE		61
#define BUTTON2_MIDIVALUE		62
#define BUTTON3_MIDIVALUE		63
#define BUTTON4_MIDIVALUE		64
#define BUTTON5_MIDIVALUE		65

#define ENCBUTTON_MIDIVALUE 66
#define SHAKESENSOR_MIDIVALUE 67

//pinout for the buttons
#define BUTTON1_pin		7
#define BUTTON2_pin		6
#define BUTTON3_pin		5
#define BUTTON4_pin		4
#define BUTTON5_pin		3

#define ENCBUTTON_pin 31
#define SHAKESENSOR_pin 29

#define ROTARY_pin1 33
#define ROTARY_pin2 35
#define ROTARY_MIDIVALUE 68

#define ENCODER_MIN 0
#define ENCODER_MAX 64


#endif
