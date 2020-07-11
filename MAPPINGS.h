#ifndef MAPPINGS_H
#define MAPPINGS_H

#define MIDICHANNEL 1

#define ROLE_pin0 0
#define ROLE_pin1 1

//radio stuff
#define RADIO_CE_pin 20
#define RADIO_CSN_pin 21
#define RADIO_INT_pin 26
#define RADIO_MOSI_pin 28
#define RADIO_MISO_pin 39
#define RADIO_SCK_pin 27

//button is toggle?
#define BUTTON1_TOGGLE false
#define BUTTON2_TOGGLE false
#define BUTTON3_TOGGLE false
#define BUTTON4_TOGGLE false
#define BUTTON5_TOGGLE false

#define ENCBUTTON_TOGGLE false
#define SHAKESENSOR_TOGGLE false

//Both when CC and when noteOn.
#define BUTTON1_MIDIVALUE 61
#define BUTTON2_MIDIVALUE 62
#define BUTTON3_MIDIVALUE 63
#define BUTTON4_MIDIVALUE 64
#define BUTTON5_MIDIVALUE 65

#define ENCBUTTON_MIDIVALUE 66
#define SHAKESENSOR_MIDIVALUE 67

//pinout for the buttons
#define BUTTON1_pin 7
#define BUTTON2_pin 6
#define BUTTON3_pin 5
#define BUTTON4_pin 4
#define BUTTON5_pin 3

//leds
#define LED_pin0 30
#define LED_pin1 29
#define LED_pin2 36
#define LED_pin3 37
#define LED_pin4 38

const uint8_t ledBrightness[5] = {
    1100,
    255,
    50,
    100,
    100};
// const uint8_t ledBrightness[5] = {
//     255,
//     255,
//     255,
//     255,
//     255};

#define ENCBUTTON_pin 32
#define ENCBUTTON_groundPin 31
// #define SHAKESENSOR_pin 32

#define ROTARY_groundPin 34
#define ROTARY_pin1 33
#define ROTARY_pin2 35
#define ROTARY_MIDIVALUE 68

#define ENCODER_MIN 0
#define ENCODER_MAX 64

#endif
