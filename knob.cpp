#include "knob.h"
// #include "led_bar.h"


Knob_Class::Knob_Class(int pin1, int pin2): enc(pin1, pin2), encValue(0), newEncValue(0){
}

void Knob_Class::init(){
	value = enc.read();
}

bool Knob_Class::updated(){

  // if(value != enc.read()){
  //   value = enc.read();
  //   DEBUG_PRINT("sending on cc: ");
  //   DEBUG_PRINT(CC);
  //   DEBUG_PRINTLN(value);
  //   return true;
  // }
  // else{return false;}

  newEncValue = enc.read();
  if(encValue != newEncValue){
    if(newEncValue < ENCODER_MIN or newEncValue > ENCODER_MAX){
      encValue = constrain(newEncValue, ENCODER_MIN, ENCODER_MAX);
      enc.write(encValue);
    }else{encValue = newEncValue;}
    value = convertToByte(encValue, ENCODER_MIN, ENCODER_MAX);
    // DEBUG_PRINT("sending: ");
    // DEBUG_PRINTLN(value);
    return true;
  }
  else {return false;}
}

void Knob_Class::write(byte param){
  if(param != value){
    value = param;
    newEncValue = map(param, 0, 127, ENCODER_MIN, ENCODER_MAX);
    encValue = newEncValue;
    // enc.write(param);
    enc.write(newEncValue);
    // DEBUG_PRINT("recieving on cc: ");
    // DEBUG_PRINT(CC);
    // DEBUG_PRINTLN(param);
    // DEBUG_PRINT("written to encoder: ");
    // DEBUG_PRINTLN(enc.read());
  }
}
