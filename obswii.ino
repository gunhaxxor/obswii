// #include <Bounce.h>

#include "MAPPINGS.h"
#include "button.h"
#include "knob.h"

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

unsigned long now = 0;

//predefine
void commonSetup();
void setupAcker(int nodeNr);
void setupPoller();

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(14,10);
const uint8_t radioIRQPin = 15;

const int8_t autoRole = -1;
int currentNode = 0;

//*******************************************************
//CONFIG
//Configurable parameters here. Set role to autoRole to automatically read hardware pins to configure role
//Set manually to 0-5 or baseStation if you want to override auto assignment.
volatile int8_t role = autoRole;
//configure how manyy nodes we have
const int nrOfNodes = 2;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "Node 0", "Node 1", "BaseStation"};
const int baseStation = nrOfNodes;
//if radio is not used the imu data will be spewed directly to serial port, rather than sent over radio.
const bool useRadio = true;
//Is nrf chip on pin 14? This configuration only applies if role is set manually (not autoRole)
// const bool NRFOnPin14 = false;

//radio data hopping stuff
  //For nodes
  // volatile uint8_t relayPendingTo = 0;
  volatile unsigned long irqStamp = 0;
  // volatile bool waitingForTriggerAck = false;
  // volatile bool triggerAckReceived = false;

  //For base
  int16_t relayTarget[nrOfNodes] = {0};
  int16_t bridgeFor[nrOfNodes] = {0};
  bool bridgeActive[nrOfNodes] = {false};
  bool edgeActive[nrOfNodes] = {false};
  const unsigned long relayDuration = 250;

//radio message stuffs
//base
  binaryInt16 receivedData[16] = {0};
  binaryInt16 receivedRelayData[16] = {0};

  const int commandArraySize = 0;
  uint8_t currentCommands[nrOfNodes][commandArraySize];
  bool commandReceived[nrOfNodes];
//node
  volatile binaryInt16 transmitData[23] = {0}; //Should never be filled with more than 30 bytes, but make it bigger in case.
  volatile uint8_t receivedCommands[commandArraySize];

  struct state { //Be sure to make this struct aligned!!
    uint8_t    nodeId;
    uint8_t    updateCounter;
    struct stateQuaternion {
      int16_t w;
      int16_t x;
      int16_t y;
      int16_t z;
    }quaternion;
    uint8_t    buttons[5];
    uint8_t    shake;
    uint8_t    rotaryButton;
    uint8_t    rotary;
    uint8_t    leds[5];
  };
  struct state deviceState;
  const int stateSize = sizeof(deviceState);

//Radio status information
// int receivedPollPacketsOnPipe[nrOfNodes] = {0,0};
// int receivedRelayPacketsOnPipe[nrOfNodes] = {0,0};
// int pollFailsOnPipe[nrOfNodes] = {0,0};
// int relayFailsOnPipe[nrOfNodes] = {0,0};
// unsigned long normalPollFailedStamp[nrOfNodes] = {0};
// unsigned long tryReestablishStamp[nrOfNodes] = {0,0};
// unsigned long successStamp[nrOfNodes] = {0,0};
// unsigned long lastSuccessStamp[nrOfNodes] = {0,0};
// unsigned long maxTimeBetweenSuccesses[nrOfNodes] = {0,0};
// unsigned long retrieveDuration[nrOfNodes] = {0,0};
// bool sendTriggerAck[nrOfNodes] = {false};
// bool triggerReceived[nrOfNodes] = {false};
// binaryInt16 latestNodeValues[nrOfNodes][16] = {0};
// bool isResponding[nrOfNodes] = {true, true};

//timing stuff
unsigned long mainLoopStamp = 0;
unsigned long mainLoopDuration = 0;
unsigned long mainLoopFrequency = 0;

unsigned long serialFrequency = 0;
unsigned long serialStamp = 0;

bool printDuringThisLap = false;
unsigned long printFrequency = 5;
unsigned long printStamp = 0;

unsigned long sensorFusionDuration = 0;

unsigned long midiStamp = 0;

void button1interrupt();
void button2interrupt();
void button3interrupt();
void button4interrupt();
void button5interrupt();
Button_Class button[] = {
  Button_Class(BUTTON1_pin, BOUNCEDURATION, BUTTON1_TOGGLE, button1interrupt),
  Button_Class(BUTTON2_pin, BOUNCEDURATION, BUTTON2_TOGGLE, button2interrupt),
  Button_Class(BUTTON3_pin, BOUNCEDURATION, BUTTON3_TOGGLE, button3interrupt),
  Button_Class(BUTTON4_pin, BOUNCEDURATION, BUTTON4_TOGGLE, button4interrupt),
  Button_Class(BUTTON5_pin, BOUNCEDURATION, BUTTON5_TOGGLE, button5interrupt),
};
void button1interrupt(){button[0].interrupt();}//Trick to handle that interrupts can't be attached to class member functions. Jag vet. Det är lite b. Men lite mer lättanvänd kod...
void button2interrupt(){button[1].interrupt();}
void button3interrupt(){button[2].interrupt();}
void button4interrupt(){button[3].interrupt();}
void button5interrupt(){button[4].interrupt();}
const int buttonsGroundPin = 36;

void encButtonInterrupt();
Button_Class encButton = Button_Class(ENCBUTTON_pin, BOUNCEDURATION, ENCBUTTON_TOGGLE, encButtonInterrupt);
void encButtonInterrupt(){encButton.interrupt();}
const int encButtonGroundPin = 30;

void shakeSensorInterrupt();
Button_Class shakeSensor = Button_Class(SHAKESENSOR_pin, BOUNCEDURATION, SHAKESENSOR_TOGGLE, shakeSensorInterrupt);
void shakeSensorInterrupt(){shakeSensor.interrupt();}
const int shakeSensorGroundPin = 28;

Knob_Class rotary = Knob_Class(ROTARY_pin1, ROTARY_pin2);
const int rotaryGroundPin = 34;


const int nrOfButtons = sizeof(button)/sizeof(button[0]);

const int nrOfLeds = 5;
const int ledPins[nrOfLeds] = {18, 19, 20, 21, 22};

elapsedMillis sinceFakeRadioMessage = 0;
unsigned long fakeRadioMessageInterval = 200;

elapsedMillis sincePrint = 0;
unsigned long printInterval = 150;

void configurePinAsGround( int pin ){
  pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
}

void setup(){
  noInterrupts();
  Serial.begin(115200);
  printf("Starting OBSWII firmware\n");
  //Configure peripherals and ground pins (really hope the pins are capabe of sinking enough current!!!!)
  configurePinAsGround(rotaryGroundPin);
  rotary.init();
  configurePinAsGround(shakeSensorGroundPin);
  shakeSensor.init();
  configurePinAsGround(encButtonGroundPin);
  encButton.init();

  configurePinAsGround(buttonsGroundPin);
  for (size_t i = 0; i < nrOfButtons; i++) {
    button[i].init();
  }

  for (size_t i = 0; i < nrOfLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  //ROLE. Do this first so everything that relies on it works properly
  if(role == autoRole){
    //baseStation is default
    role = baseStation;
    //Reads the pins 2-4 as a binary bcd number.
    for (int i = 0; i < nrOfNodes; ++i)
    {
      pinMode(i, INPUT_PULLUP);
      delay(2);
      if(!digitalRead(i)){
        role = i+1;
        break;
      }
    }
  }

  printf("ROLE: %s\n\r",role_friendly_name[role]);

  if(role == baseStation){

  }else{

  }

  //Radio Stuff!
  if(useRadio){
    printf("Setting up radio\n");
    commonSetup();
    if(role == baseStation){
      setupPoller();
      // setupMultiReceiver();
    }else{
      setupAcker(role);
      // setupTransmitter(role);
    }
    radio.printDetails();
  }

  now = millis();
  interrupts();
}

void loop(){
  //What is the time?
  now = millis();

  if(rotary.updated()){
    deviceState.rotary = rotary.value;
  }
  for (size_t i = 0; i < nrOfButtons; i++) {
    deviceState.buttons[i] = button[i].value;
  }
  deviceState.rotaryButton = encButton.value;
  deviceState.shake = shakeSensor.value;

  //Let's update quaternion last.


  //Let's fake a radiomessage update
  if(sinceFakeRadioMessage > fakeRadioMessageInterval){
    sinceFakeRadioMessage = 0;
    int randomLed = random(nrOfLeds);
    deviceState.leds[randomLed] = !deviceState.leds[randomLed];
  }


  //Now set the leds from the deviceState (which should be updated by received radio mesage)
  for (size_t i = 0; i < nrOfLeds; i++) {
    digitalWrite(ledPins[i], deviceState.leds[i]%2);
  }

  //Let's fake a radiomessage update
  if(sincePrint > printInterval){
    sincePrint = 0;

    printState(deviceState);
  }

}

void printState(struct state deviceState){
  printf("nodeId\t %i \n", deviceState.nodeId);
  printf("messageCounter\t %i \n", deviceState.updateCounter);
  printf("quaternion\t %f,%f,%f,%f \n", deviceState.quaternion.w, deviceState.quaternion.x, deviceState.quaternion.y, deviceState.quaternion.z);
  for (size_t buttonIndex = 0; buttonIndex < nrOfButtons; buttonIndex++) {
    printf("button %i\t %i \n",buttonIndex, deviceState.buttons[buttonIndex]);
  }
  printf("shake\t %i \n", deviceState.shake);
  printf("rotaryButton\t %i \n", deviceState.rotaryButton);
  printf("rotary\t %i \n", deviceState.rotary);
  for (size_t ledIndex = 0; ledIndex < nrOfLeds; ledIndex++) {
    printf("led %i\t %i \n",ledIndex, deviceState.leds[ledIndex]);
  }

}