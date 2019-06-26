// -----------------------OBSWII CODE
// PINOUT
//  RADIO
//  Vcc       -     Vcc
//  GND       -     GND
//  CE        -     9
//  CSN(CS)   -     10
//  SCK       -     27
//  MOSI      -     11
//  MISO      -     12
//  INT       -     26
#include "MAPPINGS.h"
#include "button.h"
#include "knob.h"
#include "helpers.h"

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>

unsigned long now = 0;

//predefine
void commonSetup();
void setupAcker(int nodeNr);
void setupPoller();

// Set up nRF24L01 radio on SPI bus
// RF24 radio(14,10);
RF24 radio(RADIO_CE_pin, RADIO_CSN_pin);
uint8_t radioIRQPin = RADIO_INT_pin;

const int8_t autoRole = -1;
int currentNode = 0;

//*******************************************************
//CONFIG
//Configurable parameters here. Set role to autoRole to automatically read hardware pins to configure role
//Set manually to 0-5 or baseStation if you want to override auto assignment.
volatile int8_t role = autoRole;
//configure how many nodes we have
const int nrOfNodes = 2;

// The debug-friendly names of those roles
const char *role_friendly_name[] = {"Node 0", "Node 1", "BaseStation"};
const int baseStation = nrOfNodes;
//if radio is not used the imu data will be spewed directly to serial port, rather than sent over radio.
bool useRadio = true;
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
// binaryInt16 receivedData[16] = {0};
// binaryInt16 receivedRelayData[16] = {0};

const int commandArraySize = 0;
uint8_t currentCommands[nrOfNodes][commandArraySize];
bool commandReceived[nrOfNodes];
//node
// volatile binaryInt16 transmitData[23] = {0}; //Should never be filled with more than 30 bytes, but make it bigger in case.
volatile uint8_t receivedCommands[commandArraySize];

struct state
{ //Be sure to make this struct aligned!!
  uint8_t nodeId;
  uint8_t updateCounter;
  struct quaternionFixedPoint
  {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
  } quaternion;
  uint8_t buttons[5];
  uint8_t shake;
  uint8_t rotaryButton;
  uint8_t rotary;
  uint8_t leds[5];
};
volatile state deviceState[2];
volatile state pushState[2];

const int stateSize = sizeof(deviceState[0]);

//Orientation sensor stuff
Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_B, I2C_MASTER, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
imu::Quaternion quat;
// I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400
quaternion currentQuaternion = {1.0f, 0, 0, 0};
const int nrOfPoseSlots = 10;
quaternion learnedPoses[nrOfPoseSlots] = {0};

// bool initialized = false;
// volatile bool radioEstablished = false;

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
void button1interrupt() { button[0].interrupt(); } //Trick to handle that interrupts can't be attached to class member functions. Jag vet. Det är lite b. Men lite mer lättanvänd kod...
void button2interrupt() { button[1].interrupt(); }
void button3interrupt() { button[2].interrupt(); }
void button4interrupt() { button[3].interrupt(); }
void button5interrupt() { button[4].interrupt(); }
// const int buttonsGroundPin = 36;

void encButtonInterrupt();
Button_Class encButton = Button_Class(ENCBUTTON_pin, BOUNCEDURATION, ENCBUTTON_TOGGLE, encButtonInterrupt);
void encButtonInterrupt() { encButton.interrupt(); }
// const int encButtonGroundPin = 30;

void shakeSensorInterrupt();
Button_Class shakeSensor = Button_Class(SHAKESENSOR_pin, BOUNCEDURATION, SHAKESENSOR_TOGGLE, shakeSensorInterrupt);
void shakeSensorInterrupt() { shakeSensor.interrupt(); }
// const int shakeSensorGroundPin = 28;

Knob_Class rotary = Knob_Class(ROTARY_pin1, ROTARY_pin2);
// const int rotaryGroundPin = 34;

const int nrOfButtons = sizeof(button) / sizeof(button[0]);

const int nrOfLeds = 5;
const int ledPins[nrOfLeds] = {LED_pin0, LED_pin1, LED_pin2, LED_pin3, LED_pin4};

elapsedMillis sinceFakeRadioMessage = 0;
unsigned long fakeRadioMessageInterval = 200;

elapsedMillis sincePrint = 0;
unsigned long printInterval = 150;

void configurePinAsGround(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void setup()
{

  noInterrupts();

  Serial.begin(115200);
  printf("Starting OBSWII firmware\n");

  //ROLE. Do this first so everything that relies on it works properly
  if (role == autoRole)
  {
    //baseStation is default
    role = baseStation;
    //Reads the pins 0-1 as a binary bcd number.
    for (int i = 0; i < nrOfNodes; ++i)
    {
      pinMode(i, INPUT_PULLUP);
      delay(2);
      if (!digitalRead(i))
      {
        role = i;
        break;
      }
    }
  }

  // if (role == autoRole)
  // {
  //   role = 0;
  //   //Reads the pins 0-1 as a binary bcd number.
  //   for (int i = 0; i < 1; ++i)
  //   {
  //     pinMode(i, INPUT_PULLUP);
  //     delay(2);
  //     if (!digitalRead(i))
  //     {
  //       role = role | 1 << i;
  //     }
  //   }
  //   //The pin config starts with node 0 having bcd 1. So, decrease with one to align index.
  //   role--;
  //   //If we are negative after reading pins and aligning index, that means we had bcd 0 = baseStation.
  //   if (role < 0)
  //     role = baseStation;
  // }

  delay(1600);
  printf("ROLE: %s\n\r", role_friendly_name[role]);

  if (role != baseStation)
  {
    //Configure peripherals and ground pins (really hope the pins are capabe of sinking enough current!!!!)
    // configurePinAsGround(rotaryGroundPin);
    rotary.init();
    // configurePinAsGround(shakeSensorGroundPin);
    shakeSensor.init();
    // configurePinAsGround(encButtonGroundPin);
    encButton.init();

    // configurePinAsGround(buttonsGroundPin);
    for (size_t i = 0; i < nrOfButtons; i++)
    {
      button[i].init();
    }

    for (size_t i = 0; i < nrOfLeds; i++)
    {
      pinMode(ledPins[i], OUTPUT);
    }

    // EM7180_setup();
    // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
    if (bno.begin())
    {
      bno.setExtCrystalUse(false);
    }
    else
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      printf("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
  }

  SPI.setSCK(RADIO_SCK_pin);
  SPI.setMOSI(RADIO_MOSI_pin);
  SPI.setMISO(RADIO_MISO_pin);
  // radioIRQPin = RADIO_INT_pin;

  if (role == baseStation)
  {
  }
  else
  {
  }

  //Radio Stuff!
  if (useRadio)
  {
    printf("Setting up radio\n");
    commonSetup();
    if (role == baseStation)
    {
      setupPoller();
      // setupMultiReceiver();
    }
    else
    {
      //Make the first node have 0 as index.
      setupAcker(role);
      // setupTransmitter(role);
    }
    radio.printDetails();
  }

  now = millis();
  interrupts();
  delay(2500);
}

void loop()
{
  //What is the time?
  now = millis();

  if (role == baseStation)
  {
    baseStationLoop();
    return;
  }
  else // is a node
  {
    nodeLoop();
  }

  // if (!initialized && radioEstablished)
  // {
  //   initialized = true;
  //   memcpy(deviceState, pushState, stateSize);
  // }

  if (sincePrint > printInterval)
  {
    sincePrint = 0;

    // printState(deviceState[role]);
  }
}

void baseStationLoop()
{
  delay(10);
  if (useRadio)
  {
    if (pollNode(0, (uint8_t *)&pushState[currentNode], (uint8_t *)&deviceState[currentNode]))
    {
      printf("poll received\n");
      currentQuaternion.w = Q15ToFloat(deviceState[currentNode].quaternion.w);
      currentQuaternion.x = Q15ToFloat(deviceState[currentNode].quaternion.x);
      currentQuaternion.y = Q15ToFloat(deviceState[currentNode].quaternion.y);
      currentQuaternion.z = Q15ToFloat(deviceState[currentNode].quaternion.z);
      currentQuaternion = quat_norm(currentQuaternion);
    }
    else
    {
      printf("pollnode failed\n");
    }
  }

  for (size_t i = 0; i < nrOfLeds; i++)
  {
    if (pushState[currentNode].leds[i] != deviceState[currentNode].buttons[i])
    {
      int value = pushState[currentNode].leds[i] = deviceState[currentNode].buttons[i];
      value %= 2;
      printf("Changed button %i\n", i);
      // usbMIDI.sendControlChange(60 + i, value * 127, MIDICHANNEL);
    }
    // deviceState[currentNode].leds[i] = pushState[currentNode].leds[i];
    // memcpy(pushState, deviceState, stateSize);
  }

  calculateParameterWeights();
  // radio.printDetails();

  if (true || sincePrint > printInterval)
  {
    sincePrint = 0;
    // printState(deviceState[currentNode]);

    Serial.println("current -----");
    printQuaternion(((currentQuaternion)));
    float currentAngle = quat_angle((currentQuaternion));
    printf("angle: %f \n", toDegrees(currentAngle));
    Serial.println("learned ----- ");
    printQuaternion(learnedPoses[0]);
    float learnedAngle = quat_angle(learnedPoses[0]);
    printf("angle: %f \n", toDegrees(learnedAngle));

    quaternion deltaQ = quat_delta_rotation(currentQuaternion, learnedPoses[0]);
    Serial.println("delta ----- ");
    printQuaternion(deltaQ);

    float angle = quat_angle(currentQuaternion, learnedPoses[0]);
    printf("delta angle: %f \n", toDegrees(angle));
    Serial.println();

    printAngleDistances();
    printParameterWeights();
  }

  handleSerial();
}

elapsedMillis sinceLastLoop = 0;
void nodeLoop()
{
  noInterrupts();
  if (radio.rxFifoFull())
  {
    Serial.print("RX FIFO FULL! Dummyreading. ");
    uint8_t data[32] = {0};
    while (radio.available())
    {
      printf("read. ");
      radio.read(data, radio.getDynamicPayloadSize());
    }
    printf("\n");
    // radio.printDetails();
  }
  interrupts();

  unsigned long t = sinceLastLoop;
  sinceLastLoop = 0;
  // printf("loopTime %ul \n", t);
  // printf(".\n");

  //handle all the inputs
  if (rotary.updated())
  {
    deviceState[role].rotary = rotary.value;
  }
  for (size_t i = 0; i < nrOfButtons; i++)
  {
    deviceState[role].buttons[i] = button[i].value;
  }
  deviceState[role].rotaryButton = encButton.value;
  deviceState[role].shake = shakeSensor.value;

  //sync the leds in deviceState to the ones in received pushState
  for (size_t i = 0; i < nrOfLeds; i++)
  {
    deviceState[role].leds[i] = pushState[role].leds[i];
  }
  //Now set the leds from the deviceState (which should be updated by received radio mesage)
  for (size_t i = 0; i < nrOfLeds; i++)
  {
    digitalWrite(ledPins[i], deviceState[role].leds[i] % 2);
  }

  quat = bno.getQuat();
  // EM7180_loop();
  // float q[4];
  // memcpy(q, EM7180_getQuaternion(), 4 * sizeof(float));
  // // printf("-------------- %f, %f, %f, %f \n", q[0], q[1], q[2], q[3]);
  deviceState[role].quaternion.w = floatToQ15((float)quat.w());
  deviceState[role].quaternion.x = floatToQ15((float)quat.x());
  deviceState[role].quaternion.y = floatToQ15((float)quat.y());
  deviceState[role].quaternion.z = floatToQ15((float)quat.z());

  // EM7180_printAlgorithmDetails();

  if (sincePrint > printInterval)
  {
    sincePrint = 0;
    // radio.printDetails();
    // writeAckPacks((void *)&deviceState, stateSize);
  }
}

const float clampAngle = 10;
bool activePoseSlots[nrOfPoseSlots] = {0};
bool fullyTriggeredPoses[nrOfPoseSlots]{0};
float parameterWeights[nrOfPoseSlots] = {0};
float distances[nrOfPoseSlots] = {0};
float clampedDistances[nrOfPoseSlots] = {0};
void calculateParameterWeights()
{
  float invertedDistancesSum = 0.f;
  float InvertedDistances[nrOfPoseSlots] = {0};
  bool anyPoseIsFullyTriggered = false;
  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if (activePoseSlots[i])
    {
      distances[i] = toDegrees(quat_angle(currentQuaternion, learnedPoses[i]));
      if (distances[i] < clampAngle)
      {
        fullyTriggeredPoses[i] = true;
        anyPoseIsFullyTriggered = true;
      }
      else
      {
        fullyTriggeredPoses[i] = false;
      }
    }
  }

  if (anyPoseIsFullyTriggered)
  {
    for (size_t i = 0; i < nrOfPoseSlots; i++)
    {
      if (activePoseSlots[i])
      {
        if (fullyTriggeredPoses[i])
        {
          if (distances[i] < 0.001f)
          {
            InvertedDistances[i] = INFINITY;
          }
          else
          {
            InvertedDistances[i] = 1.0f / distances[i];
          }
          invertedDistancesSum += InvertedDistances[i];
        }
        else
        {
          InvertedDistances[i] = 0.0f;
        }
      }
    }
  }
  else
  {
    for (size_t i = 0; i < nrOfPoseSlots; i++)
    {
      if (activePoseSlots[i])
      {
        clampedDistances[i] = distances[i] - clampAngle;
        InvertedDistances[i] = 1.0f / clampedDistances[i];
        invertedDistancesSum += InvertedDistances[i];
      }
    }
  }

  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if (activePoseSlots[i])
    {
      parameterWeights[i] = InvertedDistances[i] / invertedDistancesSum;
    }
  }
}

void printAngleDistances()
{
  printf("angleDistances: ");
  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if (activePoseSlots[i])
    {
      printf("%f, ", distances[i]);
    }
  }
  Serial.println();
}

void printParameterWeights()
{
  printf("parameterWeights: ");
  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if (activePoseSlots[i])
    {
      printf("%f, ", parameterWeights[i]);
    }
  }
  Serial.println();
}

void printState(struct state deviceState)
{
  printf("nodeId\t %i \n", deviceState.nodeId);
  printf("messageCounter\t %i \n", deviceState.updateCounter);
  printf("quaternion\t %f,%f,%f,%f \n", Q15ToFloat(deviceState.quaternion.w), Q15ToFloat(deviceState.quaternion.x), Q15ToFloat(deviceState.quaternion.y), Q15ToFloat(deviceState.quaternion.z));
  for (size_t buttonIndex = 0; buttonIndex < nrOfButtons; buttonIndex++)
  {
    printf("button %i\t %i \n", buttonIndex, deviceState.buttons[buttonIndex]);
  }
  printf("shake\t %i \n", deviceState.shake);
  printf("rotaryButton\t %i \n", deviceState.rotaryButton);
  printf("rotary\t %i \n", deviceState.rotary);
  for (size_t ledIndex = 0; ledIndex < nrOfLeds; ledIndex++)
  {
    printf("led %i\t %i \n", ledIndex, deviceState.leds[ledIndex]);
  }
}

void printQuaternion(quaternion q)
{
  printf("quaternion\t %f,%f,%f,%f \n", q.w, q.x, q.y, q.z);
}

void handleSerial()
{
  if (Serial.available())
  {
    uint8_t c = Serial.read();
    switch (c)
    {
    case '1':
      learnedPoses[0] = currentQuaternion;
      activePoseSlots[0] = true;
      break;
    case '2':
      learnedPoses[1] = currentQuaternion;
      activePoseSlots[1] = true;
      break;
    case '3':
      learnedPoses[2] = currentQuaternion;
      activePoseSlots[2] = true;
      break;
    case '4':
      learnedPoses[3] = currentQuaternion;
      activePoseSlots[3] = true;
      break;
    case 'r':
      useRadio = !useRadio;
      break;
    }
  }
}
