// -----------------------OBSWII CODE
// PINOUT
//  RADIO
//  Vcc       -     Vcc
//  GND       -     GND
//  CE        -
//  CSN(CS)   -
//  SCK       -
//  MOSI      -
//  MISO      -
//  INT       -
#include "MAPPINGS.h"
#include "button.h"
#include "helpers.h"
#include "knob.h"

#include "RF24.h"
#include "nRF24L01.h"
#include <SPI.h>

#include <Adafruit_BNO055_t3.h>
#include <Adafruit_Sensor.h>
#include <i2c_t3.h>

#include "SD.h"

unsigned long now = 0;

// predefine
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
// CONFIG
// Configurable parameters here. Set role to autoRole to automatically read
// hardware pins to configure role Set manually to 0-5 or baseStation if you want
// to override auto assignment.
volatile int8_t role = autoRole;
// configure how many nodes we have
const int nrOfNodes = 2;

// The debug-friendly names of those roles
const char *role_friendly_name[] = {"Node 0", "Node 1", "BaseStation"};
const int baseStation = nrOfNodes;
// if radio is not used the imu data will be spewed directly to serial port,
// rather than sent over radio.
bool useRadio = true;
bool shouldRestartAcker = false;
bool shouldRestartPoller = false;
// Is nrf chip on pin 14? This configuration only applies if role is set
// manually (not autoRole)
// const bool NRFOnPin14 = false;

// radio data hopping stuff
// For nodes
// volatile uint8_t relayPendingTo = 0;
volatile unsigned long irqStamp = 0;
// volatile bool waitingForTriggerAck = false;
// volatile bool triggerAckReceived = false;

// For base
int16_t relayTarget[nrOfNodes] = {0};
int16_t bridgeFor[nrOfNodes] = {0};
bool bridgeActive[nrOfNodes] = {false};
bool edgeActive[nrOfNodes] = {false};
const unsigned long relayDuration = 250;

// radio message stuffs
// base
// binaryInt16 receivedData[16] = {0};
// binaryInt16 receivedRelayData[16] = {0};

// const int commandArraySize = 0;
// uint8_t currentCommands[nrOfNodes][commandArraySize];
// bool commandReceived[nrOfNodes];
// node
// volatile binaryInt16 transmitData[23] = {0}; //Should never be filled with
// more than 30 bytes, but make it bigger in case. volatile uint8_t
// receivedCommands[commandArraySize];

// bool initialized = false;
// volatile bool radioEstablished = false;

// Radio status information
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

struct state
{ // Be sure to make this struct aligned!!
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
  // uint8_t shake;
  uint8_t rotaryButton;
  int16_t rotary;
  uint8_t leds[5];
};
state deviceState[2];
state previousDeviceState[2];
state pushState[2];

const int stateSize = sizeof(deviceState[0]);

// Orientation sensor stuff
Adafruit_BNO055 bno =
    Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_B, I2C_MASTER, I2C_PINS_16_17,
                    I2C_PULLUP_EXT, I2C_RATE_400);
imu::Quaternion quat;
quaternion currentQuaternion = {1.0f, 0, 0, 0};
quaternion absoluteQuaternion = {1.0f, 0, 0, 0};
// const int nrOfPoseSlots = 10;
// quaternion learnedPoses[nrOfPoseSlots] = {0};

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
void button1interrupt()
{
  button[0].interrupt();
} // Trick to handle that interrupts can't be attached to class member
// functions. Jag vet. Det är lite b. Men lite mer lättanvänd kod...
void button2interrupt() { button[1].interrupt(); }
void button3interrupt() { button[2].interrupt(); }
void button4interrupt() { button[3].interrupt(); }
void button5interrupt() { button[4].interrupt(); }
// const int buttonsGroundPin = 36;

void encButtonInterrupt();
Button_Class encButton = Button_Class(ENCBUTTON_pin, BOUNCEDURATION,
                                      ENCBUTTON_TOGGLE, encButtonInterrupt);
void encButtonInterrupt() { encButton.interrupt(); }

Knob_Class rotary = Knob_Class(ROTARY_pin1, ROTARY_pin2);
// const int rotaryGroundPin = 34;

const int nrOfButtons = sizeof(button) / sizeof(button[0]);

const int nrOfLeds = 5;
const int ledPins[nrOfLeds] = {LED_pin0, LED_pin1, LED_pin2, LED_pin3,
                               LED_pin4};

// MIDI stuff
bool shouldSendPoseMidi = false;

// timing stuff
elapsedMillis sinceFakeRadioMessage = 0;
unsigned long fakeRadioMessageInterval = 200;

elapsedMillis sincePrint = 0;
unsigned long printInterval = 150;

elapsedMillis sinceMidiSend = 0;
unsigned long midiSendInterval = 10;

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

  pinMode(13, OUTPUT);

  // ROLE. Do this first so everything that relies on it works properly
  if (role == autoRole)
  {
    // baseStation is default
    role = baseStation;
    // Reads the pins 0-1 as a binary bcd number.
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

  printf("ROLE: %s\n\r", role_friendly_name[role]);

  if (role == baseStation)
  {
    if (!(SD.begin(BUILTIN_SDCARD)))
    {
      Serial.println("Unable to access the SD card! You cry now!!");
      delay(500);
    }
    else
    {
      printf("SD card opened. Hurray!\n");
    }

    // MIDI STUFF
    usbMIDI.setHandleControlChange(onControlChange);
    usbMIDI.setHandleNoteOn(onNoteOn);
    usbMIDI.setHandleNoteOff(onNoteOff);
  }
  else
  {
    // Configure peripherals and ground pins (really hope the pins are capabe of
    // sinking enough current!!!!)
    configurePinAsGround(ROTARY_groundPin);
    rotary.init();
    // configurePinAsGround(shakeSensorGroundPin);
    // shakeSensor.init();
    configurePinAsGround(ENCBUTTON_groundPin);
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
    // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT,
    // I2C_RATE_400);
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

  // Radio Stuff!
  if (useRadio)
  {
    printf("Setting up radio\n");
    SPI.setSCK(RADIO_SCK_pin);
    SPI.setMOSI(RADIO_MOSI_pin);
    SPI.setMISO(RADIO_MISO_pin);
    // radioIRQPin = RADIO_INT_pin;
    commonSetup();
    if (role == baseStation)
    {
      setupPoller();
      // setupMultiReceiver();
    }
    else
    {
      // Make the first node have 0 as index.
      setupAcker(role);
      // setupTransmitter(role);
    }
    radio.printDetails();
  }

  now = millis();
  interrupts();
  // delay(2500);
}

bool recordNotes = false;
struct note
{
  bool active;
  int note;
  int velocity;
};

const int nrOfNotes = 127;
note notes[nrOfNotes] = {0};

void onNoteOn(byte channel, byte note, byte velocity)
{
  digitalWrite(13, !digitalRead(13));
  usbMIDI.sendNoteOn(note, velocity, channel);

  if (recordNotes && !notes[note].active)
  {
    printf("recording note number %i \n", note);
    notes[note].active = true;
    notes[note].note = note;
  }
  notes[note].velocity = velocity;
}

void onNoteOff(byte channel, byte note, byte velocity)
{
  digitalWrite(13, !digitalRead(13));
  usbMIDI.sendNoteOff(note, velocity, channel);

  if (recordNotes && !notes[note].active)
  {
    printf("recording note number %i \n", note);
    notes[note].active = true;
    notes[note].note = note;
  }
  notes[note].velocity = 0;
}

bool recordControlChanges = false;
struct controlChange
{
  bool active;
  int cc;
  int value;
};

// const int nrOfControlChanges = 15
const int nrOfCCs = 128;
controlChange controlChanges[nrOfCCs] = {0};
void onControlChange(byte channel, byte control, byte value)
{
  // digitalWrite(13, !digitalRead(13));
  digitalWrite(13, HIGH);
  // midiThru
  usbMIDI.sendControlChange(control, value, channel);
  // printf("control change received\n");
  if (recordControlChanges && !controlChanges[control].active)
  {
    printf("recording CC number %i \n", control);
    sincePrint = 2000;
    controlChanges[control].active = true;
    controlChanges[control].cc = control;
  }
  controlChanges[control].value = value;
}

const int maxNrOfCCsInParameterGroup = 15;
const int maxNrOfNotesInParameterGroup = 15;
struct parameterGroupState
{
  int slot;
  bool active;
  quaternion savedPose;
  float outerRadius;
  controlChange savedControlChanges[maxNrOfCCsInParameterGroup];
  note savedNotes[maxNrOfNotesInParameterGroup];
};
const int nrOfParameterGroups = 5;

struct preset
{
  bool active = false;
  parameterGroupState savedParameterGroups[nrOfParameterGroups];
};
const int nrOfPresetSlots = 5;
preset presets[nrOfPresetSlots] = {0};
preset *currentPreset = &presets[0];

bool anyParamGroupSaved = false;
void saveParameterGroup(int slot)
{
  // disable addition of more control changes after first saved parameter group
  recordControlChanges = false;
  recordNotes = false;

  currentPreset->savedParameterGroups[slot].active = true;
  currentPreset->savedParameterGroups[slot].slot = slot;
  currentPreset->savedParameterGroups[slot].savedPose = currentQuaternion;

  // save the recorded cc values
  int k = 0;
  for (size_t i = 0; i < nrOfCCs; i++)
  {
    if (controlChanges[i].active)
    {
      currentPreset->savedParameterGroups[slot].savedControlChanges[k] =
          controlChanges[i];
      k++;
    }
  }
  // clear the rest of the cc slots in this param group
  while (k < maxNrOfCCsInParameterGroup)
  {
    currentPreset->savedParameterGroups[slot].savedControlChanges[k].active =
        false;
    k++;
  }

  // save the recorded notes
  k = 0;
  for (size_t i = 0; i < nrOfNotes; i++)
  {
    if (notes[i].active)
    {
      currentPreset->savedParameterGroups[slot].savedNotes[k] = notes[i];
      k++;
    }
  }
  // clear the rest of the notes slots in this param group
  while (k < maxNrOfNotesInParameterGroup)
  {
    currentPreset->savedParameterGroups[slot].savedNotes[k].active = false;
    k++;
  }

  calculatePoseMinDistances();

  anyParamGroupSaved = true;
};

void clearParameterGroup(int slot)
{
  currentPreset->savedParameterGroups[slot].active = false;
  currentPreset->savedParameterGroups[slot].savedPose = currentQuaternion;
}

void savePoseForParameterGroup(int slot)
{
  currentPreset->savedParameterGroups[slot].savedPose = currentQuaternion;
};

// This function checks for each pose which is the closest pose to it and saves it.
void calculatePoseMinDistances()
{
  sincePrint = 0;

  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      float minDistance = 90.f;
      for (size_t j = 0; j < nrOfParameterGroups; j++)
      {
        if (i != j && currentPreset->savedParameterGroups[j].active)
        {
          printf("gonna calc angle distance between %i and %i\n", i, j);
          float aDistance = toDegrees(quat_angle(
              currentPreset->savedParameterGroups[i].savedPose, currentPreset->savedParameterGroups[j].savedPose));
          printf("aDistance: %f\n", aDistance);
          minDistance = aDistance < minDistance ? aDistance : minDistance;
        }
      }
      currentPreset->savedParameterGroups[i].outerRadius = minDistance;
      printf("outerRadius for %i is %f\n", i, currentPreset->savedParameterGroups[i].outerRadius);
    }
  }
}

const float clampAngle = 15;
bool activePoseSlots[nrOfParameterGroups] = {0};
bool fullyTriggeredPoses[nrOfParameterGroups]{0};
float distances[nrOfParameterGroups] = {0};
float clampedDistances[nrOfParameterGroups] = {0};
float clampedOuterRadius[nrOfParameterGroups] = {0};
float rawParameterWeights[nrOfParameterGroups] = {0};
float parameterWeights[nrOfParameterGroups] = {0};

//THIS IS where the magic happens! Gunnar is the smartest dude for sure!!!!
void calculateParameterWeights()
{
  float weightSum = 0.f;
  // float weights[nrOfParameterGroups] = {0};

  // I've split the algorithm into several parts/loops to make it easier to reason about. Not best performance but fuck it.

  // First, let's just calculate the distances
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      distances[i] = toDegrees(quat_angle(
          currentQuaternion, currentPreset->savedParameterGroups[i].savedPose));
    }
  }

  /// Loop through and check which poses are fully triggered
  // bool anyPoseIsFullyTriggered = false;
  // for (size_t i = 0; i < nrOfParameterGroups; i++) {
  //   if (currentPreset->savedParameterGroups[i].active) {
  //     if (distances[i] < clampAngle) {
  //       fullyTriggeredPoses[i] = true;
  //       anyPoseIsFullyTriggered = true;
  //     } else {
  //       fullyTriggeredPoses[i] = false;
  //     }
  //   }
  // }

  // if at least one is fully triggered we calculate weight
  // distribution only on the triggered one(s)
  // if (false && anyPoseIsFullyTriggered) {
  //   for (size_t i = 0; i < nrOfParameterGroups; i++) {
  //     if (currentPreset->savedParameterGroups[i].active) {
  //       if (fullyTriggeredPoses[i]) {
  //         if (distances[i] < 0.001f) {
  //           rawParameterWeights[i] = INFINITY;
  //         } else {
  //           rawParameterWeights[i] = 1.0f / distances[i];
  //         }
  //         weightSum += rawParameterWeights[i];
  //       } else {
  //         rawParameterWeights[i] = 0.0f;
  //       }
  //     }
  //   }
  // } else /// Otherwise calculate weight on all poses

  // Loop through and check if any is within outerRadius
  bool noneWithinOuterRadius = true;
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      if (distances[i] < currentPreset->savedParameterGroups[i].outerRadius - clampAngle)
      {
        noneWithinOuterRadius = false;
      }
    }
  }
  if (noneWithinOuterRadius)
  {
    for (size_t i = 0; i < nrOfParameterGroups; i++)
    {
      if (currentPreset->savedParameterGroups[i].active)
      {
        float distanceToOuterRadius = distances[i] - (currentPreset->savedParameterGroups[i].outerRadius - clampAngle);
        if (distanceToOuterRadius < 0.001f)
        {
          rawParameterWeights[i] = INFINITY;
        }
        else
        {
          rawParameterWeights[i] = 1.0f / distanceToOuterRadius;
        }
        weightSum += rawParameterWeights[i];
      }
    }
  }
  else

  {
    for (size_t i = 0; i < nrOfParameterGroups; i++)
    {
      if (currentPreset->savedParameterGroups[i].active)
      {
        // if (true || distances[i] < currentPreset->savedParameterGroups[i].outerRadius) {
        clampedDistances[i] = distances[i] - clampAngle;
        clampedOuterRadius[i] = currentPreset->savedParameterGroups[i].outerRadius - clampAngle;

        // TODO: Try with quadratic or cubic weighting i.e. 1.0 / (
        // clampedDistances[i] * clampedDistances[i])
        // rawParameterWeights[i] = 1.0f / clampedDistances[i];

        rawParameterWeights[i] = constrain(1.f - (clampedDistances[i] / clampedOuterRadius[i]), 0.f, INFINITY);

        // } else {
        //   rawParameterWeights[i] = 0.0f;
        // }

        weightSum += rawParameterWeights[i];
      }
    }
  }

  // normalize weights to percentages
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      parameterWeights[i] = rawParameterWeights[i] / weightSum;
    }
  }
}

void printSavedParameterGroups()
{
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      printParameterGroup(currentPreset->savedParameterGroups[i]);
    }
  }
}

void printParameterGroup(struct parameterGroupState paramGroup)
{
  // printf("parameter group slot %i \n", &paramGroup->)
  printf("savedParamGroup slot %i -------------\n", paramGroup.slot);
  printf("w: %f, x:%f, y:%f, z:%f  \n", paramGroup.savedPose.w,
         paramGroup.savedPose.x, paramGroup.savedPose.y,
         paramGroup.savedPose.z);
  for (size_t i = 0; i < maxNrOfCCsInParameterGroup; i++)
  {
    if (paramGroup.savedControlChanges[i].active)
    {
      printf("%i: control=%i \t value=%i \n", i,
             paramGroup.savedControlChanges[i].cc,
             paramGroup.savedControlChanges[i].value);
    }
  }

  for (size_t i = 0; i < maxNrOfNotesInParameterGroup; i++)
  {
    if (paramGroup.savedNotes[i].active)
    {
      printf("%i: note=%i \t velocity=%i \n", i, paramGroup.savedNotes[i].note,
             paramGroup.savedNotes[i].velocity);
    }
  }
}

void sendPoseMidi()
{
  // printf("sending pose MIDI\n");
  controlChange controlChangesToSend[maxNrOfCCsInParameterGroup] = {0};
  for (size_t controlIndex = 0; controlIndex < maxNrOfCCsInParameterGroup;
       controlIndex++)
  {
    for (size_t groupIndex = 0; groupIndex < nrOfParameterGroups;
         groupIndex++)
    {
      if (currentPreset->savedParameterGroups[groupIndex].active)
      {
        // printf("adding cc to list of sending ccs\n");
        controlChangesToSend[controlIndex].active =
            currentPreset->savedParameterGroups[groupIndex]
                .savedControlChanges[controlIndex]
                .active;
        controlChangesToSend[controlIndex].cc =
            currentPreset->savedParameterGroups[groupIndex]
                .savedControlChanges[controlIndex]
                .cc;
        controlChangesToSend[controlIndex].value +=
            parameterWeights[groupIndex] *
            currentPreset->savedParameterGroups[groupIndex]
                .savedControlChanges[controlIndex]
                .value;
      }
    }
    if (controlChangesToSend[controlIndex].active)
    {
      usbMIDI.sendControlChange(controlChangesToSend[controlIndex].cc,
                                controlChangesToSend[controlIndex].value, 1);
    }
  }

  note notesToSend[maxNrOfNotesInParameterGroup] = {0};
  // for (size_t noteIndex = 0; noteIndex < maxNrOfNotesInParameterGroup;
  // noteIndex++)
  // {
  //   notesToSend[maxNrOfNotesInParameterGroup].velocity = 0;
  // }
  for (size_t noteIndex = 0; noteIndex < maxNrOfNotesInParameterGroup;
       noteIndex++)
  {
    for (size_t groupIndex = 0; groupIndex < nrOfParameterGroups;
         groupIndex++)
    {
      if (currentPreset->savedParameterGroups[groupIndex].active)
      {
        // printf("adding cc to list of sending ccs\n");
        notesToSend[noteIndex].active =
            currentPreset->savedParameterGroups[groupIndex]
                .savedNotes[noteIndex]
                .active;
        notesToSend[noteIndex].note =
            currentPreset->savedParameterGroups[groupIndex]
                .savedNotes[noteIndex]
                .note;
        // notesToSend[noteIndex].velocity += parameterWeights[groupIndex] *
        // currentPreset->savedParameterGroups[groupIndex].savedNotes[noteIndex].velocity;
        // if(currentPreset->savedParameterGroups[groupIndex].savedNotes[noteIndex].velocity
        // > 0){

        // }
        if (parameterWeights[groupIndex] > 0.80)
        {
          if (currentPreset->savedParameterGroups[groupIndex]
                  .savedNotes[noteIndex]
                  .velocity > 0)
          {
            notesToSend[noteIndex].velocity = 127;
          }
          else
          {
            notesToSend[noteIndex].velocity = 0;
          }
        }
      }
    }
    if (notesToSend[noteIndex].active)
    {
      usbMIDI.sendNoteOn(notesToSend[noteIndex].note,
                         notesToSend[noteIndex].velocity, 1);
    }
  }
}

void loop()
{
  // What is the time?
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

int16_t rotaryModeReferenceValue;
int prevRotaryValue = 0;
int prevEncButton = 0;
bool modeChooserActive = false;
int modeCandidate = 0;
int currentMode = 0;

void checkModeChooser()
{
  if (deviceState[currentNode].rotaryButton != prevEncButton)
  {
    printf("new encbutton value\n");
    prevEncButton = deviceState[currentNode].rotaryButton;
    if (!(deviceState[currentNode].rotaryButton % 2))
    {
      if (modeChooserActive)
      {
        modeChooserActive = false;
        currentMode = modeCandidate;
        // clearAllLeds();
        if (currentMode == 1 && !anyParamGroupSaved)
        {
          recordControlChanges = true;
          recordNotes = true;
        }
        else
        {
          recordControlChanges = false;
          recordNotes = false;
        }
      }
      else
      {
        rotaryModeReferenceValue = deviceState[currentNode].rotary;
        modeChooserActive = true;
      }
    }
  }
}

uint16_t rotaryValue;
void updateModeChooser()
{
  rotaryValue =
      deviceState[currentNode].rotary - rotaryModeReferenceValue + 10240;
  // rotaryValue /= 4;
  modeCandidate = (rotaryValue / 4) % 5;
  clearAllLeds();
  pulsateLed(modeCandidate, 0.2f);
}

void turnOnLedsForActiveParameterGroups()
{
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      turnOnLed(i);
    }
  }
}

void fadeLedsFromWeights()
{
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      fadeLed(i, parameterWeights[i] * parameterWeights[i]);
    }
    else
    {
      turnOffLed(i);
    }
  }
}

void clearAllLeds()
{
  for (size_t i = 0; i < nrOfLeds; i++)
  {
    pushState[currentNode].leds[i] = 0;
  }
}

void turnOnLed(int i)
{
  if (i < 0 || i >= nrOfLeds)
    return;
  pushState[currentNode].leds[i] = ledBrightness[i];
}

void turnOffLed(int i)
{
  if (i < 0 || i >= nrOfLeds)
    return;
  pushState[currentNode].leds[i] = 0;
}

void fadeLed(int i, float intensity)
{
  pushState[currentNode].leds[i] = ledBrightness[i] * intensity;
}

void pulsateLed(int i, float interval)
{
  unsigned long t = millis();
  float intensity = sin((float)t * 0.001 * PI / interval) * 0.5 + 0.5;
  pushState[currentNode].leds[i] = ledBrightness[i] * intensity;
};

bool wasButtonPressed(int i)
{
  bool changed = deviceState[currentNode].buttons[i] !=
                 previousDeviceState[currentNode].buttons[i];
  return changed && (deviceState[currentNode].buttons[i] % 2);
}

bool wasButtonReleased(int i)
{
  bool changed = deviceState[currentNode].buttons[i] !=
                 previousDeviceState[currentNode].buttons[i];
  return changed && !(deviceState[currentNode].buttons[i] % 2);
}

void baseStationLoop()
{
  noInterrupts();
  if (shouldRestartPoller || getNrOfFailedPolls() > 150 ||
      radio.failureDetected)
  {
    shouldRestartPoller = false;
    printf("restarting radio!!");
    restartPoller();
  }
  interrupts();

  digitalWrite(13, LOW);
  usbMIDI.read();
  // delay(10);
  bool radioSuccess = false;
  if (useRadio)
  {
    radioSuccess = pollNode(0, (uint8_t *)&pushState[currentNode],
                            (uint8_t *)&deviceState[currentNode]);
    if (radioSuccess)
    {
      // printf("poll received\n");
      currentQuaternion.w = Q15ToFloat(deviceState[currentNode].quaternion.w);
      currentQuaternion.x = Q15ToFloat(deviceState[currentNode].quaternion.x);
      currentQuaternion.y = Q15ToFloat(deviceState[currentNode].quaternion.y);
      currentQuaternion.z = Q15ToFloat(deviceState[currentNode].quaternion.z);
      currentQuaternion = quat_norm(currentQuaternion);
    }
    else
    {
      // printf("pollnode failed\n");
    }
  }

  checkModeChooser();
  if (modeChooserActive)
  {
    updateModeChooser();
  }
  else
  {

    calculateParameterWeights();
    clearAllLeds();
    if (currentMode == 0)
    {

      // for (size_t i = 0; i < nrOfLeds; i++)
      // {
      // }
      shouldSendPoseMidi =
          wasButtonReleased(0) ? !shouldSendPoseMidi : shouldSendPoseMidi;
      if (shouldSendPoseMidi)
      {

        fadeLedsFromWeights();
        // turnOnLed(0);
        if (sinceMidiSend > midiSendInterval)
        {
          sinceMidiSend = 0;
          sendPoseMidi();
        }
      }
    }
    else if (currentMode == 1) // pose creation mode
    {
      turnOnLedsForActiveParameterGroups();
      for (size_t slot = 0; slot < nrOfButtons; slot++)
      {
        if (wasButtonPressed(slot))
        {
          saveParameterGroup(slot);
        }
      }
    }
    else if (currentMode == 2) // pose repositioning mode
    {
      fadeLedsFromWeights();
      for (size_t slot = 0; slot < nrOfButtons; slot++)
      {
        if (wasButtonPressed(slot))
        {
          savePoseForParameterGroup(slot);
        }
      }
    }
    else if (currentMode == 4) // pose repositioning mode
    {
      for (size_t slot = 0; slot < nrOfButtons; slot++)
      {
        if (wasButtonPressed(slot))
        {
          currentPreset = &presets[slot];
        }
        if (currentPreset == &presets[slot])
        {
          pulsateLed(slot, 0.5);
        }
      }
    }
  }

  if (sincePrint > printInterval + 5000)
  {
    sincePrint = 5000;

    if (useRadio)
      if (radioSuccess)
      {
        printf("poll received\n");
      }
      else
      {
        printf("pollnode failed\n");
      }

    printState(deviceState[currentNode]);

    // printf("rotaryreference: %i \n", rotaryModeReferenceValue);
    // printf("rotaryValue: %i \n", rotaryValue);
    // printf("mode chooser active: %i \n", modeChooserActive);
    // printf("modeChooser: %i\n", modeCandidate);
    // printf("mode: %i\n", currentMode);

    // Serial.println("current -----");
    // printQuaternion(((currentQuaternion)));
    // float currentAngle = quat_angle((currentQuaternion));
    // printf("angle: %f \n", toDegrees(currentAngle));
    // Serial.println("learned ----- ");
    // printQuaternion(learnedPoses[0]);
    // float learnedAngle = quat_angle(learnedPoses[0]);
    // printf("angle: %f \n", toDegrees(learnedAngle));

    // quaternion deltaQ = quat_delta_rotation(currentQuaternion,
    // learnedPoses[0]); Serial.println("delta ----- ");
    // printQuaternion(deltaQ);

    // float angle = quat_angle(currentQuaternion, learnedPoses[0]);
    // printf("delta angle: %f \n", toDegrees(angle));
    // Serial.println();

    printSavedParameterGroups();
    printOuterRadius();
    printAngleDistances();
    printClampedAngleDistances();
    printRawParameterWeights();
    printParameterWeights();
    printFullyTriggered();
  }

  handleSerialBaseStation();

  previousDeviceState[currentNode] = deviceState[currentNode];
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
  if (shouldRestartAcker || getNrOfCorruptPackets() > 50 ||
      radio.failureDetected)
  {
    shouldRestartAcker = false;
    printf("restarting radio!!");
    restartAcker(role);
  }
  interrupts();

  unsigned long t = sinceLastLoop;
  sinceLastLoop = 0;
  // printf("loopTime %ul \n", t);
  // printf(".\n");

  // handle all the inputs
  if (rotary.updated())
  {
    deviceState[role].rotary = rotary.value;
  }
  for (size_t i = 0; i < nrOfButtons; i++)
  {
    deviceState[role].buttons[i] = button[i].value;
  }
  deviceState[role].rotaryButton = encButton.value;
  // deviceState[role].shake = shakeSensor.value;

  // sync the leds in deviceState to the ones in received pushState
  for (size_t i = 0; i < nrOfLeds; i++)
  {
    deviceState[role].leds[i] = pushState[role].leds[i];
  }
  // Now set the leds from the deviceState (which should be updated by received
  // radio mesage)
  for (size_t i = 0; i < nrOfLeds; i++)
  {
    // float scaler = (sin((float)millis()/1500.f) + 1)/2.f;
    // scaler = 1.f + scaler;
    analogWrite(ledPins[i], deviceState[role].leds[i]);
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

  handleSerialNode();
}

void printAngleDistances()
{
  printf("angleDistances: ");
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      printf("%f, ", distances[i]);
    }
  }
  Serial.println();
}

void printClampedAngleDistances()
{
  printf("clampedAngleDistances: ");
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      printf("%f, ", clampedDistances[i]);
    }
  }
  Serial.println();
}

void printRawParameterWeights()
{
  printf("rawParameterWeights: ");
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      printf("%f, ", rawParameterWeights[i]);
    }
  }
  Serial.println();
}

void printParameterWeights()
{
  printf("parameterWeights: ");
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      printf("%f, ", parameterWeights[i]);
    }
  }
  Serial.println();
}

void printFullyTriggered()
{
  printf("fully Triggered: ");
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      printf("%i, ", fullyTriggeredPoses[i]);
    }
  }
  Serial.println();
}

void printOuterRadius()
{
  printf("minDistances (angle): ");
  for (size_t i = 0; i < nrOfParameterGroups; i++)
  {
    if (currentPreset->savedParameterGroups[i].active)
    {
      printf("%f, ", currentPreset->savedParameterGroups[i].outerRadius);
    }
  }
  Serial.println();
}

void printState(struct state deviceState)
{
  printf("nodeId\t %i \n", deviceState.nodeId);
  printf("messageCounter\t %i \n", deviceState.updateCounter);
  printf("quaternion\t %f,%f,%f,%f \n", Q15ToFloat(deviceState.quaternion.w),
         Q15ToFloat(deviceState.quaternion.x),
         Q15ToFloat(deviceState.quaternion.y),
         Q15ToFloat(deviceState.quaternion.z));
  for (size_t buttonIndex = 0; buttonIndex < nrOfButtons; buttonIndex++)
  {
    printf("button %i\t %i \n", buttonIndex, deviceState.buttons[buttonIndex]);
  }
  // printf("shake\t %i \n", deviceState.shake);
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

void handleSerialBaseStation()
{
  if (Serial.available())
  {
    uint8_t c = Serial.read();
    switch (c)
    {
    case '1':
      // learnedPoses[0] = currentQuaternion;
      // activePoseSlots[0] = true;
      saveParameterGroup(0);
      break;
    case '2':
      // learnedPoses[1] = currentQuaternion;
      // activePoseSlots[1] = true;
      saveParameterGroup(1);
      break;
    case '3':
      // learnedPoses[2] = currentQuaternion;
      // activePoseSlots[2] = true;
      saveParameterGroup(2);
      break;
    case '4':
      // learnedPoses[3] = currentQuaternion;
      // activePoseSlots[3] = true;
      saveParameterGroup(3);
      break;
    case '5':
      // learnedPoses[3] = currentQuaternion;
      // activePoseSlots[3] = true;
      saveParameterGroup(4);
      break;
    case 'r':
      shouldRestartPoller = true;
      break;
    case 'o':
      useRadio = !useRadio;
      break;
    case 'c':
      recordControlChanges = !recordControlChanges;
      recordNotes = !recordNotes;
      break;
    case 's':
      saveToSD();
      break;
    case 'l':
      loadFromSD();
      break;
    }
  }
}

void handleSerialNode()
{
  if (Serial.available())
  {
    uint8_t c = Serial.read();
    switch (c)
    {
    case 'r':
      shouldRestartAcker = true;
      break;
    }
  }
}

char fileNames[nrOfPresetSlots][14] = {"saveData1.bin", "saveData2.bin",
                                       "saveData3.bin", "saveData4.bin",
                                       "saveData5.bin"};
char fileName[] = {"saveData.bin"};
File saveFile;
bool saveToSD()
{
  saveFile = SD.open(fileName, FILE_WRITE);
  if (!saveFile)
  {
    printf("Failed to open/create saveFile in write mode! Your CRYYY!\n");
    delay(500);
    return;
  }
  if (!saveFile.seek(0))
  {
    printf("failed to seek\n");
    return false;
  }
  // int writeCount = saveFile.write((uint8_t
  // *)currentPreset->savedParameterGroups, sizeof(parameterGroupState) *
  // nrOfParameterGroups);
  int writeCount =
      saveFile.write((uint8_t *)presets, sizeof(preset) * nrOfPresetSlots);
  printf("wrote %i bytes to the file. Wuuuhuu!\n", writeCount);
  delay(250);
  saveFile.close();
}

bool loadFromSD()
{
  saveFile = SD.open(fileName);
  if (!saveFile)
  {
    printf("Failed to open saveFile in read mode! Your CRYYY!\n");
    delay(500);
    return;
  }
  // saveFile.read((uint8_t *)currentPreset->savedParameterGroups,
  // sizeof(parameterGroupState) * nrOfParameterGroups);
  saveFile.read((uint8_t *)presets, sizeof(preset) * nrOfPresetSlots);
  saveFile.close();
}
