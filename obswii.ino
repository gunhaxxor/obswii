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
#define NOSTOP I2C_NOSTOP
#include "EM7180.h"

EM7180 em7180;

// #include "usfs.cpp"

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
state deviceState[2];
state pushState[2];

const int stateSize = sizeof(deviceState[0]);

//Orientation sensor stuff
quaternion currentQuaternion;
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
    //Orientation sensor stuff -------------------------
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
    // Should detect SENtral at 0x28
    I2Cscan();

    // Start EM7180 interaction
    em7180.begin();

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    for (uint8_t k = 0; k < 10; ++k)
    {

      uint8_t stat = em7180.getSentralStatus() & 0x01;

      if (stat & 0x01)
        Serial.println("EEPROM detected on the sensor bus!");
      if (stat & 0x02)
        Serial.println("EEPROM uploaded config file!");
      if (stat & 0x04)
        Serial.println("EEPROM CRC incorrect!");
      if (stat & 0x08)
        Serial.println("EM7180 in initialized state!");
      if (stat & 0x10)
        Serial.println("No EEPROM detected!");

      if (stat)
        break;

      em7180.requestReset();

      delay(500);
    }

    if (!(em7180.getSentralStatus() & 0x04))
      Serial.println("EEPROM upload successful!");

    // em7180.setPassThroughMode();
    // // Fetch the WarmStart data from the M24512DFM I2C EEPROM
    // readSenParams();
    // // Take Sentral out of pass-thru mode and re-start algorithm
    // em7180.setMasterMode();

    // // Put the Sentral in pass-thru mode
    // em7180.setPassThroughMode();

    // // Fetch the WarmStart data from the M24512DFM I2C EEPROM
    // readAccelCal();
    // Serial.print("X-acc max: ");
    // Serial.println(global_conf.accZero_max[0]);
    // Serial.print("Y-acc max: ");
    // Serial.println(global_conf.accZero_max[1]);
    // Serial.print("Z-acc max: ");
    // Serial.println(global_conf.accZero_max[2]);
    // Serial.print("X-acc min: ");
    // Serial.println(global_conf.accZero_min[0]);
    // Serial.print("Y-acc min: ");
    // Serial.println(global_conf.accZero_min[1]);
    // Serial.print("Z-acc min: ");
    // Serial.println(global_conf.accZero_min[2]);

    // // Take Sentral out of pass-thru mode and re-start algorithm
    // em7180.setMasterMode();

    // Set SENtral in initialized state to configure registers

    // em7180.setRunDisable();
    // // Load Accel Cal
    //   EM7180_acc_cal_upload();
    // // Force initialize
    // em7180.setRunEnable();

    // Load Warm Start parameters
    // EM7180_set_WS_params();

    // Set SENtral in initialized state to configure registers
    em7180.setRunDisable();

    //Setup LPF bandwidth (BEFORE setting ODR's)
    em7180.setAccelLpfBandwidth(0x03); // 41Hz
    em7180.setGyroLpfBandwidth(0x01);  // 184Hz

    // Set accel/gyro/mage desired ODR rates
    em7180.setQRateDivisor(0x02);    // 100 Hz
    em7180.setMagRate(0x64);         // 100 Hz
    em7180.setAccelRate(0x14);       // 200/10 Hz
    em7180.setGyroRate(0x14);        // 200/10 Hz
    em7180.setBaroRate(0x80 | 0x32); // set enable bit and set Baro rate to 25 Hz

    // Configure operating mode
    em7180.algorithmControlReset(); // read scale sensor data

    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    em7180.enableEvents(0x07);

    // Enable EM7180 run mode
    em7180.setRunEnable();
    delay(100);

    // EM7180 parameter adjustments
    Serial.println("Beginning Parameter Adjustments");

    // Read sensor default FS values from parameter space
    uint8_t param[4];
    readParams(0x4A, param);

    uint16_t EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
    uint16_t EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
    Serial.print("Magnetometer Default Full Scale Range: +/-");
    Serial.print(EM7180_mag_fs);
    Serial.println("uT");
    Serial.print("Accelerometer Default Full Scale Range: +/-");
    Serial.print(EM7180_acc_fs);
    Serial.println("g");
    readParams(0x4B, param); // Request to read  parameter 75
    uint16_t EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
    Serial.print("Gyroscope Default Full Scale Range: +/-");
    Serial.print(EM7180_gyro_fs);
    Serial.println("dps");
    em7180.requestParamRead(0x00);  //End parameter transfer
    em7180.algorithmControlReset(); // re-enable algorithm

    // Disable stillness mode
    EM7180_set_integer_param(0x49, 0x00);

    // Write desired sensor full scale ranges to the EM7180
    em7180.setMagAccFs(0x3E8, 0x08); // 1000 uT, 8 g
    em7180.setGyroFs(0x7D0);         // 2000 dps

    // Read sensor new FS values from parameter space
    readParams(0x4A, param); // Request to read  parameter 74
    EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
    Serial.print("Magnetometer New Full Scale Range: +/-");
    Serial.print(EM7180_mag_fs);
    Serial.println("uT");
    Serial.print("Accelerometer New Full Scale Range: +/-");
    Serial.print(EM7180_acc_fs);
    Serial.println("g");
    readParams(0x4B, param); // Request to read  parameter 75
    EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
    Serial.print("Gyroscope New Full Scale Range: +/-");
    Serial.print(EM7180_gyro_fs);
    Serial.println("dps");
    em7180.requestParamRead(0x00);  //End parameter transfer
    em7180.algorithmControlReset(); // re-enable algorithm

    // Read EM7180 status
    if (em7180.getRunStatus() & 0x01)
      Serial.println(" EM7180 run status = normal mode");
    uint8_t algoStatus = em7180.getAlgorithmStatus();
    if (algoStatus & 0x01)
      Serial.println(" EM7180 standby status");
    if (algoStatus & 0x02)
      Serial.println(" EM7180 algorithm slow");
    if (algoStatus & 0x04)
      Serial.println(" EM7180 in stillness mode");
    if (algoStatus & 0x08)
      Serial.println(" EM7180 mag calibration completed");
    if (algoStatus & 0x10)
      Serial.println(" EM7180 magnetic anomaly detected");
    if (algoStatus & 0x20)
      Serial.println(" EM7180 unreliable sensor data");
    if (em7180.getPassThruStatus() & 0x01)
      Serial.print(" EM7180 in passthru mode!");
    uint8_t eventStatus = em7180.getEventStatus();
    if (eventStatus & 0x01)
      Serial.println(" EM7180 CPU reset");
    if (eventStatus & 0x02)
      Serial.println(" EM7180 Error");

    // Give some time to read the screen
    delay(1000);

    // Check sensor status
    uint8_t sensorStatus = em7180.getSensorStatus();
    if (sensorStatus & 0x01)
      sensorError("Magnetometer not acknowledging!");
    if (sensorStatus & 0x02)
      sensorError("Accelerometer not acknowledging!");
    if (sensorStatus & 0x04)
      sensorError("Gyro not acknowledging!");
    if (sensorStatus & 0x10)
      sensorError("Magnetometer ID not recognized!");
    if (sensorStatus & 0x20)
      sensorError("Accelerometer ID not recognized!");
    if (sensorStatus & 0x40)
      sensorError("Gyro ID not recognized!");
  }

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
  delay(100);

  if (pollNode(0, 0, (uint8_t *)&pushState[currentNode], (uint8_t *)&deviceState[currentNode]))
  {
    printf("poll received\n");
  }
  else
  {
    printf("pollnode failed\n");
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

  currentQuaternion.w = Q14ToFloat(deviceState[currentNode].quaternion.w);
  currentQuaternion.x = Q14ToFloat(deviceState[currentNode].quaternion.x);
  currentQuaternion.y = Q14ToFloat(deviceState[currentNode].quaternion.y);
  currentQuaternion.z = Q14ToFloat(deviceState[currentNode].quaternion.z);

  calculateParameterWeights();

  if (true || sincePrint > printInterval)
  {
    sincePrint = 0;
    // printState(deviceState[currentNode]);

    // float currentQuat[4] = {currentQuaternion.w, currentQuaternion.x, currentQuaternion.y, currentQuaternion.z};
    // float learnedQuat[4] = {currentQuaternion.w, currentQuaternion.x, currentQuaternion.y, currentQuaternion.z};
    // float learnedQuat[4] = {learnedPoses[0].w, learnedPoses[0].x, learnedPoses[0].y, learnedPoses[0].z};
    Serial.println("current -----");
    printQuaternion((quat_uniform_w(currentQuaternion)));
    float currentAngle = quat_angle(quat_uniform_w(currentQuaternion));
    printf("angle: %f \n", toDegrees(currentAngle));
    Serial.println("learned ----- ");
    printQuaternion(learnedPoses[0]);
    float learnedAngle = quat_angle(learnedPoses[0]);
    printf("angle: %f \n", toDegrees(learnedAngle));

    float angle = quat_angle(currentQuaternion, learnedPoses[0]);
    printf("delta angle: %f \n", toDegrees(angle));
    Serial.println();

    printAngleDistances();
    printParameterWeights();
  }

  handleSerial();
}

void nodeLoop()
{
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

  //syn the leds in deviceState to the ones in received pushState
  for (size_t i = 0; i < nrOfLeds; i++)
  {
    deviceState[role].leds[i] = pushState[role].leds[i];
  }
  //Now set the leds from the deviceState (which should be updated by received radio mesage)
  for (size_t i = 0; i < nrOfLeds; i++)
  {
    digitalWrite(ledPins[i], deviceState[role].leds[i] % 2);
  }

  //handle orientation sensor
  static float Quat[4];

  static float ax;
  static float ay;
  static float az;
  // Check event status register, way to check data ready by polling rather than interrupt
  uint8_t eventStatus = em7180.getEventStatus(); // reading clears the register

  // Check for errors
  // Error detected, what is it?
  if (eventStatus & 0x02)
  {

    uint8_t errorStatus = em7180.getErrorStatus();
    if (!errorStatus)
    {
      Serial.print(" EM7180 sensor status = ");
      Serial.println(errorStatus);
      if (errorStatus == 0x11)
        Serial.print("Magnetometer failure!");
      if (errorStatus == 0x12)
        Serial.print("Accelerometer failure!");
      if (errorStatus == 0x14)
        Serial.print("Gyro failure!");
      if (errorStatus == 0x21)
        Serial.print("Magnetometer initialization failure!");
      if (errorStatus == 0x22)
        Serial.print("Accelerometer initialization failure!");
      if (errorStatus == 0x24)
        Serial.print("Gyro initialization failure!");
      if (errorStatus == 0x30)
        Serial.print("Math error!");
      if (errorStatus == 0x80)
        Serial.print("Invalid sample rate!");
    }
    // Handle errors ToDo
  }

  // if no errors, see if new data is ready
  // new acceleration data available
  if (eventStatus & 0x10)
  {

    int16_t accelCount[3];

    em7180.readAccelerometer(accelCount[0], accelCount[1], accelCount[2]);

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * 0.000488; // get actual g value
    ay = (float)accelCount[1] * 0.000488;
    az = (float)accelCount[2] * 0.000488;

    // // Manages accelerometer calibration; is active when calibratingA > 0
    // Accel_cal_check(accelCount);
  }

  if (eventStatus & 0x04)
  {
    em7180.readQuaternion(Quat[0], Quat[1], Quat[2], Quat[3]);
    // printf("quat: x= %f, y=%f, z=%f, w=%f \n", Quat[0], Quat[1], Quat[2], Quat[3]);
    deviceState[role].quaternion.w = floatToQ14(Quat[0]);
    deviceState[role].quaternion.x = floatToQ14(Quat[1]);
    deviceState[role].quaternion.y = floatToQ14(Quat[2]);
    deviceState[role].quaternion.z = floatToQ14(Quat[3]);
  }

  if (sincePrint > printInterval)
  {
    sincePrint = 0;
    printAlgorithmStatus();
  }
}

const float clampAngle = 5;
bool activePoseSlots[nrOfPoseSlots] = {0};
bool fullyTriggeredPoses[nrOfPoseSlots]  {0};
float parameterWeights[nrOfPoseSlots] = {0};
float distances[nrOfPoseSlots] = {0};
float clampedDistances[nrOfPoseSlots] = {0};
void calculateParameterWeights(){
  float invertedDistancesSum = 0.f;
  float InvertedDistances[nrOfPoseSlots] = {0};
  bool anyPoseIsFullyTriggered = false;
  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if(activePoseSlots[i]){
      distances[i] = toDegrees(quat_angle(currentQuaternion, learnedPoses[i]));
      if(distances[i] < clampAngle){
        fullyTriggeredPoses[i] = true;
        anyPoseIsFullyTriggered = true;
      }else{
        fullyTriggeredPoses[i] = false;
      }
    }
  }

  if(anyPoseIsFullyTriggered){
    for (size_t i = 0; i < nrOfPoseSlots; i++)
    {
      if(activePoseSlots[i]){
        if(fullyTriggeredPoses[i]){
          InvertedDistances[i] = 1.0f/distances[i];
          invertedDistancesSum += InvertedDistances[i];
        }else{
          InvertedDistances[i] = 0.0f;
        }
      }
    }
  }else{
    for (size_t i = 0; i < nrOfPoseSlots; i++)
    {
      if(activePoseSlots[i]){
        clampedDistances[i] = distances[i] - clampAngle;
        InvertedDistances[i] = 1.0f/clampedDistances[i];
        invertedDistancesSum += InvertedDistances[i];
      }
    }
  }


  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if(activePoseSlots[i]){
      parameterWeights[i] = InvertedDistances[i] / invertedDistancesSum;
    }
  }
}

// const float poseClampThreshold = 0.9f;
// const float poseClampMultiplier = 1.0f/poseClampThreshold;
// void clampParameterWeights(){
//   for (size_t i = 0; i < nrOfPoseSlots; i++)
//   {
//     if(activePoseSlots[i]){
//       if(parameterWeights[i] > poseClampThreshold){

//       }
//     }
//   }
// }

void printAngleDistances(){
  printf("angleDistances: ");
  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if(activePoseSlots[i]){
      printf("%f, ", distances[i]);
    }
  }
  Serial.println();
}

void printParameterWeights(){
  printf("parameterWeights: ");
  for (size_t i = 0; i < nrOfPoseSlots; i++)
  {
    if(activePoseSlots[i]){
      printf("%f, ", parameterWeights[i]);
    }
  }
  Serial.println();
}

void printAlgorithmStatus(){
  uint8_t algoStatus = em7180.getAlgorithmStatus();
    if (algoStatus & 0x01)
      Serial.println(" EM7180 standby status");
    if (algoStatus & 0x02)
      Serial.println(" EM7180 algorithm slow");
    if (algoStatus & 0x04)
      Serial.println(" EM7180 in stillness mode");
    if (algoStatus & 0x08)
      Serial.println(" EM7180 mag calibration completed");
    if (algoStatus & 0x10)
      Serial.println(" EM7180 magnetic anomaly detected");
    if (algoStatus & 0x20)
      Serial.println(" EM7180 unreliable sensor data");
}

void printState(struct state deviceState)
{
  printf("nodeId\t %i \n", deviceState.nodeId);
  printf("messageCounter\t %i \n", deviceState.updateCounter);
  printf("quaternion\t %f,%f,%f,%f \n", Q14ToFloat(deviceState.quaternion.w), Q14ToFloat(deviceState.quaternion.x), Q14ToFloat(deviceState.quaternion.y), Q14ToFloat(deviceState.quaternion.z));
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
    }
  }
}
