#include "Arduino.h"
#include "usfs_registers.h"
#include <i2c_t3.h>

#define SerialDebug true
// General purpose variables
int16_t serial_input;
static int16_t warm_start = 0;
static int16_t warm_start_saved = 0;
// used for param transfer
uint8_t param[4];
// EM7180 sensor full scale ranges
uint16_t EM7180_mag_fs;
uint16_t EM7180_acc_fs;
uint16_t EM7180_gyro_fs;

// // variables to hold latest sensor data values
float ax;
float ay;
float az;
float gx;
float gy;
float gz;
float mx;
float my;
float mz;

// MPU9250 variables
// Stores the 16-bit signed accelerometer sensor output
int16_t accelCount[3];

// Stores the 16-bit signed gyro sensor output
int16_t gyroCount[3];

// Stores the 16-bit signed magnetometer sensor output
int16_t magCount[3];

// quaternion data register
float Quat[4] = {0, 0, 0, 0};

// // Pressure, temperature raw count output
int16_t rawPressure;
int16_t rawTemperature;

// // Stores the MPU9250 internal chip temperature in degrees Celsius
float temperature;
float pressure;
float altitude;

Sentral_WS_params WS_params;

elapsedMillis sinceUSFSPrint = 0;
// elapsedMillis calibrationTimeout = 0;

void EM7180_setup()
{
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(100);

  I2Cscan();

  // Read SENtral device information
  uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
  uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
  Serial.print("EM7180 ROM Version: 0x");
  Serial.print(ROM1, HEX);
  Serial.println(ROM2, HEX);
  Serial.println("Should be: 0xE609");
  uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
  uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
  Serial.print("EM7180 RAM Version: 0x");
  Serial.print(RAM1);
  Serial.println(RAM2);
  uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
  Serial.print("EM7180 ProductID: 0x");
  Serial.print(PID, HEX);
  Serial.println(" Should be: 0x80");
  uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
  Serial.print("EM7180 RevisionID: 0x");
  Serial.print(RID, HEX);
  Serial.println(" Should be: 0x02");

  // Give some time to read the screen
  delay(1000);

  // Check which sensors can be detected by the EM7180
  uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
  if (featureflag & 0x01)
    Serial.println("A barometer is installed");
  if (featureflag & 0x02)
    Serial.println("A humidity sensor is installed");
  if (featureflag & 0x04)
    Serial.println("A temperature sensor is installed");
  if (featureflag & 0x08)
    Serial.println("A custom sensor is installed");
  if (featureflag & 0x10)
    Serial.println("A second custom sensor is installed");
  if (featureflag & 0x20)
    Serial.println("A third custom sensor is installed");

  // Give some time to read the screen
  delay(1000);

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)
    Serial.println("EEPROM detected on the sensor bus!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)
    Serial.println("EEPROM uploaded config file!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)
    Serial.println("EEPROM CRC incorrect!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)
    Serial.println("EM7180 in initialized state!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)
    Serial.println("No EEPROM detected!");
  int count = 0;
  while (!STAT)
  {
    writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
    delay(500);
    count++;
    STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)
      Serial.println("EEPROM detected on the sensor bus!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)
      Serial.println("EEPROM uploaded config file!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)
      Serial.println("EEPROM CRC incorrect!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)
      Serial.println("EM7180 in initialized state!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)
      Serial.println("No EEPROM detected!");
    if (count > 10)
      break;
  }
  if (!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))
    Serial.println("EEPROM upload successful!");

  // Take user input to choose Warm Start or not...
  // "1" from the keboard is ASCII "1" which gives integer value 49
  // "0" from the keboard is ASCII "0" which gives integer value 48

  Serial.println();
  Serial.println("Send 'c' quickly for no warm start, else warm start will be automatically chosen");
  serial_input = Serial.read();

  elapsedMillis calibrationTimeout = 0;
  while (calibrationTimeout < 1000 && !(serial_input == 'c'))
  {
    serial_input = Serial.read();
    delay(50);
  }
  if (serial_input == 'c')
  {
    Serial.println("***No Warm Start***");
  }
  else
  {
    Serial.println("!!!Warm Start active!!!");

    // Put the Sentral in pass-thru mode
    WS_PassThroughMode();

    // Fetch the WarmStart data from the M24512DFM I2C EEPROM
    readSenParams();

    // Take Sentral out of pass-thru mode and re-start algorithm
    WS_Resume();
  }

  // Reset Sentral after

  // Give some time to read the screen
  delay(1000);

  // Set SENtral in initialized state to configure registers
  writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00);

  // Insert Acc Cal upload here when the time comes...

  // Force initialize
  writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01);

  // Load Warm Start parameters
  if (warm_start)
  {
    EM7180_set_WS_params();
  }

  // Set SENtral in initialized state to configure registers
  writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00);

  //Setup LPF bandwidth (BEFORE setting ODR's)
  writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03);  // 41 Hz
  writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 42 Hz

  // Set accel/gyro/mage desired ODR rates
  writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02);    // 100 Hz
  writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x64);         // 100 Hz
  writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14);       // 200/10 Hz
  writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14);        // 200/10 Hz
  writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x32); // set enable bit and set Baro rate to 25 Hz

  // Configure operating mode
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data

  // Enable interrupt to host upon certain events
  // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
  // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
  writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);

  // Enable EM7180 run mode
  writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
  delay(100);

  // EM7180 parameter adjustments
  Serial.println("Beginning Parameter Adjustments");

  // Read sensor default FS values from parameter space
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A);     // Request to read parameter 74
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
  byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(param_xfer == 0x4A))
  {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
  EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
  Serial.print("Magnetometer Default Full Scale Range: +/-");
  Serial.print(EM7180_mag_fs);
  Serial.println("uT");
  Serial.print("Accelerometer Default Full Scale Range: +/-");
  Serial.print(EM7180_acc_fs);
  Serial.println("g");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
  param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(param_xfer == 0x4B))
  {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
  Serial.print("Gyroscope Default Full Scale Range: +/-");
  Serial.print(EM7180_gyro_fs);
  Serial.println("dps");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);     //End parameter transfer
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

  //Disable stillness mode
  EM7180_set_integer_param(0x49, 0x00);

  //Write desired sensor full scale ranges to the EM7180
  EM7180_set_mag_acc_FS(0x3E8, 0x08); // 1000 uT, 8 g
  EM7180_set_gyro_FS(0x7D0);          // 2000 dps

  // Read sensor new FS values from parameter space
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A);     // Request to read  parameter 74
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
  param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(param_xfer == 0x4A))
  {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
  EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
  Serial.print("Magnetometer New Full Scale Range: +/-");
  Serial.print(EM7180_mag_fs);
  Serial.println("uT");
  Serial.print("Accelerometer New Full Scale Range: +/-");
  Serial.print(EM7180_acc_fs);
  Serial.println("g");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
  param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(param_xfer == 0x4B))
  {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
  Serial.print("Gyroscope New Full Scale Range: +/-");
  Serial.print(EM7180_gyro_fs);
  Serial.println("dps");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);     //End parameter transfer
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

  // Read EM7180 status
  uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
  if (runStatus & 0x01)
    Serial.println(" EM7180 run status = normal mode");
  uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
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
  uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
  if (passthruStatus & 0x01)
    Serial.print(" EM7180 in passthru mode!");
  uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
  if (eventStatus & 0x01)
    Serial.println(" EM7180 CPU reset");
  if (eventStatus & 0x02)
    Serial.println(" EM7180 Error");
  if (eventStatus & 0x04)
    Serial.println(" EM7180 new quaternion result");
  if (eventStatus & 0x08)
    Serial.println(" EM7180 new mag result");
  if (eventStatus & 0x10)
    Serial.println(" EM7180 new accel result");
  if (eventStatus & 0x20)
    Serial.println(" EM7180 new gyro result");

  // Give some time to read the screen
  delay(1000);

  // Check sensor status
  uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
  Serial.print(" EM7180 sensor status = ");
  Serial.println(sensorStatus);
  if (sensorStatus & 0x01)
    Serial.print("Magnetometer not acknowledging!");
  if (sensorStatus & 0x02)
    Serial.print("Accelerometer not acknowledging!");
  if (sensorStatus & 0x04)
    Serial.print("Gyro not acknowledging!");
  if (sensorStatus & 0x10)
    Serial.print("Magnetometer ID not recognized!");
  if (sensorStatus & 0x20)
    Serial.print("Accelerometer ID not recognized!");
  if (sensorStatus & 0x40)
    Serial.print("Gyro ID not recognized!");

  Serial.print("Actual MagRate = ");
  Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate));
  Serial.println(" Hz");
  Serial.print("Actual AccelRate = ");
  Serial.print(10 * readByte(EM7180_ADDRESS, EM7180_ActualAccelRate));
  Serial.println(" Hz");
  Serial.print("Actual GyroRate = ");
  Serial.print(10 * readByte(EM7180_ADDRESS, EM7180_ActualGyroRate));
  Serial.println(" Hz");
  Serial.print("Actual BaroRate = ");
  Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate));
  Serial.println(" Hz");
  Serial.println("");
  Serial.println("*******************************************");
  Serial.println("Send '1' to store Warm Start configuration");
  Serial.println("*******************************************");
  Serial.println("");

  // Give some time to read the screen
  delay(1000);
}

void EM7180_loop()
{
  serial_input = Serial.read();
  if (serial_input == 49)
  {
    delay(100);
    EM7180_get_WS_params();

    // Put the Sentral in pass-thru mode
    WS_PassThroughMode();

    // Store WarmStart data to the M24512DFM I2C EEPROM
    writeSenParams();

    // Take Sentral out of pass-thru mode and re-start algorithm
    WS_Resume();
    warm_start_saved = 1;
  }

  // Check event status register, way to chech data ready by polling rather than interrupt
  uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

  // Check for errors
  // Error detected, what is it?
  if (eventStatus & 0x02)
  {
    uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
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
    readSENtralAccelData(accelCount);

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * 0.000488; // get actual g value
    ay = (float)accelCount[1] * 0.000488;
    az = (float)accelCount[2] * 0.000488;
  }

  if (eventStatus & 0x20)
  {
    // new gyro data available
    readSENtralGyroData(gyroCount);

    // Now we'll calculate the gyro value into actual dps's
    gx = (float)gyroCount[0] * 0.153; // get actual dps value
    gy = (float)gyroCount[1] * 0.153;
    gz = (float)gyroCount[2] * 0.153;
  }

  if (eventStatus & 0x08)
  {
    // new mag data available
    readSENtralMagData(magCount);

    // Now we'll calculate the mag value into actual G's
    // get actual G value
    mx = (float)magCount[0] * 0.305176;
    my = (float)magCount[1] * 0.305176;
    mz = (float)magCount[2] * 0.305176;
  }

  if (eventStatus & 0x04) // new quaternions available
  {
    readSENtralQuatData(Quat);
  }

  // get BMP280 pressure
  // new baro data available

  if (eventStatus & 0x40)
  {
    rawPressure = readSENtralBaroData();
    pressure = (float)rawPressure * 0.01f + 1013.25f; // pressure in mBar

    // get BMP280 temperature
    rawTemperature = readSENtralTempData();
    temperature = (float)rawTemperature * 0.01; // temperature in degrees C
  }

  if (sinceUSFSPrint > 100)
  {
    sinceUSFSPrint = 0;

    if (SerialDebug)
    {
      Serial.print("ax = ");
      Serial.print((int)1000 * ax);
      Serial.print(" ay = ");
      Serial.print((int)1000 * ay);
      Serial.print(" az = ");
      Serial.print((int)1000 * az);
      Serial.println(" mg");
      Serial.print("gx = ");
      Serial.print(gx, 2);
      Serial.print(" gy = ");
      Serial.print(gy, 2);
      Serial.print(" gz = ");
      Serial.print(gz, 2);
      Serial.println(" deg/s");
      Serial.print("mx = ");
      Serial.print((int)mx);
      Serial.print(" my = ");
      Serial.print((int)my);
      Serial.print(" mz = ");
      Serial.print((int)mz);
      Serial.println(" mG");
      Serial.println("Hardware quaternions (NED):");
      Serial.print("Q0 = ");
      Serial.print(Quat[0]);
      Serial.print(" Qx = ");
      Serial.print(Quat[1]);
      Serial.print(" Qy = ");
      Serial.print(Quat[2]);
      Serial.print(" Qz = ");
      Serial.println(Quat[3]);
    }
    // if (passThru)
    // {
    //   rawPress = readBMP280Pressure();
    //   pressure = (float)bmp280_compensate_P(rawPress) / 25600.; // Pressure in mbar
    //   rawTemp = readBMP280Temperature();
    //   temperature = (float)bmp280_compensate_T(rawTemp) / 100.;
    // }

    //Hardware AHRS:
    float Yaw = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);
    float Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
    float Roll = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    Pitch *= 180.0f / PI;
    Yaw *= 180.0f / PI;
    // Yaw += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if (Yaw < 0)
      Yaw += 360.0f; // Ensure yaw stays between 0 and 360
    Roll *= 180.0f / PI;

    // Or define output variable according to the Android system, where heading (0 to 360) is defined by the angle between the y-axis
    // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
    // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
    // points toward the right of the device.

    if (SerialDebug)
    {
      Serial.print("Hardware Yaw, Pitch, Roll: ");
      Serial.print(Yaw, 2);
      Serial.print(", ");
      Serial.print(Pitch, 2);
      Serial.print(", ");
      Serial.println(Roll, 2);
      Serial.println("BMP280:");
      Serial.print("Altimeter temperature = ");
      Serial.print(temperature, 2);
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = ");
      Serial.print(9. * temperature / 5. + 32., 2);
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = ");
      Serial.print(pressure, 2);
      Serial.println(" mbar"); // pressure in millibar
      altitude = 145366.45f * (1.0f - pow((pressure / 1013.25f), 0.190284f));
      Serial.print("Altitude = ");
      Serial.print(altitude, 2);
      Serial.println(" feet");
      Serial.println(" ");
      if (warm_start_saved)
      {
        Serial.println("Warm Start configuration saved!");
      }
      else
      {
        Serial.println("Send '1' to store Warm Start configuration");
      }
    }
  }
}

float *EM7180_getQuaternion()
{
  return Quat;
}

//===================================================================================================================
//====== Sentral parameter management functions
//===================================================================================================================

void EM7180_set_gyro_FS(uint16_t gyro_fs)
{
  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused

  // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(STAT == 0xCB))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);     //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_mag_acc_FS(uint16_t mag_fs, uint16_t acc_fs)
{
  uint8_t bytes[4], STAT;
  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB

  // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA);

  //Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(STAT == 0xCA))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_integer_param(uint8_t param, uint32_t param_val)
{
  uint8_t bytes[4], STAT;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(STAT == param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_float_param(uint8_t param, float param_val)
{
  uint8_t bytes[4], STAT;
  float_to_bytes(param_val, &bytes[0]);

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(STAT == param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, WS_params.Sen_param[0][0]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, WS_params.Sen_param[0][1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, WS_params.Sen_param[0][2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, WS_params.Sen_param[0][3]);
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(STAT == param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  for (uint8_t i = 1; i < 35; i++)
  {
    param = (i + 1) | 0x80;
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, WS_params.Sen_param[i][0]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, WS_params.Sen_param[i][1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, WS_params.Sen_param[i][2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, WS_params.Sen_param[i][3]);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(STAT == param))
    {
      STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
}

void EM7180_get_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;

  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  delay(10);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);
  delay(10);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while (!(STAT == param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }

  // Parameter is the decimal value with the MSB set low (default) to indicate a paramter read processs
  WS_params.Sen_param[0][0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  WS_params.Sen_param[0][1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  WS_params.Sen_param[0][2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  WS_params.Sen_param[0][3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);

  for (uint8_t i = 1; i < 35; i++)
  {
    param = (i + 1);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    delay(10);

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(STAT == param))
    {
      STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    WS_params.Sen_param[i][0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    WS_params.Sen_param[i][1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    WS_params.Sen_param[i][2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    WS_params.Sen_param[i][3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);

  // Re-start algorithm
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00);
}

void WS_PassThroughMode()
{
  uint8_t stat = 0;

  // First put SENtral in standby mode
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x01);
  delay(5);

  // Place SENtral in pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
  while (!(stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    delay(5);
  }
}

void WS_Resume()
{
  uint8_t stat = 0;

  // Cancel pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
  while ((stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    delay(5);
  }

  // Re-start algorithm
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
  while ((stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    delay(5);
  }
}

void readSenParams()
{
  uint8_t data[140];
  uint8_t paramnum;
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
  delay(100);
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
  for (paramnum = 0; paramnum < 35; paramnum++)                          // 35 parameters
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      WS_params.Sen_param[paramnum][i] = data[(paramnum * 4 + i)];
    }
  }
}

void writeSenParams()
{
  uint8_t data[140];
  uint8_t paramnum;
  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      data[(paramnum * 4 + i)] = WS_params.Sen_param[paramnum][i];
    }
  }
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
  delay(100);
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float uint32_reg_to_float(uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 = (((uint32_t)buf[0]) +
            (((uint32_t)buf[1]) << 8) +
            (((uint32_t)buf[2]) << 16) +
            (((uint32_t)buf[3]) << 24));
  return u.f;
}

void float_to_bytes(float param_val, uint8_t *buf)
{
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i = 0; i < sizeof(float); i++)
  {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i = 0; i < sizeof(float); i++)
  {
    buf[i] = buf[(sizeof(float) - 1) - i];
  }
}

void readSENtralQuatData(float *destination)
{
  uint8_t rawData[16];                                   // x/y/z quaternion register data stored here
  readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]); // Read the sixteen raw data registers into data array
  destination[0] = uint32_reg_to_float(&rawData[0]);
  destination[1] = uint32_reg_to_float(&rawData[4]);
  destination[2] = uint32_reg_to_float(&rawData[8]);
  destination[3] = uint32_reg_to_float(&rawData[12]); // SENtral stores quats as qx, qy, qz, q0!
}

void readSENtralAccelData(int16_t *destination)
{
  uint8_t rawData[6];                                                  // x/y/z accel register data stored here
  readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);                // Read the six raw data registers into data array
  destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
}

void readSENtralGyroData(int16_t *destination)
{
  uint8_t rawData[6];                                                  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);                // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
}

void readSENtralMagData(int16_t *destination)
{
  uint8_t rawData[6];                                                  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);                // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
}

int16_t readSENtralBaroData()
{
  uint8_t rawData[2];                                        // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);    // Read the two raw data registers sequentially into data array
  return (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
}

int16_t readSENtralTempData()
{
  uint8_t rawData[2];                                        // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);    // Read the two raw data registers sequentially into data array
  return (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
}

void SENtralPassThroughMode()
{
  // First put SENtral in standby mode
  uint8_t c = readByte(EM7180_ADDRESS, EM7180_AlgorithmControl);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, c | 0x01);
  //  c = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
  //  Serial.print("c = "); Serial.println(c);
  // Verify standby status
  // if(readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & 0x01) {
  Serial.println("SENtral in standby mode");
  // Place SENtral in pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01);
  if (readByte(EM7180_ADDRESS, EM7180_PassThruStatus) & 0x01)
  {
    Serial.println("SENtral in pass-through mode");
  }
  else
  {
    Serial.println("ERROR! SENtral not in pass-through mode!");
  }
}

//===================================================================================================================
//====== I2C Communication Support Functions
//===================================================================================================================

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t data)
{
  Wire.beginTransmission(device_address); // Initialize the Tx buffer
  Wire.write(data_address1);              // Put slave register address in Tx buffer
  Wire.write(data_address2);              // Put slave register address in Tx buffer
  Wire.write(data);                       // Put data in Tx buffer
  Wire.endTransmission();                 // Send the Tx buffer
}

void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t *dest)
{
  if (count > 128)
  {
    count = 128;
    Serial.print("Page count cannot be more than 128 bytes!");
  }
  Wire.beginTransmission(device_address); // Initialize the Tx buffer
  Wire.write(data_address1);              // Put slave register address in Tx buffer
  Wire.write(data_address2);              // Put slave register address in Tx buffer
  for (uint8_t i = 0; i < count; i++)
  {
    Wire.write(dest[i]); // Put data in Tx buffer
  }
  Wire.endTransmission(); // Send the Tx buffer
}

uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
  uint8_t data;                                // `data` will store the register data
  Wire.beginTransmission(device_address);      // Initialize the Tx buffer
  Wire.write(data_address1);                   // Put slave register address in Tx buffer
  Wire.write(data_address2);                   // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);            // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(device_address, (size_t)1); // Read one byte from slave register address
  data = Wire.read();                          // Fill Rx buffer with result
  return data;                                 // Return data read from slave register
}

void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t *dest)
{
  Wire.beginTransmission(device_address); // Initialize the Tx buffer
  Wire.write(data_address1);              // Put slave register address in Tx buffer
  Wire.write(data_address2);              // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(device_address, (size_t)count); // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  } // Put read results in the Rx buffer
}

// simple function to scan for I2C devices on the bus
void I2Cscan()
{
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

// I2C read/write functions for the MPU9250 and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.write(data);                // Put data in Tx buffer
  Wire.endTransmission();          // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                         // `data` will store the register data
  Wire.beginTransmission(address);      // Initialize the Tx buffer
  Wire.write(subAddress);               // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);     // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t)1); // Read one byte from slave register address
  data = Wire.read();                   // Fill Rx buffer with result
  return data;                          // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP); // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, (size_t)count); // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  } // Put read results in the Rx buffer
}