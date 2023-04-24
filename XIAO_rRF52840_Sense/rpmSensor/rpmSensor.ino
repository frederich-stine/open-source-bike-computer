// Frederich Stine
// EN.525.743 Open Source Bike Computer
// rpmSensor.ino

#include <LSM6DS3.h>
#include <bluefruit.h>


/******************* MACRO DEFINITIONS ***********************/
/* These macros NEED to match with the Bluetooth co-processor code
*/
#define MANUFACTURER_ID 0xFFFF
#define BLE_MAJOR 0xDEAD
#define BLE_MINOR 0x0000

/******************* GLOBAL VARIABLES ***********************/
// IMU sensor connection
LSM6DS3 IMU(I2C_MODE, 0x6A);

// Global bluetooth packet parameters
uint8_t beaconUuid[16];
float* beaconUuidFloat = (float*) &beaconUuid;

// Bluetooth beacon datatype
BLEBeacon beacon(beaconUuid, BLE_MAJOR, BLE_MINOR, -54);

/******************* FUNCTION DEFINITIONS ***********************/
/* Setup function
*  This function initializes the IMU and the bluetooth functionality
*/
void setup() {
  Serial.begin(115200);

  if (IMU.begin() != 0) {
    Serial.println("IMU Error");
  }

  Bluefruit.begin();
  Bluefruit.setName("RPM_Sensor");
  beacon.setManufacturer(MANUFACTURER_ID);
}

/* Main loop
*  This code creates new advertising beacons with RPM data
*  and sends them over Bluetooth advertising for 100ms
*/
void loop() {
  float rpm;

  rpm = get_avg_rpm();

  beaconUuidFloat[0] = rpm;

  beacon.setUuid(beaconUuid);
  Bluefruit.Advertising.setBeacon(beacon);
  Bluefruit.Advertising.start();
  delay(100);
  Bluefruit.Advertising.stop();

  Serial.printf("MAX AVG RPM: %frpm\n", rpm);
}

/* Get average RPM function
*  This function reads in values from the IMU and averages them
*  to remove any irregular results. This function delays for a total
*  of 500ms.
*/
float get_avg_rpm() {
  float gx[10], gy[10], gz[10];

  for (int i=0; i<10; i++) {
    gx[i] = IMU.readFloatGyroX() / 6;
    gy[i] = IMU.readFloatGyroY() / 6;
    gz[i] = IMU.readFloatGyroZ() / 6;
    delay(50);
  }

  // There is some manual calibration in here specific to my sensor
  // If values are off this process may need to be redone
  float avgGx, avgGy, avgGz;
  for (int i=0; i<10; i++) {
    avgGx += gx[i];
    avgGy += (gy[i]+.5);
    avgGz += gz[i];
  }

  avgGx = abs(avgGx) / 10;
  avgGy = abs(avgGy) / 10;
  avgGz = abs(avgGz) / 10;

  if (avgGx >= avgGy && avgGx >= avgGz) {
    return avgGx;
  }
  else if (avgGy >= avgGx && avgGy >= avgGz) {
    return avgGy;
  }
  else if (avgGz >= avgGx && avgGz >= avgGy) {
    return avgGz;
  }
  return avgGx;
}