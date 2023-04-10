#include <LSM6DS3.h>
#include <bluefruit.h>

#define MANUFACTURER_ID 0xFFFF
#define BLE_MAJOR 0xDEAD
#define BLE_MINOR 0x0001

LSM6DS3 IMU(I2C_MODE, 0x6A);

uint8_t beaconUuid[16];
float* beaconUuidFloat = (float*) &beaconUuid;

BLEBeacon beacon(beaconUuid, BLE_MAJOR, BLE_MINOR, -54);

void setup() {
  Serial.begin(115200);

  if (IMU.begin() != 0) {
    Serial.println("IMU Error");
  }

  Bluefruit.begin();
  Bluefruit.setName("RPM_Sensor");
  beacon.setManufacturer(MANUFACTURER_ID);
}

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

float get_avg_rpm() {
  float gx[10], gy[10], gz[10];

  for (int i=0; i<10; i++) {
    gx[i] = IMU.readFloatGyroX() / 6;
    gy[i] = IMU.readFloatGyroY() / 6;
    gz[i] = IMU.readFloatGyroZ() / 6;
    delay(50);
  }

  float avgGx, avgGy, avgGz;
  for (int i=0; i<10; i++) {
    avgGx += gx[i];
    avgGy += gy[i];
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