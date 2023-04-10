#include "BLEDevice.h"

int dataCount = 0;
BLEAdvertData foundDevice;

// Reference: https://www.cateye.com/data/resources/Tire_size_chart_ENG_151106.pdf
#define TIRE_CIRCUMFERENCE_CM 222

#define MANUFACTURER_ID 0xFFFF
#define BLE_MAJOR_RPM   0xDEAD
#define BLE_MINOR_RPM   0x0000
#define BLE_MAJOR_SPEED 0xDEAD
#define BLE_MINOR_SPEED 0x0001

void scanFunction(T_LE_CB_DATA* p_data) {
    foundDevice.parseScanInfo(p_data);
    if (!foundDevice.hasManufacturer()) {
      return;
    }
    if (foundDevice.getManufacturerDataLength() != 23) {
      return;
    }
    uint8_t* manufacturerData = foundDevice.getManufacturerData();

    uint16_t major = (manufacturerData[18] << 8) | (manufacturerData[19]);
    uint16_t minor = (manufacturerData[20] << 8) | (manufacturerData[21]);

    if (major == BLE_MAJOR_RPM && minor == BLE_MINOR_RPM) {
      float* value = (float*) &manufacturerData[2];
      Serial.print("R:");
      Serial.print(*value);
      Serial.print("\n");
    }
    else if (major == BLE_MAJOR_SPEED && minor == BLE_MINOR_SPEED) {
      float* value = (float*) &manufacturerData[2];
      float speed = ((*value*TIRE_CIRCUMFERENCE_CM)*.0003728);
      Serial.print("S:");
      Serial.print(speed);
      Serial.print("\n");
    }
}

void setup() {
    Serial.begin(115200);
    BLE.init();
    BLE.configScan()->setScanMode(GAP_SCAN_MODE_PASSIVE);    // Active mode requests for scan response packets
    BLE.configScan()->setScanInterval(300);   // Start a scan every 500ms
    BLE.configScan()->setScanWindow(299);     // Each scan lasts for 250ms
    BLE.configScan()->updateScanParams();
    // Provide a callback function to process scan data.
    // If no function is provided, default BLEScan::printScanInfo is used
    BLE.setScanCallback(scanFunction);
    BLE.beginCentral(0);
}

void loop() {
  BLE.configScan()->startScan(1200);    // Repeat scans for 5 seconds
}
