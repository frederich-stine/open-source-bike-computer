// Frederich Stine
// EN.525.743 Open Source Bike Computer
// BLEScan.ino

#include "BLEDevice.h"

/******************* GLOBAL VARIABLES ***********************/
// Variable for CB function
BLEAdvertData foundDevice;

/******************* MACRO DEFINITIONS ***********************/
/* These variables need to change on different set-ups of the open source bike computer
*  The TIRE_CIRCUMFERENCE_CM is related to the wheel/tire size of the bike and the
*  BLE flags should be changed on the sensors and matched here. It is good to select
*  random values as these can conflict with nearby sensors.
*/
// Reference: https://www.cateye.com/data/resources/Tire_size_chart_ENG_151106.pdf
#define TIRE_CIRCUMFERENCE_CM 222

#define MANUFACTURER_ID 0xFFFF
#define BLE_MAJOR_RPM   0xDEAD
#define BLE_MINOR_RPM   0x0000
#define BLE_MAJOR_SPEED 0xDEAD
#define BLE_MINOR_SPEED 0x0001


/******************* FUNCTION DEFINITIONS ***********************/
/* BLE Callback function
*  This function processes advertisement packets found and checks for whether
*  they are advertisements from the bike computer sensors.
*/
void scanFunction(T_LE_CB_DATA* p_data) {
    foundDevice.parseScanInfo(p_data);
    if (!foundDevice.hasManufacturer()) {
      return;
    }
    // This is the length of our data packet
    if (foundDevice.getManufacturerDataLength() != 23) {
      return;
    }
    uint8_t* manufacturerData = foundDevice.getManufacturerData();

    uint16_t major = (manufacturerData[18] << 8) | (manufacturerData[19]);
    uint16_t minor = (manufacturerData[20] << 8) | (manufacturerData[21]);

    // Check for each sensor and if so, send rpc message back to AP
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

/* Setup function
*  This function sets up the bluetooth module for scanning
*/
void setup() {
    Serial.begin(115200);
    BLE.init();
    BLE.configScan()->setScanMode(GAP_SCAN_MODE_PASSIVE);    // Active mode requests for scan response packets
    BLE.configScan()->setScanInterval(300);   // Start a scan every 300ms
    BLE.configScan()->setScanWindow(299);     // Each scan lasts for 299ms
    BLE.configScan()->updateScanParams();
    // Provide a callback function to process scan data.
    // If no function is provided, default BLEScan::printScanInfo is used
    BLE.setScanCallback(scanFunction);
    BLE.beginCentral(0);
}

/* Main loop
*  This function simply scans infinitely
*/
void loop() {
  BLE.configScan()->startScan(1200);    // Repeat scans for 5 seconds
}
