#include "Free_Fonts.h"

#include <rpcBLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// Arduino.h header file included here redefines min and max functions and breaks gcc compilation
// Make sure this include is the LAST include to stop previous libraries from failing to compile 
#include <Seeed_Arduino_FreeRTOS.h>

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

// Reference: https://www.cateye.com/data/resources/Tire_size_chart_ENG_151106.pdf
#define TIRE_CIRCUMFERENCE_CM 222

#define MANUFACTURER_ID 0xFFFF
#define BLE_MAJOR_RPM 0xDEAD
#define BLE_MINOR_RPM 0x0000
#define BLE_MAJOR_SPEED 0xDEAD
#define BLE_MINOR_SPEED 0x0001

#define ERROR_LED_PIN            LED_BUILTIN   //Led Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE  LOW // the state that makes the led light up on your board, either low or high

// Select the Serial port the project should use and communicate over
// Sombe boards use SerialUSB, some use Serial
#define Terminal          Serial

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;
TaskHandle_t Handle_sensorTask;
TaskHandle_t Handle_buttonsTask;
TaskHandle_t Handle_monitorTask;

float rpm = 0;
float speed = 0;

bool rideStarted = 0;
bool ridePaused = 0;

int scanTime = 1; //In seconds
BLEScan* pBLEScan;

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account procesor speed
//**************************************************************************
void myDelayUs(int us) {
    vTaskDelay(us / portTICK_PERIOD_US);
}

void myDelayMsUntil(TickType_t* previousWakeTime, int ms) {
    vTaskDelayUntil(previousWakeTime, (ms * 1000) / portTICK_PERIOD_US);
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (!advertisedDevice.haveManufacturerData()) {
        return;
      }
      if (advertisedDevice.getManufacturerDataLength() != 23) {
        return;
      }
      uint8_t* manufacturerData = advertisedDevice.getManufacturerData();

      uint16_t major = (manufacturerData[18] << 8) | (manufacturerData[19]);
      uint16_t minor = (manufacturerData[20] << 8) | (manufacturerData[21]);

      if (major == BLE_MAJOR_RPM && minor == BLE_MINOR_RPM) {
        float* value = (float*) &manufacturerData[2];
        rpm = *value;
        Terminal.printf("Received RPM: %f\r\n", *value);
      }
      else if (major == BLE_MAJOR_SPEED && minor == BLE_MINOR_SPEED) {
        float* value = (float*) &manufacturerData[2];
        // Convert RPM to MPH
        speed = ((*value*TIRE_CIRCUMFERENCE_CM)*.0022369);
        Terminal.printf("Received Speed: %f\r\n", speed);
      }
    }
};

static void threadA(void* pvParameters) {

  TFT_eSPI tft = TFT_eSPI();
  tft.begin();

  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // Update screen
  while (true) {
    int xpos =  0;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM); // Centre text on x,y position

    xpos = tft.width() / 2; // Half the screen width
    int ypos = 5;

    char message[50];

    tft.setTextPadding(tft.width());
    tft.setFreeFont(FM12);
    snprintf(message, 50, "Speed: %.2fMPH", speed);
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "Distance: %.2fM", float(0));
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "Cadence: %.2fRPM", rpm);
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "Temperature: %.2fF", float(0));
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "Elevation: %.2fFt", float(0));
    tft.drawString(message, xpos, ypos, GFXFF);    
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "AVG Speed: %.2fMPH", float(0));
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "AVG Cadence: %.2fRPM", float(0));
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "Elev Gain: %.2fRPM", float(0));
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    snprintf(message, 50, "Ride Time: %.2fH", 0);
    tft.drawString(message, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);

    delay(1000);
  }  

  vTaskDelete(NULL);
}

static void threadB(void* pvParameters) {
  Terminal.println("In thread A");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  pBLEScan->setInterval(32);
  pBLEScan->setWindow(1);  // less or equal setInterval value

  while (true) {
    //BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    pBLEScan->start(scanTime, false);
    Terminal.printf("Done scanning\n");
  }

  vTaskDelete(NULL);
}

static void buttonThread(void* pvPatameters) {

  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);
  pinMode(BUTTON_3, INPUT);

  int bState[3];
  int bStatePrev[3];

  for(;;) {
    bState[0] = digitalRead(BUTTON_1);
    bState[1] = digitalRead(BUTTON_2);
    bState[2] = digitalRead(BUTTON_3);

    if (bState[0] == HIGH && bStatePrev[0] == LOW) {
      rideStarted = 1;
    }
    if (bState[1] == HIGH && bStatePrev[1] == LOW) {
      if rideStarted {
        ridePaused = !ridePaused;
      }
    }
    if (bState[2] == HIGH && bStatePrev[2] == LOW) {
      rideStarted = 0;
    }

    bStatePrev[0] = bState[0];
    bStatePrev[1] = bState[1];
    bStatePrev[2] = bState[2];
    
    delay(50);
  }

  vTaskDelete(NULL);
}

static void sensorThread(void* pvPatameters) {

  vTaskDelete(NULL);
}

void taskMonitor(void* pvParameters) {
    int x;
    int measurement;

    Terminal.println("Task Monitor: Started");

    // run this task afew times before exiting forever
    for (;;) {

        Terminal.println("");
        Terminal.println("******************************");
        Terminal.println("[Stacks Free Bytes Remaining] ");

        measurement = uxTaskGetStackHighWaterMark(Handle_aTask);
        Terminal.print("Thread A: ");
        Terminal.println(measurement);

        measurement = uxTaskGetStackHighWaterMark(Handle_bTask);
        Terminal.print("Thread B: ");
        Terminal.println(measurement);

        measurement = uxTaskGetStackHighWaterMark(Handle_monitorTask);
        Terminal.print("Monitor Stack: ");
        Terminal.println(measurement);

        measurement = xPortGetFreeHeapSize();
        Terminal.print("Heap remaining: ");
        Terminal.println(measurement);

        Terminal.println("******************************");

        delay(2000); // print every 2 seconds
    }

    // delete ourselves.
    // Have to caTerminal this or the system crashes when you reach the end bracket and then get scheduled.
    Terminal.println("Task Monitor: Deleting");
    vTaskDelete(NULL);

}

void setup() {

    Terminal.begin(115200);

    vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this

    // Set the led the rtos will blink when we have a fatal rtos error
    // RTOS also Needs to know if high/low is the state that turns on the led.
    // Error Blink Codes:
    //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
    //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object.
    //               Probably ran out of heap.
    //    1 blink  - Stack overflow, Task needs more bytes defined for its stack!
    //               Use the taskMonitor thread to help gauge how much more you need
    vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

    // Create the threads that will be managed by the rtos
    // Sets the stack size and priority of each task
    // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
    xTaskCreate(threadA, "LCD Task", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_aTask);
    xTaskCreate(threadB, "BLE Task", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_bTask);
    xTaskCreate(buttonThread, "Buttons", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_buttonsTask);
    //xTaskCreate(sensorThread, "Sensors", 4096, NULL, tskIDLE_PRIORITY + 2, &Handle_sensorTask);

    xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);

    // Start the RTOS, this function will never return and will schedule the tasks.
    vTaskStartScheduler();

}


void loop() {

}

