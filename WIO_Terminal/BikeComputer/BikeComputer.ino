#include "Free_Fonts.h"
#include "datatypes.h"

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
// FreeRTOS task handles
TaskHandle_t Handle_displayTask;
TaskHandle_t Handle_bleTask;
TaskHandle_t Handle_sensorTask;
TaskHandle_t Handle_buttonsTask;
TaskHandle_t Handle_monitorTask;

// Global data structures
displayData dispData;
controlData contData;

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
        dispData.cadence = *value;
        Terminal.printf("Received RPM: %f\r\n", dispData.cadence);
      }
      else if (major == BLE_MAJOR_SPEED && minor == BLE_MINOR_SPEED) {
        float* value = (float*) &manufacturerData[2];
        // Convert RPM to MPH
        dispData.speed = ((*value*TIRE_CIRCUMFERENCE_CM)*.0003728);
        Terminal.printf("Received Speed: %f\r\n", dispData.speed);
      }
    }
};

static void threadDisplay(void* pvParameters) {
  TFT_eSPI tft = TFT_eSPI();
  tft.begin();

  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  int xpos, ypos;
  tft.setFreeFont(FM12);
  tft.setTextDatum(TC_DATUM); // Centre text on x,y position

  // Update screen
  while (true) {
    if (contData.started == false) {
      tft.fillScreen(TFT_BLACK);
      xpos = tft.width() / 2;
      ypos = (tft.height() / 2) - (tft.fontHeight(GFXFF) / 2);
      tft.drawString("Computer Stopped", xpos, ypos, GFXFF);

      while (contData.started == false) {
        delay(100);
      }
    }     

    if (contData.started == true) {
      tft.fillScreen(TFT_BLACK);
      xpos = tft.width() / 2;
      ypos = (tft.height() / 2) - (tft.fontHeight(GFXFF) / 2);
      tft.drawString("Computer Started", xpos, ypos, GFXFF);
      
      delay(2000);
      tft.fillScreen(TFT_BLACK);

      while (contData.started == true) {
        xpos = tft.width() / 2; // Half the screen width
        ypos = 5;

        char message[50];

        tft.setTextPadding(tft.width());
        tft.setFreeFont(FM12);
        snprintf(message, 50, "Speed: %.2fMPH", dispData.speed);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "Distance: %.2fM", float(0));
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "Cadence: %.2fRPM", dispData.cadence);
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

        snprintf(message, 50, "Ride Time: %.2fH", float(0));
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        delay(1000);
      }
    }
  }

  vTaskDelete(NULL);
}

static void threadBLE(void* pvParameters) {
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

  for (;;) {
    bState[0] = digitalRead(BUTTON_1);
    bState[1] = digitalRead(BUTTON_2);
    bState[2] = digitalRead(BUTTON_3);

    if (bState[0] == HIGH && bStatePrev[0] == LOW) {
      Serial.println("Button 0 Pressed");
      contData.started = true;
    }
    if (bState[1] == HIGH && bStatePrev[1] == LOW) {
      Serial.println("Button 1 Pressed");
      if (contData.started == true) {
        contData.paused != contData.paused;
      }
    }
    if (bState[2] == HIGH && bStatePrev[2] == LOW) {
      Serial.println("Button 2 Pressed");
      contData.started = false;
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

        measurement = uxTaskGetStackHighWaterMark(Handle_displayTask);
        Terminal.print("Thread Disp: ");
        Terminal.println(measurement);

        measurement = uxTaskGetStackHighWaterMark(Handle_bleTask);
        Terminal.print("Thread BLE: ");
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

    contData.paused = false;
    contData.started = false;
    dispData.speed = 0;
    dispData.cadence = 0;

    // Create the threads that will be managed by the rtos
    // Sets the stack size and priority of each task
    // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
    xTaskCreate(threadDisplay, "LCD Task", 1024, NULL, tskIDLE_PRIORITY + 3, &Handle_displayTask);
    xTaskCreate(threadBLE, "BLE Task", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_bleTask);
    xTaskCreate(buttonThread, "Buttons", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_buttonsTask);
    //xTaskCreate(sensorThread, "Sensors", 4096, NULL, tskIDLE_PRIORITY + 2, &Handle_sensorTask);

    xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);

    // Start the RTOS, this function will never return and will schedule the tasks.
    vTaskStartScheduler();

}


void loop() {

}

