#include "Free_Fonts.h"
#include "datatypes.h"

#include <TFT_eSPI.h>
#include <SPI.h>
#include <AHT20.h>
#include <TinyGPS++.h>
#include <Seeed_Arduino_FS.h>
#include <Seeed_Arduino_FreeRTOS.h>

//**************************************************************************
// Type Defines and Constants
//**************************************************************************
#define ERROR_LED_PIN            LED_BUILTIN
#define ERROR_LED_LIGHTUP_STATE  LOW

#define SD SD

#ifdef _SAMD21_
#define SDCARD_SS_PIN 1
#define SDCARD_SPI SPI
#endif 

#define gpsSerial Serial1

// FreeRTOS task handles
TaskHandle_t Handle_displayTask;
TaskHandle_t Handle_sensorTask;
TaskHandle_t Handle_pollTask;
TaskHandle_t Handle_flashTask;

TimerHandle_t secondTimer;

// Global data structures
displayData dispData;
controlData contData;
timeData tData;
timeData gpsTime;
gpsData gpsInfo;
TinyGPSPlus gps;

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

        snprintf(message, 50, "Distance: %.2fM", dispData.distance);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "Cadence: %.2fRPM", dispData.cadence);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "Temperature: %.2fF", dispData.temperature);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "Elevation: %.2fFt", dispData.elevation);
        tft.drawString(message, xpos, ypos, GFXFF);    
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "AVG Speed: %.2fMPH", dispData.avgSpeed);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "AVG Cadence: %.2fRPM", dispData.avgCadence);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "Elev Gain: %.2fFt", dispData.elevationGain);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        snprintf(message, 50, "Ride Time: %02d:%02d:%02d", tData.hours, tData.minutes, tData.seconds);
        tft.drawString(message, xpos, ypos, GFXFF);
        ypos += tft.fontHeight(GFXFF);

        delay(1000);
      }
    }
  }

  vTaskDelete(NULL);
}

static void pollThread(void* pvParameters) {

  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);
  pinMode(BUTTON_3, INPUT);

  int bState[3];
  int bStatePrev[3];

  int bleBufIndex = 0;
  char bleBuffer[100];
  gpsSerial.begin(9600);
  RTL8720D.begin(115200);

  while(!RTL8720D);

  for (;;) {
    bState[0] = digitalRead(BUTTON_1);
    bState[1] = digitalRead(BUTTON_2);
    bState[2] = digitalRead(BUTTON_3);

    if (bState[0] == HIGH && bStatePrev[0] == LOW) {
      contData.started = true;
    }
    if (bState[1] == HIGH && bStatePrev[1] == LOW) {
      if (contData.started == true) {
        if (contData.paused == false) {
          contData.paused = true;
        }
        else {
          contData.paused = false;
        }
      }
    }
    if (bState[2] == HIGH && bStatePrev[2] == LOW) {
      contData.started = false;
    }

    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    while (RTL8720D.available()) {
      bleBuffer[bleBufIndex] = (char) RTL8720D.read();
      bleBufIndex++;

      if (bleBuffer[bleBufIndex-1] == '\n') {
        bleBuffer[bleBufIndex-1] = 0;
        if ((bleBuffer[0] == 'S') && (bleBuffer[1] == ':')) {
          dispData.speed = atof((const char*)&bleBuffer[2]);
          //Serial.printf("Speed: %f\n", dispData.speed);
        }
        if ((bleBuffer[0] == 'R') && (bleBuffer[1] == ':')) {
          dispData.cadence = atof((const char*)&bleBuffer[2]);
          //Serial.printf("Cadence: %f\n", dispData.cadence);
        }
        bleBufIndex = 0;
      }
    }

    bStatePrev[0] = bState[0];
    bStatePrev[1] = bState[1];
    bStatePrev[2] = bState[2];
    
    delay(50);
  }

  vTaskDelete(NULL);
}

static void sensorThread(void* pvParameters) {
  AHT20 AHT;

  AHT.begin();

  for (;;) {
    float tempC;
    AHT.getTemperature(&tempC);

    dispData.temperature = (tempC * 1.8) + 32;

    delay(1000);
  }

  vTaskDelete(NULL);
}

File rideFile;
static void flashThread(void* pvParameters) {
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    while (!SD.begin(SDCARD_SS_PIN,SDCARD_SPI,4000000UL)) {
        Serial.println("Card Mount Failed");
        vTaskDelete(NULL);
    }

    char text[200];
    // Generate a filename for the ride

    bool firstStart = true;
    while (true) {
      if (contData.started == true) {
        if (firstStart == true) {
          int i=0;
          while (true) {
            snprintf(text, 200, "ride_%d.txt", i);
            if(!SD.exists(text)) {
              Serial.printf("Filename: %s\n", text);
              rideFile = SD.open(text, "w");
              firstStart = false;
              break;
            }
            i++;
          }
        }
        else {
          if (contData.paused == true) {
            delay(1000);
            continue;
          }
          snprintf(text, 200, "D,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
            dispData.rideTime, dispData.speed,
            dispData.cadence, dispData.avgSpeed,
            dispData.avgCadence, dispData.temperature,
            dispData.elevation, dispData.elevationGain,
            dispData.distance);
          Serial.print(text);
          rideFile.write(text);
          snprintf(text, 200, "GPS,%.8f,%.8f,%.8f\n",
            gpsInfo.latitude, gpsInfo.longitude, gpsInfo.elevation);
          Serial.print(text);
          rideFile.write(text);
          snprintf(text, 200, "GPST,%d,%d,%d\n", 
            gpsTime.hours, gpsTime.minutes, gpsTime.seconds);
          Serial.print(text);
          rideFile.write(text);
        }
      }
      else {
        if (rideFile != NULL) {
          rideFile.close();
          firstStart = true;
        }
      }
      delay(1000);
    }

  vTaskDelete(NULL);
}

float oldElevation = 0;
bool firstElevation = true;
void secondTimerCallback ( TimerHandle_t xTimer ) {
  if (contData.started == true) {
    if (contData.paused == false) {
      dispData.rideTime += 1;

      tData.seconds = (int)dispData.rideTime % 60;
      tData.minutes = ((int)dispData.rideTime / 60) % 60;
      tData.hours = (int)dispData.rideTime / 3600;
      Serial.print("U\n");

      // Calculate average speed and cadence
      float divRatio = ((float)dispData.rideTime-1)/(float)dispData.rideTime;
      dispData.avgSpeed = (dispData.avgSpeed * divRatio) + \
        dispData.speed / float(dispData.rideTime);
      dispData.avgCadence = (dispData.avgCadence * divRatio) + \
        dispData.cadence / float(dispData.rideTime);
      
      dispData.distance = dispData.distance + (dispData.speed / 3600.0);
    }
  }
  else {
    dispData.rideTime = 0;
  }

  if (gps.location.isValid()) {
    gpsInfo.latitude = (float)gps.location.lat();
    gpsInfo.longitude = (float)gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    gpsInfo.elevation = (float)gps.altitude.feet();
    dispData.elevation = (float)gps.altitude.feet();

    if(firstElevation == true) {
      firstElevation = false;
    }
    else if (dispData.elevation > (oldElevation - 2.0)) {         
      dispData.elevationGain += dispData.elevation - oldElevation;
    }
    oldElevation = dispData.elevation;
  }
  if (gps.time.isValid()) {
    gpsTime.hours = (int)gps.time.hour();
    gpsTime.minutes = (int)gps.time.minute();
    gpsTime.seconds = (int)gps.time.second();
  }
}

void setup() {
    Serial.begin(115200);
    // Prevent USB driver crash
    vNopDelayMS(1000);
    //while (!Serial);

    vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

    pinMode(RTL8720D_CHIP_PU, OUTPUT);
    digitalWrite(RTL8720D_CHIP_PU, LOW);
    delay(100);
    digitalWrite(RTL8720D_CHIP_PU, HIGH);
    delay(100);    

    contData.paused = false;
    contData.started = false;
    dispData.speed = 0;
    dispData.cadence = 0;
    dispData.rideTime = 0;

    secondTimer = xTimerCreate("sTimer", 1000, pdTRUE, 0, secondTimerCallback);
    xTimerStart(secondTimer, 1000);

    xTaskCreate(threadDisplay, "LCD", 1024, NULL, tskIDLE_PRIORITY + 3, &Handle_displayTask);
    xTaskCreate(pollThread, "Poll", 512, NULL, tskIDLE_PRIORITY + 6, &Handle_pollTask);
    xTaskCreate(sensorThread, "Sensors", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_sensorTask);
    xTaskCreate(flashThread, "Flash", 4096, NULL, tskIDLE_PRIORITY + 3, &Handle_flashTask);

    vTaskStartScheduler();
}

void loop() {
}

