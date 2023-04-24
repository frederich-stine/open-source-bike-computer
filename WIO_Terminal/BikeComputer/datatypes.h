// Frederich Stine
// EN.525.743 Open Source Bike Computer
// datatypes.h

#include <stdint.h>

#ifndef __DATATYPES_H__
#define __DATATYPES_H__

/******************* STRUCT DEFINITIONS ***********************/
// displayData - Data to be displayed
typedef struct {
  float speed;
  float cadence;
  float avgSpeed;
  float avgCadence;
  float temperature;
  float elevation;
  float elevationGain;
  float rideTime;
  float distance;
  float maxSpeed;
} displayData;

// controlData - Datatype for system state
typedef struct {
  bool started;
  bool paused;
} controlData;

// timeData - Datatype for time
typedef struct {
  int hours;
  int minutes;
  int seconds;
  bool valid;
} timeData;

// gpsData - Datatype for GPS position
typedef struct {
  float latitude;
  float longitude;
  float elevation;
  bool valid;
} gpsData;

#endif
