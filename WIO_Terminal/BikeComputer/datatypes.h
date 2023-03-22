#include <stdint.h>

#ifndef __DATATYPES_H__
#define __DATATYPES_H__

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
} displayData;

typedef struct {
  bool started;
  bool paused;
} controlData;








#endif
