// controlStrategy.cpp - the wall robot's control strategy. Built by the
// `control` environment. See Strategy.h for what a strategy must define.
//
// Every movement call is non-blocking: it is called again and again from
// loop() and returns true only once, when it has finished.

#include "Strategy.h"
#include "Hardware.h"
#include "Sensors.h"

const char* strategyName = "control";

int routine = 0;
int state = 0;

unsigned long startTime;

// No camera opening: routine stays 0, which is what the telemetry status
// block reports as the opening.
void selectOpeningRoutine() {
}

void handleMicroSwitches() {
}

void updateEndgameTiming() {
}

void runRoutines() {

switch (routine) {
  case 0:
    switch(state){
      case 0:
        move.stop();
        break;
    }
    break;
}

}
