// Routines.cpp - the strategy state machine.
//
// Several behaviours here look odd but the robot is tuned around them; each is
// marked KNOWN where it appears. Changing one needs a field test.

#include "Routines.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Motion.h"
#include "Heading.h"

int routine = 4;
int state = 0;
bool first = true;
rlane lane = MIDDLE;

int beta = 8; //degree error
int alpha = 0;

int connections;

int startTime;

int pesos[NUM_FRANJAS] = {0};

bool lastRoutine = false;
bool midRoutine = false;
bool midRoutineDone = false;

// ---------------------------------------------------------------------------

int classifyLane(float x, float y, bool right) {
    // Define the two diagonal boundaries
    float mA, mB, bA, bB;

    if (right){
      mA = (24.0 - 205.0) / (128.0 - 45.0);  // ≈ -2.346
      bA = 205 - mA * 45;            // ≈ 310.57

      mB = (180.0 - 24.0) / (237.0 - 188.0); // ≈ 2.636
      bB = 24 - mB * 188;
    } else {
      mA = (61.0 - 182.0) / (144.0 - 6.0);  // ≈ -2.346
      bA = 182 - mA * 6;            // ≈ 310.57

      mB = (186.0 - 8.0) / (279.0 - 161.0); // ≈ 2.636
      bB = 8 - mB * 161;            // ≈ -500.2
    }


    // Evaluate position relative to lines
    float yA = mA * x + bA;
    float yB = mB * x + bB;

    Serial.print("lane boundary yA ");
    Serial.println(yA);

    if (y < yA) {
        return 0; // Lane 1
    } else if (y >= yA && y < yB) {
        return 2; // Lane 2
    } else {
        return 1; // Lane 3
    }
}

// ---------------------------------------------------------------------------

void selectOpeningRoutine() {
int center_y = 32;
int center_x = 200;


int noballs = 0;

for (int i = 0; i < 120; i++){
pixy.ccc.getBlocks();
if (pixy.ccc.numBlocks > 0) {
  for (int j = 0; j < pixy.ccc.numBlocks; j++) {
    if (pixy.ccc.blocks[j].m_signature == purpleSignature &&
        pixy.ccc.blocks[j].m_age > 10
        && pixy.ccc.blocks[j].m_x > 105 && pixy.ccc.blocks[j].m_x < 300 &&
        pixy.ccc.blocks[j].m_y > 20 && pixy.ccc.blocks[j].m_y < 90
        ) {

      int ix = pixy.ccc.blocks[j].m_x;
      int iy = pixy.ccc.blocks[j].m_y;
      // pick the opening routine from the quadrant
      if (ix < center_x && iy < center_y){
        routine = 0;
        pixy.ccc.blocks[j].print();
        Serial.print("upper left corner, case 1");
        return;
      } else if (ix > center_x && iy < center_y){
        routine = 1;
        pixy.ccc.blocks[j].print();
        Serial.print("upper right corner, case 3");
        return;
      } else if (ix < center_x && iy > center_y){
        routine = 2;
        pixy.ccc.blocks[j].print();
        Serial.print("lower left corner, case 0");
        return;
      } else if (ix > center_x && iy > center_y){
        routine = 3;
        pixy.ccc.blocks[j].print();
        Serial.print("lower Right corner, case 2");
        return;
      }
    }
  }
  } else if (noballs < 10){
    noballs++;
  } else {return;}
  delay(14);
}


pixy.setLamp(0, 0);
}

// ---------------------------------------------------------------------------

void handleMicroSwitches() {
  // KNOWN: microSwitchTime and currentTime are `int`, 16 bits on AVR, so both
  // truncate millis() and wrap every 32.767 seconds; the 350 ms debounce
  // misbehaves around each wrap. Changing them to unsigned long changes switch
  // timing, so it needs a field test.
  static int microSwitchTime = 0;
  int currentTime = millis();
    bool currentBackSwitchState = digitalRead(backSwitchPin);
    bool currentSideSwitchState = digitalRead(sideSwitchPin);


    if (currentTime - microSwitchTime > 350){
      // Handle switch press
      if (currentBackSwitchState == LOW && lastBackSwitchState == HIGH) {
        microSwitchTime = millis();
        // Button pressed
        onSwitchPress();
        if (!(routine == 6 && state == 2)){
          state++;
        }
        Serial.println("Heading zeroed by microswitch");
      }
      if (currentSideSwitchState == LOW && lastSideSwitchState == HIGH && routine!= 4  && !(routine == 7 && state == 8)  ) {
        microSwitchTime = millis();
        // Button pressed
        state++;
      }
      lastBackSwitchState = currentBackSwitchState;
      lastSideSwitchState = currentSideSwitchState;
    }
}

// ---------------------------------------------------------------------------

void updateEndgameTiming() {
    // Last Routine Code ----------------------------------------------
  // if (lastRoutine == false &&  (millis() > 105000 + startTime) ){ //&& (routine != 7 && routine != 5)
  //   lastRoutine = true;
  //   lane = OUTER;
  //   lenght -= 30;
  // }
  // if (midRoutine == false && midRoutineDone == false && (millis() > 45000 + startTime) ){ //&& (routine != 7 && routine != 5)
  //   midRoutine = true;
  //   if (routine != 4 && lane != OUTER){
  //     lane = OUTER;
  //   } else {
  //     lane = MIDDLE;
  //   }
  //   lenght -= 30;
  // } else if(midRoutine == true && (millis() > 61000 + startTime) && (millis() < 100000 + startTime)){
  //   midRoutine = false;
  //   midRoutineDone = true;
  //   lenght += 30;
  // }
}

// ---------------------------------------------------------------------------

void runRoutines() {

switch (routine) {//---------------------------------------------------------------------------------------ROUTINES---------------------------------------------------------//
  case 0:
    switch(state){
      case 0:
        if(move.backward(mm(100))) state++;
        break;
      case 1:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(mm(505))) state++;
        break;
      case 3:
        state++;
        break;
      case 4:
        myservo.write(openGate);
        if(move.forward(mm(350))) state++;
        break;
      case 5:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 6:
        routine = 6; state = 0;
        if (robotSide == RIGHT){
          lane = OUTER;
        } else {lane = INNER;}
      }
    break;
  case 1:
    switch(state){
      case 0:
        if (move.right(mm(200))) state++;
        break;
      case 1:
        if(move.backward(mm(100))) state++;
        break;
      case 2:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(mm(530))) state++;
        break;
      case 4:
        state++;
        break;
      case 5:
        myservo.write(openGate);
        if(move.forward(mm(350))) state++;
        break;
      case 6:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 7:
        routine = 6; state = 0;
        lane = MIDDLE;
      }
    break;
  case 2:
    switch(state){
      case 0:
        if(move.backward(mm(100))) state++;
        break;
      case 1:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(mm(550))) state++;
        break;
      case 3:
        state++;
        break;
      case 4:
        myservo.write(closedGate);
        if(move.forward(mm(250))) state++;
        break;
      case 5:
        routine = 6; state = 0;
        if (robotSide == RIGHT){
          lane = OUTER;
        } else {lane = INNER;}
    }
    break;
  case 3:
    switch(state){
      case 0:
        if(move.right(mm(200))) state++;
        break;
      case 1:
        if(move.backward(mm(100))) state++;
        break;
      case 2:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(mm(550))) state++;
        break;
      case 4:
        state++;
        break;
      case 5:
        myservo.write(closedGate);
        if(move.forward(mm(250))) state++;
        break;
      case 6:
        routine = 6; state = 0;
        lane = MIDDLE;
    }
    break;





  ////-------------------------------LOOOP-----------------------------------------------------------////
  case 4:
    switch(state){
      case -1:                //OUTER LANE
        disableDrivers();
        if (robotSide == RIGHT){
          if(move.rotate(mm(166), false)) state--;
        }else{
          if(move.rotate(mm(166), true)) state--;
        }
        break;
      case -2:
        // KNOWN: outer() converts with mm() internally, so this is mm(mm(20)),
        // about 453 counts rather than 95. The robot is tuned around it.
        if(outer(mm(20))) state--;
        break;
      case -3:
        // KNOWN: forwardp returns 2 at 14/22 of the distance and the bare `if`
        // treats that as done, so this state ends at ~255 mm of the 400 with
        // the motors still running; state -4 releases them.
        if (robotSide == RIGHT){
          if(move.forwardp(mm(400), true)) state--;
        }else{
          if(move.forwardp(mm(400), false)) state--;
        }
        break;
      case -4:
        enableDrivers();
        if(move.stopForMillis(mili)) state--;
        break;
      case -5:
        disableDrivers();
        if (robotSide == RIGHT){
          if(move.rotate(mm(166), true)) state--;
        }else{
          if(move.rotate(mm(166), false)) state--;
        }
        break;
      case -6:
        if(outer(30)) state = 0;
        break;
      case 0:
        enableDrivers();            //MIDDLE LANE
        if(move.backward(mm(280))) state = 1;
        break;
      case 1:
        if(move.stopForMillis(mili)) state = 2;
        break;
      case 2:
          if (lane == MIDDLE || lane == INNER){
            int test = move.forwardRegulated(mm(lenght + 50));
            switch(test){
              case 1:
                state++;
                break;
              case 2:
                enableSlowDrivers();
                break;
           }
          } else if(lane == OUTER){
              if (robotSide == RIGHT){
                int test = move.forwardp(mm(lenght + 50), true);
                switch(test){
                  case 1:
                    state++;
                    break;
                  case 2:
                    enableSlowDrivers();
                    break;
                }

              }else{
                int test = move.forwardp(mm(lenght + 50), false);
                if (test == 2){enableSlowDrivers();}
                switch(test){
                  case 1:
                    state++;
                    break;
                }

              }
          }
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.stopForMillis(mili)) state++;
        break;
      case 5:
        enableDrivers();

        if (lane == OUTER){
          state = -1;
        } else{state = 0;}

        if (robotSide == RIGHT && first == false){
          if (lane == OUTER){
            lane = MIDDLE;
          }else if(lane == MIDDLE){
            lane = INNER;
          }
        } else if (robotSide == RIGHT && first == true){
          lane = OUTER;
        }else if (robotSide == LEFT){
          if (lane == OUTER){
            lane = MIDDLE;
          } else if (lane == MIDDLE) {
            lane = INNER;
          } else if (lane == INNER) {
            // Bare return: deliberately skips `routine = 6` and the endgame
            // check below. This is why runRoutines() must be the last call in
            // loop().
            return;
          }
        }
        routine = 6;

        if (lastRoutine == true || midRoutine == true){
          routine  = 9;
          state = 0;
        }
      break;
    }
  break;
  case 5:
    enableDrivers();
    switch(state){
      case 0:
        if(inner(60)) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.backward(600)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.forward(mm(150))) state++;
        break;
      case 5:
        state++;
        enableSlowDrivers();
        break;
      case 6:
        if (robotSide == RIGHT){
          if(move.forwardLeft(mm(300))) state++;
        } else{
          if(move.forwardRight(mm(250))) state++;
        }
        digitalWrite(LED, HIGH);
        break;
      case 7:
        state++;
        break;
      case 8:
        if (robotSide == RIGHT){
          if(move.forwardq(mm(lenght/3 + 150), true)){state++;}
        } else {
          if(move.forwardq(mm(lenght/2 + 160), false)){state++;}
        }
        break;
      case 9:
        if(move.stopForMillis(2*mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 10:
        if (robotSide == LEFT){
          if(outer(25)) state++;
        } else{
          state++;
        }
        break;
      case 11:
        if(move.stopForMillis(mili)) state++;
        break;
      case 12:
        lane = OUTER;
        routine = 7;
        state = 0;
        break;
    }
    break;
  case 6:
    switch(state){
      case -1:
        if (robotSide == RIGHT){
          if(move.backwardp(mm(lenght + 50), true)) state = 1;
        }else{
          if(move.backwardp(mm(lenght + 100), false)) state = 1;
        }
        break;
      case 0:
        if(move.backward(mm(lenght + 250))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(outer(750)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4: // Complex logic for Ramp robot redundancy and lane correction
        if (!(lane == OUTER) && first == true){
          if(inner(180)) state++;
        } else if (first == true){
          state = 7;
          break;
        }else if(!first){
          if(inner(180)) state++;
        }
        pixy.setLamp(0, 0);
        break;
      case 5:
        if(move.backward(mm(200))) state++;
        break;
      case 6:

        pixy.ccc.getBlocks();

        if (pixy.ccc.numBlocks > 0) {
          for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == orangeSignature){
            Block block = pixy.ccc.blocks[i];

            // Classify into a franja by image position.
            // KNOWN: `true` is hard coded, so the LEFT robot classifies using
            // the RIGHT robot's boundary lines. The robot is tuned around it.
            int franja = classifyLane(block.m_x, block.m_y, true);
            franja = constrain(franja, 0, NUM_FRANJAS - 1);

            // Blob area is its weight.
            int tamano = block.m_width * block.m_height;

            pesos[franja] += tamano;
          }
        }
        }

        if(move.stopForMillis(mili/2)) {
          // Pick the heaviest franja
            int mejorFranja = 0;
            for (int i = 1; i < NUM_FRANJAS; i++) {
              if (pesos[i] > pesos[mejorFranja]) {
                mejorFranja = i;
              }
            }

            if (connections < 2){
              if (mejorFranja == 0){
                if (robotSide == RIGHT){
                  lane = INNER;
                } else {lane = OUTER;}
              } else if(mejorFranja == 2){
                if(robotSide == RIGHT){
                  lane = OUTER;
                } else {lane = INNER;}
              } else if (mejorFranja == 1){
                  lane = MIDDLE;
              }
              connections++;
            }

          state++;
          pixy.setLamp(0, 0);
        }




        break;

      case 7:
          // Reset the camera weights
          for (int i = 0; i < NUM_FRANJAS; i++) {
            pesos[i] = 0;
          }

          // KNOWN: the checks below are SEQUENTIAL ifs, not else-ifs, so a later
          // one silently overrides the routine/state chosen by an earlier one.
          // The evaluation order is load bearing. Do not convert to else-if.
          if(lane == INNER){
            routine = 5;
            state = 0;
          }else if(lane == MIDDLE){
            routine = 4;
            state = 0;
          }else if(lane == OUTER){
            routine = 4;
            state = -1;
        }

        if(lane == OUTER && robotSide == RIGHT && first == true){//----------------------------------------------------------------------------------------------------------
          routine = 4;
          state = 0;
          first = false;
        }

        if (robotSide == LEFT && first == true){
          routine = 5;
          state = 0;
          first = false;
        }


        if (lastRoutine == true){
          routine = 4;
          state = -1;
        } else if(midRoutine == true && midRoutineDone == false){
          routine = 4;
          state = -1;
        }
      break;
    }
  break;
  case 7:
    connections = 0;
    switch(state){
      case 0:
        if(move.backward(mm(lenght + 250))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(80)) state++;
        break;
      case 3:
        digitalWrite(enable34, LOW);
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        digitalWrite(LED, HIGH);

        if (headingAvailable()){
          if (robotSide == LEFT){
            alpha = -80;
          } else {alpha  = 80;}

          if (headingSinceZero() >= alpha + beta) {
              move.rotateCCW(200, 200, 200, 200);
          } else if (headingSinceZero() <= alpha - beta){
              move.rotateCW(200, 200, 200, 200);
          } else {
            state++;
          }
        } else{
          if(robotSide == RIGHT){
            if(move.rotate(mm(146), false)) state++;
          } else {
            if(move.rotate(mm(146), true)) state++;
          }
        }

        break;
      case 5:
        if(move.stopForMillis(mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 6:
        // KNOWN: mm(mm(20)) again, same as routine 4 state -2.
        if(outer(mm(20))) state++;
        break;
      case 7:
        if(move.stopForMillis(mili/2)) state++;
        break;
      case 8:
          if (robotSide == RIGHT){
            if(move.forwardp(mm(550), true) == 1) state++;
          }else{
            if(move.forwardp(mm(550), false) == 1) state++;
          }
        break;
      case 9:
        enableDrivers();
        if(move.stopForMillis(250)) state++;
        break;
      case 10:
        digitalWrite(enable34, LOW);
        if(move.backward(20)) state++;
        break;
      case 11:
        if(move.stopForMillis(mili/2)) state++;
        break;
      case 12:
        digitalWrite(LED, HIGH);

        if (headingAvailable()){
          if (robotSide == LEFT){
            alpha = -80;
          } else {alpha  = 80;}
          if (headingSinceZero() >= alpha + beta) {
              move.rotateCCW(200, 200, 200, 200);
          } else if (headingSinceZero() <= alpha - beta){
              move.rotateCW(200, 200, 200, 200);
          } else {
            state++;
          }
        } else{
          if(robotSide != RIGHT){
            if(move.rotate(mm(166), false)) state++;
          } else {
            if(move.rotate(mm(166), true)) state++;
          }
        }
        break;
      case 13:
        if(move.stopForMillis(mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 14:
        if(outer(120)) state++;
        break;
      case 15:
        if(move.stopForMillis(mili)) state++;
        break;
      case 16:
        enableDrivers();
        if(robotSide == RIGHT){
          if(move.rotate(mm(30), true)) state++;
        } else {
          if(move.rotate(mm(30), false)) state++;
        }
        break;
      case 17:
        if(move.stopForMillis(mili * 2)) state++;
        break;
      case 18:
        if(robotSide == LEFT){
          if(move.rotate(mm(20), true)) state++;
        } else {
          if(move.rotate(mm(20), false)) state++;
        }
        break;
      case 19:
        if(move.stopForMillis(mili)) state++;
        break;
      case 20:
        routine = 4;
        state = 0;
        lane = OUTER;
        break;

    }
  break;
  case 8:
    alpha = 0;
    if (headingSinceZero() >= alpha + beta) {
        move.rotateCCW(200, 200, 200, 200);
    } else if (headingSinceZero() <= alpha - beta) {
        move.rotateCW(200, 200, 200, 200);
    } else {
        routine = 7;
        state = 0;
    }
    // KNOWN: no `break` here - control falls through into case 9 on every pass.

  case 9:
    if (lastRoutine == false and millis() > 61000){
      routine = 6;
      state = 0;
      break;
    }
    move.moveBeginStrafe();
    pixy.ccc.getBlocks();
    if (robotSide == LEFT){
      if (pixy.ccc.numBlocks) {
        int maxArea = 0;
        int maxIndex = -1;
        int moveby =  40;
        int maxmove = 550;
        static int movement = 0;

        // Step 1: Find the largest orange block
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
          if (pixy.ccc.blocks[i].m_signature == orangeSignature) {
            int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
            if (area > maxArea) {
              maxArea = area;
              maxIndex = i;
            }
          }
        }

        if (maxIndex != -1) {
          int x = pixy.ccc.blocks[maxIndex].m_x;

          if (x > 240 &&
            ((robotSide == RIGHT && movement > 0) ||
            (robotSide == LEFT && movement < maxmove))) {

            // KNOWN: blocks the whole firmware until the strafe finishes -
            // switch handling and the rest of loop() do not run meanwhile.
            // The regulator still gets fresh headings (its hook polls the
            // sensor), so the strafe itself stays regulated.
            while (true){
              if(move.right(mm(moveby))) break;
            }
            Serial.print("MOVE RIGHT\n");
            pixy.ccc.blocks[maxIndex].print();
            if (robotSide == RIGHT){
              movement -= moveby;
            } else {
              movement += moveby;
            }
          } else if (x < 140 &&
            ((robotSide == LEFT && movement > 0) ||
            (robotSide == RIGHT && movement < maxmove))) {

            while (true){
              if(move.left(mm(moveby))) break;
            }
            Serial.print("MOVE LEFT\n");
            pixy.ccc.blocks[maxIndex].print();
            if (robotSide == RIGHT){
              movement += moveby;
            } else {
              movement -= moveby;
            }
          } else {
            move.stop();
            pixy.ccc.blocks[maxIndex].print();
          }
        }
      } delay(5);
    }else {
      routine = 10;
      state = 0;
    }
    // if (lastRoutine == false and millis() > 62000){
    //   routine = 6;
    //   state = 0;
    // }
    break;

  case 10:
    switch(state){
      case 0:
        if(move.backward(mm(50))) state = 1;
        break;
      case 1:
        if(move.left(mm(500))) state++;
        break;
      case 2:
        if (lastRoutine == false and millis() > 62000){
          routine = 6;
          state = 0;
        }
    }
    break;

}

}
