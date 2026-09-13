// generalStrategy.cpp - the general strategy: the lane loop with the purple
// ball opening, the camera lane choice and the microswitch handling. Built by
// the `general` environment. See Strategy.h.
//
// Several behaviours here look odd but the robot is tuned around them; each is
// marked KNOWN where it appears. Changing one needs a field test.
//
// Every distance handed to the Move library is in MILLIMETRES; the library
// converts to encoder counts for this robot. "Towards the inside / outside of
// the field" is move.inner() / move.outer(), mirrored per robot inside the
// library. A few distances are written per robot (robotSide == LEFT ? a : b)
// because the tuned distance differs per robot.

#include "Strategy.h"
#include "Hardware.h"
#include "Sensors.h"

const char* strategyName = GENERAL_ENDGAME_KICKS ? "general, kicks on" : "general, kicks off";

int routine = 4;
int state = 0;
bool first = true;
rlane lane = MIDDLE;

int connections;

unsigned long startTime;

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
// Purple ball detection
//
// The detector works on the BALL'S EDGES instead of just its centroid, and
// confirms the answer over several frames:
//
//   1. Quality gate - a blob must be purple, tracked for a couple of frames,
//      big enough and roughly round. This is the "not a grain of dust" filter.
//   2. Zone scoring - the blob's bounding box is scored against every
//      calibrated rectangle in ballZones (Hardware.cpp), with
//      BALL_ZONE_TOLERANCE px of graded slack around each one. A blob matching
//      no zone is DISCARDED rather than being forced into a quadrant, which is
//      what stops the robot from "detecting the ball where it isn't".
//   3. Voting - only the largest valid blob of each frame votes, so the ball
//      and a speck can never both score in the same frame.
//   4. Early commit - the scan stops the moment one zone is clearly ahead, so
//      a clean ball is decided in roughly 5 camera frames (~85 ms).
//
// Sensitivity. Tuned to be as permissive as possible WITHOUT letting specks of
// dust vote: the quality gates below only reject things that cannot physically
// be the ball (too small, too thin, only seen for a single frame), while the
// multi-frame vote is what actually guarantees the decision is correct.
// ---------------------------------------------------------------------------

static const uint8_t BALL_MIN_AGE     = 2;    // camera frames the blob must have been tracked for.
                                              //   2 only rejects one-frame flicker; the vote
                                              //   below does the real confirming.
static const int  BALL_MIN_SIDE       = 5;    // px. Blob must be at least this wide AND this tall.
static const long BALL_MIN_AREA       = 45;   // px^2. Anti-dust floor - a real ball is far bigger.
static const int  BALL_MAX_ASPECT_X10 = 30;   // 3.0 : rejects long thin smears / glare streaks.
                                              //   Deliberately lenient so a ball clipped by the
                                              //   frame edge still passes.
static const int  BALL_ZONE_TOLERANCE = 20;   // px of slack around every measured rectangle, on
                                              //   all four sides. A ball whose centre lands
                                              //   outside a zone still matches it, so the numbers
                                              //   typed in do not have to be perfect.
                                              //   It is NOT a plain box inflation: matches inside
                                              //   the band are graded by distance, so when two
                                              //   zones are both within reach the closer one wins
                                              //   outright instead of the two fighting. That is
                                              //   what keeps touching zones unambiguous no matter
                                              //   how wide this gets. See scoreBallZone().
                                              //   Widening this only costs false-POSITIVE margin
                                              //   (a stray purple object further from a real
                                              //   position can now reach a zone), never
                                              //   zone-vs-zone accuracy. BALL_CONFIRM_SOFT below
                                              //   is what pays that back.
static const long BALL_MIN_SCORE      = 20;   // see scoreBallZone(): >=200 means "centre inside
                                              //   a zone", 100..199 "centre within tolerance of
                                              //   one", 20..99 "clearly overlapping one".
static const int  BALL_CONFIRM_FRAMES = 3;    // frames one zone must win before we commit, when
                                              //   that zone has had at least one solid hit
                                              //   (centre truly inside the measured rectangle,
                                              //   score >= 200).
static const int  BALL_CONFIRM_SOFT   = 5;    // ...but this many when ALL the evidence came from
                                              //   the tolerance band. Evidence that leans on the
                                              //   slack is held to a higher bar, which is what
                                              //   lets the tolerance above be generous without
                                              //   getting loose.
static const int  BALL_VOTE_MARGIN    = 2;    // frames the leader must lead the rest by, either way.
static const unsigned long BALL_SCAN_MS   = 900;  // hard ceiling on the whole scan.
static const unsigned long BALL_GIVEUP_MS = 350;  // if NOTHING ball-like has been seen by now,
                                                  //   stop early so an empty field costs almost
                                                  //   nothing.

// Area of the overlap between two axis-aligned rectangles; 0 when they do not
// touch. Returns long because a full-frame box (316 x 208) overflows a 16-bit
// AVR int.
static long rectOverlap(int aL, int aT, int aR, int aB, int bL, int bT, int bR, int bB) {
  int w = min(aR, bR) - max(aL, bL);
  int h = min(aB, bB) - max(aT, bT);
  if (w <= 0 || h <= 0) return 0;
  return (long)w * (long)h;
}

// Scores one blob box against one zone. Higher is better, 0 means no relation.
// The result falls into three tiers that never cross, so the ranking between
// zones is always well defined:
//
//   200..300 : the blob's centre is INSIDE the measured rectangle. Strongest
//              evidence. The 0..100 on top is the box overlap, which is what
//              separates two zones that both contain the centre (touching
//              zones) - the rectangle holding more of the ball wins, which is
//              the physically correct answer.
//   100..199 : the centre is OUTSIDE the rectangle but within
//              BALL_ZONE_TOLERANCE of it. Graded by how far outside it is, so
//              when two zones are both in reach the CLOSER one always wins
//              outright - never a coin flip between neighbours.
//     0..100 : the centre is well outside, but the boxes still overlap.
//              Normalised by the SMALLER of the two boxes so a wide zone and a
//              tight, ball-sized zone are judged on the same scale.
static long scoreBallZone(int bL, int bT, int bR, int bB, long blobArea, const int zone[4]) {
  int zL = min(zone[0], zone[2]);
  int zR = max(zone[0], zone[2]);
  int zT = min(zone[1], zone[3]);
  int zB = max(zone[1], zone[3]);

  // An un-filled { 0, 0, 0, 0 } row has no area and must never match anything.
  // This check has to happen BEFORE the tolerance is applied, otherwise the
  // slack would turn an empty row into a live region around the top-left
  // corner of the image.
  if (zR - zL <= 0 || zB - zT <= 0) return 0;

  int cx = (bL + bR) / 2;
  int cy = (bT + bB) / 2;

  // How far the centre sits outside the rectangle on each axis; 0 when inside.
  int dx = 0, dy = 0;
  if (cx < zL)      dx = zL - cx;
  else if (cx > zR) dx = cx - zR;
  if (cy < zT)      dy = zT - cy;
  else if (cy > zB) dy = cy - zB;

  long zoneArea = (long)(zR - zL) * (long)(zB - zT);
  long refArea  = (blobArea < zoneArea) ? blobArea : zoneArea;
  long overlap  = rectOverlap(bL, bT, bR, bB, zL, zT, zR, zB);
  long overlapScore = (refArea > 0) ? (overlap * 100L) / refArea : 0;

  if (dx == 0 && dy == 0) {
    return 200 + overlapScore;                    // centre inside the measured rectangle
  }

  if (dx <= BALL_ZONE_TOLERANCE && dy <= BALL_ZONE_TOLERANCE) {
    // Inside the tolerance band. miss runs 1..2*tolerance, so the penalty runs
    // 0..100 and the tier stays between 100 and 199 - always above a
    // pure-overlap match, always below a centre that is genuinely inside some
    // other zone.
    long miss    = (long)dx + (long)dy;
    long penalty = (miss * 100L) / (2L * (long)BALL_ZONE_TOLERANCE);
    return 200 - penalty - 1;
  }

  return overlapScore;                            // far outside, judged on overlap alone
}

// Strings kept in flash (F()) so the detector adds no SRAM on the Mega.
static const __FlashStringHelper *ballZoneName(int zone) {
  switch (zone) {
    case 0: return F("upper left  -> routine 0");
    case 1: return F("upper right -> routine 1");
    case 2: return F("lower left  -> routine 2");
    case 3: return F("lower right -> routine 3");
  }
  return F("no ball -> routine 4");
}

// Returns the ball position index 0..3, or -1 when no ball could be confirmed.
static int detectBallZone() {
  int  votes[NUM_BALL_ZONES]  = {0};
  long weight[NUM_BALL_ZONES] = {0};
  bool solid[NUM_BALL_ZONES]  = {false};   // zone has had >=1 centre-inside hit, not just slack
  bool seenAnything = false;

  unsigned long scanStart = millis();
  int committed = -1;

  while (millis() - scanStart < BALL_SCAN_MS) {

    // Non-blocking read. A negative result is BUSY (no new frame yet) or a
    // link error; either way it is NOT evidence of an empty field, so it must
    // not count as a sample.
    if (pixy.ccc.getBlocks(false) < 0) {
      delayMicroseconds(500);
      continue;
    }

    int  frameZone  = -1;
    long frameArea  = 0;
    long frameScore = 0;
    int  frameL = 0, frameT = 0, frameR = 0, frameB = 0;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      Block &b = pixy.ccc.blocks[i];

      if ((int)b.m_signature != purpleSignature) continue;
      if (b.m_age < BALL_MIN_AGE) continue;

      int bw = (int)b.m_width;
      int bh = (int)b.m_height;
      if (bw < BALL_MIN_SIDE || bh < BALL_MIN_SIDE) continue;

      long area = (long)bw * (long)bh;
      if (area < BALL_MIN_AREA) continue;

      // A ball is about as wide as it is tall; reject streaks and reflections.
      if ((long)bw * 10L > (long)bh * (long)BALL_MAX_ASPECT_X10) continue;
      if ((long)bh * 10L > (long)bw * (long)BALL_MAX_ASPECT_X10) continue;

      // Both edges of the blob. m_x / m_y are uint16_t, so cast BEFORE
      // subtracting or a blob near the left/top border wraps around to ~65000.
      int cx = (int)b.m_x;
      int cy = (int)b.m_y;
      int bL = cx - bw / 2;
      int bR = cx + bw / 2;
      int bT = cy - bh / 2;
      int bB = cy + bh / 2;

      int  bestZone  = -1;
      long bestScore = 0;
      for (int z = 0; z < NUM_BALL_ZONES; z++) {
        long score = scoreBallZone(bL, bT, bR, bB, area, ballZones[z]);
        if (score > bestScore) { bestScore = score; bestZone = z; }
      }

      // Purple, ball-shaped, but not at any calibrated ball position -> noise.
      if (bestZone < 0 || bestScore < BALL_MIN_SCORE) continue;

      // Only the biggest valid blob of this frame is allowed to vote.
      if (area > frameArea) {
        frameArea  = area;
        frameZone  = bestZone;
        frameScore = bestScore;
        frameL = bL; frameT = bT; frameR = bR; frameB = bB;
      }
    }

    if (frameZone < 0) {
      // Nothing ball-like in this frame. Bail out early only while we have
      // never seen a ball at all - once there is evidence, keep sampling for
      // the full budget.
      if (!seenAnything && millis() - scanStart > BALL_GIVEUP_MS) break;
      continue;
    }

    seenAnything = true;
    votes[frameZone]++;
    weight[frameZone] += frameArea;
    if (frameScore >= 200) solid[frameZone] = true;   // centre truly inside the rectangle

    Serial.print(F("blob edges L"));
    Serial.print(frameL); Serial.print(F(" T")); Serial.print(frameT);
    Serial.print(F(" R")); Serial.print(frameR); Serial.print(F(" B")); Serial.print(frameB);
    Serial.print(frameScore >= 200 ? F("  solid  -> ") : F("  in-tol -> "));
    Serial.println(ballZoneName(frameZone));

    // Leader and runner-up over everything collected so far. Ties break on
    // accumulated blob area, so the closer/bigger sighting wins.
    int leader = 0, runnerUp = -1;
    for (int z = 1; z < NUM_BALL_ZONES; z++) {
      if (votes[z] > votes[leader] ||
         (votes[z] == votes[leader] && weight[z] > weight[leader])) leader = z;
    }
    for (int z = 0; z < NUM_BALL_ZONES; z++) {
      if (z == leader) continue;
      if (runnerUp < 0 || votes[z] > votes[runnerUp]) runnerUp = z;
    }

    // A zone that has been hit dead-on at least once commits quickly. One that
    // has only ever matched through the tolerance slack has to prove itself
    // over more frames - that is the price of the generous BALL_ZONE_TOLERANCE,
    // and it is only paid in the rare case where the calibration is off enough
    // that the ball never lands inside the box.
    int needed = solid[leader] ? BALL_CONFIRM_FRAMES : BALL_CONFIRM_SOFT;

    if (votes[leader] >= needed &&
        votes[leader] - votes[runnerUp] >= BALL_VOTE_MARGIN) {
      committed = leader;
      break;
    }
  }

  // Budget ran out without a clear winner: use the best evidence we have
  // rather than throwing away a ball we definitely saw.
  if (committed < 0) {
    for (int z = 0; z < NUM_BALL_ZONES; z++) {
      if (votes[z] == 0) continue;
      if (committed < 0 || votes[z] > votes[committed] ||
         (votes[z] == votes[committed] && weight[z] > weight[committed])) committed = z;
    }
  }

  Serial.print(F("ball scan "));
  Serial.print(millis() - scanStart);
  Serial.print(F(" ms  votes "));
  for (int z = 0; z < NUM_BALL_ZONES; z++) {
    Serial.print(votes[z]);
    Serial.print(' ');
  }
  Serial.print(F(" => "));
  Serial.println(ballZoneName(committed));

  return committed;
}

void selectOpeningRoutine() {
  int ballZone = detectBallZone();
  if (ballZone >= 0) {
    routine = ballZone;   // routines 0..3 are the four ball-capture sequences
  }
  // routine stays 4 (the plain lane loop) when nothing was confirmed.

  pixy.setLamp(0, 0);
}

// ---------------------------------------------------------------------------

void handleMicroSwitches() {
  // KNOWN: microSwitchTime and currentTime are `int`, 16 bits on AVR, so both
  // truncate millis() and wrap every 32.767 seconds; the 350 ms debounce
  // misbehaves around each wrap. Changing them to unsigned long changes switch
  // timing, so it needs a field test.
  static int microSwitchTime = 0;
  static bool lastBackSwitchState = HIGH;   // for edge detection
  static bool lastSideSwitchState = HIGH;   // for edge detection
  int currentTime = millis();
    bool currentBackSwitchState = digitalRead(backSwitchPin);
    bool currentSideSwitchState = digitalRead(sideSwitchPin);


    if (currentTime - microSwitchTime > 350){
      // Handle switch press
      if (currentBackSwitchState == LOW && lastBackSwitchState == HIGH) {
        microSwitchTime = millis();
        // Button pressed: the robot is square against the back wall, so the
        // heading is zeroed there.
        headingZero();
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
  if (!GENERAL_ENDGAME_KICKS) return;   // constant: everything below is compiled out

  unsigned long t = millis() - startTime;

  if (lastRoutine == false && t > GENERAL_LATE_KICK_MS){
    lastRoutine = true;
    lane = OUTER;
    lenght -= 30;
  }
  if (midRoutine == false && midRoutineDone == false && t > GENERAL_MID_KICK_MS){
    midRoutine = true;
    if (routine != 4 && lane != OUTER){
      lane = OUTER;
    } else {
      lane = MIDDLE;
    }
    lenght -= 30;
  } else if(midRoutine == true && t > GENERAL_MID_DONE_MS && t < GENERAL_MID_END_MS){
    midRoutine = false;
    midRoutineDone = true;
    lenght += 30;
  }
}

// ---------------------------------------------------------------------------

void runRoutines() {

switch (routine) {//---------------------------------------------------------------------------------------ROUTINES---------------------------------------------------------//
  case 0:
    switch(state){
      case 0:
        if(move.backward(100)) state++;
        break;
      case 1:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(505)) state++;
        break;
      case 3:
        state++;
        break;
      case 4:
        myservo.write(openGate);
        if(move.forward(350)) state++;
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
        if (move.right(200)) state++;
        break;
      case 1:
        if(move.backward(100)) state++;
        break;
      case 2:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(530)) state++;
        break;
      case 4:
        state++;
        break;
      case 5:
        myservo.write(openGate);
        if(move.forward(350)) state++;
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
        if(move.backward(100)) state++;
        break;
      case 1:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(650)) state++;
        break;
      case 3:
        state++;
        break;
      case 4:
        myservo.write(closedGate);
        enableSlowDrivers();
        if(move.forward(150)) state++;
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
        if(move.right(200)) state++;
        break;
      case 1:
        if(move.backward(100)) state++;
        break;
      case 2:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(550)) state++;
        break;
      case 4:
        state++;
        break;
      case 5:
        myservo.write(closedGate);
        enableSlowDrivers();
        if(move.forward(250)) state++;
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
          if(move.rotate(166, false)) state--;
        }else{
          if(move.rotate(166, true)) state--;
        }
        break;
      case -2:
        // KNOWN: per-robot distance the course is tuned around.
        if(move.outer(robotSide == LEFT ? 95 : 143)) state--;
        break;
      case -3:
        // KNOWN: forwardp returns 2 at 14/22 of the distance and the bare `if`
        // treats that as done, so this state ends at ~255 mm of the 400 with
        // the motors still running; state -4 releases them.
        if (robotSide == RIGHT){
          if(move.forwardp(400, true)) state--;
        }else{
          if(move.forwardp(400, false)) state--;
        }
        break;
      case -4:
        enableDrivers();
        if(move.stopForMillis(mili)) state--;
        break;
      case -5:
        disableDrivers();
        if (robotSide == RIGHT){
          if(move.rotate(166, true)) state--;
        }else{
          if(move.rotate(166, false)) state--;
        }
        break;
      case -6:
        if(move.outer(30)) state = 0;
        break;
      case 0:
        enableDrivers();            //MIDDLE LANE
        if(move.backward(280)) state = 1;
        break;
      case 1:
        if(move.stopForMillis(mili)) state = 2;
        break;
      case 2:
          if (lane == MIDDLE || lane == INNER){
            int test = move.forwardRegulated(lenght + 50);
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
                int test = move.forwardp(lenght + 50, true);
                switch(test){
                  case 1:
                    state++;
                    break;
                  case 2:
                    enableSlowDrivers();
                    break;
                }

              }else{
                int test = move.forwardp(lenght + 50, false);
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
        if(move.inner(60)) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        // KNOWN: per-robot distance the course is tuned around.
        if(move.backward(robotSide == LEFT ? 126 : 84)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.forward(150)) state++;
        break;
      case 5:
        state++;
        enableSlowDrivers();
        break;
      case 6:
        if (robotSide == RIGHT){
          if(move.forwardLeft(300)) state++;
        } else{
          if(move.forwardRight(250)) state++;
        }
        break;
      case 7:
        state++;
        break;
      case 8:
        if (robotSide == RIGHT){
          if(move.forwardq(lenght/3 + 150, true)){state++;}
        } else {
          if(move.forwardq(lenght/2 + 160, false)){state++;}
        }
        break;
      case 9:
        if(move.stopForMillis(2*mili)) state++;
        break;
      case 10:
        if (robotSide == LEFT){
          if(move.outer(25)) state++;
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
          if(move.backwardp(lenght + 50, true)) state = 1;
        }else{
          if(move.backwardp(lenght + 50, false)) state = 1;
        }
        break;
      case 0:
        if(move.backward(lenght + 250)) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.outer(750)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4: // Complex logic for Ramp robot redundancy and lane correction
        if (!(lane == OUTER) && first == true){
          if(move.inner(180)) state++;
        } else if (first == true){
          state = 7;
          break;
        }else if(!first){
          if(move.inner(180)) state++;
        }
        pixy.setLamp(0, 0);
        break;
      case 5:
        if(move.backward(200)) state++;
        break;
      case 6:

        pixy.ccc.getBlocks();

        if (pixy.ccc.numBlocks > 0) {
          for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == orangeSignature){
            Block block = pixy.ccc.blocks[i];

            // Classify into a franja by image position.
            int franja = classifyLane(block.m_x, block.m_y, (robotSide == RIGHT));
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
        if(move.backward(lenght + 250)) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        // KNOWN: per-robot nudge off the wall the course is tuned around.
        if(move.forward(robotSide == LEFT ? 17 : 11)) state++;
        break;
      case 3:
        digitalWrite(enable34, LOW);
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(robotSide == RIGHT){
          if(move.rotate(146, false)) state++;
        } else {
          if(move.rotate(146, true)) state++;
        }
        break;
      case 5:
        if(move.stopForMillis(mili)) state++;
        break;
      case 6:
        // KNOWN: per-robot distance the course is tuned around, same as
        // routine 4 state -2.
        if(move.outer(robotSide == LEFT ? 95 : 143)) state++;
        break;
      case 7:
        if(move.stopForMillis(mili/2)) state++;
        break;
      case 8:
          if (robotSide == RIGHT){
            if(move.forwardp(550, true) == 1) state++;
          }else{
            if(move.forwardp(550, false) == 1) state++;
          }
        break;
      case 9:
        enableDrivers();
        if(move.stopForMillis(250)) state++;
        break;
      case 10:
        digitalWrite(enable34, LOW);
        // KNOWN: per-robot nudge the course is tuned around.
        if(move.backward(robotSide == LEFT ? 4 : 3)) state++;
        break;
      case 11:
        if(move.stopForMillis(mili/2)) state++;
        break;
      case 12:
        if(robotSide != RIGHT){
          if(move.rotate(166, false)) state++;
        } else {
          if(move.rotate(166, true)) state++;
        }
        break;
      case 13:
        if(move.stopForMillis(mili)) state++;
        break;
      case 14:
        if(move.outer(120)) state++;
        break;
      case 15:
        if(move.stopForMillis(mili)) state++;
        break;
      case 16:
        enableDrivers();
        if(robotSide == RIGHT){
          if(move.rotate(30, true)) state++;
        } else {
          if(move.rotate(30, false)) state++;
        }
        break;
      case 17:
        if(move.stopForMillis(mili * 2)) state++;
        break;
      case 18:
        if(robotSide == LEFT){
          if(move.rotate(20, true)) state++;
        } else {
          if(move.rotate(20, false)) state++;
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
              if(move.right(moveby)) break;
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
              if(move.left(moveby)) break;
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
        if(move.backward(50)) state = 1;
        break;
      case 1:
        if(move.left(500)) state++;
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
