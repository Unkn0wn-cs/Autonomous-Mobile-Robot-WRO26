// controlStrategy.cpp - the wall robot's early-game strategy.
// Built by the `control` environment. See Strategy.h for what a strategy
// must define.
//
// Every movement call is non-blocking: it is called again and again from
// loop() and returns true only once, when it has finished.
//
// Naming convention: PascalCase for all local functions and variables.

#include "Strategy.h"
#include "Hardware.h"
#include "Sensors.h"

const char* strategyName = "control";

// Keep the routine numbers compatible with generalStrategy.cpp.
enum RoutineId {
  OpeningUpperLeft  = 0,
  OpeningUpperRight = 1,
  OpeningLowerLeft  = 2,
  OpeningLowerRight = 3,
  MainLoop          = 4,
  DiagonalLane      = 5,
  ReturnAndClassify = 6,
  CornerReset       = 7,
  CameraParking     = 9,
  DebugRoutine      = 10,
  OrangeCollection  = 11,
  LaneSweep         = 12,
  CollectionCornerCheck = 13
};

int routine = MainLoop;
int state   = 0;
bool first = false;
rlane lane = OUTER;
int connections = 0;
int laneWeights[NUM_FRANJAS] = {0};
bool lastRoutine = false;
bool midRoutine = false;
bool midRoutineDone = false;

unsigned long startTime;

// Minimum PWM at which the rotor still overcomes static friction and spins.
// Determined by field test (PWM sweep 254->0, 1 PWM/s). Below this value
// the rotor stalls. Only used by this strategy.
const int RotorMinPwm = 45;

static const int RotorReversePwm = 254;
static const unsigned long GateCloseMs = 250;
static const unsigned long OrangeEjectMs = 2000;
static const unsigned long EarlyGameMs = 45000;

static bool EarlyGameExpired = false;
static rlane CollectionLane = OUTER;
static bool FinalShotPending = false;
static int RoutineAfterSweep = OrangeCollection;
static int StateAfterSweep = 9;

// Low shot: normal direction. Ejection: reverse, along the floor.
static void enableControlSlowShot() {
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);
  analogWrite(enable34, RotorMinPwm);
}

static void enableControlEjection() {
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);
  analogWrite(enable34, RotorReversePwm);
}

int classifyLane(float X, float Y, bool RightRobot) {
  float MA, MB, BA, BB;
  if (RightRobot) {
    MA = (24.0f - 205.0f) / (128.0f - 45.0f); BA = 205.0f - MA * 45.0f;
    MB = (180.0f - 24.0f) / (237.0f - 188.0f); BB = 24.0f - MB * 188.0f;
  } else {
    MA = (61.0f - 182.0f) / (144.0f - 6.0f); BA = 182.0f - MA * 6.0f;
    MB = (186.0f - 8.0f) / (279.0f - 161.0f); BB = 8.0f - MB * 161.0f;
  }
  float YA = MA * X + BA;
  float YB = MB * X + BB;
  if (Y < YA) return 0;
  if (Y < YB) return 2;
  return 1;
}

static void BeginNormalLoop() {
  lane = CollectionLane;
  first = false;
  FinalShotPending = true;
  enableControlSlowShot();
  routine = MainLoop;
  state = lane == OUTER ? -1 : -7;
}

// ---------------------------------------------------------------------------
// Purple ball detection - ported from generalStrategy, unchanged logic.
// Tuning constants below are identical to the general strategy so the
// detection behaviour is the same robot to robot.
// ---------------------------------------------------------------------------

static const uint8_t BallMinAge       = 2;
static const int     BallMinSide      = 5;
static const long    BallMinArea      = 45;
static const int     BallMaxAspectX10 = 30;
static const int     BallZoneTol      = 20;
static const long    BallMinScore     = 20;
static const int     BallConfirmFrames = 3;
static const int     BallConfirmSoft  = 5;
static const int     BallVoteMargin   = 2;
static const unsigned long BallScanMs   = 900;
static const unsigned long BallGiveupMs = 350;

static long RectOverlap(int AL, int AT, int AR, int AB,
                         int BL, int BT, int BR, int BB) {
  int W = min(AR, BR) - max(AL, BL);
  int H = min(AB, BB) - max(AT, BT);
  if (W <= 0 || H <= 0) return 0;
  return (long)W * (long)H;
}

static long ScoreBallZone(int BlobL, int BlobT, int BlobR, int BlobB,
                           long BlobArea, const int Zone[4]) {
  int ZL = min(Zone[0], Zone[2]);
  int ZR = max(Zone[0], Zone[2]);
  int ZT = min(Zone[1], Zone[3]);
  int ZB = max(Zone[1], Zone[3]);

  if (ZR - ZL <= 0 || ZB - ZT <= 0) return 0;

  int Cx = (BlobL + BlobR) / 2;
  int Cy = (BlobT + BlobB) / 2;

  int Dx = 0, Dy = 0;
  if      (Cx < ZL) Dx = ZL - Cx;
  else if (Cx > ZR) Dx = Cx - ZR;
  if      (Cy < ZT) Dy = ZT - Cy;
  else if (Cy > ZB) Dy = Cy - ZB;

  long ZoneArea    = (long)(ZR - ZL) * (long)(ZB - ZT);
  long RefArea     = (BlobArea < ZoneArea) ? BlobArea : ZoneArea;
  long Overlap     = RectOverlap(BlobL, BlobT, BlobR, BlobB, ZL, ZT, ZR, ZB);
  long OverlapScore = (RefArea > 0) ? (Overlap * 100L) / RefArea : 0;

  if (Dx == 0 && Dy == 0) return 200 + OverlapScore;

  if (Dx <= BallZoneTol && Dy <= BallZoneTol) {
    long Miss    = (long)Dx + (long)Dy;
    long Penalty = (Miss * 100L) / (2L * (long)BallZoneTol);
    return 200 - Penalty - 1;
  }

  return OverlapScore;
}

// Returns 0..3 for the confirmed ball zone, or -1 when nothing was confirmed.
static int DetectBallZone() {
  int  Votes[NUM_BALL_ZONES]  = {0};
  long Weight[NUM_BALL_ZONES] = {0};
  bool Solid[NUM_BALL_ZONES]  = {false};
  bool SeenAnything = false;

  unsigned long ScanStart = millis();
  int Committed = -1;

  while (millis() - ScanStart < BallScanMs) {
    if (pixy.ccc.getBlocks(false) < 0) {
      delayMicroseconds(500);
      continue;
    }

    int  FrameZone  = -1;
    long FrameArea  = 0;
    long FrameScore = 0;

    for (int I = 0; I < pixy.ccc.numBlocks; I++) {
      Block &Blk = pixy.ccc.blocks[I];

      if ((int)Blk.m_signature != purpleSignature) continue;
      if (Blk.m_age < BallMinAge) continue;

      int BW = (int)Blk.m_width;
      int BH = (int)Blk.m_height;
      if (BW < BallMinSide || BH < BallMinSide) continue;

      long Area = (long)BW * (long)BH;
      if (Area < BallMinArea) continue;
      if ((long)BW * 10L > (long)BH * (long)BallMaxAspectX10) continue;
      if ((long)BH * 10L > (long)BW * (long)BallMaxAspectX10) continue;

      int Cx  = (int)Blk.m_x;
      int Cy  = (int)Blk.m_y;
      int BlobL = Cx - BW / 2;
      int BlobR = Cx + BW / 2;
      int BlobT = Cy - BH / 2;
      int BlobB = Cy + BH / 2;

      int  BestZone  = -1;
      long BestScore = 0;
      for (int Z = 0; Z < NUM_BALL_ZONES; Z++) {
        long Score = ScoreBallZone(BlobL, BlobT, BlobR, BlobB, Area, ballZones[Z]);
        if (Score > BestScore) { BestScore = Score; BestZone = Z; }
      }

      if (BestZone < 0 || BestScore < BallMinScore) continue;

      if (Area > FrameArea) {
        FrameArea  = Area;
        FrameZone  = BestZone;
        FrameScore = BestScore;
      }
    }

    if (FrameZone < 0) {
      if (!SeenAnything && millis() - ScanStart > BallGiveupMs) break;
      continue;
    }

    SeenAnything = true;
    Votes[FrameZone]++;
    Weight[FrameZone] += FrameArea;
    if (FrameScore >= 200) Solid[FrameZone] = true;

    int Leader = 0, RunnerUp = -1;
    for (int Z = 1; Z < NUM_BALL_ZONES; Z++) {
      if (Votes[Z] > Votes[Leader] ||
         (Votes[Z] == Votes[Leader] && Weight[Z] > Weight[Leader])) Leader = Z;
    }
    for (int Z = 0; Z < NUM_BALL_ZONES; Z++) {
      if (Z == Leader) continue;
      if (RunnerUp < 0 || Votes[Z] > Votes[RunnerUp]) RunnerUp = Z;
    }

    int Needed = Solid[Leader] ? BallConfirmFrames : BallConfirmSoft;
    if (Votes[Leader] >= Needed &&
        Votes[Leader] - (RunnerUp >= 0 ? Votes[RunnerUp] : 0) >= BallVoteMargin) {
      Committed = Leader;
      break;
    }
  }

  // Use best evidence if budget ran out without a clear winner.
  if (Committed < 0) {
    for (int Z = 0; Z < NUM_BALL_ZONES; Z++) {
      if (Votes[Z] == 0) continue;
      if (Committed < 0 || Votes[Z] > Votes[Committed] ||
         (Votes[Z] == Votes[Committed] && Weight[Z] > Weight[Committed]))
        Committed = Z;
    }
  }

  return Committed;
}

// ---------------------------------------------------------------------------
// selectOpeningRoutine
// ---------------------------------------------------------------------------

void selectOpeningRoutine() {
  startTime = millis(); // Start the 45 s clock before detection.
  int BallZone = DetectBallZone();
  state = 0;
  CollectionLane = (BallZone == OpeningUpperRight || BallZone == OpeningLowerRight)
                     ? INNER : OUTER;

  // --- Bluetooth flag so we know which case was entered ---
  if (BallZone >= 0) {
    routine = BallZone;
    Serial2.print(F("[ctrl] Purple detected -> routine "));
    Serial2.println(BallZone);
  } else {
    routine = OrangeCollection;
    state = 0;
    Serial2.println(F("[ctrl] No purple -> orange collection"));
  }

  pixy.setLamp(0, 0);
}

// ---------------------------------------------------------------------------
// handleMicroSwitches
// Back switch: zeros heading (robot is square on wall) and advances state.
// Side switch: advances state (same rule as generalStrategy, excluding r4).
// ---------------------------------------------------------------------------

void handleMicroSwitches() {
  static unsigned long MicroSwitchTime = 0;
  static bool LastBackSwitchState  = HIGH;
  static bool LastSideSwitchState  = HIGH;

  unsigned long CurrentTime = millis();
  bool CurrentBackSwitchState = digitalRead(backSwitchPin);
  bool CurrentSideSwitchState = digitalRead(sideSwitchPin);

  if (CurrentTime - MicroSwitchTime > 350) {
    if (CurrentBackSwitchState == LOW && LastBackSwitchState == HIGH) {
      MicroSwitchTime = CurrentTime;
      headingZero();
      if (!(routine == ReturnAndClassify && state == 2)) state++;
      Serial.println(F("Heading zeroed by back switch"));
    }
    if (CurrentSideSwitchState == LOW && LastSideSwitchState == HIGH &&
        routine != MainLoop && !(routine == CornerReset && state == 8)) {
      MicroSwitchTime = CurrentTime;
      state++;
    }
  }
  LastBackSwitchState = CurrentBackSwitchState;
  LastSideSwitchState = CurrentSideSwitchState;
}

// ---------------------------------------------------------------------------
// Mark 45 s; finish the current route and return before the final shot.
// ---------------------------------------------------------------------------

void updateEndgameTiming() {
  if (!EarlyGameExpired && millis() - startTime >= EarlyGameMs) {
    EarlyGameExpired = true;
    Serial2.println(F("[ctrl] 45 s reached; normal loop pending"));
  }
}

// ---------------------------------------------------------------------------
// runRoutines
// ---------------------------------------------------------------------------

void runRoutines() {
  switch (routine) {
    case OpeningUpperLeft: // MIDDLE capture, OUTER collection.
      switch (state) {
        case 0: if (move.backward(100)) state++; break;
        case 1: // Standard shot for the orange before the purple.
          myservo.write(closedGate);
          enableControlSlowShot();
          enableDrivers();
          if (move.stopForMillis(mili)) state++;
          break;
        case 2: if (move.forward(505)) state++; break;
        case 3: // Let the orange leave before opening the gate.
          if (move.stopForMillis(3000)) state++;
          break;
        case 4: // Capture purple + clearance; tune this distance.
          myservo.write(openGate);
          enableDrivers();
          if (move.forward(350)) state++;
          break;
        case 5: // Store purple; immediately slow the rotor.
          myservo.write(closedGate);
          enableControlSlowShot();
          if (move.stopForMillis(mili)) state++;
          break;
        case 6: // Remaining MIDDLE route to the front.
          if (move.forward(295)) state++;
          break;
        case 7: // Eject towards OUTER, then sweep INNER.
          lane = MIDDLE;
          RoutineAfterSweep = OrangeCollection;
          StateAfterSweep = 8;
          routine = LaneSweep;
          state = 2;
          break;
      }
      break;

    case OpeningUpperRight: // INNER capture and collection.
      switch (state) {
        case 0: if (move.right(200)) state++; break;
        case 1: if (move.backward(100)) state++; break;
        case 2: // Low shot sends the first orange towards INNER.
          myservo.write(closedGate);
          enableControlSlowShot();
          if (move.stopForMillis(mili)) state++;
          break;
        case 3: if (move.forward(530)) state++; break;
        case 4: // Low-speed transit time; tune 3000 ms here.
          if (move.stopForMillis(3000)) state++;
          break;
        case 5: // Capture purple + clearance; tune this distance.
          myservo.write(openGate);
          enableDrivers();
          if (move.forward(350)) state++;
          break;
        case 6: // Store purple; immediately slow the rotor.
          myservo.write(closedGate);
          enableControlSlowShot();
          if (move.stopForMillis(mili)) state++;
          break;
        case 7: // Remaining INNER route to the front.
          if (move.forward(270)) state++;
          break;
        case 8: // Eject towards INNER, then sweep MIDDLE.
          lane = INNER;
          RoutineAfterSweep = OrangeCollection;
          StateAfterSweep = 8;
          routine = LaneSweep;
          state = 2;
          break;
      }
      break;

    case OpeningLowerLeft: // Purple first on MIDDLE.
      switch (state) {
        case 0: if (move.backward(100)) state++; break;
        case 1: // Open for capture at full rotor power.
          myservo.write(openGate);
          enableControlSlowShot();
          enableDrivers();
          if (move.stopForMillis(mili)) state++;
          break;
        case 2: // Capture purple + clearance; tune this distance.
          if (move.forward(550)) state++;
          break;
        case 3: // Store purple; immediately slow the rotor.
          myservo.write(closedGate);
          enableControlSlowShot();
          if (move.stopForMillis(mili)) state++;
          break;
        case 4: // Collect the next orange at minimum power.
          if (move.forward(250)) state++;
          break;
        case 5: if (move.forward(350)) state++; break;
        case 6: // Eject towards OUTER, then sweep INNER.
          lane = MIDDLE;
          RoutineAfterSweep = OrangeCollection;
          StateAfterSweep = 8;
          routine = LaneSweep;
          state = 2;
          break;
      }
      break;

    case OpeningLowerRight: // Purple first on INNER.
      switch (state) {
        case 0: if (move.right(200)) state++; break;
        case 1: if (move.backward(100)) state++; break;
        case 2: // Open for capture at full rotor power.
          myservo.write(openGate);
          enableControlSlowShot();
          enableDrivers();
          if (move.stopForMillis(mili)) state++;
          break;
        case 3: // Capture purple + clearance; tune this distance.
          if (move.forward(550)) state++;
          break;
        case 4: // Store purple; immediately slow the rotor.
          myservo.write(closedGate);
          enableControlSlowShot();
          if (move.stopForMillis(mili)) state++;
          break;
        case 5: // Collect the next orange at minimum power.
          if (move.forward(250)) state++;
          break;
        case 6: if (move.forward(350)) state++; break;
        case 7: // Eject towards INNER, then sweep MIDDLE.
          lane = INNER;
          RoutineAfterSweep = OrangeCollection;
          StateAfterSweep = 8;
          routine = LaneSweep;
          state = 2;
          break;
      }
      break;

    // No-purple opening, then collection shared by all five cases.
    case OrangeCollection:
      switch (state) {
        case 0:
          if (move.backward(100)) {
            headingZero(); state++;
          }
          break;
        case 1:
          myservo.write(closedGate);
          enableControlSlowShot();
          if (move.stopForMillis(mili)) state++;
          break;
        case 2:
          if (move.forward(505)) state++;
          break;
        case 3: // LEFT side switch faces the OUTER wall.
          if (move.left(750)) state++;
          break;
        case 4:
          myservo.write(closedGate);
          if (move.stopForMillis(GateCloseMs)) state++;
          break;
        case 5:
          enableControlEjection();
          if (move.stopForMillis(OrangeEjectMs)) state++;
          break;
        case 6:
          enableControlSlowShot();
          if (move.backward(lenght + 250)) {
            headingZero(); state++;
          }
          break;
        case 7:
          if (move.left(180)) state++;
          break;
        case 8: // Sweep the other row of initial oranges.
          if (EarlyGameExpired) { BeginNormalLoop(); break; }
          lane = CollectionLane == OUTER ? INNER : MIDDLE;
          RoutineAfterSweep = OrangeCollection;
          StateAfterSweep = 9;
          routine = LaneSweep;
          state = 0;
          break;
        case 9: // Rear OUTER reference: ignore the collection lane.
          if (EarlyGameExpired) { BeginNormalLoop(); break; }
          laneWeights[0] = laneWeights[1] = laneWeights[2] = 0;
          enableControlSlowShot();
          state++;
          break;
        case 10:
          pixy.ccc.getBlocks();
          for (int I = 0; I < pixy.ccc.numBlocks; I++) {
            Block &Blk = pixy.ccc.blocks[I];
            if ((int)Blk.m_signature != orangeSignature) continue;
            int laneIndex = constrain(classifyLane(Blk.m_x, Blk.m_y, false), 0, NUM_FRANJAS - 1);
            if (laneIndex != CollectionLane)
              laneWeights[laneIndex] += (int)Blk.m_width * (int)Blk.m_height;
          }
          if (move.stopForMillis(mili / 2)) {
            if (EarlyGameExpired) { BeginNormalLoop(); break; }
            if (CollectionLane == OUTER)
              lane = (laneWeights[2] > laneWeights[1]) ? INNER : MIDDLE;
            else
              lane = (laneWeights[0] > laneWeights[1]) ? OUTER : MIDDLE;
            RoutineAfterSweep = CollectionCornerCheck;
            StateAfterSweep = 0;
            routine = LaneSweep;
            state = 0;
          }
          break;
      }
      break;

    // Full sweep; rear OUTER is the reference for the camera and side switch.
    case LaneSweep:
      switch (state) {
        case 0:
          if (lane == INNER) {
            if (move.right(360)) state++;
          } else if (lane == MIDDLE) {
            if (move.right(180)) state++;
          } else state++;
          break;
        case 1:
          enableControlSlowShot();
          if (move.forwardRegulated(lenght + 50) == 1) state++;
          break;
        case 2: disableDrivers(); if (move.stopForMillis(mili)) state++; break;
        case 3: // Face the collection corner: OUTER left, INNER right.
          if (move.rotate(166, CollectionLane == OUTER)) state++;
          break;
        case 4: myservo.write(closedGate); if (move.stopForMillis(GateCloseMs)) state++; break;
        case 5:
          enableControlEjection();
          if (move.stopForMillis(OrangeEjectMs)) state++;
          break;
        case 6:
          enableControlSlowShot();
          state++;
          break;
        case 7: // Restore the forward heading.
          if (move.rotate(166, CollectionLane == INNER)) state++;
          break;
        case 8:
          if (move.backward(lenght + 50)) {
            headingZero(); state++;
          }
          break;
        case 9:
          if (lane == INNER) {
            if (move.left(360)) state++;
          } else if (lane == MIDDLE) {
            if (move.left(180)) state++;
          } else state++;
          break;
        case 10:
          if (move.backward(100)) {
            headingZero(); state++;
          }
          break;
        case 11:
          if (move.left(180)) state++;
          break;
        case 12:
          routine = RoutineAfterSweep;
          state = StateAfterSweep;
          break;
      }
      break;

    case CollectionCornerCheck:
      switch (state) {
        case 0: // Decide at the rear reference, before moving.
          if (EarlyGameExpired) { BeginNormalLoop(); break; }
          enableControlSlowShot();
          state++;
          break;
        case 1: // Reach the selected collection lane.
          if (CollectionLane == INNER) {
            if (move.right(360)) state++;
          } else state++;
          break;
        case 2: // Short corner check; tune 550 mm here.
          if (move.forward(550)) state++;
          break;
        case 3: myservo.write(closedGate); if (move.stopForMillis(GateCloseMs)) state++; break;
        case 4:
          enableControlEjection();
          if (move.stopForMillis(OrangeEjectMs)) state++;
          break;
        case 5:
          enableControlSlowShot();
          if (move.backward(550)) state++;
          break;
        case 6:
          if (move.backward(100)) {
            headingZero(); state++;
          }
          break;
        case 7: // Reset against the left wall, also after an INNER check.
          if (CollectionLane == INNER) {
            if (move.left(540)) state++;
          } else {
            if (move.left(180)) state++;
          }
          break;
        case 8: routine = OrangeCollection; state = 9; break;
      }
      break;

    // Compact LEFT version of general routine 4.  It keeps the familiar
    // outer-wall pass, then calls the same camera-return routine.
    case MainLoop:
      switch (state) {
        case -7: // Final shot on INNER, starting from the rear OUTER reference.
          if (move.right(360)) state = 0;
          break;
        case -1: disableDrivers(); if (move.rotate(166, true)) state--; break;
        case -2: if (move.outer(95)) state--; break;
        case -3: if (move.forwardp(400, false)) state--; break;
        case -4: enableDrivers(); if (move.stopForMillis(mili)) state--; break;
        case -5: disableDrivers(); if (move.rotate(166, false)) state--; break;
        case -6: if (move.outer(30)) state = 0; break;
        case 0: enableDrivers(); if (move.backward(280)) state++; break;
        case 1: if (move.stopForMillis(mili)) state++; break;
        case 2: {
          int Result = lane == OUTER ? move.forwardp(lenght + 50, false)
                                     : move.forwardRegulated(lenght + 50);
          // Keep standard power through the accumulated corner at 45 s.
          if (Result == 2 && !FinalShotPending) enableSlowDrivers();
          if (Result == 1) state++;
          break;
        }
        case 3:
          if (move.stopForMillis(FinalShotPending ? 3000 : mili)) state++;
          break;
        case 4: if (move.stopForMillis(mili)) state++; break;
        case 5:
          FinalShotPending = false;
          enableDrivers();
          if (lane == OUTER) lane = MIDDLE;
          else if (lane == MIDDLE) lane = INNER;
          else lane = OUTER;
          routine = ReturnAndClassify;
          state = 0;
          break;
      }
      break;

    // General's return-and-camera decision, kept short for the LEFT robot.
    case ReturnAndClassify:
      switch (state) {
        case 0: if (move.backward(lenght + 250)) state++; break;
        case 1: if (move.stopForMillis(mili)) state++; break;
        case 2: if (move.outer(750)) state++; break;
        case 3: if (move.stopForMillis(mili)) state++; break;
        case 4: if (move.inner(180)) state++; break;
        case 5: if (move.backward(200)) state++; break;
        case 6:
          pixy.ccc.getBlocks();
          for (int I = 0; I < pixy.ccc.numBlocks; I++) {
            Block &Blk = pixy.ccc.blocks[I];
            if ((int)Blk.m_signature == orangeSignature) {
              int laneIndex = constrain(classifyLane(Blk.m_x, Blk.m_y, false), 0, NUM_FRANJAS - 1);
              laneWeights[laneIndex] += (int)Blk.m_width * (int)Blk.m_height;
            }
          }
          if (move.stopForMillis(mili / 2)) state++;
          break;
        case 7: {
          int Best = 0;
          for (int I = 1; I < NUM_FRANJAS; I++) if (laneWeights[I] > laneWeights[Best]) Best = I;
          lane = Best == 0 ? OUTER : (Best == 1 ? MIDDLE : INNER);
          laneWeights[0] = laneWeights[1] = laneWeights[2] = 0;
          routine = MainLoop;
          state = lane == OUTER ? -1 : 0;
          break;
        }
      }
      break;

    default:
      move.stop();
      break;
  }
}
