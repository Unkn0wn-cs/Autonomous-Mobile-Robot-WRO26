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
  OuterCornerCheck  = 13
};

enum ExpectedSwitch { ExpectNoSwitch, ExpectBackSwitch, ExpectSideSwitch };

int routine = MainLoop;
int state   = 0;
bool first = false;
rlane lane = OUTER;
int connections = 0;
int pesos[NUM_FRANJAS] = {0};
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

// Initial values from the nearest proven movements in generalStrategy.cpp.
static const int InitialBackAlignmentMM = 100;
static const int UpperLeftAdvanceMM = 505;
static const int OuterWallSearchMM = 750;
static const int RedundantWallSearchMM = 180;
static const int LaneSpacingMM = 180;
static const int FullLaneExtraMM = 50;
static const int CornerCheckMM = 550;
static const float QuarterTurnDeg = 90.0f;
static const unsigned long TurnTimeoutMs = 2500;

static ExpectedSwitch ExpectedContact = ExpectNoSwitch;
static int ExpectedContactNextState = 0;
static bool EarlyGameExpired = false;
static int SelectedLaneOffsetMM = LaneSpacingMM;
static int RoutineAfterSweep = OrangeCollection;

static void ExpectContact(ExpectedSwitch Contact, int NextState) {
  ExpectedContact = Contact;
  ExpectedContactNextState = NextState;
}

static void SetRotorMinimum() {
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);
  analogWrite(enable34, RotorMinPwm);
}

static void SetRotorReverseFull() {
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);
  analogWrite(enable34, RotorReversePwm);
}

static void StopRotor() { analogWrite(enable34, 0); }

static bool RotateQuarterTurn(bool Clockwise) {
  static bool Turning = false;
  static bool ActiveClockwise = false;
  static unsigned long TurnStarted = 0;

  if (!headingAvailable()) {
    move.stop();
    Turning = false;
    return false;
  }
  if (!Turning || ActiveClockwise != Clockwise) {
    headingZero();
    ActiveClockwise = Clockwise;
    TurnStarted = millis();
    Turning = true;
  }
  if (Clockwise) move.rotateCW(pwms[0], pwms[1], pwms[2], pwms[3]);
  else           move.rotateCCW(pwms[0], pwms[1], pwms[2], pwms[3]);

  float Turned = headingSinceZero();
  bool Reached = Clockwise ? Turned >= QuarterTurnDeg : Turned <= -QuarterTurnDeg;
  if (!Reached && millis() - TurnStarted < TurnTimeoutMs) return false;
  move.stop();
  Turning = false;
  return true;
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
  ExpectedContact = ExpectNoSwitch;
  lane = OUTER;
  first = false;
  SetRotorMinimum();
  routine = MainLoop;
  state = -1;
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
  int BallZone = DetectBallZone();

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
    if (ExpectedContact == ExpectBackSwitch &&
        CurrentBackSwitchState == LOW && LastBackSwitchState == HIGH) {
      MicroSwitchTime = CurrentTime;
      ExpectedContact = ExpectNoSwitch;
      move.stop();
      headingZero();
      state = ExpectedContactNextState;
      Serial.println(F("Heading zeroed by back switch"));
    }
    if (ExpectedContact == ExpectSideSwitch &&
        CurrentSideSwitchState == LOW && LastSideSwitchState == HIGH) {
      MicroSwitchTime = CurrentTime;
      ExpectedContact = ExpectNoSwitch;
      move.stop();
      state = ExpectedContactNextState;
    }
  }
  LastBackSwitchState = CurrentBackSwitchState;
  LastSideSwitchState = CurrentSideSwitchState;
}

// ---------------------------------------------------------------------------
// updateEndgameTiming  (stub - will gate the 45 s transition)
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
    // Purple openings stay safe until their own routines are added.
    case OpeningUpperLeft:
    case OpeningUpperRight:
    case OpeningLowerLeft:
    case OpeningLowerRight:
      ExpectedContact = ExpectNoSwitch;
      move.stop();
      SetRotorMinimum();
      break;

    // No-purple opening, then the first mandatory INNER sweep.
    case OrangeCollection:
      switch (state) {
        case 0:
          ExpectContact(ExpectBackSwitch, 1);
          if (move.backward(InitialBackAlignmentMM)) {
            ExpectedContact = ExpectNoSwitch; headingZero(); state++;
          }
          break;
        case 1:
          myservo.write(closedGate);
          SetRotorMinimum();
          if (move.stopForMillis(mili)) state++;
          break;
        case 2:
          if (move.forward(UpperLeftAdvanceMM)) state++;
          break;
        case 3: // LEFT side switch faces the OUTER wall.
          ExpectContact(ExpectSideSwitch, 4);
          if (move.left(OuterWallSearchMM)) {
            ExpectedContact = ExpectNoSwitch; state++;
          }
          break;
        case 4:
          myservo.write(closedGate);
          if (move.stopForMillis(GateCloseMs)) state++;
          break;
        case 5:
          SetRotorReverseFull();
          if (move.stopForMillis(OrangeEjectMs)) state++;
          break;
        case 6:
          SetRotorMinimum();
          ExpectContact(ExpectBackSwitch, 7);
          if (move.backward(lenght + 250)) {
            ExpectedContact = ExpectNoSwitch; headingZero(); state++;
          }
          break;
        case 7:
          ExpectContact(ExpectSideSwitch, 8);
          if (move.left(RedundantWallSearchMM)) {
            ExpectedContact = ExpectNoSwitch; state++;
          }
          break;
        case 8:
          lane = INNER;
          SelectedLaneOffsetMM = 2 * LaneSpacingMM;
          RoutineAfterSweep = OrangeCollection;
          routine = LaneSweep;
          state = 0;
          break;
        case 9: // Safe corner: scan only MIDDLE and INNER.
          if (EarlyGameExpired) { BeginNormalLoop(); break; }
          pesos[0] = pesos[1] = pesos[2] = 0;
          SetRotorMinimum();
          state++;
          break;
        case 10:
          pixy.ccc.getBlocks();
          for (int I = 0; I < pixy.ccc.numBlocks; I++) {
            Block &Blk = pixy.ccc.blocks[I];
            if ((int)Blk.m_signature != orangeSignature) continue;
            int Franja = constrain(classifyLane(Blk.m_x, Blk.m_y, false), 0, NUM_FRANJAS - 1);
            if (Franja == 1 || Franja == 2)
              pesos[Franja] += (int)Blk.m_width * (int)Blk.m_height;
          }
          if (move.stopForMillis(mili / 2)) {
            lane = (pesos[2] > pesos[1]) ? INNER : MIDDLE;
            SelectedLaneOffsetMM = lane == INNER ? 2 * LaneSpacingMM : LaneSpacingMM;
            RoutineAfterSweep = OuterCornerCheck;
            routine = LaneSweep;
            state = 0;
          }
          break;
      }
      break;

    // Reusable full forward/backward sweep.  It always starts and ends at
    // the rear OUTER corner, which keeps competition tuning local to constants.
    case LaneSweep:
      switch (state) {
        case 0: if (move.right(SelectedLaneOffsetMM)) state++; break;
        case 1:
          SetRotorMinimum();
          if (move.forwardRegulated(lenght + FullLaneExtraMM) == 1) state++;
          break;
        case 2: StopRotor(); if (move.stopForMillis(mili)) state++; break;
        case 3: if (RotateQuarterTurn(false)) state++; break;
        case 4: myservo.write(closedGate); if (move.stopForMillis(GateCloseMs)) state++; break;
        case 5: SetRotorReverseFull(); if (move.stopForMillis(OrangeEjectMs)) state++; break;
        case 6: SetRotorMinimum(); state++; break;
        case 7: if (RotateQuarterTurn(true)) state++; break;
        case 8:
          ExpectContact(ExpectBackSwitch, 9);
          if (move.backward(lenght + FullLaneExtraMM)) {
            ExpectedContact = ExpectNoSwitch; headingZero(); state++;
          }
          break;
        case 9:
          ExpectContact(ExpectSideSwitch, 10);
          if (move.left(SelectedLaneOffsetMM)) {
            ExpectedContact = ExpectNoSwitch; state++;
          }
          break;
        case 10:
          ExpectContact(ExpectBackSwitch, 11);
          if (move.backward(InitialBackAlignmentMM)) {
            ExpectedContact = ExpectNoSwitch; headingZero(); state++;
          }
          break;
        case 11:
          ExpectContact(ExpectSideSwitch, 12);
          if (move.left(RedundantWallSearchMM)) {
            ExpectedContact = ExpectNoSwitch; state++;
          }
          break;
        case 12:
          routine = RoutineAfterSweep;
          state = RoutineAfterSweep == OrangeCollection ? 9 : 0;
          break;
      }
      break;

    case OuterCornerCheck:
      switch (state) {
        case 0:
          if (EarlyGameExpired) { BeginNormalLoop(); break; }
          SetRotorMinimum();
          if (move.forward(CornerCheckMM)) state++;
          break;
        case 1: myservo.write(closedGate); if (move.stopForMillis(GateCloseMs)) state++; break;
        case 2: SetRotorReverseFull(); if (move.stopForMillis(OrangeEjectMs)) state++; break;
        case 3: SetRotorMinimum(); if (move.backward(CornerCheckMM)) state++; break;
        case 4:
          ExpectContact(ExpectBackSwitch, 5);
          if (move.backward(InitialBackAlignmentMM)) {
            ExpectedContact = ExpectNoSwitch; headingZero(); state++;
          }
          break;
        case 5:
          ExpectContact(ExpectSideSwitch, 6);
          if (move.left(RedundantWallSearchMM)) {
            ExpectedContact = ExpectNoSwitch; state++;
          }
          break;
        case 6: routine = OrangeCollection; state = 9; break;
      }
      break;

    // Compact LEFT version of general routine 4.  It keeps the familiar
    // outer-wall pass, then calls the same camera-return routine.
    case MainLoop:
      switch (state) {
        case -1: disableDrivers(); if (RotateQuarterTurn(false)) state--; break;
        case -2: if (move.outer(95)) state--; break;
        case -3: if (move.forwardp(400, false)) state--; break;
        case -4: enableDrivers(); if (move.stopForMillis(mili)) state--; break;
        case -5: disableDrivers(); if (RotateQuarterTurn(true)) state--; break;
        case -6: if (move.outer(30)) state = 0; break;
        case 0: enableDrivers(); if (move.backward(280)) state++; break;
        case 1: if (move.stopForMillis(mili)) state++; break;
        case 2: {
          int Result = lane == OUTER ? move.forwardp(lenght + 50, false)
                                     : move.forwardRegulated(lenght + 50);
          if (Result == 2) SetRotorMinimum();
          if (Result == 1) state++;
          break;
        }
        case 3: if (move.stopForMillis(mili)) state++; break;
        case 4: if (move.stopForMillis(mili)) state++; break;
        case 5:
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
              int Franja = constrain(classifyLane(Blk.m_x, Blk.m_y, false), 0, NUM_FRANJAS - 1);
              pesos[Franja] += (int)Blk.m_width * (int)Blk.m_height;
            }
          }
          if (move.stopForMillis(mili / 2)) state++;
          break;
        case 7: {
          int Best = 0;
          for (int I = 1; I < NUM_FRANJAS; I++) if (pesos[I] > pesos[Best]) Best = I;
          lane = Best == 0 ? OUTER : (Best == 1 ? MIDDLE : INNER);
          pesos[0] = pesos[1] = pesos[2] = 0;
          routine = MainLoop;
          state = lane == OUTER ? -1 : 0;
          break;
        }
      }
      break;

    default:
      ExpectedContact = ExpectNoSwitch;
      move.stop();
      break;
  }
}
