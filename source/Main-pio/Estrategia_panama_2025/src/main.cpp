    int LED = 34;

#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "move.h"
#include <Pixy2.h>

//structs

enum rlane {
  OUTER, 
  MIDDLE,
  INNER
}; rlane lane = MIDDLE;

enum side {
  RIGHT,
  LEFT
}; 



//move
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
//LEFT - WALL
  int pwmf[4] = {245, 236, 236, 245};
  int pwms[4] = {220, 225, 220, 225};
  const long pulses = 900; // Number of pulses for each movement step
  side robotSide = LEFT;
  int slowRotorSpeed = 90; 
  int lenght = 0;
  int closedGate = 170;
  int openGate = 55;
  //Position of the purple Balls 🟣🟣🟣 -> see ballZonesLeft in the Pixy Cam section below

//RIGHT - RAMP
  //  int pwmf[4] = {230, 243, 243, 230};
  //  int pwms[4] = {200, 200, 200, 200};
  // const long pulses = 1650; // Number of pulses for each movement step
  // side robotSide = RIGHT;
  // int slowRotorSpeed = 180;
  // int closedGate = 96;
  // int openGate = 0;
  //Position of the purple Balls 🟣🟣🟣 -> see ballZonesRight in the Pixy Cam section below

  

Encoders encoderLeft(A15, A14);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders encoderRight(A13 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 
Move move(
  motor1, motor2, motor3, motor4, encoderLeft, encoderRight,
  pwmf[0], pwmf[1], pwmf[2], pwmf[3],      // Forward/backward PWM values
  pwms[0], pwms[1], pwms[2], pwms[3]       // Left/right/diagonal PWM values
);

int startTime;
//define objects----------------------------------------
MPU6050 sensor;
Pixy2 pixy;
Servo myservo;

bool cam;
bool mpu;

//Pixy Cam---------------------------------------------
int purpleSignature = 2;
int orangeSignature = 1;
int redSignature = 3;

const int NUM_FRANJAS = 3;
const int ANCHO_IMAGEN = 316; // Pixy2 horizontal resolution
int pesos[NUM_FRANJAS] = {0};
int camera = 0;

// --- Purple ball position zones 🟣 --------------------------------------------------------
//
// The Pixy2 image is 316 x 208 and its ORIGIN IS THE TOP-LEFT corner, so x grows to the
// right and y grows DOWNWARD: a point that looks "lower" on screen has the LARGER y.
//
// Every ball position is one rectangle written as two OPPOSITE CORNERS:
//                              { xA, yA, xB, yB }
// The order of the two corners does not matter - the scorer normalises to min/max - so
// {lower-left, upper-right} and {upper-left, bottom-right} are both accepted. Write down
// whichever pair is easier to read off PixyMon.
//
// HOW TO CALIBRATE
//   1. Put a purple ball at one position and leave the robot exactly where it starts.
//   2. Read the block Pixy reports (PixyMon, or the "blob" line this code prints on Serial
//      which already gives you the four edges: left, top, right, bottom).
//   3. Enter a rectangle that CONTAINS that blob. You do not need to pad it by hand - the
//      code already allows BALL_ZONE_TOLERANCE px of slack on every side (see the sensitivity
//      block below), so write down what you actually measured.
//   4. Zones may overlap or touch; the scorer picks the best match, not the first one, and
//      grades near-misses by distance so the closer rectangle always wins. What must NOT
//      happen is a zone stretching over a neighbouring ball position.
//   5. Anything outside every zone (plus its tolerance) is treated as noise and ignored, so
//      keep the rectangles away from purple-ish reflections on the mat or the walls.
//
// The row index IS the routine that will be executed for that ball:
//   row 0 -> routine 0 : ball high & left  in the image ("upper left")
//   row 1 -> routine 1 : ball high & right in the image ("upper right", strafes right 200 mm first)
//   row 2 -> routine 2 : ball low  & left  in the image ("lower left")
//   row 3 -> routine 3 : ball low  & right in the image ("lower right", strafes right 200 mm first)
// No match at all leaves routine 4, the normal no-ball lane loop.
//
// A row left as { 0, 0, 0, 0 } is an empty rectangle: it has no area, so it can never match
// anything. An un-filled row therefore just means "this ball position is never chosen" and
// the robot falls through to routine 4 - it will never guess a position it has no numbers
// for.

const int NUM_BALL_ZONES = 4;

// LEFT / WALL robot - measured values.
const int ballZonesLeft[NUM_BALL_ZONES][4] = {
  { 135,  20,  160,  0  },   // routine 0 - upper left
  { 235,  25,  260,  10 },   // routine 1 - upper right
  { 135,  40,  160,  20 },   // routine 2 - lower left
  { 260,  55,  290,  25 }    // routine 3 - lower right
};

// RIGHT / RAMP robot - measured values.
const int ballZonesRight[NUM_BALL_ZONES][4] = {
  { 160,  20,  200,  32 },   // routine 0 - upper left
  { 200,  20,  300,  32 },   // routine 1 - upper right
  { 105,  32,  200,  90 },   // routine 2 - lower left
  { 200,  32,  300,  90 }    // routine 3 - lower right
};

// --- Ball detector sensitivity ------------------------------------------------------------
// Tuned to be as permissive as possible WITHOUT letting specks of dust vote: the quality
// gates below only reject things that cannot physically be the ball (too small, too thin,
// only seen for a single frame), while the multi-frame vote is what actually guarantees the
// decision is correct.
const uint8_t BALL_MIN_AGE     = 2;    // camera frames the blob must have been tracked for.
                                       //   Was 10, which cost ~165 ms before a ball counted.
                                       //   2 only rejects one-frame flicker; the vote below
                                       //   does the real confirming.
const int  BALL_MIN_SIDE       = 5;    // px. Blob must be at least this wide AND this tall.
const long BALL_MIN_AREA       = 45;   // px^2. Anti-dust floor - a real ball is far bigger.
const int  BALL_MAX_ASPECT_X10 = 30;   // 3.0 : rejects long thin smears / glare streaks.
                                       //   Deliberately lenient so a ball clipped by the
                                       //   frame edge still passes.
const int  BALL_ZONE_TOLERANCE = 20;   // px of slack around every measured rectangle, on all
                                       //   four sides. A ball whose centre lands outside a
                                       //   zone still matches it, so the numbers you type in
                                       //   do not have to be perfect.
                                       //   It is NOT a plain box inflation: matches inside the
                                       //   band are graded by distance, so when two zones are
                                       //   both within reach the closer one wins outright
                                       //   instead of the two fighting. That is what keeps
                                       //   touching zones - like LEFT rows 0 and 2, which
                                       //   share the edge y=20 - unambiguous no matter how
                                       //   wide this gets. See scoreBallZone().
                                       //   Widening this only costs false-POSITIVE margin (a
                                       //   stray purple object further from a real position
                                       //   can now reach a zone), never zone-vs-zone accuracy.
                                       //   BALL_CONFIRM_SOFT below is what pays that back.
const long BALL_MIN_SCORE      = 20;   // see scoreBallZone(): >=200 means "centre inside a
                                       //   zone", 100..199 "centre within tolerance of one",
                                       //   20..99 "clearly overlapping one".
const int  BALL_CONFIRM_FRAMES = 3;    // frames one zone must win before we commit, when that
                                       //   zone has had at least one solid hit (centre truly
                                       //   inside the measured rectangle, score >= 200).
const int  BALL_CONFIRM_SOFT   = 5;    // ...but this many when ALL the evidence came from the
                                       //   tolerance band. Evidence that leans on the slack is
                                       //   held to a higher bar, which is what lets the
                                       //   tolerance above be generous without getting loose.
const int  BALL_VOTE_MARGIN    = 2;    // frames the leader must lead the rest by, either way.
const unsigned long BALL_SCAN_MS   = 900;  // hard ceiling on the whole scan (was ~3.5 s).
const unsigned long BALL_GIVEUP_MS = 350;  // if NOTHING ball-like has been seen by now, stop
                                           //   early so an empty field costs almost nothing.

// --- Microswitch Gyro Reset Extension ----------------------------------------------------------------

const byte backSwitchPin = 18; // For interrupt on Mega
const byte sideSwitchPin = 19; // For interrupt on Mega
const byte switchPin = 14; 


volatile bool backSwitchPressed = false;
volatile bool sideSwitchPressed = false;
bool lastBackSwitchState = HIGH;    // for edge detection
bool lastSideSwitchState = HIGH;    // for edge detection

//Rotor variables -------------------------------------------------------------------------
const int enable34 = 9; // L293D pin 9
const int input4   = 46; // L293D pin 10
const int input3   = 48; // L293D pin 15

// Declaración de variables para la velocidad
int velocidad = 210; // Valor inicial de velocidad (0 a 255)

// --- Buzzer debug 🔊 -----------------------------------------------------------------------
// DEBUG ONLY. Beeps out which routine the ball detector picked, so you can check the camera
// decision without a serial cable attached.
//
// Wiring: buzzer + on pin 47, buzzer - on pin 46.
//   Pin 46 is input4 above, the L293D rotor direction line. setup() drives it LOW and never
//   touches it again, so it works as a permanent ground for the buzzer. If the rotor logic
//   is ever changed to drive input4 HIGH, move the buzzer's negative leg to a real GND pin.
//
// TO TURN IT OFF: set BUZZER_DEBUG to 0. Every buzzer call then compiles away to nothing -
// no flash, no RAM, no delay - so it cannot interfere with a competition run. Leave it at 0
// for real runs: the announcement blocks for up to ~1.5 s and it happens AFTER startTime is
// set, so it does eat into the strategy timers in loop().
#define BUZZER_DEBUG    1      // 1 = beep the routine, 0 = compiled out entirely
#define BUZZER_PIN      47
#define BUZZER_TONE_HZ  2500   // square wave bit-banged in software; audible on both passive
                               //   and active buzzers. Set to 0 to hold the pin flat HIGH
                               //   instead, which is the cleaner drive for an ACTIVE buzzer.
                               //   NOTE: tone() is deliberately NOT used here - on the Mega
                               //   it owns Timer2, and AFMotor already uses Timer2 for the
                               //   motor PWM, so tone() would fight the drive train.

// variables for GyroScope ----------------------------------------------------------------------------
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

//Variables usadas por el filtro pasa bajos
  long f_ax,f_ay, f_az;
  int p_ax, p_ay, p_az;
  long f_gx,f_gy, f_gz;
  int p_gx, p_gy, p_gz;

  //Valor de los offsets
  int ax_o,ay_o,az_o;
  int gx_o,gy_o,gz_o;

  long tiempo_prev;
  float dt;
  float ang_x, ang_y, ang_z; // Added ang_z for Z-axis rotation
  float ang_x_prev, ang_y_prev, ang_z_prev; // Added ang_z_prev for Z-axis rotation
//

//---------------------------------
static int routine = 4;
int state = 0;
bool first = true;

int beta = 8; //degree error
int alpha = 0;

int connections;

const int mili = 250; //delay
const int diameter = 60; //Diameter of the wheel in mm

// functions-----------------------------------------------------------------------------

int mm(int mm){
  return(move.mmToPulses(mm, diameter, pulses));
}

bool inner(int mili){
  if (robotSide == RIGHT){
    if(move.left(mm(mili))) return true;
  }else if (robotSide == LEFT){
    if(move.right(mm(mili)))return true;
  }
  return false;
}

bool outer(int mili){
  if (robotSide == RIGHT){
    if(move.right(mm(mili)))return true;
  }else if (robotSide == LEFT){
    if(move.left(mm(mili))) return true;
  }
  return false;
}

int testI2C() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } 
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan done\n");
  }

  return nDevices; // return how many devices were found
}

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

    Serial.print("antes de la formula ");
    Serial.println(yA);

    if (y < yA) {
        return 0; // Lane 1
    } else if (y >= yA && y < yB) {
        return 2; // Lane 2
    } else {
        return 1; // Lane 3
    }
}
// --- Buzzer debug helpers 🔊 ---------------------------------------------------------------
// Everything in here disappears when BUZZER_DEBUG is 0. See the wiring / on-off notes next to
// the BUZZER_PIN define at the top of the file.
#if BUZZER_DEBUG

void buzzerBeep(unsigned int onMs) {
#if BUZZER_TONE_HZ > 0
  // Bit-banged square wave. Blocking, but this only ever runs from setup().
  unsigned long halfPeriod = 500000UL / (unsigned long)BUZZER_TONE_HZ;   // microseconds
  unsigned long cycles     = ((unsigned long)onMs * 1000UL) / (halfPeriod * 2UL);
  for (unsigned long i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(halfPeriod);
  }
#else
  digitalWrite(BUZZER_PIN, HIGH);
  delay(onMs);
#endif
  digitalWrite(BUZZER_PIN, LOW);
}

// Announces a number: one long lead-in beep, then `count` short beeps.
// The lead-in is there so that count == 0 (routine 0) is still clearly distinguishable from
// the buzzer simply not working.
void buzzerAnnounce(int count) {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  buzzerBeep(300);
  delay(300);

  for (int i = 0; i < count; i++) {
    buzzerBeep(100);
    delay(160);
  }
}

#else
  #define buzzerAnnounce(count)  do {} while (0)
#endif

// --- Purple ball detection 🟣 --------------------------------------------------------------
//
// Replaces the old "first purple blob to cross a centre line wins" logic. That version could
// be fooled by a single bad frame and, because it demanded m_age > 10 while giving up after
// 10 empty frames, it could also abort before a real ball ever became eligible.
//
// The new detector works on the BALL'S EDGES instead of just its centroid, and confirms the
// answer over several frames:
//
//   1. Quality gate - a blob must be purple, tracked for a couple of frames, big enough and
//      roughly round. This is the "not a grain of dust" filter.
//   2. Zone scoring - the blob's bounding box is scored against every calibrated rectangle,
//      with BALL_ZONE_TOLERANCE px of graded slack around each one. A blob matching no zone
//      is DISCARDED rather than being forced into a quadrant, which is what stops the robot
//      from "detecting the ball where it isn't".
//   3. Voting - only the largest valid blob of each frame votes, so the ball and a speck can
//      never both score in the same frame.
//   4. Early commit - the scan stops the moment one zone is clearly ahead, so a clean ball is
//      decided in roughly 5 camera frames (~85 ms) instead of the old 10+ forced frames.

// Area of the overlap between two axis-aligned rectangles; 0 when they do not touch.
// Returns long because a full-frame box (316 x 208) overflows a 16-bit AVR int.
long rectOverlap(int aL, int aT, int aR, int aB, int bL, int bT, int bR, int bB) {
  int w = min(aR, bR) - max(aL, bL);
  int h = min(aB, bB) - max(aT, bT);
  if (w <= 0 || h <= 0) return 0;
  return (long)w * (long)h;
}

// Scores one blob box against one zone. Higher is better, 0 means no relation.
// The result falls into three tiers that never cross, so the ranking between zones is
// always well defined:
//
//   200..300 : the blob's centre is INSIDE the measured rectangle. Strongest evidence.
//              The 0..100 on top is the box overlap, which is what separates two zones
//              that both contain the centre (touching zones) - the rectangle holding more
//              of the ball wins, which is the physically correct answer.
//   100..199 : the centre is OUTSIDE the rectangle but within BALL_ZONE_TOLERANCE of it.
//              Graded by how far outside it is, so when two zones are both in reach the
//              CLOSER one always wins outright - never a coin flip between neighbours.
//     0..100 : the centre is well outside, but the boxes still overlap. Normalised by the
//              SMALLER of the two boxes so a wide zone and a tight, ball-sized zone are
//              judged on the same scale.
long scoreBallZone(int bL, int bT, int bR, int bB, long blobArea, const int zone[4]) {
  int zL = min(zone[0], zone[2]);
  int zR = max(zone[0], zone[2]);
  int zT = min(zone[1], zone[3]);
  int zB = max(zone[1], zone[3]);

  // An un-filled { 0, 0, 0, 0 } row has no area and must never match anything. This check
  // has to happen BEFORE the tolerance is applied, otherwise the slack would turn an empty
  // row into a live region around the top-left corner of the image.
  if (zR - zL <= 0 || zB - zT <= 0) return 0;

  int cx = (bL + bR) / 2;
  int cy = (bT + bB) / 2;

  // How far the centre sits outside the rectangle on each axis; 0 when it is inside.
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
    // Inside the tolerance band. miss runs 1..2*tolerance, so the penalty runs 0..100 and
    // the tier stays between 100 and 199 - always above a pure-overlap match, always below
    // a centre that is genuinely inside some other zone.
    long miss    = (long)dx + (long)dy;
    long penalty = (miss * 100L) / (2L * (long)BALL_ZONE_TOLERANCE);
    return 200 - penalty - 1;
  }

  return overlapScore;                            // far outside, judged on overlap alone
}

const char *ballZoneName(int zone) {
  switch (zone) {
    case 0: return "upper left  -> routine 0";
    case 1: return "upper right -> routine 1";
    case 2: return "lower left  -> routine 2";
    case 3: return "lower right -> routine 3";
  }
  return "no ball -> routine 4";
}

// Returns the ball position index 0..3, or -1 when no ball could be confirmed.
int detectBallZone() {
  const int (*zones)[4] = (robotSide == RIGHT) ? ballZonesRight : ballZonesLeft;

  int  votes[NUM_BALL_ZONES]  = {0};
  long weight[NUM_BALL_ZONES] = {0};
  bool solid[NUM_BALL_ZONES]  = {false};   // zone has had >=1 centre-inside hit, not just slack
  bool seenAnything = false;

  unsigned long scanStart = millis();
  int committed = -1;

  while (millis() - scanStart < BALL_SCAN_MS) {

    // Non-blocking read. A negative result is BUSY (no new frame yet) or a link error;
    // either way it is NOT evidence of an empty field, so it must not count as a sample.
    // The old code conflated the two, which is what made its give-up counter misbehave.
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

      // Both edges of the blob. m_x / m_y are uint16_t, so cast BEFORE subtracting or a
      // blob near the left/top border wraps around to ~65000.
      int cx = (int)b.m_x;
      int cy = (int)b.m_y;
      int bL = cx - bw / 2;
      int bR = cx + bw / 2;
      int bT = cy - bh / 2;
      int bB = cy + bh / 2;

      int  bestZone  = -1;
      long bestScore = 0;
      for (int z = 0; z < NUM_BALL_ZONES; z++) {
        long score = scoreBallZone(bL, bT, bR, bB, area, zones[z]);
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
      // Nothing ball-like in this frame. Bail out early only while we have never seen a
      // ball at all - once there is evidence, keep sampling for the full budget.
      if (!seenAnything && millis() - scanStart > BALL_GIVEUP_MS) break;
      continue;
    }

    seenAnything = true;
    votes[frameZone]++;
    weight[frameZone] += frameArea;
    if (frameScore >= 200) solid[frameZone] = true;   // centre truly inside the rectangle

    Serial.print("blob edges L");
    Serial.print(frameL); Serial.print(" T"); Serial.print(frameT);
    Serial.print(" R");    Serial.print(frameR); Serial.print(" B"); Serial.print(frameB);
    Serial.print(frameScore >= 200 ? "  solid  -> " : "  in-tol -> ");
    Serial.println(ballZoneName(frameZone));

    // Leader and runner-up over everything collected so far. Ties break on accumulated
    // blob area, so the closer/bigger sighting wins.
    int leader = 0, runnerUp = -1;
    for (int z = 1; z < NUM_BALL_ZONES; z++) {
      if (votes[z] > votes[leader] ||
         (votes[z] == votes[leader] && weight[z] > weight[leader])) leader = z;
    }
    for (int z = 0; z < NUM_BALL_ZONES; z++) {
      if (z == leader) continue;
      if (runnerUp < 0 || votes[z] > votes[runnerUp]) runnerUp = z;
    }

    // A zone that has been hit dead-on at least once commits quickly. One that has only
    // ever matched through the tolerance slack has to prove itself over more frames - that
    // is the price of the generous BALL_ZONE_TOLERANCE, and it is only paid in the rare
    // case where the calibration is off enough that the ball never lands inside the box.
    int needed = solid[leader] ? BALL_CONFIRM_FRAMES : BALL_CONFIRM_SOFT;

    if (votes[leader] >= needed &&
        votes[leader] - votes[runnerUp] >= BALL_VOTE_MARGIN) {
      committed = leader;
      break;
    }
  }

  // Budget ran out without a clear winner: use the best evidence we have rather than
  // throwing away a ball we definitely saw.
  if (committed < 0) {
    for (int z = 0; z < NUM_BALL_ZONES; z++) {
      if (votes[z] == 0) continue;
      if (committed < 0 || votes[z] > votes[committed] ||
         (votes[z] == votes[committed] && weight[z] > weight[committed])) committed = z;
    }
  }

  Serial.print("ball scan ");
  Serial.print(millis() - scanStart);
  Serial.print(" ms  votes ");
  for (int z = 0; z < NUM_BALL_ZONES; z++) {
    Serial.print(votes[z]);
    Serial.print(' ');
  }
  Serial.print(" => ");
  Serial.println(ballZoneName(committed));

  return committed;
}

void enableSlowDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, slowRotorSpeed);
}
void enableDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, 254);   
}
void disableDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, 0);   
}
//----------------------------------------------------------------------------------------
int filterGyro(MPU6050_Base sensor){
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_z_prev = ang_z;
  return ang_z;
} 

void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
  tiempo_prev = millis(); 
}

void onSwitchPress() {
  backSwitchPressed = true;
  resetGyroAngles();
}


  void blink(){
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
        return;
  }

void setup() { //-----------------------------------------------------------------------------------------------------------------------------------///

  Serial.begin(9600);
  
  pixy.init();


  if(robotSide == RIGHT){
    lenght = 640;
  }else{
    lenght = 1100;
  } 
  //servo--------------------------------------------
  myservo.attach(10); 
  
  //rotor
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

    // LED DEBUGER SUPERIOR GRAN RESERVA PRO MAX ROJO TRUMP MAGA UNIMET #FORMAFALICA
    pinMode(LED, OUTPUT);

  // //mpu 
  Wire.begin();           // Iniciando I2C
  sensor.initialize();    // Iniciando el sensor

  if (sensor.testConnection()) {Serial.println("Sensor iniciado correctamente"); mpu = true;}
  else{Serial.println("Error al iniciar el sensor"); mpu = false;} 

  tiempo_prev = millis();
  resetGyroAngles();

  //microSwitch
  pinMode(backSwitchPin, INPUT_PULLUP);
  pinMode(sideSwitchPin, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);

  if (robotSide == LEFT){
    bool pinpressed = false;
    while(!pinpressed){
      if ((digitalRead(switchPin)) == HIGH){
        pinpressed = true;
      }
    }
  }

  startTime = millis();

  myservo.write(closedGate);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);
  enableDrivers();


  //Calculate purple position ---------------------------------------------------------------
  // The zone rectangles and the sensitivity constants live in the Pixy Cam section at the
  // top of this file; detectBallZone() confirms the answer over several camera frames and
  // returns the ball position, or -1 when there is no ball to chase.
  int ballZone = detectBallZone();
  if (ballZone >= 0) {
    routine = ballZone;   // routines 0..3 are the four ball-capture sequences
  }
  // routine stays 4 (the plain lane loop) when nothing was confirmed.

  pixy.setLamp(0, 0);

  // Beep out the chosen routine: long beep, then `routine` short beeps.
  // Debug aid only - set BUZZER_DEBUG to 0 at the top of the file to remove it completely.
  buzzerAnnounce(routine);


}

void loop() {//---------------------------------------------------------------------------------------------------------------------------------------------////

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
        Serial.println("Gyro angles reset by microswitch!");
      }
      if (currentSideSwitchState == LOW && lastSideSwitchState == HIGH && routine!= 4  && !(routine == 7 && state == 8)  ) {
        microSwitchTime = millis();
        // Button pressed
        state++;
      }
      lastBackSwitchState = currentBackSwitchState;
      lastSideSwitchState = currentSideSwitchState;
    }

  //

    // Last Routine Code ----------------------------------------------
  static bool lastRoutine = false;
  // if (lastRoutine == false &&  (millis() > 105000 + startTime) ){ //&& (routine != 7 && routine != 5)
  //   lastRoutine = true;
  //   lane = OUTER;
  //   lenght -= 30;
  // }
  static bool midRoutine = false;
  static bool midRoutineDone = false;
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
  int devices = testI2C();

  if (devices > 0) {
    const float GYRO_SENSITIVITY = 131.0; // MPU6050 scale factor for ±250°/s

    // Get filtered Z-axis rotation rate
    float gz_filtered = filterGyro(sensor); // Should return raw gyro Z in deg/s

    // Time delta in seconds
    unsigned long now = millis();
    dt = (now - tiempo_prev) / 1000.0;
    tiempo_prev = now;


    // Integrate angular velocity to get angle
    ang_z = ang_z_prev + (gz_filtered / GYRO_SENSITIVITY) * dt;
    ang_z_prev = ang_z;
  } else {
    Serial.println("I2C FAILED or no devices responding!");

    mpu = false;
  }


    
    // routine 0-3 = case 0-3 ball 
    // routine 4 Simple Lanes
    // routine 5 Diagonal Lane
    // routine 6 simple reset
    // routine 7 Corner checking reset
    // routine 8 Reorient itse
    // routine 9 Parring
    // routine 10 debugging


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
        // if(move.stopForMillis(mili)) 
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
        // if(move.stopForMillis(mili))
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
        // if(move.stopForMillis(mili)) 
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
        // if(move.stopForMillis(mili)) 
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
        if(outer(mm(20))) state--;
        break;
      case -3:
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
            // if(move.forward(mm(lenght + 50))) state++;  
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

                // if(move.forwardp(mm(lenght + 50), true) == 1) state++;  //no middle signal
              }else{
                int test = move.forwardp(mm(lenght + 50), false);
                if (test == 2){enableSlowDrivers();}
                switch(test){
                  case 1:
                    state++;
                    break;
                }
                
                // if(move.forwardp(mm(lenght + 100), false) == 1) state++;  
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
          // first = false;
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

            // Clasifica en franja según posición X
            int franja = classifyLane(block.m_x, block.m_y, true);
            franja = constrain(franja, 0, NUM_FRANJAS - 1);

            // Calcula tamaño como área
            int tamano = block.m_width * block.m_height;

            // Suma al peso de la franja
            pesos[franja] += tamano;
          }
        }
          camera++;
        } else {
          camera = 0;
        }

        if(move.stopForMillis(mili/2)) {
          // Encuentra la franja con mayor peso
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
          //Reset Camera weights
          for (int i = 0; i < NUM_FRANJAS; i++) {
            pesos[i] = 0;
          }

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

        if (mpu == true){
          if (robotSide == LEFT){
            alpha = -80;
          } else {alpha  = 80;}
          
          if (ang_z >= alpha + beta) {
              move.rotateCCW(200, 200, 200, 200);
          } else if (ang_z <= alpha - beta){
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
        // rotation = false;
        break;
      case 12:
        digitalWrite(LED, HIGH);

        if (mpu == true){
          if (robotSide == LEFT){
            alpha = -80;
          } else {alpha  = 80;}
          if (ang_z >= alpha + beta) {
              move.rotateCCW(200, 200, 200, 200);
          } else if (ang_z <= alpha - beta){
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
    if (ang_z >= alpha + beta) {
        move.rotateCCW(200, 200, 200, 200);
    } else if (ang_z <= alpha - beta) {
        move.rotateCW(200, 200, 200, 200);
    } else {
        routine = 7;
        state = 0;
    }

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
