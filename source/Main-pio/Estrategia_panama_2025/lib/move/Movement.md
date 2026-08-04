# Movement System — Architecture & Maintenance Guide

Team Outer Heaven · WRO 2026 · `Estrategia_panama_2025`

This document explains how the whole firmware fits together, so that changes can
be made later without breaking something three layers away. It is split into:

1. [Hardware map](#1-hardware-map) — every pin, verified against the libraries
2. [Which robot am I building?](#2-which-robot-am-i-building)
3. [Layer architecture](#3-layer-architecture)
4. [The movement stack](#4-the-movement-stack)
5. [What changed in the 2026 redesign](#5-what-changed-in-the-2026-redesign)
6. [Behaviour changes that need field retesting](#6-behaviour-changes-that-need-field-retesting)
7. [Tuning the regulator](#7-tuning-the-regulator)
8. [The rest of the system](#8-the-rest-of-the-system)
9. [Rules for changing this code safely](#9-rules-for-changing-this-code-safely)
10. [Known weak points and open questions](#10-known-weak-points-and-open-questions)

---

## 1. Hardware map

Arduino Mega 2560 + Adafruit Motor Shield **v1** (L293D + 74HCT595 shift register).

Every pin below was read out of the libraries rather than assumed. **Before using
any "free" pin, check it against this table.**

| Pin(s) | Used by | Notes |
|---|---|---|
| 3, 5, 6, 11 | Motor PWM (AFMotor) | M2=3, M4=5, M3=6, M1=11 |
| 4, 7, 8, 12 | Shield shift register | CLK=4, ENABLE=7, DATA=8, LATCH=12 |
| 9 | Rotor L293D enable | `analogWrite`, speed of the shooter/storer |
| 10 | Servo (gate) | `myservo.attach(10)` |
| 14 | Start switch | `INPUT_PULLUP`, LEFT robot only |
| 18, 19 | Back / side microswitches | `INPUT_PULLUP`, **polled, not interrupts** |
| 20, 21 | I2C (SDA/SCL) | MPU6050 gyro |
| 34 | Debug LED | |
| 46, 48 | Rotor L293D input4 / input3 | direction, fixed in `setup()` |
| 50–53 | SPI | Pixy2 (MISO/MOSI/SCK/SS; `SS`=53 on Mega) |
| A8–A15 | 4 quadrature encoders | **PORTK is now completely full** |

### Motor and wheel layout

Viewed from above, front of robot pointing up:

```
        FRONT
   motor3     motor4      <- front pair, these two measure distance
   motor2     motor1      <- rear pair, added 2026 for synchronisation
        BACK
```

### Encoder assignment

| Object | Pins | Motor | Position | Role |
|---|---|---|---|---|
| `encoderLeft` | A15, A14 | motor3 | front left | **distance + completion** |
| `encoderRight` | A13, A12 | motor4 | front right | **distance + completion** |
| `encoderRearRight` | A11, A10 | motor1 | rear right | synchronisation only |
| `encoderRearLeft` | A9, A8 | motor2 | rear left | synchronisation only |

> ⚠️ **The two rear pin pairs are UNCONFIRMED placeholders.** They were chosen
> because A8–A11 sit on PORTK alongside the existing pairs and nothing else uses
> them. Verify against the actual wiring before driving. They are at
> `src/main.cpp` lines 63–64.

### Timer usage — the silent killer

On the Mega, AFMotor claims three 16-bit timers. Anything else that grabs one of
these will kill motor PWM without any compile error:

| Timer | Owner | Consequence if stolen |
|---|---|---|
| Timer0 | `millis()` / `delay()` | AFMotor does **not** touch it on Mega (it does on Uno) |
| Timer1 | motor1 PWM (OC1A) | motor1 stops responding to `setSpeed` |
| Timer2 | `analogWrite(9)` — rotor | rotor speed control dies |
| Timer3 | motor2 (OC3C) + motor4 (OC3A) | two wheels stop responding |
| Timer4 | motor3 PWM (OC4A) | motor3 stops responding |
| Timer5 | Servo library | gate servo dies |

The Servo library on Mega allocates **Timer5 first**, then Timer1, Timer3, Timer4,
at 12 servos per timer. One servo is attached, so only Timer5 is used and nothing
collides. **If you ever attach a 13th servo it will take Timer1 and silently break
motor1.**

### Encoder interrupts

`QuadratureEncoder` uses the `EnableInterrupt` library with pin-change interrupts.
All four encoder pairs live on PORTK (A8–A15). `MAX_NUM_ENCODERS` is **4**, and all
four slots are now used — there is no room for a fifth encoder without editing
`lib/QuadratureEncoder/QuadratureEncoder.h`.

`Encoders` assigns its interrupt slot from a **static counter incremented in the
constructor**, so *declaration order in `main.cpp` determines which slot each
encoder gets*. Do not reorder those four declarations casually.

---

## 2. Which robot am I building?

Both robots share one `main.cpp`. Selection is by **commenting / uncommenting**
one of two config blocks near the top of the file:

```
//LEFT  - WALL   → src/main.cpp lines 33-42   (currently ACTIVE)
//RIGHT - RAMP   → src/main.cpp lines 44-52   (currently COMMENTED)
```

Exactly one block must be active or the file will not compile (duplicate symbols)
or will behave as the wrong robot. What differs between them:

| Symbol | LEFT (wall) | RIGHT (ramp) | Meaning |
|---|---|---|---|
| `pwmf[4]` | 245,243,243,245 | 230,243,243,230 | forward/backward PWM per motor |
| `pwms[4]` | 220,225,220,225 | 200,200,200,200 | strafe/diagonal PWM per motor |
| `pulses` | 900 | 1650 | fed to `mmToPulses` as counts/revolution |
| `robotSide` | `LEFT` | `RIGHT` | flips nearly every left/right decision |
| `slowRotorSpeed` | 90 | 180 | rotor PWM in "slow" mode |
| `closedGate` / `openGate` | 170 / 55 | 96 / 0 | servo angles |

`lenght` is set in `setup()`, not in the block: **1100** for LEFT, **640** for
RIGHT. It is the length of the main straight in mm.

`robotSide` is the master switch. It is read in `inner()`, `outer()`, and in most
routine cases to mirror the whole strategy. **Any new directional logic must
branch on it.**

---

## 3. Layer architecture

```
main.cpp
  setup()  ── Pixy scan picks the opening routine (0-3), or defaults to 4
  loop()   ── microswitch edges → state++
           ── gyro integration  → ang_z
           └─ switch(routine) → switch(state) → move.<primitive>(...)
                                                      │
                              ┌───────────────────────┘
                              ▼
                          move.h  (class Move)
                            armMotion()      decides "is this a new move?"
                            runSynchronised() drives motors + applies regulation
                            checkDone*()     decides "am I there yet?"
                              │                        │
                              ▼                        ▼
                    WheelRegulator.h            encoderLeft / encoderRight
                    (ramp + wheel sync)         (front pair = ground truth)
                              │
                              ▼
                        AFMotor (L293D shield)
```

**The single most important structural fact:** the routines never talk to motors
or encoders directly. They only call `Move` methods, and every `Move` method is
non-blocking — it is called *repeatedly* from `loop()` and returns "done yet?"
This is why `loop()` must never block.

---

## 4. The movement stack

### 4.1 The non-blocking contract

Every motion primitive follows the same pattern:

```cpp
case 2:
  if (move.forward(mm(505))) state++;   // called every loop until it returns true
  break;
```

- **First call** — `armMotion()` records start counts, sets nominal PWM, arms the
  acceleration profile, stamps `moveStartTime`.
- **Every call** — motors are re-commanded and regulated PWM is re-asserted.
- **Final call** — the target is reached (or the timeout fires), `stop()` is
  called, `moving` goes false, and `true` is returned exactly once.

Return conventions differ and **must be preserved**:

| Method | Returns | Meaning |
|---|---|---|
| `forward`, `backward`, `left`, `right`, `rotate`, diagonals, `forwardq`, `backwardp` | `bool` | true = finished |
| `forwardp`, `forwardRegulated` | `int` | `0`=running, `1`=finished, `2`=passed the "far" threshold (14/22 of the way) |

Return code `2` is what triggers `enableSlowDrivers()` mid-move so the rotor slows
before arriving. Losing it would break routines 4 and 7.

### 4.2 Distance measurement — unchanged, and deliberately so

Distance and completion are measured **only** on the two front encoders
(`encoderLeft` / `encoderRight`), using `abs()` of the delta, and completing when
**either** one reaches the target (`||`, not `&&`).

This is exactly how it worked before 2026, and it was kept that way on purpose:
**every distance already tuned into the routines keeps its meaning.** The two new
rear encoders feed synchronisation only and never influence when a move ends.

`checkDoneWithTimeout()` also enforces `moveTimeoutMs` (4000 ms), which stops the
move regardless. Any move that legitimately takes longer than 4 s will be cut
short — worth remembering when adding long moves.

### 4.3 Unit conversion

```cpp
int mm(int millimetres)  →  move.mmToPulses(mm, diameter=60, pulses)
```

`pulses` (900 or 1650) is passed as *pulses per wheel revolution*, with a 60 mm
wheel (188.5 mm circumference). So LEFT ≈ 4.77 counts/mm, RIGHT ≈ 8.75 counts/mm.

> ⚠️ Not every call is wrapped in `mm()`. Several pass **raw encoder counts**:
> `move.forward(80)`, `move.backward(20)`, `move.backward(600)` (routines 5 and 7),
> and `outer(750)` / `outer(30)` / `inner(180)` pass raw numbers into helpers that
> then wrap them again. Read carefully before "fixing" a number — some of these are
> raw counts and some are millimetres, and the distinction is not obvious.

### 4.4 WheelRegulator — how synchronisation works

The whole design rests on one geometric fact about this drivetrain:

> For **every** motion this robot performs — forward, backward, strafe, rotate and
> the two-wheel diagonals — each *participating* wheel is supposed to turn through
> the **same number of encoder counts**.

So keeping the robot straight and smooth reduces to one rule: **make every driven
wheel travel the same distance.** Two consequences worth understanding:

- Because only *magnitude* matters, the regulator compares `abs(delta)`. It
  therefore **does not care which way round any encoder's A/B pair was wired** —
  no polarity calibration is needed, ever.
- Wheels commanded `RELEASE` (the two idle wheels in a diagonal) are excluded from
  the comparison, as are wheels with no encoder.

**Correction is downward only.** The reference is the *slowest* driven wheel; any
wheel ahead of it has its PWM trimmed down in proportion to how far ahead it is.
No wheel is ever driven above its nominal PWM. This is not a stylistic choice —
nominal PWM is already 243–245 out of 255, so there is no headroom to speed a
lagging wheel up. Trimming down can never saturate.

**Acceleration** is a trapezoid applied to nominal PWM, measured in *encoder
counts* rather than milliseconds, so it behaves identically no matter how fast
`loop()` happens to be running. A move too short to reach full speed collapses
into a triangle automatically, with no special case.

Safety behaviour worth knowing: if an encoder ever fails and reads zero, every
other wheel appears infinitely far ahead. `maxTrim` caps the damage — the robot
crawls rather than stopping dead, and the 4 s timeout still ends the move.

### 4.5 Methods that are NOT regulated

Deliberately left open-loop; do not assume everything is smooth:

- `simpleForward/Backward/Left/Right` — no encoders, no target, run until stopped
- `rotateCW` / `rotateCCW` — used with **gyro** feedback in routines 7 and 8, not
  encoder feedback. They set `moving = false` on every call by design.
- `forwarda` — superseded by the regulator's ramp. Unused. Its `static` variables
  are never reset between moves, so it is buggy; do not adopt it.
- `accelerateToPWM` — unused, also `static`-based.

---

## 5. What changed in the 2026 redesign

### New file: `lib/move/WheelRegulator.h`

Self-contained, no dependencies beyond `Arduino.h`. Holds the acceleration profile
and the per-wheel trim calculation. Owns no hardware — `Move` feeds it progress
numbers and asks it what PWM to use.

### `lib/move/move.h`

- **Added a four-encoder constructor.** The original two-encoder constructor still
  exists and still works — with it you still get acceleration ramps and front-axle
  synchronisation, just not rear-wheel sync.
- **Added `armMotion()`** — centralises "is this a new move or a continuation?".
- **Added `runSynchronised()`** — replaces the bare `setMotors()` call in every
  primitive; commands directions, gathers all four encoder deltas, updates the
  regulator, re-asserts regulated PWM.
- **`startMove(long target)`** now also zeroes all four wheel counters and arms the
  profile. The parameter defaults to 0, so old call sites still compile.
- **Every public method kept its exact name, signature and return semantics.**
  This is why all eleven routines were converted without editing the state machine
  at all.

### `src/main.cpp`

- Two rear `Encoders` objects declared; `Move` switched to the 4-encoder
  constructor (lines 58–71).
- `i2cDeviceCount` global added.
- **The I2C bus scan moved out of `loop()` into `setup()`.** It previously probed
  all 126 addresses on *every single pass*, which made the control period both long
  and wildly irregular. A regulator cannot be tuned against a loop whose period
  keeps changing. `loop()` now reads the stored count; all downstream logic
  (`devices > 0` → read gyro, else `mpu = false`) is untouched.

---

## 6. Behaviour changes that need field retesting

These are real, intended, and will be visible on the field. **Do not assume the
robot is broken when it behaves differently here.**

### 6.1 Distances will come out shorter than before ⚠️ biggest one

The loop was previously spending roughly 100 ms per pass inside the I2C scan.
Completion was therefore only checked every ~100 ms, and the robot overshot its
target by up to a full loop-period of travel. Detection now happens within a
millisecond or two, and the deceleration ramp further reduces run-on.

Same encoder counts, far less overshoot ⇒ **shorter real-world distances.**
Re-verify the tuned numbers. To temporarily restore the old stopping behaviour
while checking, disable the ramp-down:

```cpp
move.regulator.endFactor = 255;   // in setup()
```

### 6.2 Diagonals now get their full timeout

`forwardLeft`, `forwardRight`, `backwardLeft` and `backwardRight` never set
`moveStartTime`. They inherited it from the *previous* move, so
`checkDoneWithTimeout` was already partway through its 4 s budget before the
diagonal even started — routine 5 case 6 was very likely cutting its diagonal
short. These now get a full, fresh timeout and **will travel further than before.**

### 6.3 Abandoned moves no longer corrupt the next move

A move interrupted by a microswitch `state++` left `moving == true`. The next
movement then skipped `startMove()` and measured from **stale start counts**,
travelling the wrong distance. `armMotion()` now detects that the motion type or
target changed and re-arms cleanly.

---

## 7. Tuning the regulator

All fields are public on `move.regulator` and can be set from `setup()`.

| Field | Default | Raise it when | Lower it when |
|---|---|---|---|
| `syncGain` | 4 | robot still drifts off straight | wheels hunt / stutter |
| `maxTrim` | 70 | — | — (safety cap; keeps a dead encoder from stalling the robot) |
| `minPWM` | 110 | a trimmed wheel stalls instead of slowing | — |
| `startFactor` | 145 | starts feel sluggish | starts feel jerky |
| `endFactor` | 130 | stopping short | overshooting |
| `accelFraction` | 64 (=25%) | want gentler starts | want snappier starts |
| `decelFraction` | 77 (=30%) | want gentler stops | want snappier stops |
| `maxRampCounts` | 300 | long straights should ramp over more distance | |
| `updateIntervalMs` | 10 | — | — (changing this changes what `syncGain` means) |

Fractions are in 1/256ths. Recommended order: get `syncGain` right first (drives
straightness), then shape `accelFraction`/`decelFraction` (drives smoothness),
then re-verify distances.

---

## 8. The rest of the system

### 8.1 Routine / state machine

`loop()` runs `switch (routine)` → `switch (state)`. `routine` picks the strategy;
`state` steps through it. Both are plain globals; transitions are just assignments.

| Routine | Line | Purpose |
|---|---|---|
| 0–3 | 498, 529, 561, 588 | Opening purple-ball handling, one per quadrant |
| 4 | 623 | Main lane loop. `state` **counts down** (−1…−6) for the OUTER lane and up (0…5) for MIDDLE/INNER |
| 5 | 742 | Diagonal lane |
| 6 | 803 | Return + Pixy orange-ball weighting → picks next `lane` |
| 7 | 937 | Corner reset, gyro-based rotation to ±80° |
| 8 | 1067 | Re-orient to 0° then fall into routine 7 |
| 9 | 1078 | Pixy ball tracking / parking |
| 10 | 1152 | Debug |

`setup()` chooses the opening routine by scanning up to 120 Pixy frames for the
purple ball and classifying it into a quadrant around `(center_x=200, center_y=32)`
→ routine 0, 1, 2 or 3. If nothing is found it stays at the default, **routine 4**.

> ⚠️ `case 8:` at line 1067 has **no `break`** — it deliberately falls through into
> `case 9:`. Verify whether that is intentional before adding code there.

### 8.2 Lane selection

`lane` (`OUTER` / `MIDDLE` / `INNER`) is the strategic variable. Routine 6 case 6
reads the Pixy, classifies each orange blob into one of three *franjas* via
`classifyLane()` (two diagonal boundary lines, different constants per side), sums
blob **area** per franja, and picks the heaviest. `robotSide` then maps franja →
lane, mirrored between the two robots. Guarded by `connections < 2` so the camera
decision is only taken twice per run.

### 8.3 Sensors

- **Pixy2** — SPI. Used in `setup()` (purple ball) and routines 6 and 9 (orange
  balls). `pixy.setLamp(0,0)` turns the lamp off.
- **MPU6050** — I2C, gyro only, Z axis integrated into `ang_z`. Zeroed by
  `resetGyroAngles()`, which is called on every back-microswitch press. `mpu`
  records whether the sensor answered; routines 7 and 8 fall back to
  encoder-counted `rotate()` when it did not.
- **Microswitches** — despite the "For interrupt on Mega" comments, pins 18/19 are
  **polled** in `loop()` with a 350 ms debounce window, not attached as interrupts.
  A back-switch press zeroes the gyro **and** advances `state`. `backSwitchPressed`
  / `sideSwitchPressed` are written but never meaningfully read.

### 8.4 Rotor and gate

Direction is fixed in `setup()` (`input3` HIGH, `input4` LOW); only speed varies,
via `analogWrite(enable34, …)`:

- `enableDrivers()` → 254 (full, storing)
- `enableSlowDrivers()` → `slowRotorSpeed` (90 LEFT / 180 RIGHT)
- `disableDrivers()` → 0

The servo on pin 10 switches between `closedGate` (store) and `openGate` (shoot).

---

## 9. Rules for changing this code safely

1. **Never block inside `loop()`.** Every motion primitive is a state machine that
   must be re-entered. `while (true) { if (move.right(...)) break; }` in routine 9
   (lines ~1105 and ~1119) violates this and freezes all sensing and switch
   handling for the duration. Do not copy that pattern.
2. **Never add `delay()` to the movement path.** It stalls regulation and switch
   debouncing alike.
3. **Preserve return semantics.** `forwardp` and `forwardRegulated` return `int`
   with a meaningful `2`. Changing them to `bool` silently breaks the rotor
   pre-slowing in routines 4 and 7.
4. **Distances are measured on the front encoders only.** If you ever change that,
   every tuned distance in every routine becomes invalid at once.
5. **Branch on `robotSide` for anything directional**, and check the *other* robot's
   config block still compiles when you touch the shared globals.
6. **Do not reorder the four `Encoders` declarations** — construction order assigns
   interrupt slots.
7. **Check the timer table** before using `analogWrite` on a new pin or attaching
   another servo.
8. **`mm()` vs raw counts** — confirm which one an existing call uses before
   changing its number.
9. **Adding a new motion primitive**: copy the shape of `forward()` exactly —
   `armMotion(...)` with a *new* `MotionId`, then `runSynchronised(...)`, then
   `checkDoneWithTimeout(...)`. A duplicated `MotionId` will defeat the re-arm
   detection and reintroduce the stale-counts bug.
10. **Rebuild for both robots** after touching shared code:
    `pio run` with each config block active in turn.

---

## 10. Known weak points and open questions

**Unresolved — needs an answer from the team:**

- **Rear encoder pins are unverified placeholders** (A11/A10 and A9/A8). This is
  the one thing blocking a real drive test.
- **Why does `pulses` differ so much between robots** (900 vs 1650)? Different
  gearing, or different encoder CPR? It matters if the two robots are ever meant
  to share tuned distances.
- **What are the `position` / `d` trims in `forwardp`, `backwardp`, `forwardq`
  physically compensating for?** They bias one diagonal pair of motors by ±6–9 PWM.
  They were kept untouched so nothing changed underneath, but the synchroniser now
  does this job properly and closed-loop. Once the robot drives well, they are
  likely redundant and can probably be set to 0 — test before removing.

**Latent issues, pre-existing, not introduced by the redesign:**

- Blocking `while (true)` loops in routine 9.
- `case 8:` falls through into `case 9:` with no `break`.
- `filterGyro()` integrates `ang_z` **and** updates `tiempo_prev`; `loop()` then
  integrates again using a `dt` computed from the just-updated timestamp, so the
  second integration adds nearly zero. Harmless today, confusing to read, and it
  will bite whoever changes the gyro code.
- `forwarda` and `accelerateToPWM` use function-level `static` state that is never
  reset between moves. Both are unused. Prefer deleting them over fixing them.
- `lenght` is spelled that way throughout. Renaming it is a safe, mechanical
  change, but touches many lines.
