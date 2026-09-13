# Movement System

How the robot moves: what is wired where, what happens on every pass of
`loop()` while a move runs, how the BNO08x keeps it straight, and which numbers
to change for which symptom. Strategy and the game are in
[README.md](../../README.md).

---

## 1. In one picture

```
encoders ──► distance travelled ──► phase of the move: accel / cruise / decel
         │                        ──► move complete (front pair reaches the count)
         ├─► mean speed          ──► deceleration loop → common PWM
         └─► per-motor speed     ──► telemetry only
BNO08x   ──► heading error       ──► PID → differential between the wheel pairs

        pwm_i = common + trim_i + differential · ROT_i · dirSign_i
```

Two sensors, two jobs, no overlap:

- **Encoders** say how far and how fast. They time the acceleration and the
  deceleration and end the move. They never steer.
- **BNO08x** says which way the robot points. One PID on its heading error is
  the only thing that keeps the robot straight.

Everything in this file is the code in `lib/move/`, `src/Hardware.*` and the
heading part of `src/Sensors.*`.

---

## 2. Hardware map

### Motor and wheel layout

Four 60 mm 45° omni wheels on a 200mm (across) × 130mm (back to front) base, Adafruit Motor Shield v1
(L293D, 12 V). Seen from above, front of the robot pointing up:

```
        FRONT
   motor3   motor4      front pair: measure distance, decide when a move ends
   motor2   motor1      rear pair:  speed measurement only
        BACK
```

Direction patterns (motor1..motor4):

| Move | m1 | m2 | m3 | m4 |
|---|---|---|---|---|
| forward / forwardp / forwardq | F | F | F | F |
| backward / backwardp | B | B | B | B |
| left (strafe) | B | F | B | F |
| right (strafe) | F | B | F | B |
| forwardLeft | – | F | – | F |
| forwardRight | F | – | F | – |
| backwardLeft | B | – | B | – |
| backwardRight | – | B | – | B |
| `rotate(x, true)`, `rotateCCW()` | F | B | B | F |
| `rotate(x, false)`, `rotateCW()` | B | F | F | B |

Moving and turning are done by these patterns. The regulator only adds PWM
levels and a small differential on top of a pattern.

### Encoders

One quadrature encoder per motor, all on PORTK (`A8`–`A15`):

| Object | Pins | Motor | Role |
|---|---|---|---|
| `encoderLeft` | A15, A14 | motor3, front left | distance + speed |
| `encoderRight` | A13, A12 | motor4, front right | distance + speed |
| `encoderRearRight` | A11, A10 | motor1, rear right | speed |
| `encoderRearLeft` | A9, A8 | motor2, rear left | speed |

The `Encoders` constructor takes its interrupt slot from a **static counter**,
so the declaration order in `Sensors.cpp` decides which slot each object gets.
All four are declared in that file, in that order, and must stay together. The
library counts every edge (x4) and keeps a per-encoder count of skipped
transitions (`getEncoderErrorCount()`), which `square_test` prints.

`pulses` (`Hardware.h`, per robot) is counts per wheel revolution: 900 on LEFT,
1350 on RIGHT. With the 60 mm wheel (`diameter`) that is 4.775 counts/mm (LEFT)
and 7.162 counts/mm (RIGHT). `initHardware()` computes it once into
`move.regulator.countsPerMM`, and every move converts the millimetres it is
given into counts with it (rounded to the nearest count). The routines never
see counts.

### Other pins

Actuators (`Hardware.h`):

| Pin | Use |
|---|---|
| 3, 5, 6, 11 | motor PWM (AFMotor: M2=3, M4=5, M3=6, M1=11) |
| 4, 7, 8, 12 | motor shield shift register (CLK, ENABLE, DATA, LATCH) |
| 9 | rotor L293D enable (`analogWrite`, Timer2) |
| 10 | gate servo (Servo library, Timer5) |
| 46, 48 | rotor L293D input4 / input3 |

Sensors (`Sensors.h`):

| Pin | Use |
|---|---|
| 14 | start switch |
| 18, 19 | back / side microswitches (polled) |
| 20, 21 | I2C: BNO08x |
| 50–53 | SPI: Pixy2 (SS = 53) |
| A8–A15 | the four encoders |

Serial2 on 16, 17 (TX2 / RX2) carries the Bluetooth telemetry (`Sensors.cpp`).

### Timers

| Timer | Owner |
|---|---|
| Timer1 | motor1 PWM |
| Timer2 | `analogWrite(9)`, the rotor |
| Timer3 | motor2 + motor4 PWM |
| Timer4 | motor3 PWM |
| Timer5 | Servo library |

The Servo library allocates Timer5 first, then Timer1/3/4 at 12 servos each.
Attaching a 13th servo would take Timer1 and silently kill motor1.

---

## 3. Which robot am I building?

Exactly one block at the top of `src/Hardware.cpp` is uncommented. Everything
that differs between the robots lives there: `pwmf`, `pwms`, `pulses`,
`robotSide`, `slowRotorSpeed`, `closedGate`, `openGate`, `lenght` and the
purple-ball `ballZones`.

`robotSide` is the master switch: the routines mirror left/right decisions on
it, and `initHardware()` sets `move.innerIsLeft` from it so that
`move.inner()` / `move.outer()` translate "towards the centre wall" into a
physical strafe direction per robot.

---

## 4. A move, pass by pass: `Move` (`lib/move/move.h`)

### The non-blocking contract

Every distance-counted primitive takes its distance in millimetres of wheel
travel, is called every pass of `loop()` and returns `true` once, when the
move has finished.

- **Every call, first thing** (`toCounts`): the millimetres become encoder
  counts, `mm × regulator.countsPerMM + 0.5` truncated, i.e. the nearest
  count. Everything below works in counts.
- **First call** (`armMotion`): record which motion and target this is, zero
  the four wheel start counts, arm the regulator with the target and mode (a
  backward move longer than `longBackwardMM` — converted the same way — also
  passes `backwardEnd`, the wall approach in §5), capture the heading to hold
  (translations only — a rotation is meant to change it), stamp
  `moveStartTime`.
- **Every call** (`runRegulated`): assert the direction pattern, give the
  regulator each wheel's travel (|counts| since the move began), whether it is
  driven, and its direction sign; feed it the fresh BNO08x error; take the four
  PWMs it computes and write them to the motors.
- **Completion** (`checkDoneWithTimeout`): either front encoder reaching the
  target count ends the move and `stop()` brakes in the same pass. `moveTimeoutMs`
  (4 s) is a hard cap on every counted primitive, rotations included — a move
  pressed against a wall or with a blocked wheel is stopped and reported done.

If a routine switches to a different motion or target while one is running (a
microswitch advanced `state`), the next call re-arms from the current counts.

### Primitives

| Primitive | Mode | Heading hold | Completion |
|---|---|---|---|
| `forward`, `backward`, `left`, `right` | Hold | yes | front encoder ≥ target, or timeout |
| `forwardRegulated` | Hold | yes | returns 1 at target, 2 at 14/22 of it |
| `forwardp`, `backwardp`, `forwardq` | Profile | no | as above (`forwardp` also returns 2 at 14/22) |
| `forwardLeft`, `forwardRight`, `backwardLeft`, `backwardRight` | Profile | no | front encoder ≥ target, or timeout |
| `rotate` | Profile | no | front encoder ≥ target, or timeout |
| `rotateCW`, `rotateCCW` | none | no | open-loop at a fixed PWM; the caller stops it |
| `stop` | – | – | brakes all four motors and holds them |
| `stopForMillis` | – | – | brakes, returns true after the delay (one shared timer) |

`forwardp` / `backwardp` / `forwardq` trim one diagonal pair by ±9 / ±6 / ±9 so
the robot presses against the wall it runs along; the wall aligns them, so they
run without heading hold.

### Trims

`pwmf[]` / `pwms[]` are not sent to the motors as absolute values. `Move` turns
the four numbers a motion asks for into trims: `trim[i] = n[i] − mean(n)`. The
regulator adds each trim to the common PWM. `{220, 243, 243, 220}` therefore
means "wheels 2 and 3 run 23 counts above wheels 1 and 4"; raising all four by
the same amount changes nothing. They are the static per-motor balance; the
BNO08x PID does the dynamic part.

### Braking

On the L293D, both inputs low (AFMotor `RELEASE`) with the enable held high is
the datasheet's *fast motor stop*: the motor is shorted through the driver and
brakes; with the enable low the outputs float and the motor coasts. The enable
is the PWM line, so `stop()` and `stopForMillis()` set `RELEASE` **and** PWM 255:
a full-duty brake, held until the next move sets new directions. A stationary
braked motor draws no current.

`frontTravelCounts()` is the front-wheel travel since the move began; after a
move it is target + overshoot, which `square_test` prints.

---

## 5. The regulator: `WheelRegulator` (`lib/move/WheelRegulator.h`)

Runs every `updateIntervalMs = 4` ms with a measured `dt`. All parameters are
public members, set in `initHardware()` (`src/Hardware.cpp`).

### Modes

| Mode | Speed profile | Heading PID | Used by |
|---|---|---|---|
| Burst | no — straight to `cruisePWM`, brake at the target | no | any move shorter than 30 mm (wall nudges) |
| Profile | yes | no | wall-hugging straights, diagonals, rotations |
| Hold | yes | yes | forward / backward / left / right / forwardRegulated |

### Speed profile (encoders)

Distance is the mean travel of the driven wheels; speed is its rate, filtered
0.75/0.25. Three phases:

| Phase | When | Common PWM |
|---|---|---|
| **Accel** | first 22 % of the move (clamped 25–220 mm, ≤ 45 %) | open-loop `rampStartPWM → cruisePWM` along a smoothstep of distance |
| **Cruise** | until `remaining ≤ decelCounts + creepCounts` | `cruisePWM` |
| **Decel** | last 30 % (clamped 40–200 mm, ≤ 50 %), plus the creep zone of a wall approach | closed-loop on speed, below |

Accel is open-loop because the wheels need ~200 PWM to break free, so the ramp
starts at 205. Cruise is 232.

**Decel** is where distance accuracy comes from. When the phase begins the
current speed is recorded as `vPeak`, and the commanded speed follows a
constant-deceleration curve down to a creep at the target:

```
vEnd = max(0.15 · vPeak, 40 mm/s)
vCmd = vEnd + (vPeak − vEnd) · sqrt(remaining / decelCounts)
```

### Wall approach (backward moves)

Every backward move in the routines ends on the back wall, and the routines
command more distance than there is (routine 6 reverses `lenght + 250`) so
that the back microswitch, not the count, ends the move. The wall therefore
comes *before* the target — where the curve above would still be fast. So a
backward move (`backward`, `backwardp`, `backwardLeft`, `backwardRight`)
longer than 200 mm finishes differently, with `Move::backwardEnd`
(a `WheelRegulator::EndSpec`, set in `initHardware()`):

| Field | Value | Meaning |
|---|---|---|
| `decelCounts` | 100 mm | length of the deceleration curve |
| `endSpeedMMs` | 200 mm/s | speed at the end of the curve — the approach speed |
| `creepCounts` | 100 mm | the approach speed is held over the last 100 mm of the commanded distance |

```
toCreep = max(remaining − creepCounts, 0)
vCmd    = vEnd + (vPeak − vEnd) · sqrt(toCreep / decelCounts)      vEnd = 200 mm/s
```

so the robot stays at cruise until 200 mm before the target, brakes hard over
100 mm and is at 200 mm/s for the last 100 mm: a wall inside that stretch is
met at 200 mm/s, one that comes earlier is met while still braking.
`creepCounts` is clamped to 60 % of the move and `decelCounts` to what is left
after the accel ramp and the creep; for the 890 mm reverses that is ramp 196 →
cruise 494 → curve 100 → hold 100, for `backward(280)` ramp 62 → cruise 18 →
curve 100 → hold 100. The hold takes 0.5 s of the 4 s `moveTimeoutMs`.

`Move::longBackwardMM` (200) is compared in counts, converted the same way the
move's own distance is, so `backward(200)` and everything shorter keep the
normal profile. An `EndSpec` with every field at 0 (the default `begin()`
argument) is the normal profile.

A PI loop tracks it with the common PWM: `common = 0.15 · e + I`, `e = vCmd −
vMeasured` in mm/s, `I` integrating at 2.0 PWM/(mm/s)/s from the PWM in force
when the phase began, anti-windup at the clamps, output 0..255. It needs no
motor constants: it finds whatever PWM — typically far below 200 — holds each
commanded speed, and if the robot slows under half the creep speed it adds
2 PWM per tick until it rolls again. The profile is relative to the robot's own
speed, so it is the same on either robot. The robot arrives at creep speed and
`stop()` brakes it there.

### Heading PID (BNO08x)

In `Hold` mode only. The error comes from `Sensors.cpp`: degrees from the
heading captured when the move began, −180..+180, sign convention below.

| Term | Value | Notes |
|---|---|---|
| P | `kHeadingP` 12 PWM/deg | on the error outside a ±0.12° deadband, so the robot does not dither at sensor noise |
| I | `kHeadingI` 0 | raw error, frozen while the output is saturated, limited to ±12 PWM. Start at 3.0 when enabling |
| D | `kHeadingD` 0 | raw error rate, low-passed 0.7/0.3. Start at 0.6 when enabling |
| Output | ±`maxHeadingCorrection` 40 PWM | the differential |

A jump of more than 30° between two ticks is a sensor re-reference (reset, or a
stale read returning), not the robot turning: I and D restart from there.

### From differential to wheels

```
pwm_i = common + trim_i + differential · ROT_i · dirSign_i     ROT = {+1, −1, −1, +1}
```

`ROT` puts wheels 1 and 4 against 2 and 3 — the `F B B F` rotation pattern.
`dirSign_i` is +1 for a wheel commanded forward, −1 backward, 0 released, so the
same differential turns the robot the same way whether it is driving forward,
backward or strafing.

**The differential is never clipped.** The four PWMs are computed together; if
the highest would exceed 255 the whole set is shifted down by the excess. A
correction always arrives in full — the robot slows a little instead of losing
steering. The floor is 0 in every phase: the 200 breakaway matters only for
starting, which the accel ramp handles; a rolling wheel keeps rolling below it.

Positive differential = the `F B B F` direction (the `rotateCCW()` pattern),
and the heading sign convention makes the reading **decrease** under that
pattern, so a positive error is corrected by a positive differential.

---

## 6. Heading: the BNO08x part of `src/Sensors.*`

### Sensor

BNO08x, horizontal, chip side up, I2C (tries 0x4A then 0x4B). Report:
`SH2_GAME_ROTATION_VECTOR` (accelerometer + gyroscope, no magnetometer, so the
motors cannot disturb it; heading is relative to power-on). Yaw is taken from
the quaternion with `atan2(2(ij + kr), i² − j² − k² + r²)`.

| Setting | Value | Where |
|---|---|---|
| I2C clock | 400 kHz, set after `begin_I2C()` (the driver's own `Wire.begin()` would reset it to 100 kHz) | `I2C_CLOCK_HZ` |
| I2C timeout | 10 ms, bus reset on timeout | `I2C_TIMEOUT_US` |
| Report interval requested | 2.5 ms (400 Hz); `heading_test` prints the delivered rate | `REPORT_INTERVAL_US` |
| Poll gate | 2 ms | `POLL_INTERVAL_MS` |
| Control tick | 4 ms | `WheelRegulator::updateIntervalMs` |

Worst-case latency from a heading change to the first PWM response:
2.5 + 2 + 4 = 8.5 ms.

`headingUpdate()` is called from `loop()` and from the regulator's heading hook
(`regulatorHeadingError()` in `Hardware.cpp`), so a move keeps getting fresh
readings even inside code that does not return to `loop()` between passes
(routine 9's `while (true)` strafes).

### References

| Reference | Set by | Read by |
|---|---|---|
| **target** | `armMotion()` at the start of every translation | `headingError()` → regulator |
| **zero** | `handleMicroSwitches()` on a back microswitch press — robot square on the wall | `headingSinceZero()`, heading relative to that wall |
| **boot** | `headingBegin()` | `headingSinceBoot()` → telemetry `deg` |

A rotation captures nothing; the translation after it captures the new heading.

### Sign convention

`HEADING_SIGN` (Sensors.cpp, `−1.0f`) is applied to every heading difference.
It is chosen so that the reading **increases** under the `B F F B` pattern
(`rotate(x, false)`, `rotateCW()`) and **decreases** under `F B B F`
(`rotate(x, true)`, `rotateCCW()`). The regulator depends on exactly that: its
positive differential drives the `F B B F` direction and must lower a positive
error.

`square_test` measures each turn against the pattern it used and prints
`SIGN: OK` or `SIGN: FLIP HEADING_SIGN in Sensors.cpp`.

### Fail-safes

| Condition | Effect |
|---|---|
| no report for 100 ms | `headingError()` returns 0 — heading hold idles, move continues on the profile |
| no report for 1000 ms | `headingAvailable()` false; `headingSinceZero()` / `headingSinceBoot()` return 0 |
| sensor reset (`wasReset()`) | re-subscribe; readings distrusted until the next report; **target, zero and boot re-captured** from that report (the sensor's frame is new) |
| I2C line stuck | `Wire` times out and resets the TWI peripheral; the read fails, `age` climbs |
| sensor absent at boot | `headingBegin()` false; everything runs on the profile alone |

`headingResetCount()` is printed as `rst` by `heading_test`. Any reset during a
run re-references the heading mid-move and points at a supply problem.

---

## 7. Tuning

| Symptom | Change |
|---|---|
| Robot arcs during straights, `corr` pinned at ±40 | wrong sign — `square_test` says FLIP; set `HEADING_SIGN` |
| Weaves / oscillates about the heading | lower `kHeadingP` |
| Corrects, but too slowly | raise `kHeadingP`; if it then weaves, enable `kHeadingD` (0.6) for damping |
| Settles with a constant small offset | enable `kHeadingI` (3.0) |
| Twitches while already straight | raise `headingDeadbandDeg` |
| Starts with a jolt | lower `rampStartPWM` toward 200, or raise `rampFraction` |
| Overshoots the target (square_test `overshoot`) | lower `endSpeedFraction` / `minEndSpeedMMs` (slower arrival), or raise `decelFraction` / `maxDecelCounts` (earlier slowdown); if `encoder errors` climb during the move, the counting itself is the problem |
| Stops short / crawls the last part | raise `minEndSpeedMMs`, or lower `decelFraction` |
| Reverses hit the back wall too hard | lower `backwardEnd.endSpeedMMs` (200); if the wall comes more than 100 mm before the commanded distance, raise `backwardEnd.creepCounts` — each costs time against the 4 s `moveTimeoutMs` |
| Reverses slow down too early / crawl for too long | shorten `backwardEnd.creepCounts` (100) and `decelCounts` (100), or raise `backwardEnd.endSpeedMMs` |
| Speed hunts during decel (`v` oscillates about the command) | lower `kSpeedP`, then `kSpeedI` |
| Too slow overall | raise `cruisePWM` — the differential still arrives in full, the set is shifted down when needed |
| Long moves cut short | they hit `moveTimeoutMs` (4 s) — telemetry `s` stuck then advancing at 4 s |
| `age` climbs, `rst` counts up in `heading_test` | drop `I2C_CLOCK_HZ` to 100000; check the sensor's supply |

---

## 8. Changing this code safely

1. Everything in `generalStrategy.cpp` marked `KNOWN` is behaviour the robot
   is tuned around. Changing one means re-running the course.
2. Distances in the routines are millimetres; the library converts them with
   `regulator.countsPerMM` and measures completion on the front encoder pair.
   Keep both, or the tuned numbers stop meaning what they mean.
3. Any new directional logic must branch on `robotSide`.
4. Do not reorder the four `Encoders` declarations in `Sensors.cpp` or split
   them across files.
5. Do not attach more servos or take a timer (see §2).
6. `runRoutines()` stays the last call in `loop()`.
7. New test programs go in `src/test/` with their own `[env:...]` in
   `platformio.ini`; they are excluded from the competition build.
