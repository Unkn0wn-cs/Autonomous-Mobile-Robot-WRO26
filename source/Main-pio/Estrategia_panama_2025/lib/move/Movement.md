# Movement System — hardware map, drive layer, heading, tuning

How the robot moves: what is wired where, what `Move` and `WheelRegulator`
do on every pass of `loop()`, how the BNO08x is used, and which numbers to
change for which symptom. Strategy and the game are in
[README.md](../../README.md).

---

## 1. Hardware map

### Motor and wheel layout

Four 60 mm 45° omni wheels on a 200 × 200 mm base, Adafruit Motor Shield v1
(L293D, 12 V). Seen from above, front of the robot pointing up:

```
        FRONT
   motor3   motor4      front pair: measure distance, decide when a move ends
   motor2   motor1      rear pair:  wheel synchronisation only
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

Steering and strafing are done by these direction patterns. The regulator only
ever adds small PWM differences on top of a pattern.

### Encoders

One quadrature encoder per motor, all on PORTK (`A8`–`A15`):

| Object | Pins | Motor | Role |
|---|---|---|---|
| `encoderLeft` | A15, A14 | motor3, front left | distance + sync |
| `encoderRight` | A13, A12 | motor4, front right | distance + sync |
| `encoderRearRight` | A11, A10 | motor1, rear right | sync |
| `encoderRearLeft` | A9, A8 | motor2, rear left | sync |

The `Encoders` constructor takes its interrupt slot from a **static counter**,
so the declaration order in `Hardware.cpp` decides which slot each object gets.
All four are declared in that file, in that order, and must stay there.

`pulses` (RobotConfig) is counts per wheel revolution: 900 on LEFT, 1650 on
RIGHT. With the 60 mm wheel that is 4.775 counts/mm (LEFT) and 8.754 counts/mm
(RIGHT). `mm()` in `Motion.cpp` does the conversion; some routine call sites
pass raw counts instead (`forward(80)`, `backward(600)`, `outer(750)`,
`inner(180)`).

### Other pins

| Pin | Use |
|---|---|
| 3, 5, 6, 11 | motor PWM (AFMotor: M2=3, M4=5, M3=6, M1=11) |
| 4, 7, 8, 12 | motor shield shift register (CLK, ENABLE, DATA, LATCH) |
| 9 | rotor L293D enable (`analogWrite`, Timer2) |
| 10 | gate servo (Servo library, Timer5) |
| 14 | start switch |
| 18, 19 | back / side microswitches (polled) |
| 20, 21 | I2C: BNO08x |
| 34 | debug LED |
| 46, 48 | rotor L293D input4 / input3 |
| 50–53 | SPI: Pixy2 (SS = 53) |

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

## 2. Which robot am I building?

Exactly one block in `src/RobotConfig.cpp` is uncommented. Everything that
differs between the robots lives there: `pwmf`, `pwms`, `pulses`, `robotSide`,
`slowRotorSpeed`, `closedGate`, `openGate`. `lenght` is defined outside both
blocks and assigned in `setup()`.

`robotSide` is the master switch: the routines mirror left/right decisions on
it, and `inner()` / `outer()` translate "towards the centre wall" into a
physical strafe direction per robot.

---

## 3. The drive layer: `Move` (`lib/move/move.h`)

### The non-blocking contract

Every distance-counted primitive is called every pass of `loop()` and returns
`true` once, when the move has finished. On the first call it arms itself
(`armMotion`): records which motion and target it is, zeroes all four wheel
start counts, arms the regulator, captures the heading target (translations
only), and stamps `moveStartTime`. Every call after that re-asserts the motor
directions and the regulated PWM for this instant (`runSynchronised`).

If a routine switches to a different motion or target while one is running
(a microswitch advanced `state`), the next call re-arms from the current
counts.

### Primitives

| Primitive | Mode | Heading hold | Completion |
|---|---|---|---|
| `forward`, `backward`, `left`, `right` | Full | yes | front encoder ≥ target, or timeout |
| `forwardRegulated` | Full | yes | returns 1 at target, 2 at 14/22 of it |
| `forwardp`, `backwardp`, `forwardq` | Ramp | no | as above (`forwardp` also returns 2 at 14/22) |
| `forwardLeft`, `forwardRight`, `backwardLeft`, `backwardRight` | Ramp | no | front encoder ≥ target, or timeout |
| `rotate` | Full, hold off | no | front encoder ≥ target, or timeout |
| `rotateCW`, `rotateCCW` | none | no | caller stops it (routines 7/8 use the sensor) |
| `stop` | – | – | releases all four motors |
| `stopForMillis` | – | – | releases, returns true after the delay (one shared timer) |

`forwardp` / `backwardp` / `forwardq` trim one diagonal pair by ±9 / ±6 / ±9 so
the robot presses against the wall it runs along; the wall does the aligning,
so they run without wheel sync or heading hold.

### Distance, completion, timeout

Completion is measured on the **front pair only**: either `encoderLeft` or
`encoderRight` reaching the target count ends the move. `moveTimeoutMs = 4000`
is a hard cap on every counted primitive, rotations included: a move that has
not reached its count in 4 s (usually because it is pressed against a wall, or
a wheel is blocked) is stopped and reported as done.

### PWM trims

`pwmf[]` / `pwms[]` are not sent to the motors as absolute values. `Move`
turns the four numbers a motion asks for into trims (`setNominalSpeeds`):
`trim[i] = n[i] − mean(n)`. The regulator adds each trim to its own
`cruisePWM`. `{220, 243, 243, 220}` therefore means "wheels 2 and 3 run 23
counts above wheels 1 and 4"; raising all four by the same amount changes
nothing.

---

## 4. The regulator: `WheelRegulator` (`lib/move/WheelRegulator.h`)

Three loops, each on its own sensor. All parameters are members, set in
`initHardware()` (`src/Hardware.cpp`) or at their defaults in the header.

### Modes

| Mode | Ramp | Wheel sync | Heading hold | Used by |
|---|---|---|---|---|
| Burst | no — straight to cruise | no | no | any move shorter than 120 mm |
| Ramp | yes | no | no | wall-hugging straights, diagonals |
| Full | yes | yes | yes (translations) | forward/backward/left/right, rotate (hold off) |

### The PWM band

| Parameter | Value | Meaning |
|---|---|---|
| `minMovePWM` | 200 | wheels do not turn below this; every output is clamped to it |
| `maxPWM` | 255 | ceiling |
| `rampStartPWM` | 205 | where a ramp begins |
| `cruisePWM` | 232 | regulated cruise; `base_i = cruisePWM + trim_i` |

Authority with the RIGHT robot's trims (bases 221 / 244 / 244 / 221): a
correction that raises wheels 1 and 4 has 34 counts of room, one that raises
wheels 2 and 3 has 11 before clamping at 255. The regulator clamps per wheel,
so in the second direction a ±20 request comes out as +11 / −20.

### 1. Speed shaping (encoders)

`profile = min(smoothstep(travelled / ramp), smoothstep(remaining / ramp))`,
where `travelled` is the furthest driven wheel. The PWM for wheel *i* is
`rampStartPWM + (base_i − rampStartPWM) · profile`. The ramp length is 22 % of
the move (`rampFraction`), clamped to 25–220 mm and to 45 % of the move, so the
profile is a function of **position**: deceleration always starts at a known
distance from the target.

### 2. Wheel synchronisation (encoders)

For every driven wheel, `sync_i = kSync · (mean − progress_i)`, clamped to
±`maxSyncCorrection` (0.30 PWM/count, ±12 PWM). Deviations are measured against
the **mean** of the driven wheels, so corrections sum to zero and cannot change
the robot's overall speed.

With heading hold on, the rotation component is projected out of `sync[]`:
`dot = mean_i(sync_i · ROT_i · dirSign_i)`, then `sync_i −= dot · ROT_i · dirSign_i`
with `ROT = {+1, −1, −1, +1}`. Rotation is left entirely to the heading loop,
which measures it; the sync loop equalises the wheels in every other direction.

### 3. Heading hold (BNO08x)

Runs in Full mode for translations, every `updateIntervalMs = 4` ms with a
measured `dt`:

| Parameter | Value |
|---|---|
| `kHeadingP` | 9.0 PWM/deg |
| `kHeadingI` | 0 (proportional-only while P is being observed; start at 3.0 when enabling) |
| `kHeadingD` | 0 (start at 0.6 when enabling; derivative is low-passed 0.7/0.3) |
| `headingDeadbandDeg` | 0.12° — only the P term sees it; I and D see the raw error |
| `maxHeadingCorrection` | ±20 PWM; integral frozen while saturated |
| `headingIntegralLimit` | ±8 PWM |

An error jump of more than 30° between two ticks is treated as a sensor
re-reference (reset, or a stale read returning): D and I restart from there.

The correction is a rotation superimposed on the translation:
`pwm_i = ramped_i + sync_i + corr · ROT_i · dirSign_i`, clamped to the band.
`dirSign` is +1 for a wheel commanded forward, −1 backward, 0 released, so the
same `corr` turns the robot the same way whether it is driving forward,
backward or strafing.

Positive `corr` drives the `F B B F` pattern (wheels 1 and 4 up, 2 and 3
down) — the `rotateCCW()` direction. The sign convention in `Heading.h` makes
the heading reading **decrease** under that pattern, so a positive error is
corrected by a positive `corr`.

---

## 5. Heading: `src/Heading.*`

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

`headingUpdate()` is called from `loop()` and also from the regulator's heading
hook (`regulatorHeadingError()` in `Hardware.cpp`), so a move keeps getting
fresh readings even inside code that does not return to `loop()` between
passes (routine 9's `while (true)` strafes).

### References

| Reference | Set by | Read by |
|---|---|---|
| **target** | `armMotion()` at the start of every translation | `headingError()` → regulator |
| **zero** | `onSwitchPress()` — back microswitch, robot square on the wall | `headingSinceZero()` → routines 7/8 |

A rotation captures nothing; the translation after it captures the new heading.

### Sign convention

`HEADING_SIGN` (Heading.cpp, `−1.0f`) is applied to both `headingError()` and
`headingSinceZero()`. It is chosen so that the reading **increases** under the
`rotateCW()` pattern (`B F F B`) and **decreases** under `rotateCCW()`
(`F B B F`). Two things depend on exactly that:

- routines 7/8: `reading ≥ alpha + beta → rotateCCW()`, `reading ≤ alpha − beta
  → rotateCW()` — converges only if rotateCCW lowers the reading;
- the regulator: positive `corr` = the `F B B F` direction, must lower a
  positive error.

`square_test` measures each turn against the pattern it used and prints
`SIGN: OK` or `SIGN: FLIP HEADING_SIGN in Heading.cpp`.

### Fail-safes

| Condition | Effect |
|---|---|
| no report for 100 ms | `headingError()` returns 0 — heading hold idles, move continues open-loop |
| no report for 1000 ms | `headingAvailable()` false — routines 7/8 use encoder-counted turns |
| sensor reset (`wasReset()`) | re-subscribe; readings distrusted until the next report; **target and zero re-captured** from that report (the sensor's frame is new) |
| I2C line stuck | `Wire` times out and resets the TWI peripheral; the read fails, `age` climbs |
| sensor absent at boot | `headingBegin()` false; everything runs without heading hold |

`headingResetCount()` is printed as `rst` by the telemetry line and by
`heading_test`. Any reset during a run re-references the heading mid-move and
points at a supply problem.

---

## 6. Tuning

| Symptom | Change |
|---|---|
| Robot arcs during straights, `corr` pinned at ±20 | wrong sign — `square_test` says FLIP; set `HEADING_SIGN` |
| Weaves / oscillates about the heading | lower `kHeadingP`; if I/D are enabled, lower `kHeadingD` first |
| Settles with a constant small offset | enable `kHeadingI` (3.0) |
| Twitches while already straight | raise `headingDeadbandDeg` |
| One wheel visibly lags at start | raise `kSync` (0.30) or `maxSyncCorrection` (12) |
| Starts with a jolt | lower `rampStartPWM` toward `minMovePWM`, or raise `rampFraction` |
| Too slow overall | raise `cruisePWM` — but every count up removes a count of upward authority for wheels 2/3 on RIGHT (11 today) |
| Long moves cut short | they hit `moveTimeoutMs` (4 s) — see telemetry `s` stuck then advancing at 4 s |
| `age` climbs, `rst` counts up | drop `I2C_CLOCK_HZ` to 100000; check the sensor's supply |

All regulator parameters are public members and can be set from `initHardware()`
or a test program without touching the library.

---

## 7. Changing this code safely

1. Everything in `Routines.cpp` marked `KNOWN` is behaviour the robot is tuned
   around. Changing one means re-running the course.
2. Distances tuned into the routines are in front-encoder counts. Keeping
   completion on the front pair keeps them meaningful.
3. Any new directional logic must branch on `robotSide`.
4. Do not reorder or move the four `Encoders` declarations.
5. Do not attach more servos or take a timer (see §1).
6. `runRoutines()` stays the last call in `loop()`.
7. New test programs go in `src/test/` with their own `[env:...]` in
   `platformio.ini`; they are excluded from the competition build.
