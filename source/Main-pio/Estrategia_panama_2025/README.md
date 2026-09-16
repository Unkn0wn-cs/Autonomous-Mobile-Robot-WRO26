# Estrategia Panamá 2025 — WRO RoboSports firmware

Team Outer Heaven. Firmware for both autonomous ping-pong robots.

- **[README.md](README.md)** (this file) — the game, the strategy, how to build, what to watch
- **[lib/move/Movement.md](lib/move/Movement.md)** — hardware map, movement and heading system, tuning

---

## 1. The game — WRO RoboSports "Double Tennis"

Two teams face each other. **Each team fields two robots, and both of a team's
robots stay on their own half** of the field. The halves are separated by a
barrier, and each half contains a ramp (300 × 563 × 50 mm, green slope).

**Scoring is inverted — the lowest score wins.**

| Ball | Counted as | So you want it |
|---|---|---|
| Orange | **+1** | on the *opponent's* half |
| Purple | **−2** | on *your own* half |

At the start each half holds **4 orange balls and 1 purple ball**. For the 2026
season **a fifth orange ball sits on the barrier in the middle** — it is not
counted at all, provided it never moves.

> Push your orange balls across to the other half — then **keep doing it**, because
> the opposing team is pushing theirs back at you for the whole match. Meanwhile
> the purple ball must never leave your own half.

A match lasts a **random 1 to 2 minutes**, decided immediately before it starts.
A game is three matches.

Sources: [WRO 2026 RoboSports General Rules](https://wro-association.org/wp-content/uploads/WRO-2026-RoboSports-Double-Tennis-General-Rules.pdf) ·
[WRO 2025 rules](https://wro-association.org/wp-content/uploads/WRO-2025-RoboSports-Double-Tennis-General-Rules.pdf) ·
[WRO India category overview](https://wroindia.org/wro-india-season-2025-robosports-category/)

---

## 2. The two robots

One firmware, one robot selected by commenting a block at the top of
[`src/Hardware.cpp`](src/Hardware.cpp).

| | LEFT — "WALL" | RIGHT — "RAMP" |
|---|---|---|
| `lenght` (main straight) | 1100 mm | 640 mm |
| `pulses` (encoder counts per wheel rev) | 900 | 1350 |
| `pwmf` forward trims | 245, 243, 243, 245 | 220, 243, 243, 220 |
| `pwms` strafe trims | 220, 225, 220, 225 | 200, 200, 200, 200 |
| Rotor slow speed | 90 | 180 |
| Gate closed / open | 180° / 55° | 116° / 0° |
| Waits for start switch | **yes** | no |

`pwmf`/`pwms` are per-wheel **trims**: the drive layer runs every wheel from its
own cruise PWM and uses only the differences between the four numbers (see
Movement.md).

Both robots: Arduino Mega 2560, Adafruit Motor Shield v1 (L293D, 12 V), four
12 V motors with a quadrature encoder each, four 60 mm 45° omni wheels on a
200 mm (across) × 130 mm (front to back) base, a BNO08x heading sensor on I2C, a Pixy2 camera on SPI, a
rotor that shoots or stores balls, and a servo gate that selects between those.

---

## 3. How the code plays the game

### The building block: lanes

The robot sweeps its half in **lanes** — `OUTER`, `MIDDLE`, `INNER` — parallel
runs at different distances from the centre wall:

- **OUTER** = furthest from the centre wall. For the RIGHT robot that is to its
  right; for the LEFT robot, to its left.
- **INNER** = closest to the centre wall.

`move.outer()` strafes right on RIGHT and left on LEFT; `move.inner()` does the
opposite ([`lib/move/move.h`](lib/move/move.h), fed the robot's side by
`initHardware()`).

One lap is:

```
   drive the straight  →  return  →  look at the camera  →  pick the next lane
        routine 4/5           routine 6                        routine 6 state 7
```

`lane` is chosen by *where the orange balls actually are*, not by a fixed
rotation.

### Choosing a lane from the camera (routine 6, states 5–7)

The robot backs up, then reads the Pixy. Every orange blob is classified into one
of three *franjas* by `classifyLane()`, which splits the image with two diagonal
boundary lines. Each blob contributes its **area** (`width × height`) to that
franja's weight. The heaviest franja wins and maps to a lane, mirrored per robot:

| Heaviest franja | LEFT picks | RIGHT picks |
|---|---|---|
| 0 | OUTER | INNER |
| 1 | MIDDLE | MIDDLE |
| 2 | INNER | OUTER |

`connections < 2` limits this to two camera decisions per corner reset.

### The opening: the purple ball (routines 0–3)

Before the match runs, `selectOpeningRoutine()` scans the Pixy for up to 900 ms
for the purple ball and matches its **bounding box** against four calibrated
rectangles, `ballZones` in `src/Hardware.cpp` (one table per robot, inside the
robot's block; the format and a calibration guide are in `src/Hardware.h`). The
row index of the matched rectangle is the routine:

| Purple ball seen | Routine | Then continues on lane |
|---|---|---|
| upper-left | 0 | INNER (LEFT) / OUTER (RIGHT) |
| upper-right | 1 | MIDDLE |
| lower-left | 2 | INNER (LEFT) / OUTER (RIGHT) |
| lower-right | 3 | MIDDLE |
| not found | 4 | keeps default MIDDLE |

The decision is confirmed over several frames rather than taken from the first
blob that appears:

1. **Quality gate** — a blob must be purple, tracked for at least 2 frames, at
   least 5 px on each side and 45 px² in area, and no more than 3:1 in aspect.
   This is the "not a grain of dust" filter.
2. **Zone scoring** — the blob's box is scored against every rectangle, with
   20 px of graded slack (`BALL_ZONE_TOLERANCE`) around each: centre inside a
   rectangle beats centre within the slack, which beats mere overlap, and
   between two rectangles in reach the closer one wins outright. A blob that
   matches no zone is **discarded**, not forced into a quadrant.
3. **Voting** — only the largest valid blob of each frame votes.
4. **Early commit** — the scan stops once one zone leads by 2 votes and has 3
   votes (5 if every one of them came from the slack band), so a clean ball is
   decided in about 5 camera frames. If nothing ball-like has been seen after
   350 ms the scan gives up early. If the budget runs out without a clear
   winner, the zone with the most votes is used.

The sensitivity constants live at the top of the detector in
`src/generalStrategy.cpp`. It prints one `blob edges L.. T.. R.. B..` line per voting
frame and a final `ball scan .. ms votes ..` summary on Serial, which is what
you use to read off the rectangles when calibrating.

**The gate captures the purple ball inside the robot.** Opening it lets the
purple ball be taken in and held, so each opening routine schedules the gate
to be open at the moment the robot passes over where the ball was seen:

| Routine | Purple seen | Gate schedule |
|---|---|---|
| 0 | upper-left | closed → forward 430 → pause → **open** → forward 430 → closed |
| 1 | upper-right | strafe right 250, then the same as routine 0 |
| 2 | lower-left | **open** → forward 550 → closed → forward 250 |
| 3 | lower-right | **open** → forward 550 → closed → forward 250 |

### The main lane run (routine 4)

**MIDDLE / INNER** — states 0 → 5:

1. `backward(280)` — reverse to square up against the back wall
2. pause
3. `forwardRegulated(lenght + 50)` — the sweep up the field. At **14/22** of the
   distance the primitive returns `2` and the code calls `enableSlowDrivers()`,
   dropping the rotor to 90/180 so balls clear the barrier instead of flying out
   of the field.
4. two pauses
5. rotor back to full, advance the lane, go to routine 6

**OUTER** — states −1 → −6 (the state counts *down*):

1. rotor off, `rotate(166)` — turn
2. `outer(95)` on LEFT / `outer(143)` on RIGHT — strafe outward (KNOWN: a
   per-robot distance the course is tuned around)
3. `forwardp(400)` — wall-hugging run. KNOWN: `forwardp` returns `2` at
   14/22 of the distance and the bare `if` accepts it, so this state ends at
   ≈255 mm with the motors still running; state −4 releases them.
4. rotor on, pause
5. rotor off, `rotate(166)` back
6. strafe outward again → drop into state 0 and run the straight above

On the OUTER lane the straight (routine 4 state 2) uses `forwardp` and the
return (routine 6 state −1) uses `backwardp`: the heading PID holds the robot
leaning `wallHugDeg` (per robot, `Hardware.cpp`) toward the wall, so the
leading corner stays pressed on it without the robot ever turning into it.

### Returning and re-localising (routines 6 and 7)

Routine 6 drives back `lenght + 250` — **further than the field is long**. The
**back microswitch** fires when the robot reaches the wall and the switch handler
advances `state`; the bumper, not the encoder, normally ends the move. The same
switch press zeroes the heading reference (`headingZero()`), because the robot
is square against the wall at that moment. Because the wall comes before the
commanded distance, backward moves longer than 200 mm brake hard over 100 mm
down to 200 mm/s and hold that speed over the last 100 mm of the command (the
wall approach in [Movement.md](lib/move/Movement.md)), so the bumper is met at
that speed.

The switches are accurate enough to stop movement just before the wall — but
only when the robot arrives square to it. If it arrives misaligned, one corner
touches first, and the encoder distance or the 4 s move timeout ends the move
instead. That is what the corner reset in routine 7 recovers from.

Routine 7 is the full corner reset: reverse into the wall, nudge forward, turn
by encoder count (`rotate(146)`), strafe out, run 550 mm, reverse, turn
again (`rotate(166)`), and finish with two small trim rotations. Every turn
in the routines is encoder-counted; the heading sensor holds the heading
during straights and strafes, and drives the recovery turn below.

### Heading recovery (routine 8)

In every step where the robot must point north (up the field), `runRoutines()`
checks the heading against the mat's north: the heading captured by the last
back-wall squaring (`headingZero()` on the back switch, and at power-on). The
corner legs — routine 4 states −5..−1 and routine 7 states 4..12, where the
robot is deliberately turned — are not checked. A stale sensor reads 0, so it
never triggers without a reading.

`headingLostDeg` (70°) or more off north and routine 8 takes over, whatever the
robot was doing: stop, rotor off, then turn open-loop at `headingTurnPWM` (230)
while watching the sensor until within `headingSquareDeg` (8°) of north, then
run the return (routine 6): reverse to the back wall — which squares the robot
and re-captures north on the switch — read the camera, pick the next lane. The
spin is capped at the 4 s move timeout like every other move; after it the
return runs regardless and the check runs again on the next straight. The three
numbers are in the robot's block in [`src/Hardware.cpp`](src/Hardware.cpp).

Routine 5 is the diagonal lane: strafe in, reverse, forward, then a genuine
diagonal (`forwardLeft` / `forwardRight`, which drives only two of the four
wheels at full power to the count, with no ramp or deceleration), then a long
`forwardq`.

### What the rotor and gate are doing

`enableDrivers()` = 254, `enableSlowDrivers()` = 90/180, `disableDrivers()` = 0.
Direction is fixed once at boot and never changes — only speed varies. The rotor
is off during rotations, full speed on the straights, and slowed near the end of
a sweep.

### Mid and late game kicks (an option, off by default)

`GENERAL_ENDGAME_KICKS` in [`src/Strategy.h`](src/Strategy.h) switches the
match clock in `updateEndgameTiming()` on. Times count from `startTime`, set
at the end of `setup()` before the camera scan:

| Constant | Default | What happens |
|---|---|---|
| `GENERAL_MID_KICK_MS` | 45 s | lane forced (OUTER, or MIDDLE if already in routine 4 on OUTER), straight −30 mm; the next time routine 4 finishes a straight it goes to **routine 9** (LEFT: camera ball tracking; RIGHT: routine 10 — back 50, left 500) |
| `GENERAL_MID_DONE_MS` … `GENERAL_MID_END_MS` | 61 s … 100 s | straight restored, normal laps resume |
| `GENERAL_LATE_KICK_MS` | 105 s | lane OUTER, straight −30 mm, routine 9 for the rest of the match |

With the option off nothing ever sets the flags (`lastRoutine`, `midRoutine`,
`midRoutineDone`) and routines 9 and 10 are unreachable. The boot line and the
Bluetooth status block show `general, kicks on` / `general, kicks off`.

---

## 4. Code architecture

| File | Owns |
|---|---|
| `src/main.cpp` | `setup()` / `loop()` |
| `src/Hardware.*` | everything physical that *moves*: **the robot selector** + per-robot tuning, wheel geometry, motors, `Move`, servo, rotor and their pins, `initHardware()` |
| `src/Sensors.*` | everything that *senses* and its pins: the four encoders, BNO08x heading (polling, references, sign, fail-safes), Pixy2, microswitches, `initSensors()`, I2C bus scan — and the telemetry (status block + table on USB and Bluetooth) |
| `src/Strategy.h` | what a strategy must define (the state globals and the four functions `main.cpp` calls), which strategies exist, the general strategy's options, `mili` |
| `src/generalStrategy.cpp` | **the general strategy**: the routine/state machine, purple ball detector, camera lane choice, microswitch handling, endgame clock |
| `src/controlStrategy.cpp` | **the control strategy** (wall robot): the same interface, its own routines |
| `lib/move/move.h` | motion primitives (non-blocking, in millimetres), `inner()`, `outer()` |
| `lib/move/WheelRegulator.h` | speed profile (encoders) + heading PID (BNO08x) |
| `src/test/` | bench programs, excluded from the competition builds |

Dependencies point one way: a strategy → `Hardware` / `Sensors` →
`lib/move`. `lib/move` knows nothing about the sensor or which robot it is on:
`initHardware()` hands it the heading through two function pointers, the
encoder counts per millimetre (`move.regulator.countsPerMM`, which every move
uses to convert its millimetres) and which way is inner. `Sensors` depends on
nothing else in `src/`, so the bench tests can compile it alone.

`loop()` order is fixed: `handleMicroSwitches()`, `updateEndgameTiming()`,
`headingUpdate()`, telemetry, then `runRoutines()` **last** (routine 4 state 5
contains a bare `return` that is expected to skip everything after it).

---

## 5. Building and running

Five PlatformIO environments in [`platformio.ini`](platformio.ini):

| Environment | What it builds | Baud |
|---|---|---|
| `general` | competition firmware with the **general strategy** (`src/` minus `src/test/` and `controlStrategy.cpp`) | 115200 |
| `control` | competition firmware with the **control strategy** (`src/` minus `src/test/` and `generalStrategy.cpp`) | 115200 |
| `square_test` | 500 mm square bench test — same `Move`, regulator and sensor | 115200 |
| `heading_test` | BNO08x readout only, the robot does not move | 115200 |
| `bt_passthrough` | AT-command bridge to the Bluetooth module, the robot does not move | 115200 |

`default_envs = general`: the VSCode toolbar buttons build and flash the general
strategy. For any other environment use the PlatformIO sidebar (Project Tasks →
env → Upload) or:

```
pio run -e control      -t upload
pio run -e square_test  -t upload
pio device monitor -e square_test
```

### Strategies

A strategy is one `.cpp` file that defines what [`src/Strategy.h`](src/Strategy.h)
declares: `routine`, `state`, `startTime`, `strategyName`, and
`selectOpeningRoutine()`, `handleMicroSwitches()`, `updateEndgameTiming()`,
`runRoutines()`. `main.cpp` calls those and nothing else, so it is the same for
every strategy. The environment picks the file; the other strategy is not
compiled, so it cannot interfere. The robot is still chosen in `Hardware.cpp`
and every strategy branches on `robotSide`.

- **Switching between matches**: upload the other environment. Then read the
  first status line on the phone (or the USB boot line): it names the robot
  and the strategy that is actually running, e.g. `robot LEFT (wall)   strategy
  general, kicks off`.
- **Adding a strategy**: copy `controlStrategy.cpp`, give it its own
  `strategyName`, copy the `[env:control]` block and exclude the other strategy
  files in its filter.

### Telemetry (competition firmware)

Everything below goes out identically on the USB serial (115200) and on
**Serial2** (TX2 = pin 16, RX2 = pin 17, **9600** — `BLUETOOTH_BAUD` in
`src/Sensors.cpp`), where a Bluetooth serial module streams it to a phone or
laptop while the robot drives. It never blocks: each `loop()` pass sends at most
one line, and only when both transmit buffers (128 bytes,
`SERIAL_TX_BUFFER_SIZE` in `platformio.ini`) have room for all of it.

**Status block** — at the end of `setup()` and again 5 s later, so a phone that
connects late still sees it:

```
----- status  t 0.0 s -----
robot     LEFT (wall)   strategy general, kicks off   straight 1100 mm
heading   OK      BNO08x 0x4A   reports 812   resets 0
camera    OK      Pixy2 firmware 3.0.11
switches  back 1  side 1  start 0   (1 = open)
encoders  0 0 0 0   errors 0 0 0 0
opening   routine 1   purple ball upper right
check     ALL OK
```

| Line | Meaning |
|---|---|
| `robot` | which build is running and its main straight — the first thing to check on the wrong robot |
| `heading` | `OK`, `STALE` (found, but silent for 1 s) or `NOT FOUND`; I2C address, reports received, resets |
| `camera` | `OK` with the Pixy2 firmware version, or `FAIL` with the `pixy.init()` error code |
| `switches` | raw level of the back, side and start switch pins; pulled up, so 1 = open |
| `encoders` | count and skipped-transition errors of motor1..motor4 — errors must stay 0 |
| `opening` | the routine `selectOpeningRoutine()` chose (0–3 with the ball position, or 4 with no ball) |
| `check` | `ALL OK`, or `PROBLEM:` followed by what failed — `heading`, `camera`, `encoder-errors` |

**Table** — one row every 250 ms, header repeated every 20 rows:

```
  r   s     hdg     err   corr  pwm1 pwm2 pwm3 pwm4     v1    v2    v3    v4
  4   2    -1.2   -0.35    3.2   232  240  228  235    310   305   312   300
  4   2    -1.3   -0.41    3.7   233  239  229  234    312   301   315   298
```

| Column | Meaning |
|---|---|
| `r`, `s` | routine and state |
| `hdg` | heading since power-on, degrees (0 when the sensor is stale) |
| `err` | heading error the regulator sees, degrees (0 when the sensor is stale) |
| `corr` | heading differential being applied, PWM (±40 max) |
| `pwm1`–`pwm4` | PWM on motor1..motor4 — rear right, rear left, front left, front right; 0 for a released wheel |
| `v1`–`v4` | speed of motor1..motor4 in mm/s, sign follows the encoder's counting direction |

`TELEMETRY = false` in `src/Sensors.cpp` silences both ports.

**Bluetooth module** (JY-MCU carrier with an HC-05 or HC-06): module RX ← pin 16,
module TX → pin 17, GND ← GND, VCC ← 5 V. The competition firmware never reads
from it, but `bt_passthrough` does. 9600 is the factory rate of both modules, so
a new one works with no configuration: pair it on the phone (PIN `1234`, or
`0000`), connect from a Bluetooth serial-terminal app, and the lines appear.

The module is discoverable from the moment it has power, whatever the Mega is
running: a phone that cannot find it has a module, pairing or phone problem,
never a firmware one. Classic Bluetooth (HC-05/HC-06) is invisible to iPhones,
which only expose BLE to apps. Connected but silent means the Mega is not
sending (the USB monitor shows the same lines), the module RX is not on pin 16,
or the module is not at 9600.

**From a laptop (Windows)**: pair the module in Settings → Bluetooth & devices
(PIN `1234`). Windows then creates two virtual COM ports for it, listed under
Settings → Bluetooth & devices → Devices → More Bluetooth settings → COM Ports;
use the one marked **Outgoing**. Open it with the PlatformIO monitor from the
PlatformIO terminal:

```
pio device monitor -p COM5 -b 9600
```

`pio device list` shows the port numbers. Opening the port is what makes the
laptop connect — the module's LED goes solid a second or two later. The module
accepts one connection at a time, so disconnect the phone app first. The `-b`
value is ignored on a Bluetooth port (the real rate is on the wire between the
Mega and the module); the toolbar monitor button keeps opening the USB port.

`bt_passthrough` is the bench tool for the module itself. Flash it and open its
monitor: it probes the module with `AT` at boot and says whether it answered,
which alone confirms power, wiring and rate; then anything you type is sent as
an AT command. `MODULE_BAUD` and `LINE_ENDING` at the top of
[`src/test/bt_passthrough.cpp`](src/test/bt_passthrough.cpp) select HC-06 (9600,
no line ending) or HC-05 in AT mode (38400, CR+LF); the file header lists the
commands for each — rename (`AT+NAMExxx`), PIN (`AT+PIN1234`), or a faster rate
(`AT+BAUD8` = 115200, after which `BLUETOOTH_BAUD` must be changed to match).

### First run checklist

1. **`heading_test`** — `rate` should read close to 400 Hz, `age` a few ms,
   `rst` 0. If `rate` is far below 400 the sensor is delivering a lower rate;
   if `age` climbs or `rst` counts up, set `I2C_CLOCK_HZ` in `Sensors.cpp` back
   to 100000 and check the sensor's supply.
2. **`square_test`** — after the first turn the line `SIGN: OK` must appear. If
   it says `FLIP`, change `HEADING_SIGN` in `Sensors.cpp` and re-flash: with the
   wrong sign the heading loop steers away from straight instead of back to
   it. Then watch `heading` settle toward 0 on each side and the
   robot finish where it started. After each move the line `overshoot X mm`
   says how far past the target the front wheels stopped, and `encoder errors`
   shows skipped transitions per encoder — those must stay at 0 during moves;
   if they climb, the wheels are travelling further than they count.
3. **Competition firmware** — `hdg` follows the robot when it is turned by
   hand, and `corr` goes visibly non-zero when it is nudged during a straight.

---

## 6. Known behaviours the robot is tuned around

Each is marked `KNOWN` where it appears in the code.

| Where | Behaviour |
|---|---|
| routine 4 state −2, routine 7 state 6 | `outer(95)` on LEFT, `outer(143)` on RIGHT — a per-robot distance |
| routine 4 state −3 | bare `if` on `forwardp`, so the state ends at 14/22 of 400 mm |
| routine 4 state 5 | bare `return` for LEFT on INNER — `runRoutines()` must be last in `loop()` |
| routine 6 state 6 | `classifyLane(..., true)` on both robots |
| routine 6 state 7 | sequential `if`s, later ones override earlier ones |
| routine 9 | `while (true)` strafes block the whole firmware until they finish |
| `handleMicroSwitches()` | `int` timers wrap every 32.767 s |
| `startTime` | `int`, wraps every 32.767 s; only read by the disabled endgame block |
| routine 5 state 2, routine 7 states 2 and 10 | per-robot nudges: `backward(126 / 84)`, `forward(17 / 11)`, `backward(4 / 3)` (LEFT / RIGHT) |
