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

One firmware, one robot selected by commenting a block in
[`src/RobotConfig.cpp`](src/RobotConfig.cpp).

| | LEFT — "WALL" | RIGHT — "RAMP" |
|---|---|---|
| `lenght` (main straight) | 1100 mm | 640 mm |
| `pulses` (encoder counts per wheel rev) | 900 | 1350 |
| `pwmf` forward trims | 245, 243, 243, 245 | 220, 243, 243, 220 |
| `pwms` strafe trims | 220, 225, 220, 225 | 200, 200, 200, 200 |
| Rotor slow speed | 90 | 180 |
| Gate closed / open | 170° / 55° | 96° / 0° |
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

`outer()` strafes right on RIGHT and left on LEFT; `inner()` does the opposite
([`src/Motion.cpp`](src/Motion.cpp)).

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
rectangles, `ballZones` in `src/RobotConfig.cpp` (one table per robot; the
format and a calibration guide are in `src/RobotConfig.h`). The row index of the
matched rectangle is the routine:

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
`src/Routines.cpp`. It prints one `blob edges L.. T.. R.. B..` line per voting
frame and a final `ball scan .. ms votes ..` summary on Serial, which is what
you use to read off the rectangles when calibrating.

**The gate captures the purple ball inside the robot.** Opening it lets the
purple ball be taken in and held, so each opening routine schedules the gate
to be open at the moment the robot passes over where the ball was seen:

| Routine | Purple seen | Gate schedule |
|---|---|---|
| 0 | upper-left | closed → forward 505 → **open** → forward 350 → closed |
| 1 | upper-right | closed → forward 530 → **open** → forward 350 → closed |
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

1. rotor off, `rotate(mm(166))` — turn
2. `outer(mm(20))` — strafe outward (KNOWN: mm applied twice, ≈453 counts)
3. `forwardp(mm(400))` — wall-hugging run. KNOWN: `forwardp` returns `2` at
   14/22 of the distance and the bare `if` accepts it, so this state ends at
   ≈255 mm with the motors still running; state −4 releases them.
4. rotor on, pause
5. rotor off, `rotate(mm(166))` back
6. strafe outward again → drop into state 0 and run the straight above

On the OUTER lane the straight (routine 4 state 2) uses `forwardp` and the
return (routine 6 state −1) uses `backwardp`: two wheels are trimmed ±9 / ±6 so
the robot presses against the wall it runs along. Those moves run the speed profile only (no
heading hold) — the wall aligns them.

### Returning and re-localising (routines 6 and 7)

Routine 6 drives back `lenght + 250` — **further than the field is long**. The
**back microswitch** fires when the robot reaches the wall and the switch handler
advances `state`; the bumper, not the encoder, normally ends the move. The same
switch press zeroes the heading reference (`headingZero()`), because the robot
is square against the wall at that moment. Because the wall comes before the
commanded distance, backward moves longer than 200 mm brake down to 200 mm/s
and hold that speed over the last 150 mm of the command (the wall approach in
[Movement.md](lib/move/Movement.md)), so the bumper is met at that speed.

The switches are accurate enough to stop movement just before the wall — but
only when the robot arrives square to it. If it arrives misaligned, one corner
touches first, and the encoder distance or the 4 s move timeout ends the move
instead. That is what the corner reset in routine 7 recovers from.

Routine 7 is the full corner reset: reverse into the wall, nudge forward, turn
by encoder count (`rotate(mm(146))`), strafe out, run 550 mm, reverse, turn
again (`rotate(mm(166))`), and finish with two small trim rotations. Every turn
in the routines is encoder-counted; the heading sensor only holds the heading
during straights and strafes.

Routine 5 is the diagonal lane: strafe in, reverse, forward, then a genuine
diagonal (`forwardLeft` / `forwardRight`, which drives only two of the four
wheels), then a long `forwardq`.

### What the rotor and gate are doing

`enableDrivers()` = 254, `enableSlowDrivers()` = 90/180, `disableDrivers()` = 0.
Direction is fixed once at boot and never changes — only speed varies. The rotor
is off during rotations, full speed on the straights, and slowed near the end of
a sweep.

### Endgame timing (disabled)

`updateEndgameTiming()` and its flags (`lastRoutine`, `midRoutine`,
`midRoutineDone`) are commented out, so routines 9 and 10 are unreachable. The
block and both routines are kept so the behaviour can be re-enabled by
uncommenting it; `startTime` must become `unsigned long` first (it is a 16-bit
`int` today).

---

## 4. Code architecture

| File | Owns |
|---|---|
| `src/main.cpp` | `setup()` / `loop()` and the telemetry line |
| `src/RobotConfig.*` | **the robot selector** + per-robot tuning |
| `src/Hardware.*` | pins, motors, encoders, `Move`, servo, rotor, regulator band |
| `src/Heading.*` | BNO08x: polling, references, sign, fail-safes |
| `src/Sensors.*` | Pixy2, I2C bus scan, microswitches |
| `src/Motion.*` | `mm()`, `inner()`, `outer()` |
| `src/Routines.*` | the routine/state machine |
| `lib/move/move.h` | motion primitives (non-blocking) |
| `lib/move/WheelRegulator.h` | speed profile (encoders) + heading PID (BNO08x) |
| `src/test/` | bench programs, excluded from the competition build |

Dependencies point one way: `Routines` → `Motion` / `Sensors` / `Hardware` /
`Heading` → `lib/move`. `lib/move` knows nothing about the sensor; it receives
heading through two function pointers set in `initHardware()`.

`loop()` order is fixed: `handleMicroSwitches()`, `updateEndgameTiming()`,
`headingUpdate()`, telemetry, then `runRoutines()` **last** (routine 4 state 5
contains a bare `return` that is expected to skip everything after it).

---

## 5. Building and running

Four PlatformIO environments in [`platformio.ini`](platformio.ini):

| Environment | What it builds | Baud |
|---|---|---|
| `megaatmega2560` | the competition firmware (`src/` minus `src/test/`) | 115200 |
| `square_test` | 500 mm square bench test — same `Move`, regulator and sensor | 115200 |
| `heading_test` | BNO08x readout only, the robot does not move | 115200 |
| `bt_passthrough` | AT-command bridge to the Bluetooth module, the robot does not move | 115200 |

`default_envs = megaatmega2560`: the VSCode toolbar buttons build and flash the
competition firmware. For the tests use the PlatformIO sidebar (Project Tasks →
env → Upload) or:

```
pio run -e square_test  -t upload
pio device monitor -e square_test
```

### Telemetry (competition firmware)

One line every 250 ms, identical on the USB serial (115200) and on **Serial2**
(TX2 = pin 16, RX2 = pin 17, **9600** — `BLUETOOTH_BAUD` in `src/main.cpp`),
where a Bluetooth serial module streams it to a phone or laptop while the robot
drives:

```
r4 s2   MIDDLE  ball 180,25  hdg   -1.2  err  -0.35  corr   3.2  pwm 232 240 228 235  v  310  305  312  300
r4 s2   MIDDLE  ball 180,25  hdg   -1.3  err  -0.41  corr   3.7  pwm 233 239 229 234  v  312  301  315  298
```

Columns are fixed width, so lines stack into a table.

| Column | Meaning |
|---|---|
| `r`, `s` | routine and state |
| lane | `OUTER` / `MIDDLE` / `INNER` — the lane the robot is committed to. Changes when routine 6 picks the next one from the camera weighting |
| `ball` | where the purple ball was seen at boot, Pixy image pixels `x,y` (the `ballZones` rectangle it matched picked the opening routine); `none` if it never was |
| `hdg` | heading since power-on, degrees (0 when the sensor is stale) |
| `err` | heading error the regulator sees, degrees (0 when the sensor is stale) |
| `corr` | heading differential being applied, PWM (±40 max) |
| `pwm` | PWM on motor1..motor4 — rear right, rear left, front left, front right; 0 for a released wheel |
| `v` | speed of motor1..motor4 in mm/s, sign follows the encoder's counting direction |

Sensor health (report rate, age, resets) is what `heading_test` shows; the
competition line does not carry it.

`TELEMETRY = false` in `src/main.cpp` silences it. The line is built in a buffer
and written only when both transmit buffers (128 bytes, `SERIAL_TX_BUFFER_SIZE`
in `platformio.ini`) have room for all of it, so `write()` never waits for the
UART and `loop()` never stalls for it — at 9600 the longest line leaves in
about 115 ms, inside the 250 ms period.

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
   if `age` climbs or `rst` counts up, set `I2C_CLOCK_HZ` in `Heading.cpp` back
   to 100000 and check the sensor's supply.
2. **`square_test`** — after the first turn the line `SIGN: OK` must appear. If
   it says `FLIP`, change `HEADING_SIGN` in `Heading.cpp` and re-flash: with the
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
| routine 4 state −2, routine 7 state 6 | `outer(mm(20))`: `outer()` converts with `mm()` internally, so this is `mm(mm(20))` ≈ 453 counts |
| routine 4 state −3 | bare `if` on `forwardp`, so the state ends at 14/22 of 400 mm |
| routine 4 state 5 | bare `return` for LEFT on INNER — `runRoutines()` must be last in `loop()` |
| routine 6 state 6 | `classifyLane(..., true)` on both robots |
| routine 6 state 7 | sequential `if`s, later ones override earlier ones |
| routine 9 | `while (true)` strafes block the whole firmware until they finish |
| `handleMicroSwitches()` | `int` timers wrap every 32.767 s |
| `startTime` | `int`, wraps every 32.767 s; only read by the disabled endgame block |
| `Motion.cpp` | some call sites pass raw encoder counts, not mm: `forward(80)`, `backward(600)`, `outer(750)`, `inner(180)` |
