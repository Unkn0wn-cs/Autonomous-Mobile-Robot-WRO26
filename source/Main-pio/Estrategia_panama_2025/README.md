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
| `pulses` (encoder counts per wheel rev) | 900 | 1650 |
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
200 × 200 mm base, a BNO08x heading sensor on I2C, a Pixy2 camera on SPI, a
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

Before the match runs, `selectOpeningRoutine()` scans up to 120 Pixy frames for
the purple ball and classifies it into a quadrant around `(x=200, y=32)`:

| Purple ball seen | Routine | Then continues on lane |
|---|---|---|
| upper-left | 0 | INNER (LEFT) / OUTER (RIGHT) |
| upper-right | 1 | MIDDLE |
| lower-left | 2 | INNER (LEFT) / OUTER (RIGHT) |
| lower-right | 3 | MIDDLE |
| not found | 4 | keeps default MIDDLE |

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
the robot presses against the wall it runs along. Those moves run ramp-only (no
wheel sync, no heading hold) — the wall aligns them.

### Returning and re-localising (routines 6 and 7)

Routine 6 drives back `lenght + 250` — **further than the field is long**. The
**back microswitch** fires when the robot reaches the wall and the switch handler
advances `state`; the bumper, not the encoder, normally ends the move. The same
switch press zeroes the heading reference (`headingZero()`), because the robot
is square against the wall at that moment.

The switches are accurate enough to stop movement just before the wall — but
only when the robot arrives square to it. If it arrives misaligned, one corner
touches first, and the encoder distance or the 4 s move timeout ends the move
instead. That is what the corner reset in routine 7 recovers from.

Routine 7 is the full corner reset: reverse into the wall, nudge forward, then
**turn on the heading sensor to ±80°** (`headingSinceZero()` against
`alpha ± beta`, `beta` = 8°), strafe out, run 550 mm, reverse, turn again, and
finish with two small trim rotations. If the sensor is not available it falls
back to encoder-counted turns (`mm(146)` / `mm(166)`).

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
| `lib/move/WheelRegulator.h` | ramp, four-wheel sync, heading hold |
| `src/test/` | bench programs, excluded from the competition build |

Dependencies point one way: `Routines` → `Motion` / `Sensors` / `Hardware` /
`Heading` → `lib/move`. `lib/move` knows nothing about the sensor; it receives
heading through two function pointers set in `initHardware()`.

`loop()` order is fixed: `handleMicroSwitches()`, `updateEndgameTiming()`,
`headingUpdate()`, telemetry, then `runRoutines()` **last** (routine 4 state 5
contains a bare `return` that is expected to skip everything after it).

---

## 5. Building and running

Three PlatformIO environments in [`platformio.ini`](platformio.ini):

| Environment | What it builds | Baud |
|---|---|---|
| `megaatmega2560` | the competition firmware (`src/` minus `src/test/`) | 115200 |
| `square_test` | 500 mm square bench test — same `Move`, regulator and sensor | 115200 |
| `heading_test` | BNO08x readout only, the robot does not move | 115200 |

`default_envs = megaatmega2560`: the VSCode toolbar buttons build and flash the
competition firmware. For the tests use the PlatformIO sidebar (Project Tasks →
env → Upload) or:

```
pio run -e square_test  -t upload
pio device monitor -e square_test
```

### Telemetry (competition firmware)

One line every 250 ms at 115200:

```
r=4 s=2 err=-0.35 corr=3.2 age=3 rst=0 hz=1240
```

| Field | Meaning |
|---|---|
| `r`, `s` | routine and state |
| `err` | heading error the regulator sees, degrees (0 when the sensor is stale) |
| `corr` | heading correction being applied, PWM (±20 max) |
| `age` | ms since the last sensor report |
| `rst` | sensor resets since boot — any value above 0 is a power problem to chase |
| `hz` | `loop()` passes per second — must stay well above 250 for the 4 ms control tick |

`TELEMETRY = false` in `src/main.cpp` silences it.

### First run checklist

1. **`heading_test`** — `rate` should read close to 400 Hz, `age` a few ms,
   `rst` 0. If `rate` is far below 400 the sensor is delivering a lower rate;
   if `age` climbs or `rst` counts up, set `I2C_CLOCK_HZ` in `Heading.cpp` back
   to 100000 and check the sensor's supply.
2. **`square_test`** — after the first turn the line `SIGN: OK` must appear. If
   it says `FLIP`, change `HEADING_SIGN` in `Heading.cpp` and re-flash: with the
   wrong sign the heading loop steers away from straight and routines 7/8 turn
   the wrong way. Then watch `heading` settle toward 0 on each side and the
   robot finish where it started.
3. **Competition firmware** — `hz` well above 250, `age` small, `rst` 0, and
   `corr` visibly non-zero when the robot is nudged during a straight.

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
| routine 8 | no `break`, falls through into routine 9 |
| routine 9 | `while (true)` strafes block the whole firmware until they finish |
| `handleMicroSwitches()` | `int` timers wrap every 32.767 s |
| `startTime` | `int`, wraps every 32.767 s; only read by the disabled endgame block |
| `Motion.cpp` | some call sites pass raw encoder counts, not mm: `forward(80)`, `backward(600)`, `outer(750)`, `inner(180)` |
