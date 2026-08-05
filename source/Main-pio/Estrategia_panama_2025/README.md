# Estrategia Panamá 2025 — WRO RoboSports firmware

Team Outer Heaven. Firmware for both autonomous ping-pong robots.

- **[README.md](README.md)** (this file) — the game, the strategy, what changed, open questions
- **[lib/move/Movement.md](lib/move/Movement.md)** — pin map, timers, movement internals, maintenance rules

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

The job is therefore twofold, and the second half of it is what makes the game
hard:

> Push your orange balls across to the other half — then **keep doing it**, because
> the opposing team is pushing theirs back at you for the whole match. Meanwhile
> the purple ball must never leave your own half.

A match lasts a **random 1 to 2 minutes**, and the exact duration is only decided
immediately before the match starts. A game is three matches.

Sources: [WRO 2026 RoboSports General Rules](https://wro-association.org/wp-content/uploads/WRO-2026-RoboSports-Double-Tennis-General-Rules.pdf) ·
[WRO 2025 rules](https://wro-association.org/wp-content/uploads/WRO-2025-RoboSports-Double-Tennis-General-Rules.pdf) ·
[WRO India category overview](https://wroindia.org/wro-india-season-2025-robosports-category/)

### Why the match being 1–2 minutes matters to this code

The disabled endgame block keys off **45 s, 61 s and 105 s**. Those are not
arbitrary: 61 s is "we are past the earliest possible finish" and 105 s is "we are
near the latest possible finish". The robot was being taught to change lane
strategy as the match approached either possible ending. See
[question 10](#6-open-questions-and-worries).

---

## 2. The two robots

One firmware, one robot selected by commenting a block in
[`src/RobotConfig.cpp`](src/RobotConfig.cpp).

| | LEFT — "WALL" | RIGHT — "RAMP" |
|---|---|---|
| `lenght` (main straight) | 1100 mm | 640 mm |
| `pulses` per wheel rev | 900 | 1650 |
| Forward PWM | 245,243,243,245 | 230,243,243,230 |
| Strafe PWM | 220,225,220,225 | 200,200,200,200 |
| Rotor slow speed | 90 | 180 |
| Gate closed / open | 170° / 55° | 96° / 0° |
| Waits for start switch | **yes** | no |

The names and the very different `lenght` values say the two robots divide the
half between them — one works the long run along the wall, the other the shorter
region around the ramp.

Both are four-wheel omnidirectional (45° rollers) on an Adafruit Motor Shield v1,
with a rotor that shoots or stores balls and a servo-driven gate that selects
between those two modes.

---

## 3. How the code plays the game

### The building block: lanes

The robot sweeps its half in **lanes** — `OUTER`, `MIDDLE`, `INNER` — parallel
runs at different distances from the side wall (author-confirmed):

- **OUTER** = furthest from the centre wall. For the RIGHT robot that is to its
  right; for the LEFT robot, to its left.
- **INNER** = closest to the centre wall.

That is exactly what `outer()` and `inner()` encode: `outer()` strafes right on
RIGHT and left on LEFT, and `inner()` does the opposite.

One lap is:

```
   drive the straight  →  return  →  look at the camera  →  pick the next lane
        routine 4/5           routine 6                        routine 6 state 7
```

`lane` is chosen by *where the orange balls actually are*, not by a fixed
rotation. That is the core of the strategy.

### Choosing a lane from the camera (routine 6, states 5–7)

The robot backs up, then reads the Pixy. Every orange blob is classified into one
of three *franjas* by `classifyLane()`, which splits the image with two diagonal
boundary lines. Each blob contributes its **area** (`width × height`) to that
franja's weight — so nearer/bigger clusters of balls count more heavily. The
heaviest franja wins and maps to a lane, mirrored per robot:

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

**The gate exists to capture the purple ball inside the robot** (author-confirmed).
The servo physically opens and closes a gate; opening it lets the purple ball be
taken in and held. So far this is only used at the start of the match, which is
exactly why routines 0–3 are the only places the gate is scheduled.

That is also why each opening routine schedules the gate differently — the gate
has to be open at the moment the robot is passing over wherever the purple ball
was seen:

| Routine | Purple seen | Gate schedule |
|---|---|---|
| 0 | upper-left | closed → forward 505 → **open** → forward 350 → closed |
| 1 | upper-right | closed → forward 530 → **open** → forward 350 → closed |
| 2 | lower-left | **open** → forward 550 → closed → forward 250 |
| 3 | lower-right | **open** → forward 550 → closed → forward 250 |

The nearer the ball, the earlier the gate opens — routines 2 and 3 (lower half of
the camera frame, so closer to the robot) open immediately, while 0 and 1 drive
roughly 500 mm first and only then open.

### The main lane run (routine 4)

**MIDDLE / INNER** — states 0 → 5:

1. `backward(280)` — reverse to square up against the back wall
2. pause
3. `forwardRegulated(lenght + 50)` — the sweep up the field. At **~64 %** of the
   distance the primitive returns `2` and the code calls `enableSlowDrivers()`,
   dropping the rotor to 90/180. **This is deliberate** (author-confirmed):
   shooting at full speed into the front barrier launches balls clean out of the
   field, which must not happen. The rotor is throttled before the robot gets
   there so the balls just clear the barrier.
4. two pauses
5. rotor back to full, advance the lane, go to routine 6

**OUTER** — states −1 → −6 (the state counts *down*):

1. rotor off, `rotate(166)` — turn
2. strafe outward
3. `forwardp(400)` — a shorter run
4. rotor on, pause
5. rotor off, `rotate(166)` back
6. strafe outward again → drop into state 0 and run the straight above

### Returning and re-localising (routines 6 and 7)

Routine 6 drives back `lenght + 250` — **further than the field is long**. That
over-travel is deliberate (author-confirmed): the **back microswitch** fires when
the robot reaches the wall and the switch handler advances `state`. The bumper,
not the encoder, is what normally ends the move. The same pattern appears in
routine 7.

The switches are accurate enough to stop movement just before the robot hits the
wall — **but only when the robot is square to it**. If it arrives misaligned, one
corner touches first and the switch may not fire, and then the encoder distance
(or the 4 s timeout) is what ends the move instead. This is the main reason the
corner reset in routine 7 exists.

Routine 7 is the full corner reset: reverse into the wall, nudge forward, then
**rotate on the gyro to ±80°** (`ang_z` vs `alpha ± beta`), strafe out, run
550 mm, reverse, rotate again, and finish with two small trim rotations. If the
gyro never answered at boot (`mpu == false`) it falls back to encoder-counted
rotations. This is how the robot recovers its heading after a lap of accumulated
drift.

Routine 5 is the diagonal lane: strafe in, reverse, forward, then a genuine
diagonal (`forwardLeft` / `forwardRight`, which drives only two of the four
wheels), then a long `forwardq`.

### What the rotor and gate are doing

`enableDrivers()` = 254, `enableSlowDrivers()` = 90/180, `disableDrivers()` = 0.
Direction is fixed once at boot and never changes — only speed varies. The rotor
is off during rotations, full speed on the straights, and slowed near the end of
a sweep.

---

## 4. Code architecture

Six modules with a strict one-way dependency direction. Full detail, pin map and
timer ownership in **[lib/move/Movement.md](lib/move/Movement.md)**.

| File | Owns |
|---|---|
| `src/main.cpp` | `setup()` / `loop()` only |
| `src/RobotConfig.*` | **the robot selector** + per-robot tuning |
| `src/Hardware.*` | pins, motors, encoders, `Move`, servo, rotor |
| `src/Sensors.*` | gyro, Pixy2, I2C, microswitches |
| `src/Motion.*` | `mm()`, `inner()`, `outer()` |
| `src/Routines.*` | the routine/state machine |
| `lib/move/move.h` | motion primitives |
| `lib/move/WheelRegulator.h` | ramp shaping + four-wheel synchronisation |

---

## 5. What changed, and why

### 5.1 Closed-loop movement (new)

**Before:** each wheel was driven at a fixed PWM and the move stopped when the
front encoders had counted far enough. Nothing kept the four wheels turning at the
same rate, so differences in motor strength, friction, wheel load and battery sag
made the robot drift, judder and finish crooked.

**After:** [`WheelRegulator.h`](lib/move/WheelRegulator.h) adds

- **four-wheel synchronisation** — every motion turns each driven wheel through
  the same number of counts, so the rule is "make every driven wheel travel the
  same distance". The slowest wheel is the reference and anything ahead of it is
  trimmed *down*. Trimming down never saturates, which matters because nominal
  PWM is already 243–245 of 255.
- **acceleration and deceleration ramps**, measured in encoder counts rather than
  milliseconds so they behave identically regardless of loop speed.

Because the comparison uses `abs()` of each wheel's travel, **encoder A/B polarity
does not matter** and no calibration is needed.

**Distance measurement was deliberately left alone** — still the front pair, still
`abs()`, still "either one arrives". That is why every tuned distance survived.

### 5.2 Restructure (behaviour-preserving)

One 1170-line `main.cpp` became six modules. Clean-code practices applied:

| Practice | What it looks like here |
|---|---|
| Single responsibility | one module per concern; hardware never mixed with strategy |
| One-way dependencies | `Routines` → `Motion`/`Sensors`/`Hardware`; never the reverse |
| Single point of configuration | both robots live in one file, nothing else changes |
| Named over magic | pin map, timer ownership and units documented at the definition |
| Intent recorded at the site | every deliberate oddity marked `KNOWN` with the consequence of "fixing" it |
| Dead code labelled, not deleted | `UNUSED` markers so nothing is silently lost |

**This was verified mechanically, not by eye.** With comments and whitespace
stripped, the 674-line state machine has an **identical MD5** to the original;
all twelve helper functions are identical; `setup()` has an identical
78-statement set. Only three intentional changes exist, all listed in
[Movement.md §11](lib/move/Movement.md).

### 5.3 Real fixes

- **The RIGHT-RAMP build works again.** `lenght` was declared inside the LEFT
  block only, so selecting the ramp robot did not compile. Both configurations
  are now build-verified.
- **The I2C bus scan moved to `setup()`.** It was probing all 126 addresses on
  *every* pass of `loop()`, making the control period long and irregular.
- **Interrupted moves no longer measure from stale counts.** When a microswitch
  advances `state` mid-move, `moving` stayed true and the next movement skipped
  its start-count capture. Mostly masked by the `move → stopForMillis` pattern,
  but reachable in sequences like routine 5 states 4→6.
- **Diagonals get a real timeout.** `forwardLeft`/`forwardRight`/`backwardLeft`/
  `backwardRight` never stamped `moveStartTime`, inheriting it from the previous
  move — so they could time out early.

### 5.4 Deliberately preserved defects

Per instruction, field behaviour was preserved exactly. Each is marked `KNOWN` in
the code: the `outer(mm(20))` double conversion, `classifyLane(..., true)` being
hard-coded, the 16-bit `millis()` truncation, and the `case 8` fallthrough.

---

## 6. Open questions and worries

Answered by the author and folded into the sections above: the gate captures the
purple ball, the rotor slows so balls do not fly out over the barrier, the
microswitches normally end the long reverses, OUTER/INNER are measured from the
centre wall, and all four encoder pin pairs are confirmed.

### 1. `case -3` of routine 4 ends its move at ~250 mm, not 400 mm

This is the one I explained badly last time — it is `case -3`, **negative three**,
part of the OUTER-lane sequence where `state` counts *down* from −1 to −6. It is
not `case 3`.

[`Routines.cpp:333-338`](src/Routines.cpp#L333-L338):

```cpp
case -3:
  if (robotSide == RIGHT){
    if(move.forwardp(mm(400), true)) state--;
  }else{
    if(move.forwardp(mm(400), false)) state--;
  }
  break;
```

`forwardp` returns **three** values, not a bool ([`move.h:161-165`](lib/move/move.h#L161-L165)):
`1` when the full distance is reached, `2` when `(pulses/22)*14` — about **64 %** —
is reached, and `0` otherwise. A bare `if` treats `2` as true, so `state--` fires
at 64 %. **The robot travels roughly 250 mm here, not 400 mm.**

Every other site that calls `forwardp` guards against this:

| Site | Guard |
|---|---|
| routine 4 `case 2` | `switch(test)` — `case 1` advances, `case 2` slows the rotor |
| routine 7 `case 8` | `if(... == 1)` |
| **routine 4 `case -3`** | **bare `if` — no guard** |

Was the short OUTER move intentional, or is this a missing `== 1`? Preserved
exactly as-is until you say.

### 2. Why is `pulses` 900 on one robot and 1650 on the other?
Different gearboxes, different encoder CPR, or different wheels? It is used as
counts-per-wheel-revolution.

### 3. What do the `position` / `d` trims compensate for?
`forwardp`/`backwardp`/`forwardq` bias one diagonal pair by ±6–9 PWM. Is that a
straightness trim for a robot that veers? The new synchroniser should now do that
job properly and closed-loop, so these may be removable — but I left them alone.

### 4. Do the two robots coordinate at all?
The rules stress collaboration, but I see no communication anywhere in the
firmware. Do they simply work disjoint areas (`lenght` 1100 vs 640) and rely on
never meeting? How do you avoid them colliding?

### 5. Is the endgame timing coming back this season?
The 45 s / 61 s / 105 s block is commented out, which makes **routines 9 and 10
unreachable**. Given matches are a random 60–120 s, was this the "match is nearly
over, switch to the outer lane" behaviour? Worth reviving deliberately rather than
leaving dormant.

### 6. Why does LEFT wait for the start switch and RIGHT not?
Is the ramp robot started by hand, or is it triggered some other way?

### 7. `classifyLane` is always passed `true`
So the LEFT robot uses the RIGHT robot's boundary constants. Given the lane
mapping is *also* mirrored by `robotSide` downstream, is that double-mirroring
accidental or does it happen to produce the behaviour you want?

### 8. What is the `first` flag protecting?
On the first lap, LEFT always jumps to routine 5 (diagonal) and the camera read is
skipped when the lane is OUTER. Is the first lap special because the field is
still in its known starting layout?

### 9. How far should the English-only rule go?
You have set English as the official language. Comments and log strings are safe
to translate. Renaming identifiers is compiler-checked and therefore safe too, but
it touches many lines, so I want your call before doing it. Currently non-English
or misspelled:

| Kind | Symbols |
|---|---|
| Identifiers | `pesos`, `NUM_FRANJAS`, `ANCHO_IMAGEN`, `franja`, `mejorFranja`, `tamano`, `velocidad`, `mili`, `lenght` (misspelling of *length*) |
| Comments | the gyro variable block, `Clasifica en franja…`, `Calcula tamaño como área`, `Suma al peso de la franja`, `Encuentra la franja con mayor peso`, `Iniciando I2C`, `definir routine` |
| Runtime strings | `Serial.print("antes de la formula ")`, `Serial.println("Sensor iniciado correctamente")`, `Serial.println("Error al iniciar el sensor")` |

Note `mili` is a global constant (250 ms pause) *and* the parameter name of
`inner()`/`outer()`, where it shadows the global — worth renaming for that reason
alone.

### 10. Worry: distances will have changed
The much faster loop plus the new deceleration ramp both reduce overshoot, so the
robot will travel **slightly shorter** than before for the same numbers. Nothing
in the code changed — the robot just stops when it is told to now. Expect to
re-verify distances on the field, and `move.regulator.endFactor = 255` disables
the ramp-down if you want the old stopping behaviour while you check.
