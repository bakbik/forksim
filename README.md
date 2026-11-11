# forksim: when does the simulator lie about a pallet pickup?

**[Run the simulator in your browser](https://bakbik.github.io/forksim/)** (nothing to install; first load fetches about 10 MB)

A small 2D simulator of an autonomous pallet truck that finds a pallet with
lidar, drives to it and loads it. It was built to answer one question:

> The ideal simulator says the pickup always works. Which departures from ideal
> break it first, perception or control, and how much of each can it take?

The same Python package runs the browser demo, the batch experiments and the
tests. There is one implementation, not a demo and a "real" one.

- [Try it](#try-it)
- [What is simulated](#what-is-simulated)
- [Allowed inputs](#allowed-inputs)
- [The task, step by step](#the-task-step-by-step)
- [How a run is judged](#how-a-run-is-judged)
- [Sources of error](#sources-of-error)
- [Architecture](#architecture)
- [The experiment](#the-experiment)
- [Limitations](#limitations)
- [What I would measure on a real forklift](#what-i-would-measure-on-a-real-forklift)
- [Using it from Python](#using-it-from-python)
- [Repository layout](#repository-layout)

## Try it

### In the browser

Open **<https://bakbik.github.io/forksim/>**. The page downloads Python
(Pyodide) and this package, then runs the simulation locally in your tab.

| Tab | What it does |
|---|---|
| **Run** | Pick a driver, press **Start**. *Trial number* selects the situation: the same number always gives the same pallet pose, start pose and hidden errors. *Next trial* moves to a new one. Speed up to 15x. |
| **Properties** | Room size; pallet position, angle, size and weight; truck start pose, max speed, max steer angle, minimum turning radius; approach distance **D**. The allowed range is shown under each field; a value outside it is refused with a reason. *Apply and restart* to use them. |
| **Errors** | Two sliders: how wrong perception is and how badly the truck obeys, from 0 (ideal simulator) through 1 (assumed real truck) to 3. |
| **Batch** | Runs 10 to 40 trials for both drivers with the current settings and tabulates how each ended. |

Drivers:

- **Actuator-aware** and **Baseline** are the two automatic controllers compared in the experiment.
- **Manual** lets you drive. Click the view first, then `↑` `↓` for speed, `←` `→` to steer, hold `Shift` for full lock, which pivots the truck on the spot.

What you are looking at:

| On screen | Meaning |
|---|---|
| Blue dots | Lidar returns from the latest scan, where they really hit |
| Grey dots | Cached map points the truck cannot currently see |
| Green / red lines on the pallet | Its two entry edges (forks go in) and two side edges (they do not) |
| Thick translucent lines | Edges the detector found in this scan, coloured by how it classified them |
| Orange dashed outlines | Where the truck *believes* the pallet is, and the approach target it is heading for |
| Orange bar at the back of the truck | The drive wheel, turned to its current steering angle |
| "truth" vs "robot thinks" | The two alignment errors as they really are, and as the truck estimates them |

Things worth trying: put the pallet at angle 0 so the truck starts facing a
*side* edge and has to work out where the entry edges are; shrink **D** to 0.5 m
and watch the accepted angle shrink with it; push the Errors sliders to 2 and
compare the two drivers in Batch; narrow the pallet to 0.77 m, the tightest fit the forks allow.

### On your machine

```bash
pip install numpy matplotlib pytest
```

```bash
python -m http.server 8765
```

Then open <http://localhost:8765/web/>.

```bash
python -m forksim.experiment
```

Runs every experiment described below (16,200 trials, about 20 minutes on 15
worker processes), writes `results/` and draws the figures. `--quick` runs a
tenth of it, `--seed N` a different but equally reproducible set.

```bash
python -m pytest -q
```

## What is simulated

| Thing | Model |
|---|---|
| **Room** | 20 x 30 m, origin in the bottom-left corner, x right, y up. Four walls, visible to the lidar. No other obstacles. |
| **Pallet** | Static EUR pallet, 1200 x 800 x 144 mm, 400 kg by default. Three stringers with two fork pockets 227.5 mm wide. Position, angle, size and weight are configurable. |
| **Truck** | SEER SFL-CBD15 autonomous pallet truck. From its data sheet: 1712 x 932 mm, forks 1220 x 170 mm at 550 mm outer width, 2 m/s, 1500 kg rated load, 1524 mm turning radius. Derived or assumed: wheelbase 1.2 m, load wheels 0.26 m behind the fork tips, steering lock 90 deg, all actuator dynamics. |
| **Truck motion** | Pallet-truck kinematics: a single wheel under the power unit drives and steers, the forks ride on load wheels, and the truck pivots about the load axle. |
| **Lidar** | One 2D scanner between the forks at block height: 360 deg, 0.25 deg resolution, 10 Hz, 25 m range. Ray cast against the pallet blocks and the walls. |
| **Camera** | Modelled in `sensors.py` but switched off (`CameraSpec.enabled = False`). The truck runs on lidar alone. |

The forks are 170 mm wide and the pockets 227.5 mm, so a fork has **25 mm of
room on each side**. That number sets how hard the task is.

## Allowed inputs

A configuration outside these ranges is rejected before anything runs:
`TrialRunner` raises `ValueError`, the web page refuses to start and says which
value is wrong, and `forksim.validate(cfg)` returns the list of problems.
`forksim.limits(cfg)` returns the ranges for a given configuration, because
several of them follow from other values rather than being fixed numbers.

| Property | Allowed | Why |
|---|---|---|
| Pallet size y (entry edge) | 0.77 to 2.5 m | Lower bound is computed: the forks at their 550 mm outer width, plus both 100 mm outer blocks, plus 10 mm clearance each side. Narrower and the forks cannot enter. Fit 680 mm forks and it becomes 0.90 m. |
| Pallet size x (along forks) | 0.6 to 2.5 m | From a half pallet up to well past any standard one |
| Pallet size z | 0.10 to 0.25 m | Not used by the 2D model |
| Pallet weight | 1 to 1500 kg | Upper bound is the truck's rated load |
| Pallet position | whole pallet at least 0.3 m inside the walls | Follows from room size and pallet size |
| Pallet angle, truck start angle | any | |
| Truck start position | at least 1 m inside the walls, and not on the pallet | |
| Truck max speed | 0.1 to 2.0 m/s | Data sheet maximum |
| Truck max steer angle | 10 to 90 deg | 90 deg is turning on the spot |
| Truck minimum turning radius | 0 to 5 m | |
| Approach distance D | 0.3 to 5 m | |
| Accepted share of possible approach error | 0.1 to 1 | |
| Room | 5 to 100 m each way | |

Inside these ranges every pallet size was checked to load in the ideal
simulator, at both ends of each range and in combination. Valid does not mean
easy: at 0.77 m the forks have 10 mm to spare instead of 25 mm, and with
realistic errors the truck scrapes a block.

## The task, step by step

The truck knows nothing about the room, the pallet's position or even its own
position in the room: odometry starts at zero. Every 100 ms it gets one lidar
scan and nothing else. It may cache scans in a local map, but **a new scan
always replaces whatever was cached where it looked**, so if the world changes
the map follows. The only thing known in advance is what this *type* of pallet
looks like: its edge lengths and block pattern.

### 1. Detect

`perception.py` pulls straight edges out of the points (RANSAC, then a
least-squares polish). Long ones are walls and are discarded. Short ones are
classified:

- **entry edge**: about 800 mm long and its points fall on the
  block-pocket-block-pocket-block pattern;
- **side edge**: about 1200 mm long and solid.

Either kind is enough to place the pallet, because the pallet's centre is half
a pallet behind the edge. An entry edge places it more precisely. The output is
the **pallet's position and angle relative to the truck**. It is a fit to noisy
points, so it is noisy, and that noise feeds everything downstream. Five
consistent detections in a row confirm a pallet; after that the estimate keeps
being updated for the rest of the run.

### 2. Approach

The pallet has two right edges and two wrong ones. The truck picks the nearer
entry edge whose approach target is clear and reachable in a straight line
without cutting through the pallet. The **approach target** is the point a
distance **D** in front of that edge's centre (fork tips to edge), facing it
squarely.

There are no obstacles, so the trajectory is the most trivial one this truck
can do: **turn on the spot, drive straight, turn on the spot.**

On arrival the truck has some positioning error. The phase is accepted only if
that error is small enough for the next phase to close. The limit is not a
tuned number. Given D, the truck's minimum turning radius R and the lateral
offset on arrival, `behaviors.acceptable_angle` works out the largest yaw error
that can still be removed: straightening up costs `R·sin(ψ)` of the distance and
carries the truck `R·(1−cos ψ)` further sideways, and what is left of D must be
enough for an S-curve to remove the total sideways offset. A share of that
kinematic limit (half, by default) is accepted.

| D | Accepted yaw error, no lateral offset | with 50 mm lateral offset |
|---|---|---|
| 0.5 m | 3.9 deg | rejected |
| 1.0 m | 7.9 deg | 5.7 deg |
| 1.5 m | 11.9 deg | 10.6 deg |
| 2.5 m | 20.9 deg | 20.0 deg |

A small D forces tiny errors. A large D gives the align phase room to settle
but leaves less freedom in where the approach can end.

### 3. Align

The truck closes the two remaining errors **while driving in**, never turning
tighter than R:

- **lateral offset** between the pallet's centre line and the fork centre line;
- **angle** between the pallet's entry edge and the truck's carriage face.

Perception stays in the loop until the scanner is 0.35 m from the edge, where
it can no longer frame it. The phase ends when the pallet touches the carriage.
That contact, with both forks in their pockets, is the one allowed contact and
it means the pallet is **loaded**.

### Flags

Any phase can decline to hand over to the next one:

| Flag | Raised when |
|---|---|
| `no_pallet_detected` | Nothing confirmed within 15 s |
| `no_approach_position` | Neither entry edge has a usable approach target |
| `approach_error_too_large` | Arrived at the target with more yaw error than D allows |
| `lost_pallet` | No valid detection for 5 s during align |

### The two controllers

Both run exactly the behaviours above with the same gains.

- **Baseline** is what you get by tuning in the ideal simulator, where steering
  is instant.
- **Actuator-aware** knows the steering has delay, lag and a rate limit. It
  predicts its pose across the delay, and it does not drive until the steered
  wheel has caught up with its command.

Nothing else differs between them.

## How a run is judged

| Outcome | Meaning |
|---|---|
| `success` | Pallet against the carriage face, both forks in their pockets |
| `collision_fork_face` | A fork tip hit the front of a block: lateral miss |
| `collision_fork_wall` | A fork scraped a block from inside its pocket: angular miss |
| `collision_side` | The truck drove at a side (wrong) edge |
| `collision_body` | The power unit hit the pallet |
| a flag | See above |
| `left_room`, `timeout` | Drove out of the room, or ran past 90 s |

The collision check is deliberately naive: it samples the outline of the forks
and the power unit, tests the points against the pallet, tolerates 5 mm of
overlap and stops the simulation at the first contact that is not the loading
contact.

Every trial also records the two alignment errors as ground truth at three
moments (arrival at the approach target, fork tips crossing the entry edge, end
of run) next to what the truck estimated them to be, plus the error of its last
pallet detection.

Trials are reproducible: `(seed, trial)` fixes the pallet pose, the start pose,
every hidden bias and every noise sample. Trial *k* is the same situation for
both controllers and at every error level, so comparisons are paired.

## Sources of error

Twelve independent parameters, all in `config.py`. `0x` of everything is the
ideal simulator; `1x` is a guess at an uncalibrated real truck. **The 1x values
are assumptions, not measurements.**

| Perception | 1x value | What it is |
|---|---|---|
| Lidar range noise | 15 mm | White noise on every beam |
| Lidar dropout | 2 % | Beams that return nothing |
| Lidar mount yaw error | 0.5 deg | Scanner not pointing where perception thinks; fixed per trial |
| Lidar mount lateral error | 5 mm | Scanner not sitting where perception thinks; fixed per trial |
| Scan/pose time offset | 60 ms | The scan is older than the pose it is combined with |

| Control | 1x value | What it is |
|---|---|---|
| Steering zero offset | 1 deg | Wheel is not where its encoder says; fixed per trial |
| Steering lag | 0.25 s | First-order response of the steering actuator |
| Steering rate limit | 40 deg/s | Infinite in the ideal simulator |
| Command latency | 0.10 s | Pure delay between command and actuator |
| Drive lag | 0.40 s | First-order response of the drive |
| Wheel scale error | 1 % | True speed differs from encoder speed; fixed per trial |
| Floor slip | 0.2 deg/√m | Heading random walk the encoders do not see |

Truth and odometry are integrated side by side. The zero offset, wheel scale
and slip move the truck without moving its odometry, which is what "it does not
drive exactly where it is told" means here.

## Architecture

```
            truth                                  what the robot has
  ┌──────────────────────┐
  │ config.py            │   room, pallet, truck, error levels, task (D)
  └──────────┬───────────┘
             ▼
  ┌──────────────────────┐  ranges   ┌────────────┐ points ┌───────────────┐ pallet pose ┌──────────────┐
  │ trial.py TrialRunner │──────────▶│ mapping.py │───────▶│ perception.py │────────────▶│ behaviors.py │
  │  reset / step        │  lidar    │ local map, │        │ edges → entry │             │ detect →     │
  │  collision check     │ sensors.py│ new beats  │        │ or side →     │             │ approach →   │
  │  verdict             │           │ cached     │        │ pallet pose   │             │ align, flags │
  │                      │◀──────────┴────────────┴────────┴───────────────┴─────────────┤              │
  │  vehicle.py          │        drive-wheel speed and steering commands                └──────────────┘
  │  truth + odometry    │
  └──────────────────────┘
        ▲            ▲
  experiment.py    webapi.py ── web/index.html (same package, in the browser)
  report.py
```

The seams are the point of the design:

- **Perception work** lives in `perception.py`. Its contract is points in,
  pallet pose out. `experiment.perception_study` scores it on single scans with
  no truck in the loop, and every trial logs detection error against truth.
- **Control work** lives in `behaviors.py` and `vehicle.py`.
  `OraclePerception` replaces the lidar pipeline with truth plus a chosen noise,
  so a controller can be studied against a known perception error.
- **Everything** goes through the same `TrialRunner`. The web page calls it tick
  by tick; the experiments call it thousands of times. A behaviour that works in
  one works in the other.
- Ground truth never leaks to the robot side. Perception does not know the
  lidar's true mount, the controller does not know the true pose, and the
  evaluator alone decides how a run ended.

Realism was spent where the question needs it: actuator dynamics, an honest
split between truth and odometry, a lidar that is ray cast rather than a pose
with noise added, and extrinsic calibration error. It was not spent on
graphics, 3D, or contact physics.

## The experiment

Error sources are grouped as *perception* and *control* and scaled together.
Three experiments, all paired by trial number:

1. **Grid.** Both groups swept from 0x to 2x in five steps, 120 trials per cell
   per controller. Shows where success collapses and along which axis.
2. **Sensitivity.** With everything held at 1.5x, one source at a time is
   removed or doubled, 150 trials each. It is done at 1.5x rather than 1x
   because that is where pickups start to fail; at 1x there is little to
   explain.
3. **Detection alone.** 300 single scans per range and noise level, from
   anywhere within 80 deg of the pallet axis, with no truck motion. Isolates the
   lidar pipeline.

Plus 500 trials per controller at 0x, 1x and 1.5x for the headline numbers and
the failure breakdown.

**Results are being generated and will be added here**, with the figures and
`results/trials.csv` (one row per trial). To produce them yourself:
`python -m forksim.experiment`.

## Limitations

What the simulation does not capture, and what it should not be trusted for.

- **The 1x error levels are assumed, not measured.** Every success rate is
  conditional on them. The *shape* of a result (what breaks first, where the
  cliff is) is more trustworthy than any percentage.
- **Contact is geometry, not physics.** No friction, no pallet sliding, no fork
  flex, no chamfered fork tips riding into a pocket. A real pallet forgives a
  few millimetres of scrape and punishes a hard hit by moving; here both are
  "overlap over 5 mm". Nothing after first contact is modelled.
- **It is flat.** No fork height, no mast, no pallet tilt, no uneven floor. A
  fork that is laterally perfect but 20 mm low still fails in reality.
- **The pallet is clean and alone.** Straight blocks, no shrink wrap, no broken
  boards, no neighbouring pallets, no people. The edge classifier would need
  rethinking for a pallet standing against another one.
- **The lidar is idealised in kind, not only in degree.** No beam divergence,
  no mixed pixels at block edges, no reflectivity effects, no motion distortion
  within a scan, no truck body blocking part of the 360 deg view. Range noise
  is white; real range errors are partly systematic per surface.
- **Odometry is encoders only.** No gyro, no SLAM. The real truck's data sheet
  claims +/-10 mm and +/-1 deg localisation; that is not modelled.
- **Some truck numbers are derived, not specified**: wheelbase, load-wheel
  position, steering lock and all actuator dynamics.
- **The baseline is deliberately naive.** It shows what the ideal simulator
  fails to warn about; it is not a claim about any production controller.
- **Lifting is not simulated.** "Loaded" means in position. Pallet weight is
  checked against the truck's rated load as an input limit and otherwise has no
  effect; pallet height has none.

Reasonable uses: ranking error sources, finding which tolerance is the tight
one, comparing two controllers on identical situations, regression-testing
behaviour logic. Not reasonable: quoting an absolute success rate, anything
about contact forces, sign-off.

## What I would measure on a real forklift

The simulator's answers depend on a handful of numbers that were assumed. I
would measure those, then check one prediction end to end.

1. **Lidar mounting error, yaw and lateral, per truck.** A yaw error is
   multiplied by the range to the pallet, it is systematic, and re-estimating
   cannot average it away. Scan a surveyed flat target from several known ranges
   and fit the offset.
2. **Steering response: delay, lag, rate limit, zero offset.** A step and a
   chirp on the steering command with the wheel angle logged, and a long
   straight drive for the zero offset. Cheap to measure, and these decide
   whether a controller tuned in the ideal simulator survives contact with the
   real actuator.
3. **Detection error against ground truth, by range and viewing angle.** Place
   a pallet at surveyed poses and log what `detect()` reports. This tests the
   noise *model*, not only its size: white range noise was assumed, and if the
   real error is correlated or biased the simulator is wrong in kind.
4. **The two alignment errors at fork entry, over repeated real pickups.**
   Lateral offset and angle between pallet edge and carriage face, measured
   externally as the tips cross the edge, for 50 or more runs. This is the
   end-to-end check: with 1 to 3 plugged in, the simulated distribution of these
   two numbers should match the real one. If it does, the success rate becomes
   believable. If it does not, the missing physics is in the limitations list.
5. **How much misalignment a real pallet tolerates.** Push forks in at
   deliberate lateral offsets and find where it stops working. The 25 mm + 5 mm
   envelope used here is pure geometry; the real one is probably wider, and it
   is the pass line everything else is judged against.

## Using it from Python

```python
import forksim as fs

# one trial: nominal errors, improved controller, situation number 3
r = fs.run_trial(fs.nominal(), "actuator_aware", trial=3)
print(r.outcome, r.entry_lat_mm, r.entry_angle_deg)

# the ideal simulator, and one at 2x perception error with ideal control
fs.ideal()
fs.scaled(perception_scale=2.0, control_scale=0.0)

# change anything by path
cfg = fs.override(fs.nominal(), {
    "pallet.x": 6.0, "pallet.angle_deg": 30.0,
    "task.approach_distance": 1.0,
    "perception.lidar_yaw_bias_sigma_deg": 0.0,
})

# step it yourself (this is what the web page does)
run = fs.TrialRunner(cfg, "baseline", trial=0)
while not run.step():
    pass
print(run.outcome, run.alert)
```

`TrialRunner(cfg, manual=True)` takes `(drive speed, steering angle)` per step
instead of running the behaviours. `perception="oracle"` swaps the lidar
pipeline for truth plus noise.

## Repository layout

```
forksim/
  config.py       every number: room, pallet, truck, error levels, task
  geometry.py     2D poses, ray casting
  vehicle.py      truck motion and odometry, with actuator imperfections
  sensors.py      lidar (camera present but disabled)
  mapping.py      local map; new scans override cached points
  perception.py   lidar points → edges → pallet pose
  behaviors.py    detect → approach → align, flags, the two controllers
  trial.py        one trial: runner, collision check, verdict, metrics
  experiment.py   Monte Carlo experiments
  report.py       figures and results table
  webapi.py       bridge used by the web page
web/index.html    the simulator in a browser
tests/            14 checks, about 20 s
```

Built with an AI coding assistant.
