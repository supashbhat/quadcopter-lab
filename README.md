# Aerial Control Lab

A polished quadcopter controls sandbox focused on nonlinear dynamics, LQR stabilization, and Kalman state estimation. The project is meant to make modern controls feel visual and tactile instead of hidden behind equations alone.

## What It Includes

- A nonlinear 6-DOF quadcopter simulation with quaternion attitude propagation
- Hover-state linearization for LQR controller design
- A discrete Kalman filter that fuses noisy GPS, attitude, and gyro measurements
- Rotor mixing and saturation so control effort maps to four motor commands
- Optional PID baseline so LQR behavior has a direct comparison mode
- Motor spool lag and route playback so the plant feels less idealized
- Wind gust injection, live telemetry, error/control histories, and target changes
- Recovery benchmark cards for settling time, peak error, estimate RMS, and control effort
- A guided comparison flow that walks through LQR recovery, disturbance rejection, PID baseline behavior, and route tracking
- A blue-and-gold browser UI with polished motion and presentation

## Quick Start

```bash
cd quadcopter-lab
./scripts/serve.sh
```

Then open [http://localhost:4173](http://localhost:4173).

You can also open [index.html](/Users/Supash/quadcopter-lab/index.html) directly from Finder now. The repo ships a bundled browser script alongside the source modules so the intro and UI do not get stuck behind `file://` module restrictions.

## Controls

- `Reset`: restore the current scenario
- `Wind Gust`: inject a side-force disturbance
- `Target`: cycle among hover, offset, and climb setpoints
- `Scenario`: switch the initial condition between hover recovery, crosswind, and offset approach
- `Mode`: compare LQR against a tuned PID baseline
- `Route`: switch among no route, box route, and spiral route playback
- `Pause`: freeze the sim without dropping the UI
- `Export Log`: download the recent telemetry history as JSON
- Sliders: retune LQR state weight, control effort, Kalman trust, and wind strength live
- `Guided Demo`: step through the intended portfolio narrative instead of manually guessing what to test

## What To Learn From The Demo

- `LQR` uses a model of the hover dynamics and chooses feedback gains that minimize a cost balancing state error against control effort.
- `PID` is the baseline: it reacts to present, accumulated, and rate-of-change error terms without using the full plant model.
- `Kalman` is the estimator, not the controller. The gold ghost should stay fairly close to the blue true vehicle even when noise and wind are present.
- The benchmark cards summarize the recovery story so the sandbox is easier to interpret at a glance.

## Architecture

- [index.html](/Users/Supash/quadcopter-lab/index.html) - shell and layout
- [styles/main.css](/Users/Supash/quadcopter-lab/styles/main.css) - visual system and motion
- [src/linalg.js](/Users/Supash/quadcopter-lab/src/linalg.js) - matrix helpers
- [src/quaternion.js](/Users/Supash/quadcopter-lab/src/quaternion.js) - quaternion math and frame transforms
- [src/control.js](/Users/Supash/quadcopter-lab/src/control.js) - hover model, LQR, and Kalman filter
- [src/quadcopter.js](/Users/Supash/quadcopter-lab/src/quadcopter.js) - nonlinear dynamics, motor mixing, and simulation state
- [src/renderer.js](/Users/Supash/quadcopter-lab/src/renderer.js) - 3D projection, telemetry, and charts
- [src/app.js](/Users/Supash/quadcopter-lab/src/app.js) - orchestration and UI binding
- [ROADMAP.md](/Users/Supash/quadcopter-lab/ROADMAP.md) - next milestones for control, benchmarking, and native-core work

## Why LQR + Kalman

LQR turns the stabilization problem into an optimization problem: penalize state error and control effort, then solve for the best linear feedback law around hover. The Kalman filter plays the complementary role on the sensing side, estimating the underlying state when measurements are noisy and incomplete.

That combination is useful well beyond quadcopters. It is the same broad control-estimation pattern that shows up in drones, vehicles, robotics, and instrument stabilization.

## What Changed In This Pass

- Added controller-mode switching so the same plant can be flown with `LQR` or `PID`
- Added motor spool lag to make actuator response less artificially perfect
- Added route playback beyond fixed targets
- Added telemetry export for offline analysis
- Added another telemetry chart for altitude and route progress
- Added recovery benchmark cards and controller summaries for quicker interpretation
- Added a guided demo flow that stages the most useful interactions in order
