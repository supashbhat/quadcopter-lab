# Aerial Control Lab

Aerial Control Lab is an interactive quadcopter controls sandbox built to make modern control and estimation ideas easier to see, compare, and reason about. It combines a nonlinear 6-DOF rigid-body simulation with hover-state `LQR` control, a `PID` baseline, and a discrete `Kalman` filter inside a browser-based interface designed for experimentation.

The project is aimed at the space between theory and intuition: changing gains, injecting disturbances, comparing controller behavior, and seeing how state estimation tracks a noisy plant in real time.

## Highlights

- Nonlinear quadcopter dynamics with quaternion attitude propagation
- Hover-state linearization for `LQR` feedback design
- Discrete `Kalman` filter using noisy position, attitude, and rate measurements
- `PID` comparison mode for side-by-side control intuition
- Rotor mixing, saturation, and first-order motor spool dynamics
- Disturbance injection, target switching, and route playback
- Recovery benchmark cards for settling time, peak error, estimate RMS, and control effort
- Guided walkthrough mode for controller comparison and route-tracking demos
- JSON telemetry export for offline analysis

## Getting Started

Serve the project locally:

```bash
cd quadcopter-lab
./scripts/serve.sh
```

Then open [http://localhost:4173](http://localhost:4173).

Direct file-open is also supported by opening `index.html` in a browser. The repository includes a bundled browser script so the interface can run outside a local module server when needed.

## Controls

- `Reset` resets the current scenario
- `Wind Gust` injects a disturbance force
- `Target` cycles between hover, offset, and climb setpoints
- `Scenario` switches the initial condition
- `Mode` toggles between `LQR` and `PID`
- `Route` cycles between `Off`, `Box`, and `Spiral`
- `Pause` freezes the simulation without clearing the UI
- `Export Log` downloads recent telemetry as JSON
- The slider panel adjusts control weighting, estimator trust, measurement noise, and wind strength
- `Guided Demo` walks through the recommended comparison sequence

## What The Demo Shows

### LQR

`LQR` uses a linearized hover model and computes a feedback law that minimizes a cost balancing state error against control effort. In this sandbox it is meant to show how model-based optimal control can produce coordinated recovery behavior.

### PID

`PID` reacts to error directly through proportional, integral, and derivative terms. It is included as a practical baseline so the difference between local error correction and model-based control is easier to compare.

### Kalman Filtering

The `Kalman` filter estimates the vehicle state from noisy measurements. The interface visualizes both the true vehicle state and the estimated state so estimator quality is visible rather than implicit.

### Benchmark Cards

The benchmark cards summarize the current run with:

- `Settling Time`
- `Peak Error`
- `Estimate RMS`
- `Effort Peak`

These are lightweight comparison metrics, intended to make controller behavior easier to interpret at a glance.

## Project Structure

- `index.html` — interface shell and layout
- `styles/main.css` — visual system, motion, and layout styling
- `src/linalg.js` — matrix and vector helpers
- `src/quaternion.js` — quaternion math and rotation utilities
- `src/control.js` — hover linearization, `LQR`, and `Kalman` implementations
- `src/quadcopter.js` — nonlinear dynamics, routing, disturbance logic, and simulation state
- `src/renderer.js` — scene rendering, overlays, and chart drawing
- `src/app.js` — UI orchestration and guided demo logic
- `docs/control-notes.md` — control and estimator notes
- `ROADMAP.md` — next steps for controls, analysis, and native-core work

## Roadmap

Near-term directions include:

- richer `Q/R` tuning panels
- batch scenario sweeps and replayable comparisons
- estimator covariance visualization
- stronger route-tracking and trajectory-following controllers
- migration of the control/dynamics core into a native C++ library

More detail is available in [ROADMAP.md](ROADMAP.md).
