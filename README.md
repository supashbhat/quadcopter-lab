# Aerial Control Lab

A polished quadcopter controls sandbox focused on nonlinear dynamics, LQR stabilization, and Kalman state estimation. The project is meant to make modern controls feel visual and tactile instead of hidden behind equations alone.

## What It Includes

- A nonlinear 6-DOF quadcopter simulation with quaternion attitude propagation
- Hover-state linearization for LQR controller design
- A discrete Kalman filter that fuses noisy GPS, attitude, and gyro measurements
- Rotor mixing and saturation so control effort maps to four motor commands
- Wind gust injection, live telemetry, error/control histories, and target changes
- A blue-and-gold browser UI with polished motion and presentation

## Quick Start

```bash
cd quadcopter-lab
./scripts/serve.sh
```

Then open [http://localhost:4173](http://localhost:4173).

## Controls

- `Reset`: restore the current scenario
- `Wind Gust`: inject a side-force disturbance
- `Target`: cycle among hover, offset, and climb setpoints
- `Scenario`: switch the initial condition between hover recovery, crosswind, and offset approach
- Sliders: retune LQR state weight, control effort, Kalman trust, and wind strength live

## Architecture

- [index.html](/Users/Supash/quadcopter-lab/index.html) - shell and layout
- [styles/main.css](/Users/Supash/quadcopter-lab/styles/main.css) - visual system and motion
- [src/linalg.js](/Users/Supash/quadcopter-lab/src/linalg.js) - matrix helpers
- [src/quaternion.js](/Users/Supash/quadcopter-lab/src/quaternion.js) - quaternion math and frame transforms
- [src/control.js](/Users/Supash/quadcopter-lab/src/control.js) - hover model, LQR, and Kalman filter
- [src/quadcopter.js](/Users/Supash/quadcopter-lab/src/quadcopter.js) - nonlinear dynamics, motor mixing, and simulation state
- [src/renderer.js](/Users/Supash/quadcopter-lab/src/renderer.js) - 3D projection, telemetry, and charts
- [src/app.js](/Users/Supash/quadcopter-lab/src/app.js) - orchestration and UI binding

## Why LQR + Kalman

LQR turns the stabilization problem into an optimization problem: penalize state error and control effort, then solve for the best linear feedback law around hover. The Kalman filter plays the complementary role on the sensing side, estimating the underlying state when measurements are noisy and incomplete.

That combination is useful well beyond quadcopters. It is the same broad control-estimation pattern that shows up in drones, vehicles, robotics, and instrument stabilization.

