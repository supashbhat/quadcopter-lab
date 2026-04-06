# Roadmap

## Current foundation

- Nonlinear rigid-body quadcopter dynamics with quaternion attitude
- Hover-state LQR controller
- Discrete Kalman filter with noisy sensor fusion
- Blue-gold browser visualization with telemetry and scenario controls

## Next control upgrades

- Add waypoint interpolation instead of discrete target presets
- Add actuator lag and motor spool dynamics
- Expose full Q/R matrices through advanced tuning panels
- Compare hover LQR against a baseline PID controller

## Next analysis upgrades

- Log settling time and overshoot across scenarios
- Export run histories for notebook-based benchmark plots
- Add wind-rejection scorecards and batch replay
- Plot estimator covariance against actual state error

## Longer-term direction

- Move the core dynamics and control stack into a native C++ library
- Reuse the browser scene as a visualization shell around the native core
- Add path-following and trajectory-tracking controllers beyond hover linearization
