# Roadmap

## Current foundation

- Nonlinear rigid-body quadcopter dynamics with quaternion attitude
- Hover-state LQR controller plus PID comparison mode
- Discrete Kalman filter with noisy sensor fusion
- Motor spool dynamics, route playback, telemetry export
- Blue-gold browser visualization with telemetry and scenario controls

## Next control upgrades

- Expose full Q/R matrices through advanced tuning panels
- Add nonlinear trajectory-tracking beyond hover linearization
- Add integral LQR / LQI variant for offset rejection comparison

## Next analysis upgrades

- Log settling time and overshoot across scenarios
- Export run histories for notebook-based benchmark plots
- Add wind-rejection scorecards and batch replay
- Plot estimator covariance against actual state error

## Longer-term direction

- Move the core dynamics and control stack into a native C++ library
- Reuse the browser scene as a visualization shell around the native core
- Add path-following and trajectory-tracking controllers beyond hover linearization
