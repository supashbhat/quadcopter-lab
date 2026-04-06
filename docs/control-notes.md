# Control Notes

The browser sandbox uses a nonlinear rigid-body model but computes control around a hover linearization.

## Linear state

The feedback state is:

`[x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]`

around a hover operating point with nominal thrust `mg`.

## Inputs

The controller outputs:

`[delta thrust, tau_x, tau_y, tau_z]`

which are converted into four rotor thrust commands through a mixer matrix. Rotor saturation is applied, and the realized thrust/torque are reconstructed from the clamped motor values before the nonlinear dynamics step.

## Kalman measurements

The filter fuses:

- position `(x, y, z)`
- Euler attitude `(roll, pitch, yaw)`
- angular rates `(p, q, r)`

The process and measurement covariance scales are exposed in the UI to make the estimator behavior tangible.

## Controller comparison

The lab now includes two control modes:

- `LQR`: full-state feedback around hover linearization
- `PID`: a practical baseline with position-to-attitude mapping and inner-loop rate damping

The goal is not to claim that the PID mode is globally optimal. It exists to make the benefits of the LQR formulation more legible in the same environment and under the same disturbances.

## Interpreting the benchmark cards

The UI now exposes a compact recovery summary:

- `Settling Time`: how quickly the vehicle returns to a small hover-error band
- `Peak Error`: the largest positional miss during the current recovery segment
- `Estimate RMS`: average gap between the Kalman estimate and the true simulated state
- `Effort Peak`: largest control burst used during the recovery

Those numbers are intentionally lightweight rather than academically exhaustive. The goal is to help a reviewer see, in a few seconds, whether one control mode is recovering more cleanly than another.

## Actuator realism

Motor commands are no longer treated as instantaneously realized thrust. A first-order spool model smooths each rotor toward its commanded thrust, which makes aggressive corrections visibly less idealized and introduces a more realistic control-effort story.
