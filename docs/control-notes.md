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

