import { buildHoverModel, buildWeights, DiscreteKalmanFilter, solveDiscreteLQR } from "./control.js";
import { inverse, magnitude, mulMatrixVector, subVectors } from "./linalg.js";
import { fromEuler, normalizeQuaternion, quaternionDerivative, rotateVector, toEuler } from "./quaternion.js";

const DEFAULT_PARAMS = {
  mass: 1.15,
  armLength: 0.24,
  inertia: [0.018, 0.019, 0.033],
  yawCoefficient: 0.055,
  thrustLimit: 6.6,
  gravity: 9.81,
  linearDamping: 0.22,
  angularDamping: 0.19,
};

const SCENARIOS = [
  {
    name: "Recover",
    position: [-2.4, 1.4, 1.1],
    velocity: [0.2, -0.3, 0],
    euler: [0.24, -0.18, 0.28],
    omega: [0.22, -0.16, 0.12],
  },
  {
    name: "Crosswind",
    position: [1.7, -2.1, 1.8],
    velocity: [-0.6, 0.55, 0.05],
    euler: [-0.11, 0.16, -0.25],
    omega: [0.12, 0.18, -0.08],
  },
  {
    name: "Offset",
    position: [-4.2, -1.2, 0.9],
    velocity: [0.34, 0.1, 0.12],
    euler: [0.08, 0.22, 0.16],
    omega: [-0.1, 0.15, 0.16],
  },
];

const TARGETS = [
  { name: "Hover", position: [0, 0, 2.2], yaw: 0 },
  { name: "Offset", position: [3.2, -2.1, 2.6], yaw: 0.24 },
  { name: "Climb", position: [-2.6, 2.3, 3.6], yaw: -0.2 },
];

export class QuadcopterLab {
  constructor() {
    this.params = { ...DEFAULT_PARAMS };
    this.sceneIndex = 0;
    this.targetIndex = 0;
    this.tuning = {
      stateWeight: 1.0,
      effortWeight: 1.0,
      processNoise: 1.0,
      measurementNoise: 1.0,
      windScale: 1.0,
    };

    this.histories = {
      error: [],
      effort: [],
      estimateGap: [],
      trueTrace: [],
      estimatedTrace: [],
    };

    this.mixMatrix = [
      [1, 1, 1, 1],
      [0, this.params.armLength, 0, -this.params.armLength],
      [-this.params.armLength, 0, this.params.armLength, 0],
      [-this.params.yawCoefficient, this.params.yawCoefficient, -this.params.yawCoefficient, this.params.yawCoefficient],
    ];
    this.mixInverse = inverse(this.mixMatrix);
    this.model = buildHoverModel(this.params, 1 / 60);
    this.controller = null;
    this.filter = null;
    this.lastCommand = [this.params.mass * this.params.gravity, 0, 0, 0];
    this.windImpulse = [0, 0, 0];
    this.setScenario(0);
    this.setTarget(0);
    this.updateTuning(this.tuning);
  }

  updateTuning(nextTuning) {
    this.tuning = { ...this.tuning, ...nextTuning };
    this.model = buildHoverModel(this.params, 1 / 60);
    const { q, r } = buildWeights(this.tuning.stateWeight, this.tuning.effortWeight);
    this.controller = solveDiscreteLQR(this.model.Ad, this.model.Bd, q, r);
    this.filter = new DiscreteKalmanFilter(this.model.Ad, this.model.Bd, this.tuning.processNoise, this.tuning.measurementNoise);
    this.filter.state = this.makeLinearStateFromTrue();
  }

  setScenario(index) {
    this.sceneIndex = (index + SCENARIOS.length) % SCENARIOS.length;
    const scenario = SCENARIOS[this.sceneIndex];
    this.trueState = {
      position: [...scenario.position],
      velocity: [...scenario.velocity],
      quaternion: fromEuler(...scenario.euler),
      omega: [...scenario.omega],
    };
    this.windImpulse = [0, 0, 0];
    this.lastCommand = [this.params.mass * this.params.gravity, 0, 0, 0];
    this.histories.error = [];
    this.histories.effort = [];
    this.histories.estimateGap = [];
    this.histories.trueTrace = [];
    this.histories.estimatedTrace = [];
    if (this.filter) {
      this.filter.state = this.makeLinearStateFromTrue();
    }
  }

  cycleScenario() {
    this.setScenario(this.sceneIndex + 1);
  }

  setTarget(index) {
    this.targetIndex = (index + TARGETS.length) % TARGETS.length;
    this.target = TARGETS[this.targetIndex];
  }

  cycleTarget() {
    this.setTarget(this.targetIndex + 1);
  }

  injectWind() {
    const strength = 4.5 * this.tuning.windScale;
    const angle = Math.random() * Math.PI * 2;
    this.windImpulse = [
      Math.cos(angle) * strength,
      Math.sin(angle) * strength,
      (Math.random() - 0.5) * 0.9 * strength,
    ];
  }

  reset() {
    this.setScenario(this.sceneIndex);
    this.setTarget(this.targetIndex);
  }

  step(dt) {
    const measurement = this.makeMeasurement();
    this.filter.predict(this.lastCommand.map((value, index) => index === 0 ? value - this.params.mass * this.params.gravity : value));
    this.filter.update(measurement);

    const estimate = this.filter.state;
    const desired = [
      this.target.position[0],
      this.target.position[1],
      this.target.position[2],
      0, 0, 0,
      0, 0, this.target.yaw,
      0, 0, 0,
    ];

    const error = subVectors(estimate, desired);
    const correction = mulMatrixVector(this.controller, error).map((value) => -value);
    const rawCommand = [
      this.params.mass * this.params.gravity + correction[0],
      correction[1],
      correction[2],
      correction[3],
    ];
    const allocation = this.allocateMotors(rawCommand);
    this.lastCommand = [allocation.totalThrust, allocation.torque[0], allocation.torque[1], allocation.torque[2]];

    this.trueState = integrateState(this.trueState, this.lastCommand, dt, this.params, this.windImpulse);
    this.windImpulse = this.windImpulse.map((value) => value * Math.pow(0.23, dt));

    const truthLinear = this.makeLinearStateFromTrue();
    const errorNorm = magnitude(subVectors(truthLinear.slice(0, 6), desired.slice(0, 6)));
    const estimateGap = magnitude(subVectors(truthLinear.slice(0, 9), estimate.slice(0, 9)));
    const effortNorm = magnitude([
      this.lastCommand[0] - this.params.mass * this.params.gravity,
      this.lastCommand[1] * 5,
      this.lastCommand[2] * 5,
      this.lastCommand[3] * 5,
    ]);

    pushHistory(this.histories.error, errorNorm);
    pushHistory(this.histories.estimateGap, estimateGap);
    pushHistory(this.histories.effort, effortNorm);
    pushTrace(this.histories.trueTrace, [...this.trueState.position]);
    pushTrace(this.histories.estimatedTrace, estimate.slice(0, 3));

    this.motorLevels = allocation.motors.map((value) => value / this.params.thrustLimit);
    this.metrics = {
      positionError: magnitude(subVectors(truthLinear.slice(0, 3), desired.slice(0, 3))),
      attitudeError: radToDeg(magnitude(subVectors(truthLinear.slice(6, 9), desired.slice(6, 9)))),
      windForce: magnitude(this.windImpulse),
      estimateGap,
      effortNorm,
    };
  }

  makeMeasurement() {
    const [roll, pitch, yaw] = toEuler(this.trueState.quaternion);
    const gaussian = () => {
      let u = 0;
      let v = 0;
      while (u === 0) u = Math.random();
      while (v === 0) v = Math.random();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    };

    const posNoise = 0.15 * this.tuning.measurementNoise;
    const angleNoise = 0.04 * this.tuning.measurementNoise;
    const rateNoise = 0.07 * this.tuning.measurementNoise;

    return [
      this.trueState.position[0] + gaussian() * posNoise,
      this.trueState.position[1] + gaussian() * posNoise,
      this.trueState.position[2] + gaussian() * posNoise,
      roll + gaussian() * angleNoise,
      pitch + gaussian() * angleNoise,
      yaw + gaussian() * angleNoise,
      this.trueState.omega[0] + gaussian() * rateNoise,
      this.trueState.omega[1] + gaussian() * rateNoise,
      this.trueState.omega[2] + gaussian() * rateNoise,
    ];
  }

  makeLinearStateFromTrue() {
    const [roll, pitch, yaw] = toEuler(this.trueState.quaternion);
    return [
      this.trueState.position[0],
      this.trueState.position[1],
      this.trueState.position[2],
      this.trueState.velocity[0],
      this.trueState.velocity[1],
      this.trueState.velocity[2],
      roll,
      pitch,
      yaw,
      this.trueState.omega[0],
      this.trueState.omega[1],
      this.trueState.omega[2],
    ];
  }

  allocateMotors(command) {
    const desiredMotors = mulMatrixVector(this.mixInverse, command).map((value) => clamp(value, 0, this.params.thrustLimit));
    const totalThrust = desiredMotors.reduce((sum, value) => sum + value, 0);
    const torque = [
      this.params.armLength * (desiredMotors[1] - desiredMotors[3]),
      this.params.armLength * (desiredMotors[2] - desiredMotors[0]),
      this.params.yawCoefficient * (-desiredMotors[0] + desiredMotors[1] - desiredMotors[2] + desiredMotors[3]),
    ];
    return { motors: desiredMotors, totalThrust, torque };
  }

  get sceneName() {
    return SCENARIOS[this.sceneIndex].name;
  }

  get targetName() {
    return TARGETS[this.targetIndex].name;
  }

  getEstimatedPose() {
    const estimate = this.filter.state;
    return {
      position: estimate.slice(0, 3),
      quaternion: fromEuler(estimate[6], estimate[7], estimate[8]),
    };
  }
}

function pushHistory(history, value) {
  history.push(value);
  if (history.length > 220) {
    history.shift();
  }
}

function pushTrace(history, point) {
  history.push(point);
  if (history.length > 120) {
    history.shift();
  }
}

function flattenState(state) {
  return [
    ...state.position,
    ...state.velocity,
    ...state.quaternion,
    ...state.omega,
  ];
}

function unflattenState(array) {
  return {
    position: array.slice(0, 3),
    velocity: array.slice(3, 6),
    quaternion: normalizeQuaternion(array.slice(6, 10)),
    omega: array.slice(10, 13),
  };
}

function integrateState(state, command, dt, params, windImpulse) {
  const y0 = flattenState(state);
  const k1 = stateDerivative(y0, command, params, windImpulse);
  const k2 = stateDerivative(addScaled(y0, k1, dt * 0.5), command, params, windImpulse);
  const k3 = stateDerivative(addScaled(y0, k2, dt * 0.5), command, params, windImpulse);
  const k4 = stateDerivative(addScaled(y0, k3, dt), command, params, windImpulse);

  const next = y0.map(
    (value, index) => value + (dt / 6) * (k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index]),
  );
  return unflattenState(next);
}

function stateDerivative(flatState, command, params, windImpulse) {
  const state = unflattenState(flatState);
  const [qx, qy, qz] = rotateVector(state.quaternion, [0, 0, command[0]]);
  const gravity = [0, 0, -params.gravity * params.mass];
  const drag = state.velocity.map((value) => -params.linearDamping * value);
  const force = [
    qx + gravity[0] + drag[0] + windImpulse[0],
    qy + gravity[1] + drag[1] + windImpulse[1],
    qz + gravity[2] + drag[2] + windImpulse[2],
  ];

  const [ixx, iyy, izz] = params.inertia;
  const omega = state.omega;
  const inertiaOmega = [ixx * omega[0], iyy * omega[1], izz * omega[2]];
  const gyro = [
    omega[1] * inertiaOmega[2] - omega[2] * inertiaOmega[1],
    omega[2] * inertiaOmega[0] - omega[0] * inertiaOmega[2],
    omega[0] * inertiaOmega[1] - omega[1] * inertiaOmega[0],
  ];
  const torque = [
    command[1] - gyro[0] - params.angularDamping * omega[0],
    command[2] - gyro[1] - params.angularDamping * omega[1],
    command[3] - gyro[2] - params.angularDamping * omega[2],
  ];

  const acceleration = force.map((value) => value / params.mass);
  const omegaDot = [torque[0] / ixx, torque[1] / iyy, torque[2] / izz];
  const qDot = quaternionDerivative(state.quaternion, omega);

  return [
    state.velocity[0],
    state.velocity[1],
    state.velocity[2],
    acceleration[0],
    acceleration[1],
    acceleration[2],
    qDot[0],
    qDot[1],
    qDot[2],
    qDot[3],
    omegaDot[0],
    omegaDot[1],
    omegaDot[2],
  ];
}

function addScaled(vector, derivative, scale) {
  return vector.map((value, index) => value + derivative[index] * scale);
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function radToDeg(value) {
  return (value * 180) / Math.PI;
}
