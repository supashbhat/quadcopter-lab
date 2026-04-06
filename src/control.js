import {
  addMatrices,
  cloneMatrix,
  diag,
  identity,
  inverse,
  mulMatrices,
  mulMatrixVector,
  scaleMatrix,
  subMatrices,
  subVectors,
} from "./linalg.js";

export function buildHoverModel(params, dt) {
  const { gravity, mass, inertia, linearDamping, angularDamping } = params;
  const [ixx, iyy, izz] = inertia;

  const A = Array.from({ length: 12 }, () => Array(12).fill(0));
  const B = Array.from({ length: 12 }, () => Array(4).fill(0));

  A[0][3] = 1;
  A[1][4] = 1;
  A[2][5] = 1;

  A[3][3] = -linearDamping;
  A[4][4] = -linearDamping;
  A[5][5] = -linearDamping;
  A[3][7] = gravity;
  A[4][6] = -gravity;

  A[6][9] = 1;
  A[7][10] = 1;
  A[8][11] = 1;

  A[9][9] = -angularDamping;
  A[10][10] = -angularDamping;
  A[11][11] = -angularDamping;

  B[5][0] = 1 / mass;
  B[9][1] = 1 / ixx;
  B[10][2] = 1 / iyy;
  B[11][3] = 1 / izz;

  const Ad = addMatrices(identity(12), scaleMatrix(A, dt));
  const Bd = scaleMatrix(B, dt);
  return { A, B, Ad, Bd };
}

export function buildWeights(stateScale, effortScale) {
  const q = diag([
    8.5 * stateScale,
    8.5 * stateScale,
    10.0 * stateScale,
    3.1 * stateScale,
    3.1 * stateScale,
    4.1 * stateScale,
    6.2 * stateScale,
    6.2 * stateScale,
    2.8 * stateScale,
    1.4 * stateScale,
    1.4 * stateScale,
    0.9 * stateScale,
  ]);

  const r = diag([
    0.42 * effortScale,
    0.18 * effortScale,
    0.18 * effortScale,
    0.26 * effortScale,
  ]);

  return { q, r };
}

export function solveDiscreteLQR(ad, bd, q, r, iterations = 140) {
  let p = cloneMatrix(q);
  const at = ad[0].map((_, col) => ad.map((row) => row[col]));
  const bt = bd[0].map((_, col) => bd.map((row) => row[col]));

  for (let i = 0; i < iterations; i += 1) {
    const btpb = mulMatrices(bt, mulMatrices(p, bd));
    const gainTerm = inverse(addMatrices(r, btpb));
    const atpa = mulMatrices(at, mulMatrices(p, ad));
    const atpb = mulMatrices(at, mulMatrices(p, bd));
    const reduction = mulMatrices(atpb, mulMatrices(gainTerm, mulMatrices(bt, mulMatrices(p, ad))));
    p = addMatrices(q, subMatrices(atpa, reduction));
  }

  const k = mulMatrices(inverse(addMatrices(r, mulMatrices(bt, mulMatrices(p, bd)))), mulMatrices(bt, mulMatrices(p, ad)));
  return k;
}

export function buildMeasurementModel() {
  const h = Array.from({ length: 9 }, () => Array(12).fill(0));
  h[0][0] = 1;
  h[1][1] = 1;
  h[2][2] = 1;
  h[3][6] = 1;
  h[4][7] = 1;
  h[5][8] = 1;
  h[6][9] = 1;
  h[7][10] = 1;
  h[8][11] = 1;
  return h;
}

export class DiscreteKalmanFilter {
  constructor(ad, bd, processScale, measurementScale) {
    this.ad = ad;
    this.bd = bd;
    this.h = buildMeasurementModel();
    this.state = Array(12).fill(0);
    this.covariance = scaleMatrix(identity(12), 0.8);
    this.setNoise(processScale, measurementScale);
  }

  setNoise(processScale, measurementScale) {
    this.processCov = diag([
      0.002,
      0.002,
      0.0025,
      0.018,
      0.018,
      0.02,
      0.005,
      0.005,
      0.007,
      0.03,
      0.03,
      0.036,
    ].map((value) => value * processScale));

    this.measurementCov = diag([
      0.045,
      0.045,
      0.06,
      0.018,
      0.018,
      0.022,
      0.028,
      0.028,
      0.03,
    ].map((value) => value * measurementScale));
  }

  predict(controlInput) {
    this.state = addVectorsSafe(mulMatrixVector(this.ad, this.state), mulMatrixVector(this.bd, controlInput));
    this.covariance = addMatrices(mulMatrices(this.ad, mulMatrices(this.covariance, transposeInline(this.ad))), this.processCov);
  }

  update(measurement) {
    const innovation = subVectors(measurement, mulMatrixVector(this.h, this.state));
    const s = addMatrices(mulMatrices(this.h, mulMatrices(this.covariance, transposeInline(this.h))), this.measurementCov);
    const kalmanGain = mulMatrices(this.covariance, mulMatrices(transposeInline(this.h), inverse(s)));

    this.state = addVectorsSafe(this.state, mulMatrixVector(kalmanGain, innovation));
    const identityGain = subMatrices(identity(12), mulMatrices(kalmanGain, this.h));
    this.covariance = mulMatrices(identityGain, this.covariance);
  }
}

function addVectorsSafe(a, b) {
  return a.map((value, index) => value + b[index]);
}

function transposeInline(matrix) {
  return matrix[0].map((_, col) => matrix.map((row) => row[col]));
}

