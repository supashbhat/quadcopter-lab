import { magnitude, scaleVector } from "./linalg.js";

export function normalizeQuaternion(q) {
  const norm = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) || 1;
  return q.map((value) => value / norm);
}

export function quaternionMultiply(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

export function quaternionConjugate(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

export function rotateVector(q, vector) {
  const pure = [0, ...vector];
  const rotated = quaternionMultiply(quaternionMultiply(q, pure), quaternionConjugate(q));
  return rotated.slice(1);
}

export function quaternionDerivative(q, omega) {
  const omegaQuat = [0, omega[0], omega[1], omega[2]];
  return scaleVector(quaternionMultiply(q, omegaQuat), 0.5);
}

export function fromEuler(roll, pitch, yaw) {
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);

  return normalizeQuaternion([
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ]);
}

export function toEuler(q) {
  const [w, x, y, z] = normalizeQuaternion(q);

  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp);

  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  return [roll, pitch, yaw];
}

export function smoothQuaternionToward(q, toward, rate) {
  const mixed = q.map((value, index) => value * (1 - rate) + toward[index] * rate);
  return normalizeQuaternion(mixed);
}

export function axisAngle(axis, angle) {
  const axisNorm = magnitude(axis) || 1;
  const normalized = axis.map((value) => value / axisNorm);
  const half = angle * 0.5;
  const s = Math.sin(half);
  return normalizeQuaternion([Math.cos(half), normalized[0] * s, normalized[1] * s, normalized[2] * s]);
}

