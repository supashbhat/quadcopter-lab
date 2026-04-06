import { toEuler } from "./quaternion.js";

export class Renderer {
  constructor(sceneCanvas, motorCanvas, errorCanvas, effortCanvas) {
    this.sceneCanvas = sceneCanvas;
    this.scene = sceneCanvas.getContext("2d");
    this.motorCanvas = motorCanvas;
    this.motor = motorCanvas.getContext("2d");
    this.errorCanvas = errorCanvas;
    this.error = errorCanvas.getContext("2d");
    this.effortCanvas = effortCanvas;
    this.effort = effortCanvas.getContext("2d");
    this.cameraAngle = 0.85;
  }

  render(sim, elapsed) {
    this.cameraAngle += 0.05 * elapsed;
    drawScene(this.scene, this.sceneCanvas, sim, this.cameraAngle);
    drawBars(this.motor, this.motorCanvas, sim.motorLevels || [0, 0, 0, 0]);
    drawLineChart(this.error, this.errorCanvas, sim.histories.error, sim.histories.estimateGap, "#7fc8ff", "#f2c879");
    drawSingleChart(this.effort, this.effortCanvas, sim.histories.effort, "#d8ad63");
  }
}

function drawScene(ctx, canvas, sim, cameraAngle) {
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);

  const gradient = ctx.createLinearGradient(0, 0, 0, height);
  gradient.addColorStop(0, "rgba(10, 20, 32, 0.85)");
  gradient.addColorStop(1, "rgba(6, 13, 24, 0.98)");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  drawGlow(ctx, width * 0.18, height * 0.16, 220, "rgba(91, 164, 255, 0.11)");
  drawGlow(ctx, width * 0.86, height * 0.2, 170, "rgba(242, 200, 121, 0.08)");
  drawRibbon(ctx, width, height, cameraAngle);

  const projectedGrid = [];
  for (let x = -8; x <= 8; x += 1) {
    projectedGrid.push([
      project([x, -8, 0], width, height, cameraAngle),
      project([x, 8, 0], width, height, cameraAngle),
    ]);
  }
  for (let y = -8; y <= 8; y += 1) {
    projectedGrid.push([
      project([-8, y, 0], width, height, cameraAngle),
      project([8, y, 0], width, height, cameraAngle),
    ]);
  }

  ctx.strokeStyle = "rgba(112, 154, 214, 0.14)";
  ctx.lineWidth = 1;
  projectedGrid.forEach(([a, b]) => {
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  });

  drawTarget(ctx, sim.target.position, width, height, cameraAngle);
  drawTrace(ctx, sim.histories.trueTrace || [], width, height, cameraAngle, "rgba(127, 200, 255, 0.35)");
  drawTrace(ctx, sim.histories.estimatedTrace || [], width, height, cameraAngle, "rgba(242, 200, 121, 0.3)");
  drawDrone(ctx, sim.trueState, width, height, cameraAngle, false);
  drawDrone(ctx, sim.getEstimatedPose(), width, height, cameraAngle, true);
  drawTelemetryOverlay(ctx, sim, width, height);
}

function drawDrone(ctx, state, width, height, cameraAngle, ghost) {
  const center = state.position;
  const rotorLocal = [
    [0.24, 0, 0],
    [0, 0.24, 0],
    [-0.24, 0, 0],
    [0, -0.24, 0],
  ];
  const points = rotorLocal.map((point) => addVectors(center, rotateByEuler(point, toEuler(state.quaternion))));
  const center2d = project(center, width, height, cameraAngle);

  ctx.strokeStyle = ghost ? "rgba(242, 200, 121, 0.48)" : "rgba(127, 200, 255, 0.92)";
  ctx.lineWidth = ghost ? 2 : 3.2;
  ctx.beginPath();
  ctx.moveTo(project(points[0], width, height, cameraAngle).x, project(points[0], width, height, cameraAngle).y);
  ctx.lineTo(project(points[2], width, height, cameraAngle).x, project(points[2], width, height, cameraAngle).y);
  ctx.moveTo(project(points[1], width, height, cameraAngle).x, project(points[1], width, height, cameraAngle).y);
  ctx.lineTo(project(points[3], width, height, cameraAngle).x, project(points[3], width, height, cameraAngle).y);
  ctx.stroke();

  points.forEach((point, index) => {
    const rotor = project(point, width, height, cameraAngle);
    ctx.beginPath();
    ctx.strokeStyle = ghost ? "rgba(242, 200, 121, 0.32)" : "rgba(242, 200, 121, 0.9)";
    ctx.lineWidth = 2;
    ctx.arc(rotor.x, rotor.y, ghost ? 12 : 15, 0, Math.PI * 2);
    ctx.stroke();

    if (!ghost) {
      const shimmer = 0.55 + 0.45 * Math.sin(performance.now() * 0.012 + index * 1.7);
      ctx.fillStyle = `rgba(242, 200, 121, ${0.18 * shimmer})`;
      ctx.beginPath();
      ctx.arc(rotor.x, rotor.y, 20 + shimmer * 6, 0, Math.PI * 2);
      ctx.fill();
    }
  });

  ctx.fillStyle = ghost ? "rgba(242, 200, 121, 0.66)" : "#ecf5ff";
  ctx.beginPath();
  ctx.arc(center2d.x, center2d.y, ghost ? 6 : 8, 0, Math.PI * 2);
  ctx.fill();
}

function drawTarget(ctx, target, width, height, cameraAngle) {
  const point = project(target, width, height, cameraAngle);
  ctx.strokeStyle = "rgba(242, 200, 121, 0.86)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(point.x, point.y, 18, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(point.x - 24, point.y);
  ctx.lineTo(point.x + 24, point.y);
  ctx.moveTo(point.x, point.y - 24);
  ctx.lineTo(point.x, point.y + 24);
  ctx.stroke();
}

function drawTelemetryOverlay(ctx, sim, width, height) {
  const lines = [
    `Target: ${sim.targetName}`,
    `Scenario: ${sim.sceneName}`,
    `Estimated gap: ${sim.metrics.estimateGap.toFixed(2)} m`,
    `Wind: ${sim.metrics.windForce.toFixed(2)} N`,
  ];
  ctx.fillStyle = "rgba(8, 16, 26, 0.78)";
  ctx.strokeStyle = "rgba(127, 180, 242, 0.14)";
  roundRect(ctx, width - 278, height - 164, 236, 118, 18, true, true);
  ctx.fillStyle = "#edf5ff";
  ctx.font = "600 15px Inter, sans-serif";
  ctx.fillText("Flight Notes", width - 252, height - 130);
  ctx.font = "500 13px Inter, sans-serif";
  ctx.fillStyle = "rgba(180, 196, 214, 0.95)";
  lines.forEach((line, index) => ctx.fillText(line, width - 252, height - 100 + index * 22));
}

function drawTrace(ctx, trace, width, height, cameraAngle, color) {
  if (trace.length < 2) {
    return;
  }

  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  trace.forEach((point, index) => {
    const projected = project(point, width, height, cameraAngle);
    if (index === 0) {
      ctx.moveTo(projected.x, projected.y);
    } else {
      ctx.lineTo(projected.x, projected.y);
    }
  });
  ctx.stroke();
}

function drawBars(ctx, canvas, values) {
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);
  fillPanel(ctx, width, height);
  values.forEach((value, index) => {
    const barWidth = 58;
    const gap = 20;
    const x = 34 + index * (barWidth + gap);
    const maxHeight = 58;
    const current = Math.max(0.05, Math.min(1, value));
    ctx.fillStyle = "rgba(112, 154, 214, 0.18)";
    roundRect(ctx, x, height - 26 - maxHeight, barWidth, maxHeight, 12, true, false);
    const gradient = ctx.createLinearGradient(0, height - 26, 0, height - 26 - maxHeight);
    gradient.addColorStop(0, "rgba(127, 200, 255, 0.88)");
    gradient.addColorStop(1, "rgba(242, 200, 121, 0.96)");
    ctx.fillStyle = gradient;
    roundRect(ctx, x, height - 26 - current * maxHeight, barWidth, current * maxHeight, 12, true, false);
    ctx.fillStyle = "rgba(237, 245, 255, 0.84)";
    ctx.font = "600 12px Inter, sans-serif";
    ctx.fillText(`M${index + 1}`, x + 18, height - 8);
  });
}

function drawLineChart(ctx, canvas, primary, secondary, primaryColor, secondaryColor) {
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);
  fillPanel(ctx, width, height);
  drawSparkline(ctx, width, height, primary, primaryColor, 0.88);
  drawSparkline(ctx, width, height, secondary, secondaryColor, 0.64);
}

function drawSingleChart(ctx, canvas, data, color) {
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);
  fillPanel(ctx, width, height);
  drawSparkline(ctx, width, height, data, color, 0.82);
}

function drawSparkline(ctx, width, height, data, color, alpha) {
  if (!data.length) {
    return;
  }
  const max = Math.max(...data, 1e-3);
  ctx.strokeStyle = withAlpha(color, alpha);
  ctx.lineWidth = 2.4;
  ctx.beginPath();
  data.forEach((value, index) => {
    const x = 18 + (index / Math.max(1, data.length - 1)) * (width - 36);
    const y = height - 20 - (value / max) * (height - 36);
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.stroke();
}

function fillPanel(ctx, width, height) {
  const gradient = ctx.createLinearGradient(0, 0, 0, height);
  gradient.addColorStop(0, "rgba(10, 19, 30, 0.92)");
  gradient.addColorStop(1, "rgba(7, 14, 24, 0.92)");
  ctx.fillStyle = gradient;
  roundRect(ctx, 0, 0, width, height, 18, true, false);
}

function project(point, width, height, cameraAngle) {
  const cameraDistance = 10.5;
  const elevated = 4.8;
  const cos = Math.cos(cameraAngle);
  const sin = Math.sin(cameraAngle);

  const x = point[0] * cos - point[1] * sin;
  const y = point[0] * sin + point[1] * cos;
  const z = point[2];

  const depth = cameraDistance + y;
  const scale = 520 / depth;
  return {
    x: width * 0.5 + x * scale,
    y: height * 0.72 - (z - elevated) * scale,
  };
}

function rotateByEuler(vector, euler) {
  const [roll, pitch, yaw] = euler;
  const [x, y, z] = vector;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);

  const m00 = cy * cp;
  const m01 = cy * sp * sr - sy * cr;
  const m02 = cy * sp * cr + sy * sr;
  const m10 = sy * cp;
  const m11 = sy * sp * sr + cy * cr;
  const m12 = sy * sp * cr - cy * sr;
  const m20 = -sp;
  const m21 = cp * sr;
  const m22 = cp * cr;

  return [
    m00 * x + m01 * y + m02 * z,
    m10 * x + m11 * y + m12 * z,
    m20 * x + m21 * y + m22 * z,
  ];
}

function drawGlow(ctx, x, y, radius, color) {
  const gradient = ctx.createRadialGradient(x, y, 0, x, y, radius);
  gradient.addColorStop(0, color);
  gradient.addColorStop(1, "rgba(0, 0, 0, 0)");
  ctx.fillStyle = gradient;
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fill();
}

function drawRibbon(ctx, width, height, angle) {
  ctx.save();
  ctx.strokeStyle = "rgba(111, 185, 255, 0.14)";
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  for (let i = 0; i <= width; i += 18) {
    const y = 96 + Math.sin(i * 0.008 + angle * 1.8) * 22 + Math.cos(i * 0.003 + angle) * 10;
    if (i === 0) {
      ctx.moveTo(i, y);
    } else {
      ctx.lineTo(i, y);
    }
  }
  ctx.stroke();
  ctx.restore();
}

function roundRect(ctx, x, y, width, height, radius, fill, stroke) {
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.arcTo(x + width, y, x + width, y + height, radius);
  ctx.arcTo(x + width, y + height, x, y + height, radius);
  ctx.arcTo(x, y + height, x, y, radius);
  ctx.arcTo(x, y, x + width, y, radius);
  ctx.closePath();
  if (fill) {
    ctx.fill();
  }
  if (stroke) {
    ctx.stroke();
  }
}

function withAlpha(hexColor, alpha) {
  if (!hexColor.startsWith("#")) {
    return hexColor;
  }
  const value = hexColor.replace("#", "");
  const bigint = Number.parseInt(value, 16);
  const r = (bigint >> 16) & 255;
  const g = (bigint >> 8) & 255;
  const b = bigint & 255;
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

function addVectors(a, b) {
  return a.map((value, index) => value + b[index]);
}
