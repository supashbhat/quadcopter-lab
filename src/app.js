import { QuadcopterLab } from "./quadcopter.js";
import { Renderer } from "./renderer.js";

const sliderDefinitions = [
  { id: "stateWeight", label: "LQR State Weight", min: 0.4, max: 2.6, step: 0.01, value: 1.0 },
  { id: "effortWeight", label: "Control Effort Weight", min: 0.45, max: 2.8, step: 0.01, value: 1.0 },
  { id: "processNoise", label: "Kalman Process Trust", min: 0.45, max: 3.0, step: 0.01, value: 1.0 },
  { id: "measurementNoise", label: "Sensor Noise Weight", min: 0.4, max: 2.8, step: 0.01, value: 1.0 },
  { id: "windScale", label: "Wind Strength", min: 0.4, max: 2.4, step: 0.01, value: 1.0 },
];

const sim = new QuadcopterLab();
const renderer = new Renderer(
  document.getElementById("sceneCanvas"),
  document.getElementById("motorCanvas"),
  document.getElementById("errorCanvas"),
  document.getElementById("effortCanvas"),
  document.getElementById("altitudeCanvas"),
);

const statusLabel = document.getElementById("statusLabel");
const statusDot = document.getElementById("statusDot");
const positionError = document.getElementById("positionError");
const attitudeError = document.getElementById("attitudeError");
const windValue = document.getElementById("windValue");
const settleValue = document.getElementById("settleValue");
const controllerValue = document.getElementById("controllerValue");
const routeValue = document.getElementById("routeValue");
const resetButton = document.getElementById("resetButton");
const gustButton = document.getElementById("gustButton");
const targetButton = document.getElementById("targetButton");
const scenarioButton = document.getElementById("scenarioButton");
const modeButton = document.getElementById("modeButton");
const routeButton = document.getElementById("routeButton");
const pauseButton = document.getElementById("pauseButton");
const exportButton = document.getElementById("exportButton");

let paused = false;

buildSliders();
wireButtons();
kickIntro();

let last = performance.now();
requestAnimationFrame(frame);

function frame(now) {
  const dt = Math.min(0.035, (now - last) / 1000 || 1 / 60);
  last = now;
  if (!paused) {
    sim.step(dt);
  }
  renderer.render(sim, dt);
  syncText();
  requestAnimationFrame(frame);
}

function buildSliders() {
  const stack = document.getElementById("sliderStack");
  sliderDefinitions.forEach((definition) => {
    const wrapper = document.createElement("label");
    wrapper.className = "slider";

    const meta = document.createElement("div");
    meta.className = "slider__meta";

    const label = document.createElement("span");
    label.className = "slider__label";
    label.textContent = definition.label;

    const value = document.createElement("span");
    value.className = "slider__value";
    value.textContent = definition.value.toFixed(2);
    value.id = `${definition.id}Value`;

    const input = document.createElement("input");
    input.type = "range";
    input.min = definition.min;
    input.max = definition.max;
    input.step = definition.step;
    input.value = definition.value;
    input.addEventListener("input", () => {
      value.textContent = Number.parseFloat(input.value).toFixed(2);
      sim.updateTuning({ [definition.id]: Number.parseFloat(input.value) });
    });

    meta.append(label, value);
    wrapper.append(meta, input);
    stack.append(wrapper);
  });
}

function wireButtons() {
  resetButton.addEventListener("click", () => sim.reset());
  gustButton.addEventListener("click", () => sim.injectWind());
  targetButton.addEventListener("click", () => {
    sim.cycleTarget();
    targetButton.textContent = `Target: ${sim.targetName}`;
  });
  scenarioButton.addEventListener("click", () => {
    sim.cycleScenario();
    scenarioButton.textContent = `Scenario: ${sim.sceneName}`;
  });
  modeButton.addEventListener("click", () => {
    sim.toggleControllerMode();
    modeButton.textContent = `Mode: ${sim.controllerMode}`;
  });
  routeButton.addEventListener("click", () => {
    sim.cycleRoute();
    routeButton.textContent = `Route: ${sim.routeName}`;
  });
  pauseButton.addEventListener("click", () => {
    paused = !paused;
    pauseButton.textContent = paused ? "Resume" : "Pause";
  });
  exportButton.addEventListener("click", exportTelemetry);
}

function syncText() {
  positionError.textContent = `${sim.metrics.positionError.toFixed(2)} m`;
  attitudeError.textContent = `${sim.metrics.attitudeError.toFixed(1)} deg`;
  windValue.textContent = `${sim.metrics.windForce.toFixed(2)} N`;
  settleValue.textContent = paused ? "Paused" : sim.metrics.positionError < 0.45 ? "Near hover" : "Recovering";
  statusLabel.textContent = paused ? "Paused" : sim.metrics.positionError < 0.45 ? "Stable hold" : "Stabilizing";
  statusDot.style.background = paused
    ? "radial-gradient(circle, #e9f5ff, #7fc8ff)"
    : sim.metrics.positionError < 0.45
      ? "radial-gradient(circle, #fff2ca, #f2c879)"
      : "radial-gradient(circle, #ffd8cb, #ff9a7a)";
  controllerValue.textContent = sim.controllerMode;
  routeValue.textContent = sim.routeName;
}

function kickIntro() {
  const intro = document.getElementById("intro");
  setTimeout(() => intro.classList.add("intro--done"), 3200);
  setTimeout(() => intro.remove(), 4100);
}

function exportTelemetry() {
  const payload = sim.exportLog();
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `aerial-control-lab-${Date.now()}.json`;
  anchor.click();
  URL.revokeObjectURL(url);
}
