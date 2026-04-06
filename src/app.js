import { QuadcopterLab } from "./quadcopter.js";
import { Renderer } from "./renderer.js";

const sliderDefinitions = [
  { id: "stateWeight", label: "LQR State Weight", min: 0.4, max: 2.6, step: 0.01, value: 1.0 },
  { id: "effortWeight", label: "Control Effort Weight", min: 0.45, max: 2.8, step: 0.01, value: 1.0 },
  { id: "processNoise", label: "Kalman Process Trust", min: 0.45, max: 3.0, step: 0.01, value: 1.0 },
  { id: "measurementNoise", label: "Sensor Noise Weight", min: 0.4, max: 2.8, step: 0.01, value: 1.0 },
  { id: "windScale", label: "Wind Strength", min: 0.4, max: 2.4, step: 0.01, value: 1.0 },
];

let sim;
let renderer;
let statusLabel;
let statusDot;
let positionError;
let attitudeError;
let windValue;
let settleValue;
let controllerValue;
let routeValue;
let settlingTimeValue;
let peakErrorValue;
let estimateRmsValue;
let effortPeakValue;
let lqrSummary;
let pidSummary;
let demoStepTag;
let demoProgress;
let demoTitle;
let demoBody;
let demoAdvanceButton;
let demoResetButton;
let resetButton;
let gustButton;
let targetButton;
let scenarioButton;
let modeButton;
let routeButton;
let pauseButton;
let exportButton;
let paused = false;
let benchmarkTracker;
let demoDirector;

boot();

function boot() {
  try {
    sim = new QuadcopterLab();
    renderer = new Renderer(
      requireElement("sceneCanvas"),
      requireElement("motorCanvas"),
      requireElement("errorCanvas"),
      requireElement("effortCanvas"),
      requireElement("altitudeCanvas"),
    );

    statusLabel = requireElement("statusLabel");
    statusDot = requireElement("statusDot");
    positionError = requireElement("positionError");
    attitudeError = requireElement("attitudeError");
    windValue = requireElement("windValue");
    settleValue = requireElement("settleValue");
    controllerValue = requireElement("controllerValue");
    routeValue = requireElement("routeValue");
    settlingTimeValue = requireElement("settlingTimeValue");
    peakErrorValue = requireElement("peakErrorValue");
    estimateRmsValue = requireElement("estimateRmsValue");
    effortPeakValue = requireElement("effortPeakValue");
    lqrSummary = requireElement("lqrSummary");
    pidSummary = requireElement("pidSummary");
    demoStepTag = requireElement("demoStepTag");
    demoProgress = requireElement("demoProgress");
    demoTitle = requireElement("demoTitle");
    demoBody = requireElement("demoBody");
    demoAdvanceButton = requireElement("demoAdvanceButton");
    demoResetButton = requireElement("demoResetButton");
    resetButton = requireElement("resetButton");
    gustButton = requireElement("gustButton");
    targetButton = requireElement("targetButton");
    scenarioButton = requireElement("scenarioButton");
    modeButton = requireElement("modeButton");
    routeButton = requireElement("routeButton");
    pauseButton = requireElement("pauseButton");
    exportButton = requireElement("exportButton");

    buildSliders();
    wireButtons();
    benchmarkTracker = new BenchmarkTracker();
    demoDirector = new DemoDirector();
    benchmarkTracker.startEpisode(sim, "Initial hover recovery");
    demoDirector.render();
    kickIntro();

    let last = performance.now();
    const frame = (now) => {
      const dt = Math.min(0.035, (now - last) / 1000 || 1 / 60);
      last = now;
      if (!paused) {
        sim.step(dt);
      }
      benchmarkTracker.update(sim, dt, paused);
      renderer.render(sim, dt);
      syncText();
      requestAnimationFrame(frame);
    };

    window.__ACL_BOOTED = true;
    if (window.__ACL_BOOT_TIMEOUT) {
      window.clearTimeout(window.__ACL_BOOT_TIMEOUT);
    }
    requestAnimationFrame(frame);
  } catch (error) {
    console.error(error);
    if (window.__ACL_SHOW_BOOT_ERROR) {
      window.__ACL_SHOW_BOOT_ERROR(`Boot error: ${error.message}`);
    }
  }
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
  resetButton.addEventListener("click", () => {
    sim.reset();
    benchmarkTracker.startEpisode(sim, `${sim.controllerMode} reset`);
  });
  gustButton.addEventListener("click", () => {
    sim.injectWind();
    benchmarkTracker.startEpisode(sim, `${sim.controllerMode} gust recovery`);
  });
  targetButton.addEventListener("click", () => {
    sim.cycleTarget();
    targetButton.textContent = `Target: ${sim.targetName}`;
    benchmarkTracker.startEpisode(sim, `${sim.controllerMode} target retune`);
  });
  scenarioButton.addEventListener("click", () => {
    sim.cycleScenario();
    scenarioButton.textContent = `Scenario: ${sim.sceneName}`;
    benchmarkTracker.startEpisode(sim, `${sim.controllerMode} ${sim.sceneName.toLowerCase()} setup`);
  });
  modeButton.addEventListener("click", () => {
    sim.toggleControllerMode();
    modeButton.textContent = `Mode: ${sim.controllerMode}`;
    benchmarkTracker.startEpisode(sim, `${sim.controllerMode} controller`);
  });
  routeButton.addEventListener("click", () => {
    sim.cycleRoute();
    routeButton.textContent = `Route: ${sim.routeName}`;
    benchmarkTracker.startEpisode(sim, `${sim.controllerMode} ${sim.routeName.toLowerCase()} route`);
  });
  pauseButton.addEventListener("click", () => {
    paused = !paused;
    pauseButton.textContent = paused ? "Resume" : "Pause";
  });
  exportButton.addEventListener("click", exportTelemetry);
  demoAdvanceButton.addEventListener("click", () => demoDirector.advance());
  demoResetButton.addEventListener("click", () => demoDirector.reset());
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

  const benchmark = benchmarkTracker.getCurrentSnapshot();
  settlingTimeValue.textContent = benchmark.settlingTime;
  peakErrorValue.textContent = benchmark.peakError;
  estimateRmsValue.textContent = benchmark.estimateRms;
  effortPeakValue.textContent = benchmark.effortPeak;
  lqrSummary.textContent = benchmarkTracker.getControllerSummary("LQR");
  pidSummary.textContent = benchmarkTracker.getControllerSummary("PID");
  demoDirector.render();
}

function kickIntro() {
  setTimeout(() => {
    if (sim) {
      sim.reset();
    }
    if (window.__ACL_CLEAR_INTRO) {
      window.__ACL_CLEAR_INTRO();
    }
  }, 3200);
}

function exportTelemetry() {
  const payload = {
    ...sim.exportLog(),
    benchmark: benchmarkTracker.export(),
    demoStep: demoDirector.index,
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `aerial-control-lab-${Date.now()}.json`;
  anchor.click();
  URL.revokeObjectURL(url);
}

function requireElement(id) {
  const element = document.getElementById(id);
  if (!element) {
    throw new Error(`Missing required element: ${id}`);
  }
  return element;
}

class BenchmarkTracker {
  constructor() {
    this.current = null;
    this.history = {
      LQR: null,
      PID: null,
    };
  }

  startEpisode(simulation, label) {
    this.finalizeCurrent();
    this.current = {
      label,
      controller: simulation.controllerMode,
      scenario: simulation.sceneName,
      route: simulation.routeName,
      elapsed: 0,
      peakError: simulation.metrics?.positionError || 0,
      effortPeak: simulation.metrics?.effortNorm || 0,
      estimateSquareIntegral: 0,
      stableDuration: 0,
      settleSeconds: null,
      sampled: 0,
    };
  }

  update(simulation, dt, isPaused) {
    if (isPaused || !simulation.metrics) {
      return;
    }

    if (!this.current) {
      this.startEpisode(simulation, `${simulation.controllerMode} live sample`);
    }

    this.current.elapsed += dt;
    this.current.sampled += 1;
    this.current.peakError = Math.max(this.current.peakError, simulation.metrics.positionError);
    this.current.effortPeak = Math.max(this.current.effortPeak, simulation.metrics.effortNorm);
    this.current.estimateSquareIntegral += simulation.metrics.estimateGap ** 2 * dt;

    const isSettled = simulation.metrics.positionError < 0.35 && simulation.metrics.attitudeError < 6.5;
    this.current.stableDuration = isSettled ? this.current.stableDuration + dt : 0;

    if (this.current.settleSeconds === null && this.current.stableDuration > 0.9) {
      this.current.settleSeconds = Math.max(0, this.current.elapsed - this.current.stableDuration);
    }
  }

  finalizeCurrent() {
    if (!this.current || this.current.elapsed < 0.5) {
      return;
    }

    this.history[this.current.controller] = this.snapshotFromEpisode(this.current);
  }

  snapshotFromEpisode(episode) {
    const estimateRms = Math.sqrt(episode.estimateSquareIntegral / Math.max(episode.elapsed, 1e-6));
    return {
      controller: episode.controller,
      label: episode.label,
      scenario: episode.scenario,
      route: episode.route,
      settlingTime: episode.settleSeconds === null ? "Tracking..." : `${episode.settleSeconds.toFixed(2)} s`,
      peakError: `${episode.peakError.toFixed(2)} m`,
      effortPeak: episode.effortPeak.toFixed(2),
      estimateRms: `${estimateRms.toFixed(2)} m`,
      rawSettlingTime: episode.settleSeconds,
      rawPeakError: episode.peakError,
      rawEffortPeak: episode.effortPeak,
      rawEstimateRms: estimateRms,
    };
  }

  getCurrentSnapshot() {
    if (!this.current) {
      return {
        settlingTime: "Tracking...",
        peakError: "0.00 m",
        estimateRms: "0.00 m",
        effortPeak: "0.00",
      };
    }
    return this.snapshotFromEpisode(this.current);
  }

  getControllerSummary(controller) {
    const summary = this.history[controller];
    if (!summary) {
      return `No ${controller} recovery sample yet. Use the guided demo to capture one.`;
    }
    return `${summary.label}: settled ${summary.settlingTime}, peak ${summary.peakError}, estimate RMS ${summary.estimateRms}.`;
  }

  export() {
    return {
      current: this.getCurrentSnapshot(),
      history: this.history,
    };
  }
}

class DemoDirector {
  constructor() {
    this.index = 0;
    this.steps = [
      {
        tag: "Guided Demo",
        progress: "Ready",
        title: "Start the controls walkthrough",
        body:
          "This guided run stages the exact sequence worth showing in a portfolio review: LQR recovery, disturbance rejection, PID comparison, then route tracking.",
        button: "Start Demo",
        action: () => {
          paused = false;
          pauseButton.textContent = "Pause";
          sim.setControllerMode("LQR");
          sim.setRoute(0);
          sim.setTarget(0);
          sim.setScenario(0);
          sim.reset();
          modeButton.textContent = `Mode: ${sim.controllerMode}`;
          routeButton.textContent = `Route: ${sim.routeName}`;
          targetButton.textContent = `Target: ${sim.targetName}`;
          scenarioButton.textContent = `Scenario: ${sim.sceneName}`;
          benchmarkTracker.startEpisode(sim, "LQR hover recovery");
        },
      },
      {
        tag: "Step 1",
        progress: "1 / 4",
        title: "Watch the LQR hover recovery",
        body:
          "The quad starts displaced from hover. LQR uses the hover model to trade state error against control effort, so you should see a fairly direct but not too aggressive return.",
        button: "Inject Gust",
        action: () => {
          sim.injectWind();
          benchmarkTracker.startEpisode(sim, "LQR gust recovery");
        },
      },
      {
        tag: "Step 2",
        progress: "2 / 4",
        title: "Stress the estimator with a disturbance",
        body:
          "When the gust hits, compare the blue true vehicle with the gold estimate. If the Kalman filter is doing its job, the gap should stay modest while the controller regains hover.",
        button: "Switch To PID",
        action: () => {
          sim.setControllerMode("PID");
          sim.setRoute(0);
          sim.setScenario(0);
          sim.reset();
          modeButton.textContent = `Mode: ${sim.controllerMode}`;
          routeButton.textContent = `Route: ${sim.routeName}`;
          scenarioButton.textContent = `Scenario: ${sim.sceneName}`;
          benchmarkTracker.startEpisode(sim, "PID hover recovery");
        },
      },
      {
        tag: "Step 3",
        progress: "3 / 4",
        title: "Compare the PID baseline",
        body:
          "PID only reacts to local error terms. It is a strong baseline, but it usually overshoots more or uses sharper effort than LQR on the same recovery problem.",
        button: "Run Route Tracking",
        action: () => {
          sim.setControllerMode("LQR");
          sim.setRoute(2);
          sim.setScenario(0);
          sim.reset();
          modeButton.textContent = `Mode: ${sim.controllerMode}`;
          routeButton.textContent = `Route: ${sim.routeName}`;
          scenarioButton.textContent = `Scenario: ${sim.sceneName}`;
          benchmarkTracker.startEpisode(sim, "LQR spiral route");
        },
      },
      {
        tag: "Step 4",
        progress: "4 / 4",
        title: "Finish with route tracking",
        body:
          "The spiral route shows the same controller-estimator stack working beyond simple hover hold. Export the log afterward if you want a clean benchmark artifact for the repo.",
        button: "Restart Demo",
        action: () => {
          this.index = -1;
        },
      },
    ];
  }

  advance() {
    const step = this.steps[this.index];
    if (step?.action) {
      step.action();
    }
    this.index = (this.index + 1) % this.steps.length;
    this.render();
  }

  reset() {
    this.index = 0;
    paused = false;
    pauseButton.textContent = "Pause";
    sim.setControllerMode("LQR");
    sim.setRoute(0);
    sim.setTarget(0);
    sim.setScenario(0);
    sim.reset();
    modeButton.textContent = `Mode: ${sim.controllerMode}`;
    routeButton.textContent = `Route: ${sim.routeName}`;
    targetButton.textContent = `Target: ${sim.targetName}`;
    scenarioButton.textContent = `Scenario: ${sim.sceneName}`;
    benchmarkTracker.startEpisode(sim, "LQR hover recovery");
    this.render();
  }

  render() {
    const step = this.steps[this.index];
    if (!step) {
      return;
    }
    demoStepTag.textContent = step.tag;
    demoProgress.textContent = step.progress;
    demoTitle.textContent = step.title;
    demoBody.textContent = step.body;
    demoAdvanceButton.textContent = step.button;
  }
}
