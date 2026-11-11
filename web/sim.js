// RoaryX7 Web Demo — faithful 2D emulation of the autonomous runOpMode() loop
// AND the Roary non-blocking subsystem managers & state machines.
//
//   switch (AUTO) { START / LAUNCH / LOAD_SPIKE (+tremor) / PARK }   (optional full FSM)
//   mainDrive()                                                      (always — stick → mainDrivePowers)
//   if (DXEnabled)  splineToDX(TARGET, vel)                          (→ sendPower → motors = goTo·goToK + mainDrivePowers)
//   if (splineDone) goToV2(lastTarget, …)                            (final approach → same composition)
//   sensors.updateSlots(loopNr % 3)                                  (round-robin 3-slot color/distance polling)
//   intake.unloadManager() / launchControl() / Unstuck() / ocProtection()
//   outTake.step() / statusCode()                                    (slides, bascula, Farul LED)
//   updatePowerCompensator()
//
// "Roary never starts or stops anything — it just happens." The loop runs every
// frame; the stick always drives (mainDrivePowers); "Start Auto" toggles goToK
// so the spline's goTo power is added in or not.

'use strict';

// ─── Constants ───────────────────────────────────────────────────────────────
const FIELD_SIZE = 365;            // cm (Decode field ≈ 12 ft)
const ROBOT_SIZE = 20;             // cm
const CANVAS_SIZE = 640;
const CENTER = CANVAS_SIZE / 2;
const SCALE = CANVAS_SIZE / (FIELD_SIZE * 2); // px per cm
const DT = 1 / 60;                 // sim timestep (s)
const MAX_SPEED = 42;              // cm/s at |power| = 1
const MAX_ROT = 2.6;               // rad/s at |rotate| = 1

// ─── Real KirbyAirRiders waypoints (autonom/Kirby/KirbyAirRiders.java:62-86) ─
const WP_LIBRARY = {
  launch_first:      () => new goToValues(new Pose2d(-35, -124, -70), 0.71, 3),
  launch:            () => new goToValues(new Pose2d(-53, -145, -90), 0.53, 3),
  artifact_3_sliding:() => new goToValues(new Pose2d(-50, -110, -66), 0.71, 3),
  artifact_3_facing: () => new goToValues(new Pose2d(-120, -90, -90), 0.64, 3),
  artifact_3:        () => new goToValues(new Pose2d(-152, -94, -93), 0.53, 3),
  artifact_2_facing: () => new goToValues(new Pose2d(-80, -38, -93), 0.71, 3),
  artifact_2:        () => new goToValues(new Pose2d(-155, -38, -93), 0.71, 3),
  launch_close:      () => new goToValues(new Pose2d(-45, 26, -90), 0.64, 6.4),
  park:              () => new goToValues(new Pose2d(-45, 130, 0), 0.5, 3),
};

function defaultSplineTarget() {
  return [
    WP_LIBRARY.launch_first(),
    WP_LIBRARY.launch_first(),
    WP_LIBRARY.artifact_3_sliding(),
    WP_LIBRARY.artifact_3_facing(),
    WP_LIBRARY.artifact_3(),
    WP_LIBRARY.artifact_3(),
  ];
}

let TARGET = defaultSplineTarget();

// ─── Core Robot & Subsystem Instances (Kirby.java composition) ───────────────
const drive = new Roaryx7Drive('Kirby');
const sensors = new RoarySensors();
const outTake = new RoaryOutTake(sensors);
const intake = new RoaryIntake(sensors, outTake);

let DXEnabled = true;              // autonomous spline path enabled (false during tremor/manual sendPower)
let runtime = 0;                   // ElapsedTime equivalent (s)
let loopNr = 1;                    // 1..10 counter (KirbyAirRiders.java:1048)
let motif = 1;                     // 1, 2, or 3 (determines GREEN slot position in launchControl)

// ─── OpMode AUTO_STATE Machine (KirbyAirRiders.java:338-807) ─────────────────
let autoFsmEnabled = false;        // false = single-path spline mode; true = full multi-stage AUTO_STATE
let AUTO = 'IDLE';                 // IDLE | START | LAUNCH | LOAD_SPIKE | PARK
let hasStarted = true;
let currentAction = 1;
let delayTime = 0;
let safetyTime = 0;
let cycleNR = 0;
let tremor = 1;
let tremorTime = 0;
let side = 1;

// ─── Field Elements (2D artifacts to collect on the field) ───────────────────
let fieldElements = [];
function spawnFieldElements() {
  fieldElements = [
    // Spike 3 cluster (around artifact_3: -152, -94)
    { x: -138, y: -93, color: inIndexer.PURPLE, collected: false },
    { x: -148, y: -96, color: inIndexer.GREEN,  collected: false },
    { x: -158, y: -91, color: inIndexer.PURPLE, collected: false },
    // Spike 2 cluster (around artifact_2: -155, -38)
    { x: -135, y: -38, color: inIndexer.PURPLE, collected: false },
    { x: -147, y: -35, color: inIndexer.GREEN,  collected: false },
    { x: -157, y: -41, color: inIndexer.PURPLE, collected: false },
    // Spike 1 cluster (top-left)
    { x: -135, y: 29,  color: inIndexer.GREEN,  collected: false },
    { x: -146, y: 32,  color: inIndexer.PURPLE, collected: false },
    // Center-right practice elements for Intake Lab & clawToRobotPos
    { x: 45,   y: -40, color: inIndexer.PURPLE, collected: false },
    { x: 65,   y: -25, color: inIndexer.GREEN,  collected: false },
    { x: 55,   y: 20,  color: inIndexer.PURPLE, collected: false },
  ];
  outTake.scoredDrops = [];
}
spawnFieldElements();

// Odometry trail
const trail = [];
const MAX_TRAIL = 260;

// ─── Stick & Keyboard Input ──────────────────────────────────────────────────
let joyX = 0, joyY = 0, joyR = 0, joyActive = false;

const keys = Object.create(null);
window.addEventListener('keydown', (e) => {
  // Don't hijack keys while the user is typing in the waypoint list
  const t = e.target;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT')) return;
  const k = e.key.toLowerCase();
  keys[k] = true;
  if (k === 'b') toggleAuto();                                // gamepad1.bWasPressed() → toggle goToK
  if (k === 'r') resetAll();
  if (k === 'i') toggleIntakeEat();                           // toggle intake (-1 / 0)
  if (k === 'x') triggerLaunchControl();                      // gamepad1.x → launchControl
  if (k === 'u') triggerUnloadManager();                      // startUnload() → 6-step unloadManager
  if (k === 'j') triggerJamDemo();                            // simulate >7.1A jam
  if ((k === 'delete' || k === 'backspace') && selectedWaypoint >= 0) deleteWaypoint(selectedWaypoint);
  if (k === 'escape') { selectedWaypoint = -1; refreshWpSelection(); }
  if (['w', 'a', 's', 'd', 'q', 'e', 'b', 'r', 'i', 'x', 'u', 'j', ' ', 'delete', 'backspace'].includes(k)) {
    e.preventDefault();
  }
});
window.addEventListener('keyup', (e) => { keys[e.key.toLowerCase()] = false; });

// FPS
let frameCount = 0, lastFpsTime = performance.now(), currentHz = 60;

// ─── DOM References ──────────────────────────────────────────────────────────
const fieldCanvas = document.getElementById('field');
const fieldCtx = fieldCanvas.getContext('2d');
const joyCanvas = document.getElementById('joystick');
const joyCtx = joyCanvas.getContext('2d');

const btnStartAuto = document.getElementById('btn-start-auto');
const btnFullFsm = document.getElementById('btn-full-fsm');
const btnReset = document.getElementById('btn-reset');
const goToKIndicator = document.getElementById('goToK-indicator');
const chipGoToK = document.getElementById('chip-gotok');
const dotGoToK = document.getElementById('dot-gotok');

const ovVectors = document.getElementById('ov-vectors');
const ovZones = document.getElementById('ov-zones');
const ovOffsets = document.getElementById('ov-offsets');
const ovTrail = document.getElementById('ov-trail');
const ovLookahead = document.getElementById('ov-lookahead');

// ─── Auto toggle (goToK) & Full FSM toggle ───────────────────────────────────
function refreshAutoButton() {
  const on = drive.goToK === 1;
  btnStartAuto.textContent = on ? '■ Stop Auto (B)' : '▶ Start Auto (B)';
  btnStartAuto.classList.toggle('active', on);
  goToKIndicator.textContent = on
    ? 'goToK = 1 · spline + stick'
    : 'goToK = 0 · stick only';
  if (chipGoToK) chipGoToK.classList.toggle('on', on);
  if (dotGoToK) dotGoToK.classList.toggle('on', on);
  if (btnFullFsm) btnFullFsm.classList.toggle('active', autoFsmEnabled);
}

// "Start Auto" toggles goToK (faithful to gamepad1.bWasPressed()). Demo-UX addition:
// on the 0→1 edge we also re-arm the follower with resetSpline(), so a completed
// (sticky doneTranslationSpline) run re-follows the spline from wherever the robot
// is now, instead of goToV2-ing straight to the last waypoint.
function toggleAuto() {
  drive.toggleGoToK();
  if (drive.goToK === 1) {
    DXEnabled = true;
    drive.resetSpline();
  }
  refreshAutoButton();
}
btnStartAuto.onclick = toggleAuto;

function startFullAutoFsm() {
  autoFsmEnabled = !autoFsmEnabled;
  if (autoFsmEnabled) {
    AUTO = 'START';
    hasStarted = true;
    cycleNR = 0;
    DXEnabled = true;
    drive.setPosition(0, 0, 0);
    drive.vel = new Point(0, 0);
    drive.setGoToK(true);
    drive.resetSpline();
    spawnFieldElements();
    trail.length = 0;
  } else {
    AUTO = 'IDLE';
    DXEnabled = true;
  }
  refreshAutoButton();
}
if (btnFullFsm) btnFullFsm.onclick = startFullAutoFsm;

function setTargetWaypoints(newList) {
  TARGET.length = 0;
  newList.forEach(wp => TARGET.push(new goToValues(wp)));
  drive.resetSpline();
  selectedWaypoint = -1;
  rebuildWaypointList();
}

function resetAll() {
  autoFsmEnabled = false;
  AUTO = 'IDLE';
  DXEnabled = true;
  drive.setPosition(0, 0, 0);
  drive.vel = new Point(0, 0);
  drive.resetSpline();
  drive.mainDrivePowers = { FLPow: 0, FRPow: 0, BLPow: 0, BRPow: 0 };
  drive.FL = drive.FR = drive.BL = drive.BR = 0;
  drive.power = 1;
  intake.setPowerIntakeAuto(0);
  outTake.stopLauncher();
  trail.length = 0;
  refreshAutoButton();
}
btnReset.onclick = resetAll;

document.getElementById('btn-clear-trail').onclick = () => {
  trail.length = 0;
  spawnFieldElements();
};

// ─── Tabs & Scenario Presets ─────────────────────────────────────────────────
document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
    btn.classList.add('active');
    const pane = document.getElementById(btn.dataset.tab);
    if (pane) pane.classList.add('active');
  });
});

function switchTab(tabId) {
  const btn = document.querySelector(`.tab-btn[data-tab="${tabId}"]`);
  if (btn) btn.click();
}

document.querySelectorAll('.preset-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    const p = btn.dataset.preset;

    if (p === 'spline') {
      resetAll();
      setTargetWaypoints(defaultSplineTarget());
      drive.setGoToK(true);
      drive.resetSpline();
      refreshAutoButton();
      switchTab('tab-drive');
    } else if (p === 'full-auto') {
      resetAll();
      autoFsmEnabled = false;
      startFullAutoFsm();
      switchTab('tab-subsystems');
    } else if (p === 'intake-lab') {
      resetAll();
      spawnFieldElements();
      sensors.wipeSlots();
      intake.setHExtCm(18);
      document.getElementById('slider-hext').value = 18;
      document.getElementById('val-hext').textContent = '18 cm';
      intake.setPowerIntakeAuto(-1);
      // Set a path that sweeps across the Spike 3 & Spike 2 element clusters
      setTargetWaypoints([
        new goToValues(new Pose2d(-50, -110, -90), 0.65, 3),
        new goToValues(new Pose2d(-50, -110, -90), 0.65, 3),
        new goToValues(new Pose2d(-138, -94, -90), 0.55, 3),
        new goToValues(new Pose2d(-155, -92, -90), 0.50, 3),
        new goToValues(new Pose2d(-148, -38, -90), 0.55, 3),
        new goToValues(new Pose2d(-148, -38, -90), 0.55, 3),
      ]);
      drive.setPosition(-40, -115, toRadians(-90));
      drive.setGoToK(true);
      drive.resetSpline();
      refreshAutoButton();
      switchTab('tab-subsystems');
    } else if (p === 'offset-ik') {
      resetAll();
      spawnFieldElements();
      sensors.fillSlotsRobot();
      intake.setHExtCm(30);
      document.getElementById('slider-hext').value = 30;
      document.getElementById('val-hext').textContent = '30 cm';
      outTake.targetOutExtCm = 14;
      document.getElementById('slider-outext').value = 14;
      document.getElementById('val-outext').textContent = '14 cm';
      drive.setPosition(-45, -135, toRadians(-70));
      switchTab('tab-subsystems');
      triggerUnloadManager();
    }
  });
});

// ─── Subsystem UI Controls ───────────────────────────────────────────────────
function toggleIntakeEat() {
  intake.setPowerIntakeAuto(intake.intakePower < 0 ? 0 : -1);
}

function triggerLaunchControl() {
  if (sensors.areSlotsEmpty()) sensors.fillSlotsRobot();
  intake.updateRight(PaddleState.LOAD);
  intake.updateMid(PaddleState.LOAD);
  intake.updateLeft(PaddleState.LOAD);
  intake.launchFirstTime = true;
  intake.launchDone = false;
  outTake.startLauncher();
}

function triggerUnloadManager() {
  if (sensors.areSlotsEmpty()) sensors.fillSlotsRobot();
  intake.startUnload();
}

function triggerJamDemo() {
  if (intake.intakePower === 0) intake.setPowerIntakeAuto(-1);
  intake.triggerJam();
  intake.startUnstuck();
}

document.getElementById('btn-fill-slots').onclick = () => sensors.fillSlotsRobot();
document.getElementById('btn-wipe-slots').onclick = () => sensors.wipeSlots();
document.getElementById('btn-cycle-motif').onclick = () => {
  motif = (motif % 3) + 1;
  const labels = { 1: '1 (G·P·P)', 2: '2 (P·G·P)', 3: '3 (P·P·G)' };
  document.getElementById('motif-val').textContent = labels[motif];
};

document.getElementById('btn-intake-eat').onclick = () => intake.setPowerIntakeAuto(-1);
document.getElementById('btn-intake-stop').onclick = () => intake.setPowerIntakeAuto(0);
document.getElementById('btn-intake-spit').onclick = () => intake.setPowerIntakeAuto(1);
document.getElementById('btn-jam').onclick = triggerJamDemo;

document.getElementById('btn-launch-seq').onclick = triggerLaunchControl;
document.getElementById('btn-unload-seq').onclick = triggerUnloadManager;

// Compute robot center pose so the extended front claw lands directly on the nearest field element
document.getElementById('btn-snap-claw').onclick = () => {
  const uncollected = fieldElements.filter(e => !e.collected);
  if (uncollected.length === 0) spawnFieldElements();
  const pool = fieldElements.filter(e => !e.collected);
  const pos = drive.getPoint();
  let best = pool[0], bestD = Infinity;
  pool.forEach(el => {
    const d = Math.hypot(el.x - pos.x, el.y - pos.y);
    if (d < bestD) { bestD = d; best = el; }
  });
  if (!best) return;
  if (intake.targetHExtCm < 15) {
    intake.setHExtCm(24);
    document.getElementById('slider-hext').value = 24;
    document.getElementById('val-hext').textContent = '24 cm';
  }
  const headingDeg = toDegrees(Math.atan2(best.x - pos.x, best.y - pos.y));
  const desiredClaw = new goToValues(new Pose2d(best.x, best.y, -headingDeg), 0.6, 2.5);
  const robotGoal = drive.clawToRobotPos(desiredClaw, intake.targetHExtCm * 0.45);
  robotGoal.target.heading = headingDeg;
  const startPose = new goToValues(drive.getPose(), 0.65, 3);
  const midPose = new goToValues(
    new Pose2d((pos.x + robotGoal.target.x) / 2, (pos.y + robotGoal.target.y) / 2, headingDeg),
    0.65, 3
  );
  setTargetWaypoints([startPose, startPose, midPose, robotGoal, robotGoal]);
  intake.setPowerIntakeAuto(-1);
  drive.setGoToK(true);
  drive.resetSpline();
  refreshAutoButton();
};

const sliderHext = document.getElementById('slider-hext');
sliderHext.addEventListener('input', (e) => {
  const v = +e.target.value;
  intake.setHExtCm(v);
  document.getElementById('val-hext').textContent = `${v} cm`;
});

const sliderOutext = document.getElementById('slider-outext');
sliderOutext.addEventListener('input', (e) => {
  const v = +e.target.value;
  outTake.targetOutExtCm = v;
  outTake.velocityEnabled = v > 0;
  document.getElementById('val-outext').textContent = `${v} cm`;
});

const sliderVbat = document.getElementById('slider-vbat');
sliderVbat.addEventListener('input', (e) => {
  const v = parseFloat(e.target.value);
  drive.batteryVoltage = v;
  document.getElementById('val-vbat').textContent = `${v.toFixed(1)} V`;
});

// ─── Joystick input ──────────────────────────────────────────────────────────
function getJoyPos(e) {
  const rect = joyCanvas.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let x = (e.clientX - cx) / (rect.width / 2);
  let y = (e.clientY - cy) / (rect.height / 2);
  const r = Math.hypot(x, y);
  if (r > 1) { x /= r; y /= r; }
  return { x, y };
}

joyCanvas.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  joyActive = true;
  try { joyCanvas.setPointerCapture(e.pointerId); } catch (_) {}
  updateJoyFromPointer(e);
});
joyCanvas.addEventListener('pointermove', (e) => { if (joyActive) updateJoyFromPointer(e); });
joyCanvas.addEventListener('pointerup', () => { joyActive = false; joyX = 0; joyY = 0; });

function updateJoyFromPointer(e) {
  const p = getJoyPos(e);
  joyX = p.x;
  joyY = -p.y; // screen Y down → stick Y up
}

// ─── Keyboard → stick ────────────────────────────────────────────────────────
function updateJoyFromKeyboard() {
  if (joyActive) return;
  let kx = 0, ky = 0, kr = 0;
  // 0.6 stays below mainDrive's 0.89 full-stick ramp threshold (constant max speed)
  if (keys['w'] || keys['arrowup']) ky += 0.6;
  if (keys['s'] || keys['arrowdown']) ky -= 0.6;
  if (keys['a'] || keys['arrowleft']) kx -= 0.6;
  if (keys['d'] || keys['arrowright']) kx += 0.6;
  if (keys['q']) kr -= 0.5;
  if (keys['e']) kr += 0.5;

  joyX += (kx - joyX) * 0.35;
  joyY += (ky - joyY) * 0.35;
  joyR += (kr - joyR) * 0.35;

  if (Math.abs(joyX) < 0.01) joyX = 0;
  if (Math.abs(joyY) < 0.01) joyY = 0;
  if (Math.abs(joyR) < 0.01) joyR = 0;
}

// ─── Waypoint editing (TARGET — live, Roary keeps running) ───────────────────
const MIN_WAYPOINTS = 4;   // Catmull-Rom needs 4 control points
const MAX_WAYPOINTS = 32;
let selectedWaypoint = -1;
let dragWaypoint = -1;
let wpMsgTimer = null;

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

function fieldPointerPos(e) {
  const r = fieldCanvas.getBoundingClientRect();
  return {
    x: (e.clientX - r.left) * (fieldCanvas.width / r.width),
    y: (e.clientY - r.top) * (fieldCanvas.height / r.height),
  };
}

function canvasToWorld(px, py) {
  return { x: (px - CENTER) / SCALE, y: (CENTER - py) / SCALE };
}

function waypointAtCanvas(px, py, maxDist = 14) {
  let best = -1, bestD = maxDist;
  TARGET.forEach((wp, idx) => {
    const c = worldToCanvas(wp.target.x, wp.target.y);
    const d = Math.hypot(px - c.x, py - c.y);
    if (d < bestD) { bestD = d; best = idx; }
  });
  return best;
}

function distToSegment(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const l2 = dx * dx + dy * dy;
  if (l2 === 0) return Math.hypot(px - ax, py - ay);
  const t = clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0, 1);
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

function flashWpMsg(msg) {
  const el = document.getElementById('wp-msg');
  if (!el) return;
  el.textContent = msg;
  el.classList.add('show');
  if (wpMsgTimer) clearTimeout(wpMsgTimer);
  wpMsgTimer = setTimeout(() => el.classList.remove('show'), 2200);
}

// Drag a point on the field to move it (applies live)
fieldCanvas.addEventListener('pointerdown', (e) => {
  const p = fieldPointerPos(e);
  const idx = waypointAtCanvas(p.x, p.y);
  if (idx >= 0) {
    selectedWaypoint = idx;
    dragWaypoint = idx;
    try { fieldCanvas.setPointerCapture(e.pointerId); } catch (_) { /* synthetic pointer */ }
  } else {
    selectedWaypoint = -1;
  }
  refreshWpSelection();
});

fieldCanvas.addEventListener('pointermove', (e) => {
  const p = fieldPointerPos(e);
  if (dragWaypoint >= 0) {
    const w = canvasToWorld(p.x, p.y);
    const F = FIELD_SIZE / 2;
    TARGET[dragWaypoint].target.x = clamp(w.x, -F, F);
    TARGET[dragWaypoint].target.y = clamp(w.y, -F, F);
    fieldCanvas.style.cursor = 'grabbing';
    updateWpListValues(dragWaypoint);
  } else {
    fieldCanvas.style.cursor = waypointAtCanvas(p.x, p.y) >= 0 ? 'grab' : 'crosshair';
  }
});

const endWpDrag = () => { dragWaypoint = -1; };
fieldCanvas.addEventListener('pointerup', endWpDrag);
fieldCanvas.addEventListener('pointercancel', endWpDrag);

// Double-click the field → insert a new point near the click
fieldCanvas.addEventListener('dblclick', (e) => {
  const p = fieldPointerPos(e);
  const onPoint = waypointAtCanvas(p.x, p.y);
  if (onPoint >= 0) { selectedWaypoint = onPoint; refreshWpSelection(); return; }
  const w = canvasToWorld(p.x, p.y);
  if (Math.abs(w.x) > FIELD_SIZE / 2 || Math.abs(w.y) > FIELD_SIZE / 2) return;
  insertWaypoint(w.x, w.y);
});

function insertWaypoint(worldX, worldY) {
  if (TARGET.length >= MAX_WAYPOINTS) { flashWpMsg(`Max ${MAX_WAYPOINTS} points`); return; }
  const px = worldX, py = worldY;

  let bestSeg = 0, bestSegD = Infinity;
  for (let i = 0; i < TARGET.length - 1; i++) {
    const d = distToSegment(px, py, TARGET[i].target.x, TARGET[i].target.y, TARGET[i + 1].target.x, TARGET[i + 1].target.y);
    if (d < bestSegD) { bestSegD = d; bestSeg = i; }
  }
  const dFirst = Math.hypot(px - TARGET[0].target.x, py - TARGET[0].target.y);
  const last = TARGET[TARGET.length - 1];
  const dLast = Math.hypot(px - last.target.x, py - last.target.y);
  let insertAt;
  if (dFirst < bestSegD) insertAt = 0;
  else if (dLast < bestSegD) insertAt = TARGET.length;
  else insertAt = bestSeg + 1;

  const prev = TARGET[clamp(insertAt - 1, 0, TARGET.length - 1)];
  const next = TARGET[clamp(insertAt, 0, TARGET.length - 1)];
  const a1 = toRadians(prev.target.heading), a2 = toRadians(next.target.heading);
  const mx = Math.sin(a1) + Math.sin(a2), my = Math.cos(a1) + Math.cos(a2);
  const heading = Math.hypot(mx, my) > 1e-9 ? toDegrees(Math.atan2(mx, my)) : prev.target.heading;
  const wp = new goToValues(new Pose2d(px, py, heading), (prev.speed + next.speed) / 2, prev.error);

  TARGET.splice(insertAt, 0, wp);
  drive.resetSpline();          // re-seat follower + refresh drive.length
  selectedWaypoint = insertAt;
  rebuildWaypointList();
  flashWpMsg(`Inserted point ${insertAt} — selected`);
}

function deleteWaypoint(idx) {
  if (idx < 0 || idx >= TARGET.length) return;
  if (TARGET.length <= MIN_WAYPOINTS) { flashWpMsg(`Spline needs at least ${MIN_WAYPOINTS} points`); return; }
  TARGET.splice(idx, 1);
  drive.resetSpline();
  selectedWaypoint = -1;
  rebuildWaypointList();
  flashWpMsg(`Deleted point ${idx}`);
}

// ─── Waypoint list (per-point x / y / heading / speed / error) ───────────────
function rebuildWaypointList() {
  const list = document.getElementById('wp-list');
  const badge = document.getElementById('wp-count-badge');
  if (badge) badge.textContent = `${TARGET.length} pts`;
  if (!list) return;
  list.innerHTML = '';
  TARGET.forEach((wp, idx) => {
    const row = document.createElement('div');
    row.className = 'wp-row' + (idx === selectedWaypoint ? ' selected' : '');
    row.dataset.idx = idx;
    row.innerHTML =
      `<span class="wp-idx">${idx}</span>` +
      `<label>x<input data-f="x" type="number" step="1" value="${Math.round(wp.target.x * 10) / 10}"></label>` +
      `<label>y<input data-f="y" type="number" step="1" value="${Math.round(wp.target.y * 10) / 10}"></label>` +
      `<label>h°<input data-f="h" type="number" step="1" value="${Math.round(wp.target.heading * 10) / 10}"></label>` +
      `<label>spd<input data-f="s" type="number" step="0.05" min="0" max="1" value="${wp.speed}"></label>` +
      `<label>err<input data-f="e" type="number" step="0.5" min="0.5" value="${wp.error}"></label>` +
      `<button class="wp-del" title="Delete point ${idx}">✕</button>`;
    list.appendChild(row);
  });
}

function refreshWpSelection() {
  document.querySelectorAll('.wp-row').forEach((row) => {
    row.classList.toggle('selected', +row.dataset.idx === selectedWaypoint);
  });
}

function updateWpListValues(idx) {
  const row = document.querySelector(`.wp-row[data-idx="${idx}"]`);
  if (!row) return;
  const wp = TARGET[idx];
  const ix = row.querySelector('[data-f="x"]'), iy = row.querySelector('[data-f="y"]');
  if (ix && document.activeElement !== ix) ix.value = Math.round(wp.target.x * 10) / 10;
  if (iy && document.activeElement !== iy) iy.value = Math.round(wp.target.y * 10) / 10;
}

function applyWaypointInput(idx, field, value, el) {
  const v = parseFloat(value);
  if (!isFinite(v)) return;
  const wp = TARGET[idx];
  const F = FIELD_SIZE / 2;
  let out = v;
  if (field === 'x') { out = clamp(v, -F, F); wp.target.x = out; }
  else if (field === 'y') { out = clamp(v, -F, F); wp.target.y = out; }
  else if (field === 'h') { wp.target.heading = v; }
  else if (field === 's') { out = clamp(v, 0, 1); wp.speed = out; }
  else if (field === 'e') { out = Math.max(0.5, v); wp.error = out; }
  if (el && el === document.activeElement && out !== v) el.value = out;
}

const wpListEl = document.getElementById('wp-list');
if (wpListEl) {
  wpListEl.addEventListener('input', (e) => {
    const row = e.target.closest('.wp-row');
    const f = e.target.dataset.f;
    if (!row || !f) return;
    applyWaypointInput(+row.dataset.idx, f, e.target.value, e.target);
  });
  wpListEl.addEventListener('click', (e) => {
    const del = e.target.closest('.wp-del');
    if (del) { deleteWaypoint(+del.closest('.wp-row').dataset.idx); return; }
    const row = e.target.closest('.wp-row');
    if (row) { selectedWaypoint = +row.dataset.idx; refreshWpSelection(); }
  });
}

// ─── Physics: motor powers → world motion ────────────────────────────────────
// Inverse of the drive IK  (FL = rotY+rotX+hp, FR = rotY-rotX-hp, BL = rotY-rotX+hp, BR = rotY+rotX-hp):
//   forward = (FL+FR+BL+BR)/4      strafe = (FL-FR-BL+BR)/4      rotate = (FL-FR+BL-BR)/4
// World transform from Roaryx7.java:update (odometry):
//   worldX = sin(H)·forward + cos(H)·strafe
//   worldY = cos(H)·forward − sin(H)·strafe
function stepPhysics(dt) {
  const FL = drive.FL, FR = drive.FR, BL = drive.BL, BR = drive.BR;
  const forward = (FL + FR + BL + BR) / 4;
  const strafe = (FL - FR - BL + BR) / 4;
  const rotate = (FL - FR + BL - BR) / 4;

  const H = drive.H;
  const worldX = Math.sin(H) * forward + Math.cos(H) * strafe;
  const worldY = Math.cos(H) * forward - Math.sin(H) * strafe;

  drive.X = clamp(drive.X + worldX * MAX_SPEED * dt, -FIELD_SIZE / 2, FIELD_SIZE / 2);
  drive.Y = clamp(drive.Y + worldY * MAX_SPEED * dt, -FIELD_SIZE / 2, FIELD_SIZE / 2);
  drive.H = angleWrap(drive.H + rotate * MAX_ROT * dt);
  drive.vel = new Point(worldX * MAX_SPEED, worldY * MAX_SPEED);

  if (loopNr % 3 === 0 && (Math.abs(worldX) + Math.abs(worldY) > 0.01)) {
    trail.push({ x: drive.X, y: drive.Y });
    if (trail.length > MAX_TRAIL) trail.shift();
  }
}

// ─── OpMode AUTO_STATE Machine Step (KirbyAirRiders.java:338-807) ────────────
function stepAutoStateMachine() {
  if (!autoFsmEnabled) return;

  const endPt = TARGET[TARGET.length - 1].target;
  const parked = drive.splineDone && drive.doneGoTo && drive.getPoint().distanceTo(endPt) < 6.5;

  switch (AUTO) {
    case 'START': {
      sensors.fillSlotsRobot();
      AUTO = 'LAUNCH';
      hasStarted = true;
      break;
    }
    case 'LAUNCH': {
      if (hasStarted) {
        sensors.fillSlotsRobot();
        intake.setPowerIntakeAuto(-1);
        DXEnabled = true;
        if (cycleNR === 0) {
          setTargetWaypoints([
            new goToValues(new Pose2d(0, 0, 0), 0.71, 3),
            new goToValues(new Pose2d(0, 0, 0), 0.71, 3),
            WP_LIBRARY.launch_first(),
            WP_LIBRARY.launch_first(),
          ]);
        } else {
          const cur = new goToValues(drive.getPose(), 0.65, 3);
          setTargetWaypoints([cur, cur, WP_LIBRARY.launch(), WP_LIBRARY.launch()]);
        }
        delayTime = runtime + 0.45;
        currentAction = 1;
        hasStarted = false;
      }

      if (parked) {
        if (currentAction === 1) {
          intake.launchFirstTime = true;
          intake.launchDone = false;
          outTake.startLauncher();
          currentAction = 2;
        }
        if (currentAction === 2 && intake.launchDone) {
          intake.setPowerIntakeAuto(0);
          outTake.stopLauncher();
          cycleNR++;
          hasStarted = true;
          AUTO = cycleNR <= 1 ? 'LOAD_SPIKE' : 'PARK';
        }
      }
      break;
    }
    case 'LOAD_SPIKE': {
      if (hasStarted) {
        setTargetWaypoints(defaultSplineTarget());
        DXEnabled = true;
        currentAction = 1;
        hasStarted = false;
        safetyTime = runtime + 7.5;
        sensors.wipeSlots();
        intake.setHExtCm(16);
      }

      // KirbyAirRiders.java:431 — turn intake ON automatically as soon as robot leaves the small triangle
      if (!isRobotInSmallTriangle(drive.getPose(), 35, 38) && intake.intakePower === 0) {
        intake.setPowerIntakeAuto(-1);
      }

      // KirbyAirRiders.java:488-636 — when at spike (or slots full), switch DXEnabled=false & run tremor!
      if (parked || sensors.areSlotsFull() || runtime > safetyTime || !DXEnabled) {
        if (currentAction === 1) {
          DXEnabled = false;
          intake.setPowerIntakeAuto(-1);
          delayTime = runtime + 1.25;
          tremorTime = runtime + 0.35;
          currentAction = 2;
        }
        // KirbyAirRiders.java:622 — tremor oscillation in Roary space to shake artifacts into the intake
        if (!DXEnabled) {
          drive.sendPower(new Pose2d(-0.13 * side, 0.17 * tremor, -90 * side + 8 * tremor));
        }
        if (runtime > tremorTime) {
          tremorTime = runtime + 0.35;
          tremor *= -1;
        }
        if ((currentAction === 2 && runtime > delayTime) || runtime > safetyTime || sensors.areSlotsFull()) {
          intake.setHExtCm(0);
          AUTO = 'LAUNCH';
          hasStarted = true;
        }
      }
      break;
    }
    case 'PARK': {
      if (hasStarted) {
        const cur = new goToValues(drive.getPose(), 0.65, 3);
        setTargetWaypoints([cur, cur, WP_LIBRARY.launch_close(), WP_LIBRARY.park(), WP_LIBRARY.park()]);
        DXEnabled = true;
        currentAction = 1;
        hasStarted = false;
      }
      if (parked) {
        intake.setPowerIntakeAuto(0);
        outTake.stopLauncher();
      }
      break;
    }
  }
}

// ─── Field Element Collection (2D top-down intake pickup) ────────────────────
function stepFieldCollection() {
  // Intake picks up elements near the front claw Pose2d (`robotToClawPos`) when rollers < 0
  if (intake.motorOutput >= -0.05 || sensors.areSlotsFull()) return;
  const renderPose = new Pose2d(drive.X, drive.Y, -drive.getH());
  const clawPose = drive.robotToClawPos(renderPose, intake.hExtCm * 0.45);
  const robotPos = drive.getPoint();

  for (const el of fieldElements) {
    if (el.collected) continue;
    const dClaw = Math.hypot(el.x - clawPose.x, el.y - clawPose.y);
    const dRobot = Math.hypot(el.x - robotPos.x, el.y - robotPos.y);
    if (dClaw < 15 || dRobot < 14) {
      const slot = sensors.senseElement(el.color);
      if (slot >= 0) {
        el.collected = true;
        break;
      }
    }
  }
}

// ─── runOpMode() loop ────────────────────────────────────────────────────────
// One fixed-rate simulation step (60 Hz): the full runOpMode() body.
function control() {
  runtime += DT;
  updateJoyFromKeyboard();

  // 0) OpMode AUTO_STATE machine (when enabled)
  stepAutoStateMachine();

  // 1) mainDrive() — always. (stick up = forward, so feed -joyY to match the
  //    gamepad convention the Java expects; see mainDrive in KirbyAirRiders)
  drive.mainDrive(joyX, -joyY, joyR);

  // 2) spline / goTo — the autonomous path (always live, gated by goToK)
  if (drive.splineDone) drive.target = TARGET[TARGET.length - 1].target;

  if (DXEnabled) drive.splineToDX(TARGET, drive.vel);

  if (drive.splineDone) {
    if (DXEnabled) {
      drive.goToV2(
        TARGET[TARGET.length - 1].target,
        Math.min(0.24, TARGET[TARGET.length - 1].speed),
        TARGET[TARGET.length - 1].error
      );
    }
  }

  // 3) Subsystem Managers (non-blocking, run every tick just like KirbyAirRiders / KirbyTester)
  sensors.updateSlots(loopNr % 3);          // round-robin slot polling (KirbyAirRiders.java:925)
  stepFieldCollection();

  // Spit-on-full protection (KirbyTester.java:408-421)
  if (sensors.areSlotsFull()) {
    if (!intake.hasSpit) {
      intake.spitTime = runtime + 0.24;
      intake.hasSpit = true;
    }
    if (runtime < intake.spitTime) {
      intake.motorOutput = 1;               // brief 0.24s spit when 3/3 slots fill
    }
  } else {
    intake.hasSpit = false;
  }

  // Reserve right paddle when intaking into empty L/R slots (KirbyTester.java:422-426)
  if (intake.launchDone && intake.unloadDone && intake.doneUnstuck) {
    if (intake.intakePower < 0 && sensors.slots[2] === inIndexer.NOTHING && sensors.slots[1] === inIndexer.NOTHING) {
      intake.updateRight(PaddleState.RESERVED);
    } else {
      intake.updateRight(PaddleState.LOAD);
    }
  }

  intake.Unstuck(runtime);
  intake.unloadManager(runtime, drive);
  if (!intake.launchDone) {
    const pose = drive.getPose();
    const inZone = isRobotInSmallTriangle(pose, 35, 38) || isRobotInBigTriangle(pose, 35, 38) || outTake.velocityEnabled;
    intake.launchControl(runtime, inZone && drive.vel.radius() < 30, motif, drive);
  }
  intake.ocProtection(runtime);
  intake.stepSlide(DT);

  outTake.step(DT, drive);
  outTake.statusCode(drive.getPose(), runtime);

  // 4) updatePowerCompensator()
  drive.updatePowerCompensator();

  // 5) integrate the motors → motion
  stepPhysics(DT);

  loopNr++;
  if (loopNr > 10) loopNr = 1;
}

// Drawing + telemetry (runs once per animation frame).
function render() {
  drawField();
  drawZonesAndElements();
  drawWaypoints();
  drawForceVectors();
  drawRobot();
  drawJoystick();
  updateTelemetry();
}

// ─── Drawing ─────────────────────────────────────────────────────────────────
function worldToCanvas(x, y) {
  return { x: CENTER + x * SCALE, y: CENTER - y * SCALE };
}

function drawArrow(ctx, x1, y1, x2, y2, color, width = 2, head = 7) {
  const dx = x2 - x1, dy = y2 - y1;
  if (Math.hypot(dx, dy) < 2) return;
  const a = Math.atan2(dy, dx);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - head * Math.cos(a - 0.45), y2 - head * Math.sin(a - 0.45));
  ctx.lineTo(x2 - head * Math.cos(a + 0.45), y2 - head * Math.sin(a + 0.45));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawField() {
  const ctx = fieldCtx;
  ctx.clearRect(0, 0, CANVAS_SIZE, CANVAS_SIZE);

  const half = FIELD_SIZE * SCALE / 2;
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 2;
  ctx.strokeRect(CENTER - half, CENTER - half, FIELD_SIZE * SCALE, FIELD_SIZE * SCALE);

  ctx.strokeStyle = '#1b2027';
  ctx.lineWidth = 1;
  for (let i = -FIELD_SIZE / 2; i <= FIELD_SIZE / 2; i += 25) {
    let a = worldToCanvas(i, FIELD_SIZE / 2), b = worldToCanvas(i, -FIELD_SIZE / 2);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    a = worldToCanvas(-FIELD_SIZE / 2, i); b = worldToCanvas(FIELD_SIZE / 2, i);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
  }

  // center axes + circle
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(CENTER, CENTER - half); ctx.lineTo(CENTER, CENTER + half); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(CENTER - half, CENTER); ctx.lineTo(CENTER + half, CENTER); ctx.stroke();
  ctx.beginPath(); ctx.arc(CENTER, CENTER, 26, 0, Math.PI * 2); ctx.stroke();

  // Odometry trail
  if (ovTrail && ovTrail.checked && trail.length > 1) {
    ctx.strokeStyle = 'rgba(88, 166, 255, 0.28)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    trail.forEach((pt, idx) => {
      const c = worldToCanvas(pt.x, pt.y);
      if (idx === 0) ctx.moveTo(c.x, c.y);
      else ctx.lineTo(c.x, c.y);
    });
    ctx.stroke();
  }
}

function drawZonesAndElements() {
  const ctx = fieldCtx;
  const pose = drive.getPose();

  // 1) Meth.java triangle zones (isRobotInBigTriangle / isRobotInSmallTriangle)
  if (ovZones && ovZones.checked) {
    const inBig = isRobotInBigTriangle(pose, 35, 38);
    const inSmall = isRobotInSmallTriangle(pose, 35, 38);

    // Big Triangle: (-180,180), (180,180), (0,0)
    const b1 = worldToCanvas(-180, 180), b2 = worldToCanvas(180, 180), b3 = worldToCanvas(0, 0);
    ctx.fillStyle = inBig ? 'rgba(63, 185, 80, 0.11)' : 'rgba(88, 166, 255, 0.04)';
    ctx.strokeStyle = inBig ? 'rgba(63, 185, 80, 0.55)' : 'rgba(88, 166, 255, 0.18)';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(b1.x, b1.y); ctx.lineTo(b2.x, b2.y); ctx.lineTo(b3.x, b3.y); ctx.closePath();
    ctx.fill(); ctx.stroke();

    // Small Triangle: (-60,-180), (60,-180), (0,-120)
    const s1 = worldToCanvas(-60, -180), s2 = worldToCanvas(60, -180), s3 = worldToCanvas(0, -120);
    ctx.fillStyle = inSmall ? 'rgba(63, 185, 80, 0.15)' : 'rgba(240, 180, 41, 0.06)';
    ctx.strokeStyle = inSmall ? 'rgba(63, 185, 80, 0.65)' : 'rgba(240, 180, 41, 0.28)';
    ctx.beginPath();
    ctx.moveTo(s1.x, s1.y); ctx.lineTo(s2.x, s2.y); ctx.lineTo(s3.x, s3.y); ctx.closePath();
    ctx.fill(); ctx.stroke();

    ctx.fillStyle = 'rgba(139, 148, 158, 0.65)';
    ctx.font = '10px monospace';
    ctx.fillText(inBig ? 'BIG TRIANGLE (IN ZONE)' : 'bigTriangle (Meth.java)', b1.x + 8, b1.y + 14);
    ctx.fillText(inSmall ? 'SMALL TRIANGLE (IN ZONE)' : 'smallTriangle (launch)', s1.x + 12, s1.y - 6);
  }

  // 2) Field elements (collectible PURPLE / GREEN artifacts)
  fieldElements.forEach((el) => {
    if (el.collected) return;
    const c = worldToCanvas(el.x, el.y);
    ctx.save();
    ctx.fillStyle = el.color === inIndexer.GREEN ? '#3fb950' : '#a371f7';
    ctx.strokeStyle = el.color === inIndexer.GREEN ? '#7ee787' : '#d2a8ff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(c.x, c.y, 5.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  });

  // 3) Scored / unloaded elements placed on the field by unloadManager or launchControl
  outTake.scoredDrops.forEach((drop) => {
    const c = worldToCanvas(drop.x, drop.y);
    ctx.save();
    ctx.fillStyle = drop.color === inIndexer.GREEN ? 'rgba(63,185,80,0.85)' : 'rgba(163,113,247,0.85)';
    ctx.strokeStyle = '#f0f6fc';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(c.x, c.y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#e6edf3';
    ctx.font = '9px monospace';
    ctx.fillText('★', c.x - 3, c.y + 3);
    ctx.restore();
  });
}

function drawWaypoints() {
  const ctx = fieldCtx;

  // Spline path (the whole route faintly, plus the active segment i highlighted)
  if (TARGET.length > 3) {
    ctx.strokeStyle = 'rgba(88,166,255,0.32)';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < TARGET.length - 3; i++) {
      const pts = [0, 1, 2, 3].map(k => new Point(TARGET[i + k].target.x, TARGET[i + k].target.y));
      for (let t = 0; t <= 1; t += 0.02) {
        const p = drive.catmullRomSpline(pts, t);
        const c = worldToCanvas(p.x, p.y);
        if (!started) { ctx.moveTo(c.x, c.y); started = true; } else ctx.lineTo(c.x, c.y);
      }
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // Active window i highlight
    if (!drive.splineDone && drive.i >= 0 && drive.i < TARGET.length - 3) {
      const pts = [0, 1, 2, 3].map(k => new Point(TARGET[drive.i + k].target.x, TARGET[drive.i + k].target.y));
      ctx.strokeStyle = 'rgba(88, 166, 255, 0.85)';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      for (let t = 0; t <= 1; t += 0.02) {
        const p = drive.catmullRomSpline(pts, t);
        const c = worldToCanvas(p.x, p.y);
        if (t === 0) ctx.moveTo(c.x, c.y); else ctx.lineTo(c.x, c.y);
      }
      ctx.stroke();
    }
  }

  // Optional lookAhead / lookBehind / futurePos debug markers
  if (ovLookahead && ovLookahead.checked && !drive.splineDone) {
    if (drive.lookAhead) {
      const la = worldToCanvas(drive.lookAhead.x, drive.lookAhead.y);
      ctx.strokeStyle = 'rgba(240,180,41,0.45)';
      ctx.fillStyle = 'rgba(240,180,41,0.75)';
      ctx.beginPath(); ctx.arc(la.x, la.y, 3.5, 0, Math.PI * 2); ctx.fill();
      ctx.font = '9px monospace';
      ctx.fillText('lookAhead', la.x + 5, la.y - 3);
    }
    if (drive.futurePos) {
      const fp = worldToCanvas(drive.futurePos.x, drive.futurePos.y);
      ctx.fillStyle = '#58a6ff';
      ctx.beginPath(); ctx.arc(fp.x, fp.y, 3.5, 0, Math.PI * 2); ctx.fill();
      ctx.font = '9px monospace';
      ctx.fillText('futurePos', fp.x + 5, fp.y + 9);
    }
  }

  // current spline target + true tangent (Catmull-Rom derivative at localT).
  // lookAhead = spline(localT + 0.2·(length−3)) extrapolates past t=1 mid-path,
  // so drawing target→lookAhead was misleading; the derivative is the real direction.
  if (drive.target && drive.goToK === 1) {
    const c = worldToCanvas(drive.target.x, drive.target.y);
    ctx.fillStyle = '#f0b429';
    ctx.beginPath(); ctx.arc(c.x, c.y, 5, 0, Math.PI * 2); ctx.fill();
    const d = drive.derivative;
    const dl = d ? Math.hypot(d.x, d.y) : 0;
    if (dl > 1e-6 && !drive.splineDone) {
      const L = 28; // px
      const ex = c.x + (d.x / dl) * L, ey = c.y - (d.y / dl) * L; // canvas Y flipped
      drawArrow(ctx, c.x, c.y, ex, ey, 'rgba(240,180,41,0.9)', 2, 7);
    }
  }

  // waypoint dots + desired heading ticks
  TARGET.forEach((wp, idx) => {
    const c = worldToCanvas(wp.target.x, wp.target.y);
    const isLast = idx === TARGET.length - 1;
    if (idx === selectedWaypoint) {
      ctx.strokeStyle = '#f0b429';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.arc(c.x, c.y, 10, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
    }
    // Small heading tick showing waypoint target.heading
    const hRad = toRadians(wp.target.heading);
    const hx = c.x + Math.sin(hRad) * 11;
    const hy = c.y - Math.cos(hRad) * 11;
    ctx.strokeStyle = isLast ? 'rgba(63,185,80,0.7)' : 'rgba(88,166,255,0.6)';
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(hx, hy); ctx.stroke();

    ctx.fillStyle = isLast ? '#3fb950' : '#58a6ff';
    ctx.beginPath(); ctx.arc(c.x, c.y, 4, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#8b949e';
    ctx.font = '10px monospace';
    ctx.fillText(isLast ? 'END' : `WP${idx}`, c.x + 6, c.y - 4);
  });
}

// ─── Force Vector Decomposition Overlay (normalPower + derivativePower + projectionPower) ──
function drawForceVectors() {
  if (!ovVectors || !ovVectors.checked) return;
  if (drive.splineDone || !drive.totalPower) return;

  const ctx = fieldCtx;
  const pos = drive.getPoint();
  const c = worldToCanvas(pos.x, pos.y);
  const VSCALE = 58; // px per unit power

  // Note: in sendPower(), direction is negated via tVR = -(-direction.atan() - PI/2 - H) + PI/4,
  // meaning a Point in `normalPower / derivativePower / projectionPower` pushes the robot in
  // world direction (-p.x, -p.y). Draw in actual world push direction so vectors match motion!
  const drawVec = (pt, color, width = 2) => {
    if (!pt) return;
    const mag = pt.radius();
    if (mag < 0.015) return;
    const wx = -pt.x, wy = -pt.y;
    const ex = c.x + wx * VSCALE;
    const ey = c.y - wy * VSCALE;
    drawArrow(ctx, c.x, c.y, ex, ey, color, width, 6);
  };

  drawVec(drive.normalPower, '#ff7b72', 2);       // normal correction (onto spline)
  drawVec(drive.derivativePower, '#3fb950', 2);   // tangent cruise power
  drawVec(drive.projectionPower, '#d2a8ff', 2);   // centripetal lookAhead projection
  drawVec(drive.totalPower, '#58a6ff', 2.5);      // resultant totalPower
}

function drawRobot() {
  const ctx = fieldCtx;
  const pos = drive.getPoint();
  const c = worldToCanvas(pos.x, pos.y);
  const H = drive.H;
  const s = ROBOT_SIZE * SCALE;

  // 1) Draw front Claw (`robotToClawPos`) and rear OutTake (`robotToOutTakePos`) slides in world space.
  // In the demo's convention, robot forward in world is (sin H, cos H) = heading H * 180 / PI,
  // while Java's robotToClawPos / robotToOutTakePos use (-heading) for its mirrored dashboard frame.
  // Passing -getH() aligns the 2D canvas rendering with the robot's green forward arrow (+forward).
  if (ovOffsets && ovOffsets.checked) {
    const renderPose = new Pose2d(pos.x, pos.y, -drive.getH());

    // Front horizontal slide (hExt) + Claw
    if (intake.hExtCm > 0.5 || intake.motorOutput !== 0) {
      const clawWorld = drive.robotToClawPos(renderPose, intake.hExtCm * 0.45);
      const cc = worldToCanvas(clawWorld.x, clawWorld.y);
      ctx.strokeStyle = intake.motorOutput < 0 ? '#3fb950' : intake.motorOutput > 0 ? '#f85149' : '#58a6ff';
      ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(cc.x, cc.y); ctx.stroke();
      ctx.fillStyle = '#0d1117';
      ctx.beginPath(); ctx.arc(cc.x, cc.y, 5, 0, Math.PI * 2); ctx.fill();
      ctx.stroke();
    }

    // Rear outTake slide (outExt) + Bascula/Claw
    if (outTake.outExtCm > 0.5) {
      const outWorld = drive.robotToOutTakePos(renderPose, outTake.outExtCm * 0.55);
      const oc = worldToCanvas(outWorld.x, outWorld.y);
      ctx.strokeStyle = '#bc8cff';
      ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(oc.x, oc.y); ctx.stroke();
      ctx.fillStyle = outTake.clawState === 'OPEN_BASKET' ? '#3fb950' : '#bc8cff';
      ctx.beginPath(); ctx.arc(oc.x, oc.y, 5, 0, Math.PI * 2); ctx.fill();
    }
  }

  // 2) Robot chassis in local frame (rotated by H — DO NOT derive angle from worldToCanvas deltas)
  ctx.save();
  ctx.translate(c.x, c.y);
  ctx.rotate(H); // forward edge points where +forward motion goes

  ctx.fillStyle = 'rgba(88,166,255,0.18)';
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 2;
  ctx.fillRect(-s / 2, -s / 2, s, s);
  ctx.strokeRect(-s / 2, -s / 2, s, s);

  // Active intake roller bar on front edge
  if (Math.abs(intake.motorOutput) > 0.05) {
    ctx.strokeStyle = intake.motorOutput < 0 ? '#3fb950' : '#f85149';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-s / 2 + 3, -s / 2 - 2);
    ctx.lineTo(s / 2 - 3, -s / 2 - 2);
    ctx.stroke();
  }

  // forward arrow (top edge)
  ctx.fillStyle = '#3fb950';
  ctx.beginPath();
  ctx.moveTo(0, -s / 2 - 7);
  ctx.lineTo(-5, -s / 2);
  ctx.lineTo(5, -s / 2);
  ctx.closePath();
  ctx.fill();

  // 3 internal indexer slots [Left, Mid, Right] drawn inside the chassis!
  const slotCoords = [
    { idx: 0, x: 0,      y: 0 }, // Mid
    { idx: 1, x: -s * 0.22, y: 2 }, // Left
    { idx: 2, x:  s * 0.22, y: 2 }, // Right
  ];
  slotCoords.forEach(sc => {
    const val = sensors.slots[sc.idx];
    ctx.fillStyle = val === inIndexer.GREEN ? '#3fb950'
                  : val === inIndexer.PURPLE ? '#a371f7'
                  : 'rgba(13,17,23,0.75)';
    ctx.strokeStyle = '#30363d';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, 2.8, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });

  // Farul LED beacon at rear of chassis
  ctx.fillStyle = outTake.farul.hex;
  ctx.beginPath();
  ctx.arc(0, s / 2 - 3, 2.5, 0, Math.PI * 2);
  ctx.fill();

  // wheels (FL top-left, FR top-right, BL bottom-left, BR bottom-right in robot frame)
  const wheelR = 4;
  const wp = [
    { x: -s / 2 * 0.65, y: -s / 2 * 0.65, p: drive.FL }, // FL
    { x:  s / 2 * 0.65, y: -s / 2 * 0.65, p: drive.FR }, // FR
    { x: -s / 2 * 0.65, y:  s / 2 * 0.65, p: drive.BL }, // BL
    { x:  s / 2 * 0.65, y:  s / 2 * 0.65, p: drive.BR }, // BR
  ];
  const spin = performance.now() / 90;
  wp.forEach(w => {
    const p = w.p;
    const color = p > 0.05 ? '#3fb950' : p < -0.05 ? '#f85149' : '#8b949e';
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(w.x, w.y, wheelR, 0, Math.PI * 2); ctx.fill();
    if (Math.abs(p) > 0.01) {
      const a = spin * p;
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(w.x, w.y);
      ctx.lineTo(w.x + Math.cos(a) * wheelR, w.y + Math.sin(a) * wheelR);
      ctx.stroke();
    }
  });

  ctx.restore();
}

function drawJoystick() {
  const ctx = joyCtx;
  const size = 120, cx = size / 2, cy = size / 2, r = size / 2 - 10;
  ctx.clearRect(0, 0, size, size);

  ctx.fillStyle = '#0d1117';
  ctx.beginPath(); ctx.arc(cx, cy, r + 2, 0, Math.PI * 2); ctx.fill();

  // 0.89 power-ramp threshold ring
  ctx.strokeStyle = 'rgba(240, 180, 41, 0.22)';
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 3]);
  ctx.beginPath(); ctx.arc(cx, cy, r * 0.7 * 0.89, 0, Math.PI * 2); ctx.stroke();
  ctx.setLineDash([]);

  ctx.strokeStyle = '#21262d';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r);
  ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
  ctx.stroke();

  const kx = cx + joyX * r * 0.7;
  const ky = cy - joyY * r * 0.7;
  ctx.strokeStyle = 'rgba(88,166,255,0.5)';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(kx, ky); ctx.stroke();

  ctx.fillStyle = '#58a6ff';
  ctx.beginPath(); ctx.arc(kx, ky, 10, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = '#30363d';
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, Math.PI * 2); ctx.fill();

  if (Math.abs(joyR) > 0.01) {
    ctx.strokeStyle = joyR > 0 ? '#3fb950' : '#f85149';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, r - 2, -Math.PI / 2, -Math.PI / 2 + joyR * Math.PI, joyR < 0);
    ctx.stroke();
  }
}

// ─── Telemetry & Subsystem UI Updates ────────────────────────────────────────
function updateWheelCompositionBar(prefix, total, goToPow, k, stickPow) {
  const nameEl = document.getElementById(`wheel-${prefix}`);
  const gotoEl = document.getElementById(`wbar-${prefix}-goto`);
  const stickEl = document.getElementById(`wbar-${prefix}-stick`);
  const txtEl = document.getElementById(`wbar-${prefix}-txt`);

  if (nameEl) {
    nameEl.textContent = `${prefix.toUpperCase()}: ${total.toFixed(2)}`;
    nameEl.className = 'wbar-name' + (total > 0.05 ? ' pos' : total < -0.05 ? ' neg' : '');
  }
  const setSpan = (el, val) => {
    if (!el) return;
    const clamped = clamp(val, -1, 1);
    const pct = Math.abs(clamped) * 50;
    el.style.left = clamped >= 0 ? '50%' : `${50 - pct}%`;
    el.style.width = `${pct}%`;
  };
  setSpan(gotoEl, goToPow * k);
  setSpan(stickEl, stickPow);
  if (txtEl) {
    const sSign = stickPow >= 0 ? '+' : '−';
    txtEl.textContent = `${goToPow.toFixed(2)}·${k} ${sSign} ${Math.abs(stickPow).toFixed(2)}`;
  }
}

function updateTelemetry() {
  const endTarget = TARGET[TARGET.length - 1].target;
  const distToEnd = drive.getPoint().distanceTo(endTarget);
  const isParked = drive.splineDone && drive.doneGoTo && distToEnd < 6;
  const stageStr = !DXEnabled
    ? 'roarySpace (tremor)'
    : drive.splineDone
      ? (isParked ? '✓ PARKED' : '→ goToV2 approach')
      : `spline (seg ${drive.i})`;

  // Topbar chips
  document.getElementById('tel-hz').textContent = currentHz;
  document.getElementById('tel-runtime').textContent = runtime.toFixed(1);
  document.getElementById('chip-autostate').textContent = autoFsmEnabled ? `${AUTO} (c${cycleNR})` : 'MANUAL_SPLINE';

  const farulDot = document.getElementById('dot-farul');
  const farulTxt = document.getElementById('farul-indicator');
  if (farulDot) {
    farulDot.style.background = outTake.farul.hex;
    farulDot.style.boxShadow = `0 0 8px ${outTake.farul.hex}`;
  }
  if (farulTxt) farulTxt.textContent = `${outTake.farul.name} · ${outTake.farul.reason}`;

  // Power composition bars
  const badgeComp = document.getElementById('comp-state-badge');
  if (badgeComp) {
    badgeComp.textContent = stageStr;
    badgeComp.className = 'badge' + (isParked ? ' on' : '');
  }
  updateWheelCompositionBar('fl', drive.FL, drive.goTo.FLPow, drive.goToK, drive.mainDrivePowers.FLPow);
  updateWheelCompositionBar('fr', drive.FR, drive.goTo.FRPow, drive.goToK, drive.mainDrivePowers.FRPow);
  updateWheelCompositionBar('bl', drive.BL, drive.goTo.BLPow, drive.goToK, drive.mainDrivePowers.BLPow);
  updateWheelCompositionBar('br', drive.BR, drive.goTo.BRPow, drive.goToK, drive.mainDrivePowers.BRPow);

  // Tab 1: Subsystems & FSM
  const activeState = autoFsmEnabled ? AUTO : 'IDLE';
  document.querySelectorAll('#fsm-auto-nodes .fsm-node').forEach(n => {
    n.classList.toggle('active', n.dataset.st === activeState);
  });
  document.getElementById('fsm-cycle-badge').textContent = `cycleNR = ${cycleNR} · act = ${currentAction}`;
  const autoDetail = document.getElementById('fsm-auto-detail');
  if (autoDetail) {
    autoDetail.textContent = autoFsmEnabled
      ? `AUTO = ${AUTO} | DXEnabled = ${DXEnabled} | safety = ${Math.max(0, safetyTime - runtime).toFixed(1)}s | tremor = ${tremor > 0 ? '+1' : '-1'}`
      : `Single-path mode: splineToDX(TARGET, vel) → goToV2. Click "⚡ Auto FSM" to run full cycle.`;
  }

  // 3-Slot Indexer cards
  document.getElementById('sensor-poll-badge').textContent = `loopNr % 3 = ${sensors.lastPolledSlot}`;
  const paddles = [intake.MID, intake.LEFT, intake.RIGHT];
  for (let i = 0; i < 3; i++) {
    const val = sensors.slots[i];
    const card = document.getElementById(`slot-${i}`);
    const orb = document.getElementById(`slot-orb-${i}`);
    const valEl = document.getElementById(`slot-val-${i}`);
    const padEl = document.getElementById(`slot-pad-${i}`);
    if (card) card.classList.toggle('polled', sensors.lastPolledSlot === i);
    if (orb) orb.className = 'slot-orb ' + (val === inIndexer.GREEN || val === inIndexer.PURPLE ? val : '');
    if (valEl) valEl.textContent = val;
    if (padEl) padEl.textContent = `Paddle: ${paddles[i]}`;
  }

  // Intake status
  const ocActive = intake.ocTime > runtime;
  const ampsBadge = document.getElementById('intake-amps-badge');
  if (ampsBadge) {
    ampsBadge.textContent = `${intake.currentAmps.toFixed(1)} A` + (ocActive ? ' (OC REV!)' : '');
    ampsBadge.className = 'badge' + (ocActive ? ' danger' : intake.motorOutput < 0 ? ' on' : '');
  }
  document.getElementById('btn-intake-eat').classList.toggle('active', intake.intakePower < 0);
  document.getElementById('btn-intake-spit').classList.toggle('active', intake.intakePower > 0);
  document.getElementById('intake-status-bar').textContent =
    `Motor: ${intake.motorOutput.toFixed(2)} | hExt: ${intake.hExtCm.toFixed(1)}cm | ` +
    `spitOnFull: ${intake.hasSpit ? 'ACTIVE' : 'idle'} | Unstuck: ${intake.doneUnstuck ? 'DONE' : 'step ' + intake.stuckAction}`;

  // OutTake & UnloadManager steps
  const uStep = intake.unloadDone && intake.currentActionUnload > 6 ? 0 : intake.currentActionUnload;
  document.querySelectorAll('#fsm-unload-steps .fsm-node').forEach(n => {
    n.classList.toggle('active', +n.dataset.ustep === uStep);
  });
  const outBadge = document.getElementById('outtake-ready-badge');
  if (outBadge) {
    const rdy = outTake.velocityDone && outTake.turretDone;
    outBadge.textContent = rdy ? 'READY (GO)' : outTake.velocityEnabled ? 'EXTENDING…' : 'IDLE';
    outBadge.className = 'badge' + (rdy ? ' on' : outTake.velocityEnabled ? ' warn' : '');
  }
  document.getElementById('outtake-status-bar').textContent =
    `launchControl: ${intake.launchDone ? 'DONE' : 'step ' + intake.currentActionLaunch + ' (pad ' + intake.currentPaddle + ')'} | ` +
    `unloadManager: ${intake.unloadDone ? 'DONE' : 'step ' + intake.currentActionUnload} | ` +
    `Scored: ${outTake.scoredDrops.length}`;

  // Tab 2: Stick
  document.getElementById('joy-x').textContent = `X: ${joyX.toFixed(2)}`;
  document.getElementById('joy-y').textContent = `Y: ${joyY.toFixed(2)}`;
  document.getElementById('joy-r').textContent = `R: ${joyR.toFixed(2)}`;
  const pBadge = document.getElementById('power-ramp-badge');
  if (pBadge) {
    pBadge.textContent = `power = ${drive.power.toFixed(2)}`;
    pBadge.className = 'badge' + (drive.power > 1.02 ? ' warn' : '');
  }

  // Tab 3: Roary Internals Telemetry
  document.getElementById('tel-x').textContent = drive.getX().toFixed(2);
  document.getElementById('tel-y').textContent = drive.getY().toFixed(2);
  document.getElementById('tel-h').textContent = (drive.H * 180 / Math.PI).toFixed(1) + '°';
  document.getElementById('tel-vel').textContent = drive.vel.radius().toFixed(1) + ' cm/s';
  document.getElementById('tel-done').textContent = stageStr;
  document.getElementById('tel-spline-t').textContent =
    `${drive.i} / ${drive.localT.toFixed(2)} / ${drive.t.toFixed(2)}`;
  document.getElementById('tel-dist-end').textContent =
    `${drive.distanceToEnd.toFixed(1)} cm / ${drive.BRAKEPLS}`;
  document.getElementById('tel-norm-p').textContent =
    (drive.normalPower ? drive.normalPower.radius() : 0).toFixed(3);
  document.getElementById('tel-deriv-p').textContent =
    (drive.derivativePower ? drive.derivativePower.radius() : 0).toFixed(3);
  document.getElementById('tel-proj-p').textContent =
    (drive.projectionPower ? drive.projectionPower.radius() : 0).toFixed(3);
  document.getElementById('tel-mixer').textContent =
    `${drive.powerMixer.toFixed(3)} / ${drive.breakPower.toFixed(3)}`;
  document.getElementById('tel-rx').textContent = drive.rotX.toFixed(3);
  document.getElementById('tel-ry').textContent = drive.rotY.toFixed(3);
  document.getElementById('tel-hp').textContent = drive.headingPower.toFixed(3);
  document.getElementById('tel-dh').textContent = toDegrees(drive.deltaH).toFixed(1) + '°';
  document.getElementById('tel-vcomp').textContent =
    `${drive.powerCompensator.toFixed(3)} (${drive.batteryVoltage.toFixed(1)}V)`;

  const pose = drive.getPose();
  const clawP = drive.robotToClawPos(pose, intake.hExtCm);
  const outP = drive.robotToOutTakePos(pose, outTake.outExtCm);
  document.getElementById('tel-claw-pose').textContent = `${clawP.x.toFixed(1)}, ${clawP.y.toFixed(1)}`;
  document.getElementById('tel-out-pose').textContent = `${outP.x.toFixed(1)}, ${outP.y.toFixed(1)}`;
}

// ─── Robust loop ─────────────────────────────────────────────────────────────
const STEP = DT;                 // 1/60 s
const MAX_FRAME = 0.25;          // clamp elapsed time per wake (s)
const MAX_STEPS = 30;            // spiral-of-death guard
let accumulator = 0;
let lastTime = performance.now();
let lastWake = lastTime;

function frame() {
  const now = performance.now();
  let ft = (now - lastTime) / 1000;
  lastTime = now;
  if (ft > MAX_FRAME) ft = MAX_FRAME;
  accumulator += ft;

  let guard = 0;
  while (accumulator >= STEP && guard < MAX_STEPS) {
    control();
    accumulator -= STEP;
    guard++;
  }

  render();

  frameCount++;
  if (now - lastFpsTime >= 1000) { currentHz = frameCount; frameCount = 0; lastFpsTime = now; }

  lastWake = now;
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
setInterval(() => {
  if (performance.now() - lastWake > 300) frame(); // rAF is paused — keep Roary alive
}, 100);

// ─── Init ────────────────────────────────────────────────────────────────────
drive.setPosition(0, 0, 0);
drive.vel = new Point(0, 0);
drive.resetSpline();
refreshAutoButton();
rebuildWaypointList();
fieldCanvas.style.cursor = 'crosshair';
