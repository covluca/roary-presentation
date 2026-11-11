// RoaryX7 Web Demo — faithful emulation of the autonomous runOpMode() loop.
//
//   mainDrive()                                  (always — stick → mainDrivePowers)
//   if (DXEnabled)  splineToDX(TARGET, vel)      (→ sendPower → motors = goTo·goToK + mainDrivePowers)
//   if (splineDone) goToV2(lastTarget, …)        (final approach → same composition)
//   updatePowerCompensator()
//
// "Roary never starts or stops anything — it just happens." The loop runs every
// frame; the stick always drives (mainDrivePowers); "Start Auto" only toggles
// goToK so the spline's goTo power is added in or not.
//
// Physics: the 4 motor powers are decomposed into (forward, strafe, rotate) —
// the exact inverse of the drive's IK — and integrated with the odometry's own
// world transform (Roaryx7.java:update), so this code moves the robot the same
// way it does on the real robot.

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

// ─── Real auto path (autonom/Kirby/KirbyAirRiders.java) ────────────────────
// TARGET = { launch_first, launch_first, artifact_3_sliding, artifact_3_facing, artifact_3, artifact_3 }
const TARGET = [
  new goToValues(new Pose2d(-35, -124, -70), 0.71, 3),   // launch_first
  new goToValues(new Pose2d(-35, -124, -70), 0.71, 3),   // launch_first
  new goToValues(new Pose2d(-50, -110, -66), 0.71, 3),   // artifact_3_sliding
  new goToValues(new Pose2d(-120, -90, -90), 0.64, 3),   // artifact_3_facing
  new goToValues(new Pose2d(-152, -94, -93), 0.53, 3),   // artifact_3
  new goToValues(new Pose2d(-152, -94, -93), 0.53, 3),   // artifact_3
];

// ─── State ───────────────────────────────────────────────────────────────────
const drive = new Roaryx7Drive('Kirby');
let DXEnabled = true;              // autonomous path is always live (like the real opmode)

// Stick (left stick x/y in [-1,1], up = +y ; rotate in [-1,1])
let joyX = 0, joyY = 0, joyR = 0, joyActive = false;

// Keyboard
const keys = Object.create(null);
window.addEventListener('keydown', (e) => {
  // Don't hijack keys while the user is typing in the waypoint list
  const t = e.target;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT')) return;
  const k = e.key.toLowerCase();
  keys[k] = true;
  if (k === 'b') toggleAuto();          // gamepad1.bWasPressed() → toggle goToK
  if ((k === 'delete' || k === 'backspace') && selectedWaypoint >= 0) deleteWaypoint(selectedWaypoint);
  if (k === 'escape') { selectedWaypoint = -1; refreshWpSelection(); }
  if (['w', 'a', 's', 'd', 'q', 'e', 'b', ' ', 'delete', 'backspace'].includes(k)) e.preventDefault();
});
window.addEventListener('keyup', (e) => { keys[e.key.toLowerCase()] = false; });

// FPS
let frameCount = 0, lastFpsTime = performance.now(), currentHz = 0;

// ─── DOM ─────────────────────────────────────────────────────────────────────
const fieldCanvas = document.getElementById('field');
const fieldCtx = fieldCanvas.getContext('2d');
const joyCanvas = document.getElementById('joystick');
const joyCtx = joyCanvas.getContext('2d');

const btnStartAuto = document.getElementById('btn-start-auto');
const btnReset = document.getElementById('btn-reset');
const goToKIndicator = document.getElementById('goToK-indicator');

// ─── Auto toggle (goToK) ─────────────────────────────────────────────────────
function refreshAutoButton() {
  const on = drive.goToK === 1;
  btnStartAuto.textContent = on ? '■ Stop Auto' : '▶ Start Auto';
  btnStartAuto.classList.toggle('active', on);
  goToKIndicator.textContent = on
    ? 'goToK = 1  ·  spline power engaged (goTo·1 + stick)'
    : 'goToK = 0  ·  stick only (goTo·0 + stick)';
  goToKIndicator.classList.toggle('on', on);
}

// "Start Auto" only toggles goToK — the follower is always tracking the robot's
// position (that's the whole point of Roary). Reset the path via the Reset button.
function toggleAuto() {
  drive.toggleGoToK();
  refreshAutoButton();
}
btnStartAuto.onclick = toggleAuto;

btnReset.onclick = () => {
  drive.setPosition(0, 0, 0);
  drive.vel = new Point(0, 0);
  drive.resetSpline();
  drive.mainDrivePowers = { FLPow: 0, FRPow: 0, BLPow: 0, BRPow: 0 };
  drive.FL = drive.FR = drive.BL = drive.BR = 0;
  drive.power = 1;
  refreshAutoButton();
};

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
  joyCanvas.setPointerCapture(e.pointerId);
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
  if (keys['w'] || keys['arrowup']) ky += 0.35;
  if (keys['s'] || keys['arrowdown']) ky -= 0.35;
  if (keys['a'] || keys['arrowleft']) kx -= 0.35;
  if (keys['d'] || keys['arrowright']) kx += 0.35;
  if (keys['q']) kr -= 0.4;
  if (keys['e']) kr += 0.4;

  joyX += (kx - joyX) * 0.35;
  joyY += (ky - joyY) * 0.35;
  joyR += (kr - joyR) * 0.35;

  if (Math.abs(joyX) < 0.01) joyX = 0;
  if (Math.abs(joyY) < 0.01) joyY = 0;
  if (Math.abs(joyR) < 0.01) joyR = 0;
}

// ─── Waypoint editing (TARGET — live, Roary keeps running) ───────────────────
// splineToDX() re-reads TARGET every tick, so moving/editing a point changes
// the path immediately — the follower just keeps tracking (that's Roary).
// Adding/deleting changes the point count, so we re-seat the follower with
// resetSpline() (which also refreshes drive.length on the next tick).
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

  // Where to insert: after the closest segment — unless the click is clearly
  // before the first point (prepend) or past the last point (append).
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

  // Defaults from the neighbours: heading = angular mean (heading θ ↔ dir (sin θ, cos θ)),
  // speed = mean, error = previous point's.
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
  if (el && el === document.activeElement && out !== v) el.value = out; // reflect clamp
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

  drive.X += worldX * MAX_SPEED * dt;
  drive.Y += worldY * MAX_SPEED * dt;
  drive.H = angleWrap(drive.H + rotate * MAX_ROT * dt);
  drive.vel = new Point(worldX * MAX_SPEED, worldY * MAX_SPEED);
}

// ─── runOpMode() loop ────────────────────────────────────────────────────────
// One fixed-rate simulation step: the full runOpMode() body.
function control() {
  updateJoyFromKeyboard();

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

  // 3) updatePowerCompensator()
  drive.updatePowerCompensator();

  // 4) integrate the motors → motion
  stepPhysics(DT);
}

// Drawing + telemetry (runs once per animation frame).
function render() {
  drawField();
  drawWaypoints();
  drawRobot();
  drawJoystick();
  updateTelemetry();
}

// ─── Drawing ─────────────────────────────────────────────────────────────────
function worldToCanvas(x, y) {
  return { x: CENTER + x * SCALE, y: CENTER - y * SCALE };
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

  // center line + circle
  ctx.strokeStyle = '#444c56';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(CENTER, CENTER - half); ctx.lineTo(CENTER, CENTER + half); ctx.stroke();
  ctx.beginPath(); ctx.arc(CENTER, CENTER, 30, 0, Math.PI * 2); ctx.stroke();
}

function drawWaypoints() {
  const ctx = fieldCtx;

  // spline path (the segment the follower is currently on, plus the whole route faintly)
  if (TARGET.length > 3) {
    ctx.strokeStyle = 'rgba(88,166,255,0.35)';
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
  }

  // current spline target + lookahead
  if (drive.target && drive.goToK === 1) {
    const c = worldToCanvas(drive.target.x, drive.target.y);
    ctx.fillStyle = '#f0b429';
    ctx.beginPath(); ctx.arc(c.x, c.y, 5, 0, Math.PI * 2); ctx.fill();
    if (drive.lookAhead) {
      const la = worldToCanvas(drive.lookAhead.x, drive.lookAhead.y);
      ctx.strokeStyle = 'rgba(240,180,41,0.6)';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(la.x, la.y); ctx.stroke();
    }
  }

  // waypoint dots
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
    ctx.fillStyle = isLast ? '#3fb950' : '#58a6ff';
    ctx.beginPath(); ctx.arc(c.x, c.y, 4, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#8b949e';
    ctx.font = '10px monospace';
    ctx.fillText(isLast ? 'END' : `WP${idx}`, c.x + 6, c.y - 4);
  });
}

function drawRobot() {
  const ctx = fieldCtx;
  const pos = drive.getPoint();
  const c = worldToCanvas(pos.x, pos.y);
  const H = drive.H;

  // Robot forward axis in world = (sin H, cos H) (from stepPhysics: worldY = cos(H)·forward).
  // The canvas flips Y, so that direction renders as (sin H, -cos H) — which is
  // exactly the local "up" edge rotated by H. (Deriving the angle from
  // worldToCanvas deltas mixes the robot's field position into it — don't.)
  ctx.save();
  ctx.translate(c.x, c.y);
  ctx.rotate(H); // forward edge points where +forward motion goes

  const s = ROBOT_SIZE * SCALE;
  ctx.fillStyle = 'rgba(88,166,255,0.18)';
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 2;
  ctx.fillRect(-s / 2, -s / 2, s, s);
  ctx.strokeRect(-s / 2, -s / 2, s, s);

  // forward arrow (top edge)
  ctx.fillStyle = '#3fb950';
  ctx.beginPath();
  ctx.moveTo(0, -s / 2 - 7);
  ctx.lineTo(-5, -s / 2);
  ctx.lineTo(5, -s / 2);
  ctx.closePath();
  ctx.fill();

  // wheels (FL top-left, FR top-right, BL bottom-left, BR bottom-right in robot frame)
  const wheelR = 4;
  const wp = [
    { x: -s / 2 * 0.65, y: -s / 2 * 0.65, p: drive.FL }, // FL
    { x: s / 2 * 0.65, y: -s / 2 * 0.65, p: drive.FR },  // FR
    { x: -s / 2 * 0.65, y: s / 2 * 0.65, p: drive.BL },  // BL
    { x: s / 2 * 0.65, y: s / 2 * 0.65, p: drive.BR },   // BR
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

// ─── Telemetry ───────────────────────────────────────────────────────────────
function updateTelemetry() {
  document.getElementById('tel-x').textContent = drive.getX().toFixed(2);
  document.getElementById('tel-y').textContent = drive.getY().toFixed(2);
  document.getElementById('tel-h').textContent = (drive.H * 180 / Math.PI).toFixed(1) + '°';
  document.getElementById('tel-rx').textContent = drive.rotX.toFixed(3);
  document.getElementById('tel-ry').textContent = drive.rotY.toFixed(3);
  document.getElementById('tel-hp').textContent = drive.headingPower.toFixed(3);
  document.getElementById('tel-vel').textContent = drive.vel.radius().toFixed(1) + ' cm/s';
  document.getElementById('tel-done').textContent =
    drive.splineDone ? (drive.doneGoTo ? '✓ PARKED' : '→ final approach') : 'spline';
  document.getElementById('tel-hz').textContent = currentHz;

  document.getElementById('joy-x').textContent = `X: ${joyX.toFixed(2)}`;
  document.getElementById('joy-y').textContent = `Y: ${joyY.toFixed(2)}`;
  document.getElementById('joy-r').textContent = `R: ${joyR.toFixed(2)}`;

  const wheels = [
    { el: 'wheel-fl', label: 'FL', val: drive.FL },
    { el: 'wheel-fr', label: 'FR', val: drive.FR },
    { el: 'wheel-bl', label: 'BL', val: drive.BL },
    { el: 'wheel-br', label: 'BR', val: drive.BR },
  ];
  wheels.forEach(w => {
    const el = document.getElementById(w.el);
    el.textContent = `${w.label}: ${w.val.toFixed(2)}`;
    el.className = 'wheel' + (w.val > 0.05 ? ' pos' : w.val < -0.05 ? ' neg' : '');
  });
}

// ─── Robust loop ─────────────────────────────────────────────────────────────
// Roary never stops. We use a fixed-timestep accumulator:
//   • control() runs at a fixed 60 Hz (the real opmode rate), so the drive
//     logic is deterministic and matches the Java loop.
//   • physics advances by real elapsed time, so the robot moves at correct
//     real-time speed even when the tab is throttled (background/unfocused).
//   • rAF drives it when live (smooth); a setInterval heartbeat keeps it going
//     when Chromium pauses rAF in an unfocused/occluded tab.
//   • MAX_FRAME clamps catch-up so a long-hidden tab resumes without a spike.
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
