// RoaryX7 — Smart Courier & Service Robot Interactive Simulator.
//
// Features:
// 1. Modern Smart Office / Library Facility (Shelves, Executive Desk, Client Person, Lounge Table, Charging Dock).
// 2. Clear, sponsor-intuitive mission: Fetch 3 books from shelf, transport them smoothly, deliver to client desk.
// 3. Realistic Mecanum Drive Physics with mass & inertia powered by RoaryX7 (Catmull-Rom spline + Live Pilot Fusion).
// 4. High-polish visual rendering: 4 mecanum wheels with 45° rollers, top cargo basket holding 3 visible books,
//    articulated front extension arm with gripper jaws, status LED halo, and interactive speech bubbles.

'use strict';

const clamp = (v, min, max) => Math.max(min, Math.min(max, v));

// ─── Environment Dimensions & Scaling ─────────────────────────────────────────
const ROOM_WIDTH = 340;   // cm (-170..+170)
const ROOM_HEIGHT = 260;  // cm (-130..+130)
const CANVAS_WIDTH = 760;
const CANVAS_HEIGHT = 580;
const SCALE = 2.15;       // px per cm (fills canvas gorgeously!)
const CENTER_X = CANVAS_WIDTH / 2;
const CENTER_Y = CANVAS_HEIGHT / 2;
const DT = 1 / 60;        // 60 Hz simulation timestep
const MAX_SPEED = 78;     // cm/s max speed
const MAX_ROT = 2.8;      // rad/s max rotation speed

// ─── Waypoints & Poses for the Courier Mission ────────────────────────────────
// In Roary coordinates: 0° = +Y (UP), 90° = +X (RIGHT), -90° = -X (LEFT), 180° = -Y (DOWN).
// Shelf is on the west wall (-X) -> robot must face -90° so its arm extends into the shelf.
// Desk is on the east wall (+X) -> robot must face +90° so its arm extends into the desk tray.
const POSES = {
  dock:  () => new Pose2d(-105, -85, -90),
  shelf: () => new Pose2d(-95, 15, -90),
  desk:  () => new Pose2d(55, 0, 90),
};

// Multi-batch books catalog for continuous logistics demonstration
const BATCH_CATALOG = [
  [
    { id: 1, title: 'AI Navigation', color: '#0284c7', spineClass: 'loaded-blue' },
    { id: 2, title: 'Holonomic Robot', color: '#10b981', spineClass: 'loaded-green' },
    { id: 3, title: 'Sponsor Pitch', color: '#f97316', spineClass: 'loaded-orange' },
  ],
  [
    { id: 4, title: 'Cloud Logistics', color: '#a855f7', spineClass: 'loaded-purple' },
    { id: 5, title: 'Autonomous Fleet', color: '#06b6d4', spineClass: 'loaded-blue' },
    { id: 6, title: 'Vision Precision', color: '#10b981', spineClass: 'loaded-green' },
  ],
  [
    { id: 7, title: 'Sensor Fusion', color: '#6366f1', spineClass: 'loaded-blue' },
    { id: 8, title: 'Smart Kinematics', color: '#ec4899', spineClass: 'loaded-orange' },
    { id: 9, title: 'Industrial ROI', color: '#eab308', spineClass: 'loaded-orange' },
  ],
];

let batchIndex = 0;
function getBatch(idx) {
  const b = BATCH_CATALOG[idx % BATCH_CATALOG.length];
  const offset = Math.floor(idx / BATCH_CATALOG.length) * 9;
  return b.map(item => ({ ...item, id: item.id + offset }));
}

// ─── Drive & State Instances ──────────────────────────────────────────────────
const drive = new Roaryx7Drive('Kirby');
drive.setPosition(POSES.dock().x, POSES.dock().y, toRadians(POSES.dock().heading));
drive.setGoToK(true);

// Mission State Machine:
// 'STANDBY' | 'NAV_TO_SHELF' | 'PICKUP_BOOKS' | 'NAV_TO_DESK' | 'DELIVER_TO_DESK' | 'WAITING_NEXT_BATCH' | 'COMPLETE'
let missionState = 'STANDBY';
let missionActive = false;
let missionStep = 1;
let totalDelivered = 0;
let basketCargo = [];           // Array of books currently in robot basket (max 3)
let deskDeliveredBooks = [];    // Books delivered on the client's desk
let shelfBooks = getBatch(0);   // Books currently waiting on the shelf

// Extension arm & delivery animations
let armExtension = 0;           // 0..1 animated slide extension
let armGrabbing = false;
let deliveringBook = null;      // Book currently held in jaws during delivery
let pickupPhase = 0;            // 0: extend, 1: retract with book, 2: ready for transit
let pickupTimer = 0;
let deliverPhase = 0;           // 0: extend with book, 1: retract empty, 2: all delivered
let deliveryTimer = 0;
let nextBatchTimer = 0;
let clientBubble = 'Aștept 3 cărți pentru cercetare...';
let clientBubbleTimer = 0;

// Trajectory spline targets
let CURRENT_SPLINE = [];
let splinePhase = 'NONE';       // 'TO_SHELF' | 'TO_DESK' | 'TO_DOCK' | 'CUSTOM'

// Physics & Inertia state
let simVelX = 0;
let simVelY = 0;
let simOmega = 0;
const trail = [];
const MAX_TRAIL = 180;

// Shared Autonomy / Pilot state
let joyX = 0, joyY = 0, joyR = 0, joyActive = false;
let demoNudgeTimer = 0;
let demoNudgeX = 0, demoNudgeY = 0;
let lastPilotTime = 0;

// Telemetry & Timing
let runtime = 0;
let frameCount = 0, lastFpsTime = performance.now(), currentFps = 60;

// ─── DOM References ──────────────────────────────────────────────────────────
const canvas = document.getElementById('field');
const ctx = canvas.getContext('2d');
const joyCanvas = document.getElementById('joystick');
const joyCtx = joyCanvas.getContext('2d');

const btnToggleMission = document.getElementById('btn-toggle-mission');
const btnMissionIcon = document.getElementById('btn-mission-icon');
const btnMissionLabel = document.getElementById('btn-mission-label');
const btnFetchBooks = document.getElementById('btn-fetch-books');
const btnDeliverDesk = document.getElementById('btn-deliver-desk');
const btnDemoNudge = document.getElementById('btn-demo-nudge');
const btnReset = document.getElementById('btn-reset');

const missionStatusText = document.getElementById('mission-status-text');
const chipDeliveredCount = document.getElementById('chip-delivered-count');
const chipCargoCount = document.getElementById('chip-cargo-count');
const chipPilotText = document.getElementById('chip-pilot-text');
const dotPilot = document.getElementById('dot-pilot');
const liveBanner = document.getElementById('canvas-live-banner');
const missionStepBadge = document.getElementById('mission-step-badge');
const cargoStatusLabel = document.getElementById('cargo-status-label');

const ovPath = document.getElementById('ov-path');
const ovTrail = document.getElementById('ov-trail');
const ovRoom = document.getElementById('ov-room');
const ovLoop = document.getElementById('ov-loop');

// ─── Coordinate Transformations ──────────────────────────────────────────────
function worldToCanvas(x, y) {
  return {
    x: CENTER_X + x * SCALE,
    y: CENTER_Y - y * SCALE,
  };
}

function canvasToWorld(cx, cy) {
  return {
    x: (cx - CENTER_X) / SCALE,
    y: (CENTER_Y - cy) / SCALE,
  };
}

// ─── Mission Helpers ─────────────────────────────────────────────────────────
function setMissionStatus(text, stepNr, bannerText) {
  missionStatusText.textContent = text;
  missionStep = stepNr;
  if (missionStepBadge) missionStepBadge.textContent = `Etapa ${stepNr} / 4`;
  if (bannerText && liveBanner) {
    liveBanner.textContent = bannerText;
    liveBanner.classList.remove('active-nudge');
  }

  // Update step stepper cards
  for (let k = 1; k <= 4; k++) {
    const el = document.getElementById(`step-node-${k}`);
    if (!el) continue;
    el.classList.remove('active', 'completed');
    if (k < stepNr) el.classList.add('completed');
    else if (k === stepNr) el.classList.add('active');
  }
}

function updateCargoUI() {
  chipCargoCount.textContent = `${basketCargo.length} / 3 Cărți`;
  cargoStatusLabel.textContent = basketCargo.length === 3 ? 'Coș Plin (3/3) — Gata de Livrare' :
                                basketCargo.length === 0 ? 'Coș Gol (0/3)' : `Încărcat (${basketCargo.length}/3)`;

  for (let i = 0; i < 3; i++) {
    const slotEl = document.getElementById(`book-slot-${i}`);
    if (!slotEl) continue;
    const book = basketCargo[i];
    if (book) {
      slotEl.className = `book-slot ${book.spineClass}`;
      slotEl.innerHTML = `
        <div class="book-spine">
          <span class="book-num">#${book.id}</span>
          <span class="book-name">${book.title}</span>
        </div>
      `;
    } else {
      slotEl.className = 'book-slot';
      slotEl.innerHTML = `
        <div class="book-spine empty">
          <span class="book-num">#${i + 1}</span>
          <span class="book-name">Slot Gol</span>
        </div>
      `;
    }
  }
}

function buildShelfPath() {
  const cur = drive.getPose();
  const shelf = POSES.shelf();
  splinePhase = 'TO_SHELF';

  // Smooth Catmull-Rom spline:
  // If starting from dock or south of room: smooth path up along the west side
  // If starting from desk: smooth sweeping arc north of the center lounge table (-10, -15)
  let waypoints;
  if (cur.y < -30) {
    waypoints = [
      new goToValues(new Pose2d(cur.x, cur.y, cur.heading), 0.72, 3),
      new goToValues(new Pose2d(cur.x, cur.y, cur.heading), 0.72, 3),
      new goToValues(new Pose2d(-85, -35, -70), 0.75, 4),
      new goToValues(new Pose2d(-80, 0, -85), 0.72, 3.5),
      new goToValues(shelf, 0.62, 3),
      new goToValues(shelf, 0.62, 3),
    ];
  } else {
    // Coming from desk or center: route smoothly north of table (-10, -15)
    // Between table (y = -15) and armchair (y = 70), y = 30 gives >40cm clearance from both
    waypoints = [
      new goToValues(new Pose2d(cur.x, cur.y, cur.heading), 0.72, 3),
      new goToValues(new Pose2d(cur.x, cur.y, cur.heading), 0.72, 3),
      new goToValues(new Pose2d(15, 30, 45), 0.76, 4),
      new goToValues(new Pose2d(-45, 32, -45), 0.74, 3.5),
      new goToValues(shelf, 0.62, 3),
      new goToValues(shelf, 0.62, 3),
    ];
  }

  CURRENT_SPLINE = waypoints;
  drive.resetSpline();
}

function buildDeskPath() {
  const cur = drive.getPose();
  const desk = POSES.desk();
  splinePhase = 'TO_DESK';

  // Spline from shelf, sweeping wide south of the lounge table (-10, -15)
  // Table center is y = -15 (radius ~11cm). Sweeping through y = -45 gives 30cm clearance!
  const waypoints = [
    new goToValues(new Pose2d(cur.x, cur.y, cur.heading), 0.72, 3),
    new goToValues(new Pose2d(cur.x, cur.y, cur.heading), 0.72, 3),
    new goToValues(new Pose2d(-50, -45, -30), 0.76, 4),
    new goToValues(new Pose2d(15, -45, 45), 0.78, 4),
    new goToValues(desk, 0.62, 3),
    new goToValues(desk, 0.62, 3),
  ];

  CURRENT_SPLINE = waypoints;
  drive.resetSpline();
}

function startMission() {
  missionActive = true;
  btnToggleMission.classList.remove('paused');
  btnMissionIcon.textContent = '⏸';
  btnMissionLabel.textContent = 'Pauză Misiune';

  if (basketCargo.length === 3) {
    missionState = 'NAV_TO_DESK';
    buildDeskPath();
    setMissionStatus('Deplasare către Birou', 3, 'Coșul este plin! Roary navighează către biroul clientului...');
  } else {
    missionState = 'NAV_TO_SHELF';
    buildShelfPath();
    setMissionStatus('Deplasare către Raft', 1, 'Robotul se deplasează lin către raftul de cărți...');
  }
}

function pauseMission() {
  missionActive = false;
  btnToggleMission.classList.add('paused');
  btnMissionIcon.textContent = '▶';
  btnMissionLabel.textContent = 'Reia Misiune';
  setMissionStatus('Misiune în Pauză', missionStep, 'Misiune pusă în pauză. Poți prelua oricând controlul cu WASD.');
}

function toggleMission() {
  if (missionActive) pauseMission();
  else startMission();
}

function resetAll() {
  missionActive = false;
  missionState = 'STANDBY';
  basketCargo = [];
  deskDeliveredBooks = [];
  batchIndex = 0;
  shelfBooks = getBatch(0);
  armExtension = 0;
  armGrabbing = false;
  deliveringBook = null;
  pickupPhase = 0;
  deliverPhase = 0;
  totalDelivered = 0;
  chipDeliveredCount.textContent = '0 Cărți';
  updateCargoUI();

  const dock = POSES.dock();
  drive.setPosition(dock.x, dock.y, toRadians(dock.heading));
  drive.stopMotors();
  drive.resetSpline();
  trail.length = 0;
  CURRENT_SPLINE = [];
  splinePhase = 'NONE';

  btnToggleMission.classList.remove('paused');
  btnMissionIcon.textContent = '▶';
  btnMissionLabel.textContent = 'Pornește Misiune Curierat';
  setMissionStatus('Gata de Livrare', 1, 'Robot andocat la stația de încărcare. Apasă „Pornește Misiune”.');
}

// Direct button actions
btnToggleMission.addEventListener('click', toggleMission);
btnFetchBooks.addEventListener('click', () => {
  missionActive = true;
  missionState = 'NAV_TO_SHELF';
  buildShelfPath();
  setMissionStatus('Deplasare către Raft', 1, 'Comandă directă: robotul se deplasează la raftul de cărți...');
});
btnDeliverDesk.addEventListener('click', () => {
  missionActive = true;
  missionState = 'NAV_TO_DESK';
  buildDeskPath();
  setMissionStatus('Deplasare către Birou', 3, 'Comandă directă: robotul transportă marfa către birou...');
});
btnDemoNudge.addEventListener('click', () => {
  demoNudgeTimer = runtime + 1.6;
  demoNudgeX = 0.85;
  demoNudgeY = 0.35;
  if (liveBanner) {
    liveBanner.textContent = '👋 Simulare Pilot Nudge: Operatorul intervine pentru ocolire!';
    liveBanner.classList.add('active-nudge');
  }
});
btnReset.addEventListener('click', resetAll);

// ─── Stick & Keyboard Input Handling ─────────────────────────────────────────
const keys = Object.create(null);
window.addEventListener('keydown', (e) => {
  const t = e.target;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA')) return;
  const k = e.key.toLowerCase();
  keys[k] = true;
  if (k === ' ') {
    toggleMission();
    e.preventDefault();
  }
  if (k === 'r') {
    resetAll();
    e.preventDefault();
  }
  if (['w', 'a', 's', 'd', 'q', 'e', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) {
    e.preventDefault();
  }
});
window.addEventListener('keyup', (e) => { keys[e.key.toLowerCase()] = false; });

function updateJoyFromKeyboard() {
  let kx = 0, ky = 0, kr = 0;
  if (keys['w'] || keys['arrowup']) ky += 1;
  if (keys['s'] || keys['arrowdown']) ky -= 1;
  if (keys['a'] || keys['arrowleft']) kx -= 1;
  if (keys['d'] || keys['arrowright']) kx += 1;
  if (keys['q']) kr += 1;
  if (keys['e']) kr -= 1;

  if (demoNudgeTimer > runtime) {
    kx += demoNudgeX;
    ky += demoNudgeY;
  }

  // Combine virtual joystick + keyboard
  if (Math.hypot(kx, ky) > 1) {
    const l = Math.hypot(kx, ky);
    kx /= l; ky /= l;
  }

  if (joyActive) {
    // Virtual joystick takes precedence when dragged
  } else {
    joyX = kx;
    joyY = ky;
    joyR = kr;
  }

  const isPiloted = Math.hypot(joyX, joyY) > 0.05 || Math.abs(joyR) > 0.05;
  if (isPiloted) {
    lastPilotTime = runtime;
    chipPilotText.textContent = 'Fuziune Pilot + Auto';
    dotPilot.className = 'dot on purple';
    if (liveBanner && demoNudgeTimer <= runtime) {
      liveBanner.textContent = '🎮 Intervenție Operator Activă · Roary combină comenzile live!';
      liveBanner.classList.add('active-nudge');
    }
  } else if (runtime - lastPilotTime > 0.6) {
    chipPilotText.textContent = 'Autonom Roary';
    dotPilot.className = 'dot on';
    if (liveBanner && liveBanner.classList.contains('active-nudge') && demoNudgeTimer <= runtime) {
      liveBanner.classList.remove('active-nudge');
      liveBanner.textContent = 'Traseu autonom activ';
    }
  }

  document.getElementById('joy-readout').textContent =
    `X: ${joyX.toFixed(2)} · Y: ${joyY.toFixed(2)} · R: ${joyR.toFixed(2)}`;
}

// Virtual Joystick Canvas Mouse / Touch events
let joyTouchId = null;
function handleJoyPointer(clientX, clientY) {
  const rect = joyCanvas.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const maxR = rect.width / 2 - 8;
  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.hypot(dx, dy);
  if (dist > maxR) {
    dx = (dx / dist) * maxR;
    dy = (dy / dist) * maxR;
  }
  joyX = dx / maxR;
  joyY = -dy / maxR; // invert Y so up = forward
}

joyCanvas.addEventListener('pointerdown', (e) => {
  joyActive = true;
  joyCanvas.setPointerCapture(e.pointerId);
  handleJoyPointer(e.clientX, e.clientY);
});
joyCanvas.addEventListener('pointermove', (e) => {
  if (joyActive) handleJoyPointer(e.clientX, e.clientY);
});
const endJoy = () => { joyActive = false; joyX = 0; joyY = 0; joyR = 0; };
joyCanvas.addEventListener('pointerup', endJoy);
joyCanvas.addEventListener('pointercancel', endJoy);

// Click on field to dispatch custom destination
canvas.addEventListener('click', (e) => {
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const world = canvasToWorld(cx, cy);

  const cur = drive.getPose();
  // Roary heading convention: 0° = +Y, 90° = +X, so atan2(dx, dy)
  const targetHeading = toDegrees(Math.atan2(world.x - cur.x, world.y - cur.y));
  const targetPose = new Pose2d(world.x, world.y, targetHeading);

  CURRENT_SPLINE = [
    new goToValues(cur, 0.7, 3),
    new goToValues(cur, 0.7, 3),
    new goToValues(targetPose, 0.6, 3),
    new goToValues(targetPose, 0.6, 3),
  ];
  splinePhase = 'CUSTOM';
  drive.resetSpline();
  missionActive = true;
  setMissionStatus('Navigație către Destinație', 1, `Robotul navighează către (${world.x.toFixed(0)}, ${world.y.toFixed(0)})...`);
});

// ─── Control Step (60 Hz) ─────────────────────────────────────────────────────
function control() {
  runtime += DT;
  updateJoyFromKeyboard();

  // 1) Stick / Joystick Input fed directly into Roary
  drive.mainDrive(joyX, joyY, joyR);

  // 2) Autonomous Navigation via Roary Spline / goToV2
  if (CURRENT_SPLINE && CURRENT_SPLINE.length >= 4) {
    const endTarget = CURRENT_SPLINE[CURRENT_SPLINE.length - 1].target;

    if (!drive.splineDone) {
      drive.splineToDX(CURRENT_SPLINE, drive.vel);
    } else {
      // Final precision docking approach
      drive.goToV2(endTarget, 0.38, 3.0);
    }
  } else {
    // When no autonomous trajectory is active, ensure autonomous powers are zeroed
    drive.goTo.FLPow = 0;
    drive.goTo.FRPow = 0;
    drive.goTo.BLPow = 0;
    drive.goTo.BRPow = 0;
  }

  // 3) Mission State Machine Logic
  stepMissionStateMachine();

  // 4) Battery & Power Compensation
  drive.updatePowerCompensator();

  // 5) Integrate Physics with realistic AGV mass & inertia
  stepPhysics(DT);
}

// ─── Mission State Machine Execution ──────────────────────────────────────────
function stepMissionStateMachine() {
  if (!missionActive) return;

  const curPoint = drive.getPoint();

  switch (missionState) {
    case 'NAV_TO_SHELF': {
      const shelfPose = POSES.shelf();
      const distToShelf = curPoint.distanceTo(new Point(shelfPose.x, shelfPose.y));
      const degDiff = Math.abs(toDegrees(angleWrap(toRadians(shelfPose.heading) - drive.H)));

      if (distToShelf < 6.0 && degDiff < 6.0) {
        // Arrived at the bookshelf!
        missionState = 'PICKUP_BOOKS';
        pickupPhase = 0;
        pickupTimer = runtime + 0.3;
        setMissionStatus('Preluare Cărți din Raft', 2, 'Andocat la raft. Brațul telescopic preia cărțile pe rând...');
        CURRENT_SPLINE = [];
        drive.stopMotors();
      }
      break;
    }

    case 'PICKUP_BOOKS': {
      // Sequential 3-book pickup animation
      if (runtime > pickupTimer) {
        if (pickupPhase === 0) {
          // Extend arm
          armExtension = Math.min(1.0, armExtension + 2.4 * DT);
          if (armExtension >= 0.98) {
            armGrabbing = true;
            pickupTimer = runtime + 0.3;
            pickupPhase = 1;
          }
        } else if (pickupPhase === 1) {
          // Retract arm with book into basket
          armExtension = Math.max(0.0, armExtension - 2.4 * DT);
          if (armExtension <= 0.05) {
            armGrabbing = false;
            // Add book to basket
            if (basketCargo.length < 3 && shelfBooks.length > 0) {
              const book = shelfBooks.shift();
              basketCargo.push(book);
              updateCargoUI();
            }
            if (basketCargo.length < 3 && shelfBooks.length > 0) {
              pickupPhase = 0; // Grab next book!
              pickupTimer = runtime + 0.2;
            } else {
              // All 3 books loaded!
              pickupPhase = 2;
              pickupTimer = runtime + 0.5;
              setMissionStatus('Coș Plin (3/3)', 2, 'Toate cele 3 cărți au fost încărcate în coș! Pornim spre birou...');
            }
          }
        } else if (pickupPhase === 2) {
          // Ready to travel to desk!
          missionState = 'NAV_TO_DESK';
          buildDeskPath();
          setMissionStatus('Deplasare către Birou', 3, 'Robotul transportă cele 3 cărți către biroul clientului...');
        }
      }
      break;
    }

    case 'NAV_TO_DESK': {
      const deskPose = POSES.desk();
      const distToDesk = curPoint.distanceTo(new Point(deskPose.x, deskPose.y));
      const degDiff = Math.abs(toDegrees(angleWrap(toRadians(deskPose.heading) - drive.H)));

      if (distToDesk < 6.0 && degDiff < 6.0) {
        // Arrived at the desk!
        missionState = 'DELIVER_TO_DESK';
        deliverPhase = 0;
        deliveryTimer = runtime + 0.3;
        setMissionStatus('Predare Cărți la Birou', 4, 'Andocat la birou. Brațul transferă cărțile în tăvița clientului...');
        clientBubble = 'Descărcare comandă 3 cărți...';
        CURRENT_SPLINE = [];
        drive.stopMotors();
      }
      break;
    }

    case 'DELIVER_TO_DESK': {
      // Sequential 3-book delivery animation directly into desk tray
      if (runtime > deliveryTimer) {
        if (deliverPhase === 0) {
          // Extend arm with top book toward client tray
          armExtension = Math.min(1.0, armExtension + 2.4 * DT);
          deliveringBook = basketCargo.length > 0 ? basketCargo[0] : null;
          if (armExtension >= 0.98) {
            // Transfer book from basket to desk
            if (basketCargo.length > 0) {
              const delivered = basketCargo.shift();
              deskDeliveredBooks.push(delivered);
              updateCargoUI();
            }
            deliveringBook = null;
            deliverPhase = 1;
            deliveryTimer = runtime + 0.25;
          }
        } else if (deliverPhase === 1) {
          // Retract arm back empty
          armExtension = Math.max(0.0, armExtension - 2.4 * DT);
          if (armExtension <= 0.05) {
            if (basketCargo.length > 0) {
              // Deliver next book!
              deliverPhase = 0;
              deliveryTimer = runtime + 0.2;
            } else {
              // All 3 delivered!
              deliverPhase = 2;
              deliveryTimer = runtime + 0.4;
            }
          }
        } else if (deliverPhase === 2) {
          totalDelivered += 3;
          chipDeliveredCount.textContent = `${totalDelivered} Cărți`;
          clientBubble = '🎉 3 Cărți Primite! Mulțumesc, Roary!';
          clientBubbleTimer = runtime + 4.0;
          setMissionStatus('Livrare Finalizată cu Succes!', 4, `Lot predat cu succes! Total cărți livrate: ${totalDelivered}.`);

          if (ovLoop && ovLoop.checked) {
            missionState = 'WAITING_NEXT_BATCH';
            nextBatchTimer = runtime + 1.8;
          } else {
            missionState = 'COMPLETE';
          }
        }
      }
      break;
    }

    case 'WAITING_NEXT_BATCH': {
      if (runtime > nextBatchTimer) {
        // Continuous cycle: load next batch of books and travel to shelf!
        batchIndex++;
        shelfBooks = getBatch(batchIndex);
        missionState = 'NAV_TO_SHELF';
        buildShelfPath();
        setMissionStatus('Deplasare către Raft', 1, `Ciclu continuu activ: robotul preia lotul ${batchIndex + 1}...`);
      }
      break;
    }

    case 'COMPLETE': {
      // Waiting in place
      break;
    }
  }
}

// ─── Physics Integration with Realistic AGV Momentum ──────────────────────────
function stepPhysics(dt) {
  const FL = drive.FL, FR = drive.FR, BL = drive.BL, BR = drive.BR;
  const forward = (FL + FR + BL + BR) / 4;
  const strafe = (FL - FR - BL + BR) / 4;
  const rotate = (FL - FR + BL - BR) / 4;

  const H = drive.H;
  // FTC Mecanum forward = +Y, strafe right = +X
  const worldX = Math.sin(H) * forward + Math.cos(H) * strafe;
  const worldY = Math.cos(H) * forward - Math.sin(H) * strafe;

  const targetVx = worldX * MAX_SPEED;
  const targetVy = worldY * MAX_SPEED;
  const targetOmega = rotate * MAX_ROT;

  // Realistic mass & inertia filtering (smooth acceleration & damping)
  const accelRate = 8.5; // 1/s
  simVelX += (targetVx - simVelX) * Math.min(1.0, accelRate * dt);
  simVelY += (targetVy - simVelY) * Math.min(1.0, accelRate * dt);
  simOmega += (targetOmega - simOmega) * Math.min(1.0, 10.0 * dt);

  drive.X = clamp(drive.X + simVelX * dt, -ROOM_WIDTH / 2, ROOM_WIDTH / 2);
  drive.Y = clamp(drive.Y + simVelY * dt, -ROOM_HEIGHT / 2, ROOM_HEIGHT / 2);
  drive.H = angleWrap(drive.H + simOmega * dt);
  drive.vel = new Point(simVelX, simVelY);

  // Motion trail
  if (Math.hypot(simVelX, simVelY) > 2.0) {
    trail.push({ x: drive.X, y: drive.Y });
    if (trail.length > MAX_TRAIL) trail.shift();
  }
}

// ─── Rendering: Beautiful Room & Courier Robot ────────────────────────────────
function render() {
  ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  drawRoomEnvironment();
  if (ovTrail && ovTrail.checked) drawMotionTrail();
  if (ovPath && ovPath.checked) drawTrajectorySpline();
  drawDeskAndClient();
  drawBookshelf();
  drawObstacles();
  drawRobot();
  drawJoystickWidget();
  updateTelemetry();
}

// ─── Draw Floor & Ambient Layout ──────────────────────────────────────────────
function drawRoomEnvironment() {
  // Architectural flooring: subtle light-slate / hardwood look
  ctx.fillStyle = '#0f1726';
  ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  if (ovRoom && ovRoom.checked) {
    // Floor tile grid (30 cm tiles)
    ctx.strokeStyle = '#182336';
    ctx.lineWidth = 1;
    const tileW = 35 * SCALE;
    for (let x = 0; x < CANVAS_WIDTH; x += tileW) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, CANVAS_HEIGHT); ctx.stroke();
    }
    for (let y = 0; y < CANVAS_HEIGHT; y += tileW) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(CANVAS_WIDTH, y); ctx.stroke();
    }

    // Outer perimeter walls with soft glow
    ctx.strokeStyle = '#27354d';
    ctx.lineWidth = 3;
    ctx.strokeRect(10, 10, CANVAS_WIDTH - 20, CANVAS_HEIGHT - 20);

    // Docking station at bottom-left
    const dock = POSES.dock();
    const dc = worldToCanvas(dock.x, dock.y);
    ctx.save();
    ctx.fillStyle = 'rgba(34, 197, 94, 0.08)';
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.strokeRect(dc.x - 30, dc.y - 25, 60, 50);
    ctx.fillRect(dc.x - 30, dc.y - 25, 60, 50);
    ctx.setLineDash([]);
    ctx.fillStyle = '#4ade80';
    ctx.font = '9px sans-serif';
    ctx.fillText('⚡ STAȚIE ÎNCĂRCARE', dc.x - 45, dc.y + 35);
    ctx.restore();
  }
}

// ─── Draw Motion Trail ────────────────────────────────────────────────────────
function drawMotionTrail() {
  if (trail.length < 2) return;
  ctx.save();
  ctx.lineWidth = 2.5;
  for (let i = 1; i < trail.length; i++) {
    const alpha = (i / trail.length) * 0.45;
    ctx.strokeStyle = `rgba(56, 189, 248, ${alpha})`;
    const p1 = worldToCanvas(trail[i - 1].x, trail[i - 1].y);
    const p2 = worldToCanvas(trail[i].x, trail[i].y);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }
  ctx.restore();
}

// ─── Draw Trajectory Spline ───────────────────────────────────────────────────
function drawTrajectorySpline() {
  if (!CURRENT_SPLINE || CURRENT_SPLINE.length < 4) return;
  ctx.save();
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2.4;
  ctx.setLineDash([6, 5]);

  ctx.beginPath();
  for (let i = 0; i <= CURRENT_SPLINE.length - 4; i++) {
    const splinePoints = [
      new Point(CURRENT_SPLINE[i].target.x, CURRENT_SPLINE[i].target.y),
      new Point(CURRENT_SPLINE[i + 1].target.x, CURRENT_SPLINE[i + 1].target.y),
      new Point(CURRENT_SPLINE[i + 2].target.x, CURRENT_SPLINE[i + 2].target.y),
      new Point(CURRENT_SPLINE[i + 3].target.x, CURRENT_SPLINE[i + 3].target.y),
    ];
    for (let step = 0; step <= 25; step++) {
      const t = step / 25;
      const pt = drive.catmullRomSpline(splinePoints, t);
      const c = worldToCanvas(pt.x, pt.y);
      if (i === 0 && step === 0) ctx.moveTo(c.x, c.y);
      else ctx.lineTo(c.x, c.y);
    }
  }
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.restore();
}

// ─── Draw Bookshelf / Archive ─────────────────────────────────────────────────
function drawBookshelf() {
  const shelfPose = POSES.shelf();
  const sc = worldToCanvas(shelfPose.x, shelfPose.y);

  // Bookshelf cabinet on the left wall
  const shelfLeft = sc.x - 55;
  const shelfTop = sc.y - 75;
  const shelfW = 45;
  const shelfH = 150;

  // Drop shadow
  ctx.save();
  ctx.fillStyle = 'rgba(0,0,0,0.45)';
  ctx.fillRect(shelfLeft + 4, shelfTop + 4, shelfW, shelfH);

  // Wood shelf body
  ctx.fillStyle = '#452b1e';
  ctx.fillRect(shelfLeft, shelfTop, shelfW, shelfH);
  ctx.strokeStyle = '#6d4530';
  ctx.lineWidth = 2.5;
  ctx.strokeRect(shelfLeft, shelfTop, shelfW, shelfH);

  // Shelves horizontal dividers
  ctx.fillStyle = '#2d1a10';
  for (let y = shelfTop + 35; y < shelfTop + shelfH; y += 38) {
    ctx.fillRect(shelfLeft + 4, y, shelfW - 8, 4);
  }

  // Draw books currently on shelf
  let curY = shelfTop + 10;
  shelfBooks.forEach((b, idx) => {
    ctx.fillStyle = b.color;
    ctx.fillRect(shelfLeft + 8, curY, 28, 22);
    ctx.fillStyle = '#ffffff';
    ctx.font = '8px sans-serif';
    ctx.fillText(`Vol.${b.id}`, shelfLeft + 10, curY + 14);
    curY += 38;
  });

  // Shelf staging pad (where books are transferred)
  ctx.strokeStyle = 'rgba(245, 158, 11, 0.7)';
  ctx.fillStyle = 'rgba(245, 158, 11, 0.12)';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(sc.x - 8, sc.y - 30, 26, 60);
  ctx.fillRect(sc.x - 8, sc.y - 30, 26, 60);

  ctx.fillStyle = '#f59e0b';
  ctx.font = 'bold 9px sans-serif';
  ctx.fillText('📚 RAFT CĂRȚI', shelfLeft - 5, shelfTop - 8);

  ctx.restore();
}

// ─── Draw Desk & Client Person ────────────────────────────────────────────────
function drawDeskAndClient() {
  const deskPose = POSES.desk();
  const dc = worldToCanvas(deskPose.x, deskPose.y);

  // Executive Desk dimensions
  const deskX = dc.x + 35;
  const deskY = dc.y - 50;
  const deskW = 55;
  const deskH = 100;

  ctx.save();
  // Desk Shadow
  ctx.fillStyle = 'rgba(0,0,0,0.45)';
  ctx.fillRect(deskX + 5, deskY + 5, deskW, deskH);

  // Desk surface (Polished Mahogany Wood)
  ctx.fillStyle = '#653b24';
  ctx.fillRect(deskX, deskY, deskW, deskH);
  ctx.strokeStyle = '#8c5332';
  ctx.lineWidth = 2.5;
  ctx.strokeRect(deskX, deskY, deskW, deskH);

  // Laptop on desk
  ctx.fillStyle = '#cbd5e1';
  ctx.fillRect(deskX + 16, deskY + 36, 22, 16);
  ctx.fillStyle = '#0284c7';
  ctx.fillRect(deskX + 18, deskY + 38, 18, 12); // blue glowing laptop screen

  // Coffee mug
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(deskX + 14, deskY + 18, 4, 0, Math.PI * 2);
  ctx.fill();

  // Delivery Tray (Inbox)
  ctx.fillStyle = 'rgba(34, 197, 94, 0.15)';
  ctx.strokeStyle = '#22c55e';
  ctx.lineWidth = 1.8;
  const trayX = dc.x + 8;
  const trayY = dc.y - 25;
  ctx.strokeRect(trayX, trayY, 24, 50);
  ctx.fillRect(trayX, trayY, 24, 50);

  ctx.fillStyle = '#22c55e';
  ctx.font = 'bold 8px sans-serif';
  ctx.fillText('INBOX', trayX + 1, trayY - 4);

  // Draw books stacked on the delivery tray!
  const maxVisible = 5;
  const visibleBooks = deskDeliveredBooks.slice(-maxVisible);
  const stackOffset = Math.max(0, deskDeliveredBooks.length - maxVisible);

  visibleBooks.forEach((b, i) => {
    ctx.fillStyle = b.color;
    ctx.fillRect(trayX + 3, trayY + 6 + i * 8, 18, 6);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 0.5;
    ctx.strokeRect(trayX + 3, trayY + 6 + i * 8, 18, 6);
  });

  if (deskDeliveredBooks.length > 0) {
    ctx.fillStyle = '#4ade80';
    ctx.font = 'bold 8px sans-serif';
    ctx.fillText(`${deskDeliveredBooks.length} Cărți`, trayX - 2, trayY + 48);
  }

  // Client Person (Sitting behind desk at right)
  const px = deskX + deskW + 18;
  const py = dc.y;

  // Chair
  ctx.fillStyle = '#1e293b';
  ctx.beginPath();
  ctx.arc(px, py, 14, 0, Math.PI * 2);
  ctx.fill();

  // Person shoulders / shirt
  ctx.fillStyle = '#0284c7';
  ctx.beginPath();
  ctx.arc(px, py, 10, 0, Math.PI * 2);
  ctx.fill();

  // Person head
  ctx.fillStyle = '#fed7aa';
  ctx.beginPath();
  ctx.arc(px, py, 6, 0, Math.PI * 2);
  ctx.fill();

  // Hair
  ctx.fillStyle = '#451a03';
  ctx.beginPath();
  ctx.arc(px, py - 2, 6, Math.PI, Math.PI * 2);
  ctx.fill();

  // Speech bubble over person
  if (clientBubble) {
    const bubbleX = deskX - 10;
    const bubbleY = deskY - 32;
    ctx.font = 'bold 9px sans-serif';
    const textW = ctx.measureText(clientBubble).width;

    ctx.fillStyle = 'rgba(15, 23, 42, 0.92)';
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.5)';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.roundRect(bubbleX - textW / 2 + 10, bubbleY - 14, textW + 16, 20, 6);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#ffffff';
    ctx.fillText(clientBubble, bubbleX - textW / 2 + 18, bubbleY);
  }

  ctx.restore();
}

// ─── Draw Room Obstacles (Lounge Table, Armchair, Plant) ───────────────────────
function drawObstacles() {
  if (!ovRoom || !ovRoom.checked) return;
  ctx.save();

  // Center Lounge Table at (-10, -15)
  const tc = worldToCanvas(-10, -15);
  ctx.fillStyle = 'rgba(0,0,0,0.3)';
  ctx.beginPath(); ctx.arc(tc.x + 3, tc.y + 3, 24, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = '#1e293b';
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(tc.x, tc.y, 24, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

  // Plant on table
  ctx.fillStyle = '#10b981';
  ctx.beginPath(); ctx.arc(tc.x, tc.y, 7, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#059669';
  ctx.beginPath(); ctx.arc(tc.x + 4, tc.y - 3, 5, 0, Math.PI * 2); ctx.fill();

  // Lounge Armchair at (-15, 70)
  const ac = worldToCanvas(-15, 70);
  ctx.fillStyle = '#334155';
  ctx.fillRect(ac.x - 18, ac.y - 18, 36, 36);
  ctx.strokeStyle = '#475569';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(ac.x - 18, ac.y - 18, 36, 36);
  ctx.fillStyle = '#475569';
  ctx.fillRect(ac.x - 14, ac.y - 14, 28, 28);

  ctx.restore();
}

// ─── Draw Robot (High Polish with 4 Mecanum Wheels, Basket & Extension Arm) ────
function drawRobot() {
  const rc = worldToCanvas(drive.X, drive.Y);
  const H = drive.H; // Heading in radians

  const w = 32 * SCALE; // Chassis width (~68 px)
  const l = 32 * SCALE; // Chassis length

  ctx.save();
  ctx.translate(rc.x, rc.y);
  ctx.rotate(H); // Rotate with robot heading

  // 1) Soft drop shadow underneath chassis
  ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
  ctx.beginPath();
  ctx.roundRect(-w / 2 + 3, -l / 2 + 5, w, l, 8);
  ctx.fill();

  // 2) Four Mecanum Wheels at the 4 corners with visible 45° angled rollers!
  const wheelW = 9;
  const wheelL = 20;
  const wheelOffsets = [
    { x: -w / 2 - wheelW / 2 + 1, y: -l / 2 + wheelL / 2 - 2, angle: Math.PI / 4 },    // Front-Left
    { x: w / 2 + wheelW / 2 - 1,  y: -l / 2 + wheelL / 2 - 2, angle: -Math.PI / 4 },   // Front-Right
    { x: -w / 2 - wheelW / 2 + 1, y: l / 2 - wheelL / 2 + 2,  angle: -Math.PI / 4 },   // Back-Left
    { x: w / 2 + wheelW / 2 - 1,  y: l / 2 - wheelL / 2 + 2,  angle: Math.PI / 4 },    // Back-Right
  ];

  wheelOffsets.forEach((wo) => {
    ctx.save();
    ctx.translate(wo.x, wo.y);
    // Wheel Rim
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1.2;
    ctx.fillRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);
    ctx.strokeRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);

    // Angled 45° Mecanum Rollers
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1;
    for (let r = -wheelL / 2 + 3; r <= wheelL / 2 - 3; r += 5) {
      ctx.beginPath();
      ctx.moveTo(-wheelW / 2 + 1, r - 2);
      ctx.lineTo(wheelW / 2 - 1, r + 2);
      ctx.stroke();
    }
    ctx.restore();
  });

  // 3) Front Articulated Telescopic Arm & Gripper Jaws
  // Arm extends out the front (which is -Y in robot local space)
  const armExtPx = armExtension * 35;
  ctx.save();
  // Twin metallic slide rails
  ctx.strokeStyle = '#94a3b8';
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  ctx.moveTo(-7, -l / 2);
  ctx.lineTo(-7, -l / 2 - armExtPx);
  ctx.moveTo(7, -l / 2);
  ctx.lineTo(7, -l / 2 - armExtPx);
  ctx.stroke();

  // Gripper crossbar
  ctx.fillStyle = '#0284c7';
  ctx.fillRect(-11, -l / 2 - armExtPx - 3, 22, 5);

  // Gripper dual jaws
  ctx.strokeStyle = armGrabbing ? '#22c55e' : '#38bdf8';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-11, -l / 2 - armExtPx);
  ctx.lineTo(-11, -l / 2 - armExtPx - 7);
  ctx.moveTo(11, -l / 2 - armExtPx);
  ctx.lineTo(11, -l / 2 - armExtPx - 7);
  ctx.stroke();

  // If currently grabbing from shelf or transferring to desk, draw book in the jaws!
  if (armGrabbing && shelfBooks.length > 0) {
    ctx.fillStyle = shelfBooks[0].color;
    ctx.fillRect(-8, -l / 2 - armExtPx - 8, 16, 7);
  } else if (deliveringBook) {
    ctx.fillStyle = deliveringBook.color;
    ctx.fillRect(-8, -l / 2 - armExtPx - 8, 16, 7);
  }
  ctx.restore();

  // 4) Sleek Robot Chassis Body
  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  ctx.roundRect(-w / 2, -l / 2, w, l, 8);
  ctx.fill();
  ctx.stroke();

  // Status Light Ring (LED Halo)
  const isPilotOverride = runtime - lastPilotTime < 0.6;
  const haloColor = isPilotOverride ? '#a855f7' :
                    missionState === 'DELIVER_TO_DESK' ? '#22c55e' :
                    missionState === 'PICKUP_BOOKS' ? '#f59e0b' : '#38bdf8';
  ctx.strokeStyle = haloColor;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(-w / 2 + 3, -l / 2 + 3, w - 6, l - 6, 6);
  ctx.stroke();

  // Directional Forward Chevron
  ctx.fillStyle = haloColor;
  ctx.beginPath();
  ctx.moveTo(0, -l / 2 + 6);
  ctx.lineTo(-5, -l / 2 + 13);
  ctx.lineTo(5, -l / 2 + 13);
  ctx.closePath();
  ctx.fill();

  // 5) Top Cargo Basket (Clear Acrylic Carrier Holding up to 3 Books)
  const basketW = w - 16;
  const basketH = l - 22;
  ctx.fillStyle = 'rgba(30, 41, 59, 0.85)';
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.roundRect(-basketW / 2, -basketH / 2 + 4, basketW, basketH, 4);
  ctx.fill();
  ctx.stroke();

  // Draw books inside basket
  basketCargo.forEach((b, idx) => {
    ctx.fillStyle = b.color;
    const by = -basketH / 2 + 8 + idx * 11;
    ctx.fillRect(-basketW / 2 + 4, by, basketW - 8, 8);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 0.5;
    ctx.strokeRect(-basketW / 2 + 4, by, basketW - 8, 8);
  });

  ctx.restore();
}

// ─── Draw Virtual Joystick ───────────────────────────────────────────────────
function drawJoystickWidget() {
  const jw = joyCanvas.width;
  const jh = joyCanvas.height;
  joyCtx.clearRect(0, 0, jw, jh);

  const cx = jw / 2;
  const cy = jh / 2;
  const maxR = cx - 12;

  // Background ring
  joyCtx.fillStyle = '#0f172a';
  joyCtx.strokeStyle = '#1e293b';
  joyCtx.lineWidth = 2;
  joyCtx.beginPath();
  joyCtx.arc(cx, cy, maxR, 0, Math.PI * 2);
  joyCtx.fill();
  joyCtx.stroke();

  // Crosshairs
  joyCtx.strokeStyle = '#334155';
  joyCtx.lineWidth = 1;
  joyCtx.beginPath();
  joyCtx.moveTo(cx, cy - maxR + 4); joyCtx.lineTo(cx, cy + maxR - 4);
  joyCtx.moveTo(cx - maxR + 4, cy); joyCtx.lineTo(cx + maxR - 4, cy);
  joyCtx.stroke();

  // Thumb position
  const tx = cx + joyX * maxR;
  const ty = cy - joyY * maxR;

  // Thumb knob
  const grad = joyCtx.createRadialGradient(tx, ty, 2, tx, ty, 14);
  grad.addColorStop(0, '#38bdf8');
  grad.addColorStop(1, '#0284c7');
  joyCtx.fillStyle = grad;
  joyCtx.beginPath();
  joyCtx.arc(tx, ty, 13, 0, Math.PI * 2);
  joyCtx.fill();
  joyCtx.strokeStyle = '#ffffff';
  joyCtx.lineWidth = 1.5;
  joyCtx.stroke();
}

// ─── Update Telemetry Accordion (For Technical Reviewers) ──────────────────────
function updateTelemetry() {
  document.getElementById('tel-fps').textContent = currentFps;
  document.getElementById('tel-pos').textContent =
    `${drive.X.toFixed(1)}, ${drive.Y.toFixed(1)}`;
  document.getElementById('tel-heading').textContent =
    `${drive.getH().toFixed(1)}°`;
  document.getElementById('tel-vel').textContent =
    drive.vel.radius().toFixed(1);
  document.getElementById('tel-spline').textContent =
    `${drive.i} / ${drive.localT.toFixed(2)}`;
  document.getElementById('tel-dist').textContent =
    drive.distanceToEnd.toFixed(1);
  document.getElementById('tel-vbat').textContent =
    `${drive.powerCompensator.toFixed(2)} (${drive.batteryVoltage.toFixed(1)}V)`;
  document.getElementById('tel-arm').textContent =
    (armExtension * 35).toFixed(1);
}

// ─── Main Animation Loop (Robust 60 Hz Step) ─────────────────────────────────
let accumulator = 0;
let lastTime = performance.now();

function frame() {
  const now = performance.now();
  let dt = (now - lastTime) / 1000;
  lastTime = now;
  if (dt > 0.25) dt = 0.25;
  accumulator += dt;

  let guard = 0;
  while (accumulator >= DT && guard < 30) {
    control();
    accumulator -= DT;
    guard++;
  }

  render();

  frameCount++;
  if (now - lastFpsTime >= 1000) {
    currentFps = frameCount;
    frameCount = 0;
    lastFpsTime = now;
  }

  requestAnimationFrame(frame);
}

// Start simulation immediately in standby mode ready to run!
updateCargoUI();
requestAnimationFrame(frame);
