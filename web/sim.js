// RoaryX7 — Smart Parking Lot & Autonomous Courier Interactive Simulator.
//
// Features:
// 1. Dual Executive Sponsor Scenarios:
//    - Scenario 1 ('parking'): Complex Smart Parking Lot where a spot frees up, Roary navigates narrow
//      corridors safely around concrete islands and parked cars, a new car arrives in Roary's previous
//      spot, another spot frees up, and Roary departs again in a continuous loop.
//    - Scenario 2 ('courier'): Smart Office / Library Facility (Shelves, Executive Desk, Client Person,
//      Lounge Table, Charging Dock) with 3-book pickup and delivery cycle.
// 2. Realistic Mecanum Drive Physics with mass & inertia powered by RoaryX7 (Catmull-Rom spline + Live Pilot Fusion).
// 3. Coordinate Convention (Roary Point.java):
//    atan() is atan2(x, y) -> 0° = +Y (UP), 90° = +X (RIGHT), -90° = -X (LEFT), 180° = -Y (DOWN).

'use strict';

// clamp is defined in roaryx7.js and available globally

// ─── Environment Dimensions & Scaling ─────────────────────────────────────────
const ROOM_WIDTH = 340;   // cm (-170..+170)
const ROOM_HEIGHT = 260;  // cm (-130..+130)
const CANVAS_WIDTH = 760;
const CANVAS_HEIGHT = 580;
const SCALE = 2.15;       // px per cm
const CENTER_X = CANVAS_WIDTH / 2;
const CENTER_Y = CANVAS_HEIGHT / 2;
const DT = 1 / 60;        // 60 Hz simulation timestep
const MAX_SPEED = 78;     // cm/s max speed
const MAX_ROT = 2.8;      // rad/s max rotation speed

// ─── Waypoints & Poses for Scenario 2 (Courier Mission) ───────────────────────
const POSES = {
  dock:  () => new Pose2d(-105, -85, -90),
  shelf: () => new Pose2d(-95, 15, -90),
  desk:  () => new Pose2d(55, 0, 90),
};

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

// ─── Waypoints & Data for Scenario 1 (Smart Parking Lot) ──────────────────────
// 6 Parking Bays (3 North at y = +84, 3 South at y = -84) separated by lanes & 2 central islands
const PARKING_SPOTS = [
  { id: 'P1', index: 0, x: -96, y: 84,  heading: 0,   row: 'N', title: 'Loc P1 (Nord-Vest)' },
  { id: 'P2', index: 1, x: 0,   y: 84,  heading: 0,   row: 'N', title: 'Loc P2 (Nord-Centru)' },
  { id: 'P3', index: 2, x: 96,  y: 84,  heading: 0,   row: 'N', title: 'Loc P3 (Nord-Est)' },
  { id: 'P4', index: 3, x: -96, y: -84, heading: 175, row: 'S', title: 'Loc P4 (Sud-Vest)' },
  { id: 'P5', index: 4, x: 0,   y: -84, heading: 175, row: 'S', title: 'Loc P5 (Sud-Centru)' },
  { id: 'P6', index: 5, x: 96,  y: -84, heading: 175, row: 'S', title: 'Loc P6 (Sud-Est)' },
];

const PARKING_ISLANDS = [
  { x: -62, y: 0, w: 26, h: 13, label: 'Insulă Vest' },
  { x: 62,  y: 0, w: 26, h: 13, label: 'Insulă Est' },
];

const CAR_CATALOG = [
  { color: '#ef4444', roof: '#991b1b', name: 'Sedan Roșu',     plate: 'B-101-RX' },
  { color: '#3b82f6', roof: '#1e40af', name: 'SUV Albastru',   plate: 'CJ-24-AI' },
  { color: '#10b981', roof: '#065f46', name: 'EV Smarald',     plate: 'TM-88-EV' },
  { color: '#f59e0b', roof: '#92400e', name: 'Coupe Chihlimbar', plate: 'IS-55-RO' },
  { color: '#8b5cf6', roof: '#5b21b6', name: 'Compact Violet', plate: 'BV-77-X7' },
  { color: '#ec4899', roof: '#9d174d', name: 'Sport Rose',     plate: 'B-909-GT' },
  { color: '#06b6d4', roof: '#155e75', name: 'Crossover Cyan', plate: 'SB-42-FT' },
];

// Sequence of spots that free up (diagonal & multi-lane weaves across the lot)
const PARKING_ORDER = [5, 1, 3, 2, 4, 0];

let carSpawnCounter = 0;
function createCarInstance(idx) {
  const tpl = CAR_CATALOG[idx % CAR_CATALOG.length];
  carSpawnCounter++;
  return { ...tpl, uid: carSpawnCounter };
}

let parkingBays = [];
let robotCurrentSpotIdx = 0;    // Spot where Roary is currently parked (starts at P1 = 0)
let robotPrevSpotIdx = null;    // Spot where Roary was before (a new car arrives here!)
let targetFreeSpotIdx = 5;      // Spot that frees up for Roary to navigate to
let parkingSeqIdx = 0;
let departingCar = null;        // { spotIdx, car, progress: 0..1 }
let arrivingCar = null;         // { spotIdx, car, progress: 0..1 }
let totalRelocations = 0;
let minSafetyClearance = 34.0;

function initParkingLotState() {
  carSpawnCounter = 0;
  parkingBays = PARKING_SPOTS.map((spot, idx) => ({
    spot,
    car: idx === 0 ? null : createCarInstance(idx - 1),
  }));
  robotCurrentSpotIdx = 0;
  robotPrevSpotIdx = null;
  parkingSeqIdx = 0;
  targetFreeSpotIdx = PARKING_ORDER[0];
  departingCar = null;
  arrivingCar = null;
  totalRelocations = 0;
  minSafetyClearance = 34.0;
}

// ─── DOM References ──────────────────────────────────────────────────────────
const canvas = document.getElementById('field');
const ctx = canvas.getContext('2d');
const joyCanvas = document.getElementById('joystick');
const joyCtx = joyCanvas.getContext('2d');

const btnScenarioParking = document.getElementById('btn-scenario-parking');
const btnScenarioCourier = document.getElementById('btn-scenario-courier');
const btnToggleMission = document.getElementById('btn-toggle-mission');
const btnMissionIcon = document.getElementById('btn-mission-icon');
const btnMissionLabel = document.getElementById('btn-mission-label');
const btnDemoNudge = document.getElementById('btn-demo-nudge');
const btnReset = document.getElementById('btn-reset');

const missionStatusText = document.getElementById('mission-status-text');
const chipDeliveredIcon = document.getElementById('chip-delivered-icon');
const chipDeliveredLabel = document.getElementById('chip-delivered-label');
const chipDeliveredCount = document.getElementById('chip-delivered-count');
const chipCargoIcon = document.getElementById('chip-cargo-icon');
const chipCargoLabel = document.getElementById('chip-cargo-label');
const chipCargoCount = document.getElementById('chip-cargo-count');
const chipPilotText = document.getElementById('chip-pilot-text');
const dotPilot = document.getElementById('dot-pilot');
const liveBanner = document.getElementById('canvas-live-banner');
const missionCardTitle = document.getElementById('mission-card-title');
const missionStepBadge = document.getElementById('mission-step-badge');
const cargoTitleLabel = document.getElementById('cargo-title-label');
const cargoStatusLabel = document.getElementById('cargo-status-label');
const canvasLegend = document.getElementById('canvas-legend');

// Detect active initial scenario from DOM button state (defaults to 'parking' in browser)
let currentScenario = (btnScenarioParking && btnScenarioParking.className && btnScenarioParking.className.includes('active'))
  ? 'parking'
  : 'courier';

// ─── Drive & State Instances ──────────────────────────────────────────────────
const drive = new Roaryx7Drive('Kirby');
initParkingLotState();
if (currentScenario === 'parking') {
  const p1 = PARKING_SPOTS[0];
  drive.setPosition(p1.x, p1.y, toRadians(p1.heading));
} else {
  const dock = POSES.dock();
  drive.setPosition(dock.x, dock.y, toRadians(dock.heading));
}
drive.setGoToK(true);

// Mission State Machine:
// Courier states: 'STANDBY' | 'NAV_TO_SHELF' | 'PICKUP_BOOKS' | 'NAV_TO_DESK' | 'DELIVER_TO_DESK' | 'WAITING_NEXT_BATCH' | 'COMPLETE'
// Parking states: 'STANDBY' | 'PARKING_FREEING_SPOT' | 'PARKING_NAV_TO_SPOT' | 'PARKING_FILLING_OLD_SPOT' | 'PARKING_WAITING_NEXT'
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
let splinePhase = 'NONE';       // 'TO_SHELF' | 'TO_DESK' | 'TO_PARKING_SPOT' | 'CUSTOM'

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

// Display & cycle options: always enabled for optimal sponsor presentation
const ovPath = true;
const ovTrail = true;
const ovRoom = true;
const ovLoop = true;

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

// ─── UI & Scenario Helpers ───────────────────────────────────────────────────
function setMissionStatus(text, stepNr, bannerText) {
  missionStatusText.textContent = text;
  missionStep = stepNr;
  if (missionStepBadge) missionStepBadge.textContent = `Etapa ${stepNr} / 4`;
  if (bannerText && liveBanner) {
    liveBanner.textContent = bannerText;
    liveBanner.classList.remove('active-nudge');
  }

  for (let k = 1; k <= 4; k++) {
    const el = document.getElementById(`step-node-${k}`);
    if (!el) continue;
    el.classList.remove('active', 'completed');
    if (k < stepNr) el.classList.add('completed');
    else if (k === stepNr) el.classList.add('active');
  }
}

function syncScenarioStaticLabels() {
  const isParking = currentScenario === 'parking';
  if (btnScenarioParking) {
    if (isParking) btnScenarioParking.classList.add('active');
    else btnScenarioParking.classList.remove('active');
  }
  if (btnScenarioCourier) {
    if (!isParking) btnScenarioCourier.classList.add('active');
    else btnScenarioCourier.classList.remove('active');
  }

  const s1Title = document.getElementById('step-title-1');
  const s1Sub   = document.getElementById('step-sub-1');
  const s2Title = document.getElementById('step-title-2');
  const s2Sub   = document.getElementById('step-sub-2');
  const s3Title = document.getElementById('step-title-3');
  const s3Sub   = document.getElementById('step-sub-3');
  const s4Title = document.getElementById('step-title-4');
  const s4Sub   = document.getElementById('step-sub-4');

  if (isParking) {
    if (chipDeliveredIcon) chipDeliveredIcon.textContent = '🅿️';
    if (chipDeliveredLabel) chipDeliveredLabel.textContent = 'Relocări Parcare';
    if (chipCargoIcon) chipCargoIcon.textContent = '🚗';
    if (chipCargoLabel) chipCargoLabel.textContent = 'Ocupare Parcare';
    if (missionCardTitle) missionCardTitle.textContent = 'Navigație în Parcare & Spații Înguste';
    if (cargoTitleLabel) cargoTitleLabel.textContent = 'Monitorizare Locuri & Siguranță:';

    if (s1Title) s1Title.textContent = 'Eliberare Loc de Parcare';
    if (s1Sub)   s1Sub.textContent   = 'Un vehicul pleacă și eliberează spațiul';
    if (s2Title) s2Title.textContent = 'Navigație prin Spații Complexe';
    if (s2Sub)   s2Sub.textContent   = 'Traseu Catmull-Rom sigur printre mașini';
    if (s3Title) s3Title.textContent = 'Ocupare Loc Anterior';
    if (s3Sub)   s3Sub.textContent   = 'O mașină nouă apare unde era Roary înainte';
    if (s4Title) s4Title.textContent = 'Relocare Continuă';
    if (s4Sub)   s4Sub.textContent   = 'Se eliberează alt loc și Roary pleacă din nou';

    if (canvasLegend) {
      canvasLegend.innerHTML = `
        <span><i class="lg-swatch" style="background:#38bdf8"></i>Traseu Optimizat Roary</span>
        <span><i class="lg-swatch" style="background:#60a5fa; border-radius:50%"></i>Puncte de Control (Waypoints)</span>
        <span><i class="lg-swatch" style="background:#22c55e"></i>Loc Parcare Eliberat</span>
        <span><i class="lg-swatch" style="background:#f59e0b"></i>Loc Anterior (Apare Mașină)</span>
        <span><i class="lg-swatch" style="background:#a855f7"></i>Intervenție Manuală Pilot</span>
      `;
    }
  } else {
    if (chipDeliveredIcon) chipDeliveredIcon.textContent = '📦';
    if (chipDeliveredLabel) chipDeliveredLabel.textContent = 'Cărți Livrate';
    if (chipCargoIcon) chipCargoIcon.textContent = '🧺';
    if (chipCargoLabel) chipCargoLabel.textContent = 'Încărcătură Coș';
    if (missionCardTitle) missionCardTitle.textContent = 'Misiune Logistică în Desfășurare';
    if (cargoTitleLabel) cargoTitleLabel.textContent = 'Compartiment Marfă Robot (3 Cărți):';

    if (s1Title) s1Title.textContent = 'Deplasare la Raft';
    if (s1Sub)   s1Sub.textContent   = 'Navigație omnidirecțională lină';
    if (s2Title) s2Title.textContent = 'Preluare 3 Cărți';
    if (s2Sub)   s2Sub.textContent   = 'Braț telescopic & încărcare în coș';
    if (s3Title) s3Title.textContent = 'Transport Către Birou';
    if (s3Sub)   s3Sub.textContent   = 'Ocolire obstacole și frânare lină';
    if (s4Title) s4Title.textContent = 'Predare la Client';
    if (s4Sub)   s4Sub.textContent   = 'Descărcare în tăviță și confirmare';

    if (canvasLegend) {
      canvasLegend.innerHTML = `
        <span><i class="lg-swatch" style="background:#38bdf8"></i>Traseu Optimizat Roary</span>
        <span><i class="lg-swatch" style="background:#60a5fa; border-radius:50%"></i>Puncte de Control (Waypoints)</span>
        <span><i class="lg-swatch" style="background:#22c55e"></i>Zonă Livrare Birou</span>
        <span><i class="lg-swatch" style="background:#f59e0b"></i>Raft Cărți &amp; Preluare</span>
        <span><i class="lg-swatch" style="background:#a855f7"></i>Intervenție Manuală Pilot</span>
      `;
    }
  }
}

function updateParkingUI() {
  if (currentScenario !== 'parking') return;
  const occupiedByCars = parkingBays.filter(b => b.car !== null).length;
  chipDeliveredCount.textContent = `${totalRelocations} ${totalRelocations === 1 ? 'Parcare' : 'Parcări'}`;
  chipCargoCount.textContent = `${occupiedByCars} Mașini + Roary`;
  cargoStatusLabel.textContent = `${occupiedByCars}/6 Auto · Gardă >${minSafetyClearance.toFixed(0)}cm`;

  const slot0 = document.getElementById('book-slot-0');
  const slot1 = document.getElementById('book-slot-1');
  const slot2 = document.getElementById('book-slot-2');

  const prevLabel = robotPrevSpotIdx !== null ? PARKING_SPOTS[robotPrevSpotIdx].id : 'P1';
  const prevState = missionState === 'PARKING_FILLING_OLD_SPOT'
    ? 'Intră Mașină...'
    : (robotPrevSpotIdx !== null && parkingBays[robotPrevSpotIdx].car)
      ? 'Ocupat de Auto'
      : 'Locul Inițial';

  const targetLabel = PARKING_SPOTS[targetFreeSpotIdx].id;
  const targetState = missionState === 'PARKING_FREEING_SPOT'
    ? 'Se Eliberează...'
    : missionState === 'PARKING_NAV_TO_SPOT'
      ? 'Țintă Liberă'
      : 'Roary Parcat';

  if (slot0) {
    slot0.className = 'book-slot loaded-orange';
    slot0.innerHTML = `
      <div class="book-spine">
        <span class="book-num">LOC ANTERIOR (${prevLabel})</span>
        <span class="book-name">${prevState}</span>
      </div>
    `;
  }
  if (slot1) {
    slot1.className = 'book-slot loaded-green';
    slot1.innerHTML = `
      <div class="book-spine">
        <span class="book-num">LOC ȚINTĂ (${targetLabel})</span>
        <span class="book-name">${targetState}</span>
      </div>
    `;
  }
  if (slot2) {
    slot2.className = 'book-slot loaded-blue';
    slot2.innerHTML = `
      <div class="book-spine">
        <span class="book-num">SIGURANȚĂ TRASEU</span>
        <span class="book-name">0 Coliziuni · OK</span>
      </div>
    `;
  }
}

function updateCargoUI() {
  if (currentScenario === 'parking') {
    updateParkingUI();
    return;
  }
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

function setScenario(scen) {
  currentScenario = scen === 'parking' ? 'parking' : 'courier';
  syncScenarioStaticLabels();
  resetAll();
}

// ─── Trajectory Builders ─────────────────────────────────────────────────────
function buildParkingPath(fromIdx, toIdx) {
  const cur = drive.getPose();
  const toSpot = PARKING_SPOTS[toIdx];
  const targetPose = new Pose2d(toSpot.x, toSpot.y, toSpot.heading);
  splinePhase = 'TO_PARKING_SPOT';

  const laneYFrom = cur.y >= 0 ? 36 : -36;
  const laneYTo = toSpot.y >= 0 ? 36 : -36;

  // Smooth intermediate headings strictly inside [-175, 175] so Catmull-Rom never overshoots ±180°
  const hStart = clamp(cur.heading, -175, 175);
  const hEnd = toSpot.heading;
  const lerpH = (f) => hStart + (hEnd - hStart) * f;

  let pts = [];
  pts.push(new goToValues(new Pose2d(cur.x, cur.y, hStart), 0.68, 3));
  pts.push(new goToValues(new Pose2d(cur.x, cur.y, hStart), 0.68, 3));

  const inCentralPassage = Math.abs(cur.x) <= 28 && Math.abs(cur.y) <= 40;
  const inBay = Math.abs(cur.y) > 48;

  if ((cur.y >= 0 && toSpot.y < 0) || (cur.y < 0 && toSpot.y >= 0)) {
    const sameOuterSide = Math.abs(cur.x - toSpot.x) < 25 && Math.abs(cur.x) > 50;
    if (sameOuterSide) {
      const sideX = Math.sign(cur.x) * 116;
      pts.push(new goToValues(new Pose2d(sideX, laneYFrom, lerpH(0.25)), 0.72, 4));
      pts.push(new goToValues(new Pose2d(sideX, 0, lerpH(0.55)), 0.74, 4));
      pts.push(new goToValues(new Pose2d(sideX, laneYTo, lerpH(0.85)), 0.70, 3.5));
    } else if (inCentralPassage) {
      // Already in the central passage between the islands: go directly through (0,0) into target lane
      if (Math.abs(cur.y) > 8) {
        pts.push(new goToValues(new Pose2d(0, 0, lerpH(0.35)), 0.74, 4));
      }
      const exitCenterX = clamp(toSpot.x * 0.20, -20, 20);
      pts.push(new goToValues(new Pose2d(exitCenterX, laneYTo, lerpH(0.65)), 0.72, 4));
      if (Math.abs(toSpot.x) > 40) {
        pts.push(new goToValues(new Pose2d(toSpot.x * 0.85, laneYTo, lerpH(0.88)), 0.68, 3.5));
      }
    } else {
      // 1) Pull straight out of current bay into current driving lane
      if (inBay && Math.abs(cur.x) > 40) {
        pts.push(new goToValues(new Pose2d(cur.x * 0.85, laneYFrom, lerpH(0.18)), 0.72, 4));
      }
      // 2) Align with central passage in current lane
      const enterCenterX = clamp(cur.x * 0.20, -20, 20);
      pts.push(new goToValues(new Pose2d(enterCenterX, laneYFrom, lerpH(0.36)), 0.74, 4));
      // 3) Pass through central gap (0, 0) between concrete islands
      pts.push(new goToValues(new Pose2d(0, 0, lerpH(0.55)), 0.74, 4));
      // 4) Enter target lane from central passage
      const exitCenterX = clamp(toSpot.x * 0.20, -20, 20);
      pts.push(new goToValues(new Pose2d(exitCenterX, laneYTo, lerpH(0.74)), 0.72, 4));
      // 5) Cruise along target lane to front of target bay before turning in
      if (Math.abs(toSpot.x) > 40) {
        pts.push(new goToValues(new Pose2d(toSpot.x * 0.85, laneYTo, lerpH(0.90)), 0.68, 3.5));
      }
    }
  } else {
    // Same-row navigation along the safe lane corridor
    const pullOutX = Math.abs(cur.x) < 15 ? 0 : cur.x * 0.85;
    const alignX = Math.abs(toSpot.x) < 15 ? 0 : toSpot.x * 0.85;
    const midX = (pullOutX + alignX) / 2;
    if (inBay) {
      pts.push(new goToValues(new Pose2d(pullOutX, laneYFrom, lerpH(0.25)), 0.72, 4));
    }
    pts.push(new goToValues(new Pose2d(midX, laneYTo, lerpH(0.55)), 0.74, 4));
    pts.push(new goToValues(new Pose2d(alignX, laneYTo, lerpH(0.85)), 0.70, 3.5));
  }

  pts.push(new goToValues(targetPose, 0.58, 3));
  pts.push(new goToValues(targetPose, 0.58, 3));

  CURRENT_SPLINE = pts;
  drive.resetSpline();
}

function beginNextParkingFreeing(customTargetIdx = null) {
  // 1) If an arriving car was mid-entry into Roary's previous spot, finalize it immediately
  if (arrivingCar) {
    parkingBays[arrivingCar.spotIdx].car = arrivingCar.car;
    arrivingCar = null;
    totalRelocations++;
  }

  // 2) Clear any departing car animation from an interrupted transition
  if (departingCar) {
    departingCar = null;
  }

  // 3) Determine the origin spot (which remains empty for the next arriving car) and next target spot
  let originSpotIdx = robotCurrentSpotIdx !== null
    ? robotCurrentSpotIdx
    : (robotPrevSpotIdx !== null ? robotPrevSpotIdx : 0);

  let nextTargetIdx;
  if (customTargetIdx !== null && customTargetIdx !== robotCurrentSpotIdx) {
    nextTargetIdx = customTargetIdx;
  } else {
    let candidate = PARKING_ORDER[parkingSeqIdx % PARKING_ORDER.length];
    if (candidate === originSpotIdx) {
      parkingSeqIdx++;
      candidate = PARKING_ORDER[parkingSeqIdx % PARKING_ORDER.length];
    }
    nextTargetIdx = candidate;
  }

  // If Roary was redirected mid-transit back to the exact spot it just left, swap origin with previous target
  if (robotCurrentSpotIdx === null && nextTargetIdx === originSpotIdx) {
    originSpotIdx = (targetFreeSpotIdx !== null && targetFreeSpotIdx !== nextTargetIdx)
      ? targetFreeSpotIdx
      : (nextTargetIdx + 1) % PARKING_SPOTS.length;
    parkingBays[originSpotIdx].car = null;
  }

  // Restore a car in any other bay that was vacated by an aborted mid-transit target
  parkingBays.forEach((b, idx) => {
    if (idx !== originSpotIdx && idx !== nextTargetIdx && b.car === null) {
      b.car = createCarInstance(carSpawnCounter);
    }
  });

  robotPrevSpotIdx = originSpotIdx;
  targetFreeSpotIdx = nextTargetIdx;

  const bay = parkingBays[targetFreeSpotIdx];
  const spotId = bay.spot.id;
  buildParkingPath(robotPrevSpotIdx, targetFreeSpotIdx);

  if (bay.car) {
    departingCar = {
      spotIdx: targetFreeSpotIdx,
      car: bay.car,
      progress: 0,
    };
    missionState = 'PARKING_FREEING_SPOT';
    setMissionStatus(
      `Se Eliberează Locul ${spotId}`,
      1,
      `🚗 Mașina din locul ${spotId} părăsește parcarea · Traseul Roary către ${spotId} este calculat!`
    );
  } else {
    departingCar = null;
    robotCurrentSpotIdx = null;
    missionState = 'PARKING_NAV_TO_SPOT';
    setMissionStatus(
      `Navigație spre ${spotId}`,
      2,
      `🅿️ Locul ${spotId} este liber! Roary navighează în siguranță printre obstacole...`
    );
  }
  updateParkingUI();
}

function buildShelfPath() {
  const cur = drive.getPose();
  const shelf = POSES.shelf();
  splinePhase = 'TO_SHELF';

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
  drive.setGoToK(true);
  drive.DXEnabled = true;
  btnToggleMission.classList.remove('paused');
  btnMissionIcon.textContent = '⏸';
  btnMissionLabel.textContent = 'Pauză Misiune';

  if (currentScenario === 'parking') {
    if (missionState === 'STANDBY') {
      beginNextParkingFreeing();
    } else if (missionState === 'PARKING_FREEING_SPOT') {
      const spotId = PARKING_SPOTS[targetFreeSpotIdx].id;
      setMissionStatus(`Se Eliberează Locul ${spotId}`, 1, `🚗 Mașina din locul ${spotId} părăsește parcarea...`);
    } else if (missionState === 'PARKING_NAV_TO_SPOT') {
      const spotId = PARKING_SPOTS[targetFreeSpotIdx].id;
      buildParkingPath(robotPrevSpotIdx, targetFreeSpotIdx);
      setMissionStatus(`Navigație spre ${spotId}`, 2, `🅿️ Roary reia navigația în siguranță spre locul ${spotId}...`);
    } else if (missionState === 'PARKING_FILLING_OLD_SPOT') {
      const prevId = PARKING_SPOTS[robotPrevSpotIdx !== null ? robotPrevSpotIdx : 0].id;
      setMissionStatus(`Apare Mașină în ${prevId}`, 3, `🚙 Pe locul anterior (${prevId}) parchează o mașină nouă...`);
    }
    return;
  }

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
  drive.stopMotors();
  drive.setGoToK(false);
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
  simVelX = 0;
  simVelY = 0;
  simOmega = 0;
  trail.length = 0;
  CURRENT_SPLINE = [];
  splinePhase = 'NONE';

  if (currentScenario === 'parking') {
    initParkingLotState();
    const p1 = PARKING_SPOTS[0];
    drive.setPosition(p1.x, p1.y, toRadians(p1.heading));
    drive.stopMotors();
    drive.setGoToK(false);
    drive.resetSpline();
    updateParkingUI();

    btnToggleMission.classList.remove('paused');
    btnMissionIcon.textContent = '▶';
    btnMissionLabel.textContent = 'Pornește Parcare Autonomă';
    setMissionStatus('Parcat în Locul P1', 1, 'Roary este parcat în P1 (5/6 locuri ocupate de mașini). Apasă „Pornește”!');
    return;
  }

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
  drive.setGoToK(false);
  drive.resetSpline();

  btnToggleMission.classList.remove('paused');
  btnMissionIcon.textContent = '▶';
  btnMissionLabel.textContent = 'Pornește Misiune Curierat';
  setMissionStatus('Gata de Livrare', 1, 'Robot andocat la stația de încărcare. Apasă „Pornește Misiune”.');
}

// Direct button actions
if (btnScenarioParking) {
  btnScenarioParking.addEventListener('click', () => {
    setScenario('parking');
    startMission();
  });
}
if (btnScenarioCourier) {
  btnScenarioCourier.addEventListener('click', () => {
    setScenario('courier');
    startMission();
  });
}
btnToggleMission.addEventListener('click', toggleMission);
btnDemoNudge.addEventListener('click', () => {
  demoNudgeTimer = runtime + 1.6;
  demoNudgeX = 0.85;
  demoNudgeY = 0.35;
  if (liveBanner) {
    liveBanner.textContent = '👋 Simulare Pilot Nudge: Operatorul intervine pentru ocolire în spațiu îngust!';
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

  if (Math.hypot(kx, ky) > 1) {
    const l = Math.hypot(kx, ky);
    kx /= l; ky /= l;
  }

  if (!joyActive) {
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
  joyY = -dy / maxR;
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

// Click on field: in Parking mode, click a parking bay to free it & send Roary there
canvas.addEventListener('click', (e) => {
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const world = canvasToWorld(cx, cy);

  if (currentScenario === 'parking') {
    const clickedSpot = PARKING_SPOTS.find(
      s => Math.abs(world.x - s.x) < 32 && Math.abs(world.y - s.y) < 34
    );
    if (clickedSpot && clickedSpot.index !== robotCurrentSpotIdx) {
      missionActive = true;
      drive.setGoToK(true);
      drive.DXEnabled = true;
      btnToggleMission.classList.remove('paused');
      btnMissionIcon.textContent = '⏸';
      btnMissionLabel.textContent = 'Pauză Misiune';
      beginNextParkingFreeing(clickedSpot.index);
    }
    return;
  }

  const cur = drive.getPose();
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
  drive.setGoToK(true);
  setMissionStatus('Navigație către Destinație', 1, `Robotul navighează către (${world.x.toFixed(0)}, ${world.y.toFixed(0)})...`);
});

// ─── Control Step (60 Hz) ─────────────────────────────────────────────────────
function control() {
  runtime += DT;
  updateJoyFromKeyboard();

  // 1) Stick / Joystick Input fed directly into Roary
  drive.mainDrive(joyX, joyY, joyR);

  // 2) Autonomous Navigation via Roary Spline / goToV2
  const allowSplineDrive = missionState !== 'PARKING_FREEING_SPOT';
  if (allowSplineDrive && CURRENT_SPLINE && CURRENT_SPLINE.length >= 4) {
    const endTarget = CURRENT_SPLINE[CURRENT_SPLINE.length - 1].target;

    if (!drive.splineDone) {
      drive.splineToDX(CURRENT_SPLINE, drive.vel);
    } else {
      drive.goToV2(endTarget, 0.38, 3.0);
    }
  } else {
    drive.goTo.FLPow = 0;
    drive.goTo.FRPow = 0;
    drive.goTo.BLPow = 0;
    drive.goTo.BRPow = 0;
  }

  // 3) Mission State Machine Logic
  if (currentScenario === 'parking') {
    stepParkingStateMachine();
  } else {
    stepMissionStateMachine();
  }

  // 4) Battery & Power Compensation
  drive.updatePowerCompensator();

  // 5) Integrate Physics with realistic AGV mass & inertia
  stepPhysics(DT);
}

// ─── Parking Lot State Machine Execution ─────────────────────────────────────
function stepParkingStateMachine() {
  if (!missionActive) return;

  // Track safety clearance to central islands while moving
  const curPoint = drive.getPoint();
  PARKING_ISLANDS.forEach(isl => {
    const d = Math.hypot(curPoint.x - isl.x, curPoint.y - isl.y);
    if (d < minSafetyClearance) minSafetyClearance = d;
  });

  // Continue animating departing car along the exit lane even after Roary begins moving
  if (departingCar && missionState !== 'PARKING_FREEING_SPOT') {
    departingCar.progress = Math.min(1.0, departingCar.progress + 1.4 * DT);
    if (departingCar.progress >= 1.0) {
      departingCar = null;
    }
  }

  switch (missionState) {
    case 'PARKING_FREEING_SPOT': {
      if (departingCar) {
        departingCar.progress = Math.min(1.0, departingCar.progress + 2.6 * DT);
        if (departingCar.progress >= 0.45) {
          const freedIdx = departingCar.spotIdx;
          parkingBays[freedIdx].car = null;

          // Spot is now clear! Roary departs its current spot along the pre-built spline
          if (robotCurrentSpotIdx !== null) {
            robotPrevSpotIdx = robotCurrentSpotIdx;
          } else if (robotPrevSpotIdx === null) {
            robotPrevSpotIdx = 0;
          }
          robotCurrentSpotIdx = null;
          missionState = 'PARKING_NAV_TO_SPOT';
          buildParkingPath(robotPrevSpotIdx, freedIdx);

          const spotId = PARKING_SPOTS[freedIdx].id;
          const prevId = PARKING_SPOTS[robotPrevSpotIdx].id;
          setMissionStatus(
            `Navigație spre ${spotId}`,
            2,
            `🅿️ Locul ${spotId} s-a eliberat! Roary pleacă din ${prevId} și navighează în siguranță spre ${spotId}...`
          );
          updateParkingUI();
        }
      }
      break;
    }

    case 'PARKING_NAV_TO_SPOT': {
      const targetSpot = PARKING_SPOTS[targetFreeSpotIdx];
      const distToSpot = curPoint.distanceTo(new Point(targetSpot.x, targetSpot.y));
      const degDiff = Math.abs(toDegrees(angleWrap(toRadians(targetSpot.heading) - drive.H)));

      if (distToSpot < 6.0 && degDiff < 6.5) {
        // Robot has safely parked in the freed spot!
        robotCurrentSpotIdx = targetFreeSpotIdx;
        CURRENT_SPLINE = [];
        drive.stopMotors();

        // Now a new car appears where the robot was before (robotPrevSpotIdx)!
        const fillIdx = robotPrevSpotIdx !== null ? robotPrevSpotIdx : 0;
        arrivingCar = {
          spotIdx: fillIdx,
          car: createCarInstance(carSpawnCounter),
          progress: 0,
        };
        missionState = 'PARKING_FILLING_OLD_SPOT';

        const curId = targetSpot.id;
        const prevId = PARKING_SPOTS[fillIdx].id;
        setMissionStatus(
          `Apare Mașină în ${prevId}`,
          3,
          `🚙 Roary a parcat în ${curId}! Pe locul anterior (${prevId}) apare acum o mașină nouă...`
        );
        updateParkingUI();
      }
      break;
    }

    case 'PARKING_FILLING_OLD_SPOT': {
      if (arrivingCar) {
        arrivingCar.progress = Math.min(1.0, arrivingCar.progress + 1.55 * DT);
        if (arrivingCar.progress >= 1.0) {
          const filledIdx = arrivingCar.spotIdx;
          parkingBays[filledIdx].car = arrivingCar.car;
          arrivingCar = null;
          totalRelocations++;

          missionState = 'PARKING_WAITING_NEXT';
          nextBatchTimer = runtime + 0.7;

          const filledId = PARKING_SPOTS[filledIdx].id;
          const curId = PARKING_SPOTS[robotCurrentSpotIdx].id;
          setMissionStatus(
            `Loc ${filledId} Ocupat · Gata`,
            4,
            `✅ Locul anterior ${filledId} a fost ocupat de o mașină! Roary așteaptă în ${curId} eliberarea altui loc...`
          );
          updateParkingUI();
        }
      }
      break;
    }

    case 'PARKING_WAITING_NEXT': {
      if (runtime > nextBatchTimer) {
        parkingSeqIdx++;
        beginNextParkingFreeing();
      }
      break;
    }
  }
}

// ─── Courier Mission State Machine Execution ──────────────────────────────────
function stepMissionStateMachine() {
  if (!missionActive) return;

  const curPoint = drive.getPoint();

  switch (missionState) {
    case 'NAV_TO_SHELF': {
      const shelfPose = POSES.shelf();
      const distToShelf = curPoint.distanceTo(new Point(shelfPose.x, shelfPose.y));
      const degDiff = Math.abs(toDegrees(angleWrap(toRadians(shelfPose.heading) - drive.H)));

      if (distToShelf < 6.0 && degDiff < 6.0) {
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
      if (runtime > pickupTimer) {
        if (pickupPhase === 0) {
          armExtension = Math.min(1.0, armExtension + 2.4 * DT);
          if (armExtension >= 0.98) {
            armGrabbing = true;
            pickupTimer = runtime + 0.3;
            pickupPhase = 1;
          }
        } else if (pickupPhase === 1) {
          armExtension = Math.max(0.0, armExtension - 2.4 * DT);
          if (armExtension <= 0.05) {
            armGrabbing = false;
            if (basketCargo.length < 3 && shelfBooks.length > 0) {
              const book = shelfBooks.shift();
              basketCargo.push(book);
              updateCargoUI();
            }
            if (basketCargo.length < 3 && shelfBooks.length > 0) {
              pickupPhase = 0;
              pickupTimer = runtime + 0.2;
            } else {
              pickupPhase = 2;
              pickupTimer = runtime + 0.5;
              setMissionStatus('Coș Plin (3/3)', 2, 'Toate cele 3 cărți au fost încărcate în coș! Pornim spre birou...');
            }
          }
        } else if (pickupPhase === 2) {
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
      if (runtime > deliveryTimer) {
        if (deliverPhase === 0) {
          armExtension = Math.min(1.0, armExtension + 2.4 * DT);
          deliveringBook = basketCargo.length > 0 ? basketCargo[0] : null;
          if (armExtension >= 0.98) {
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
          armExtension = Math.max(0.0, armExtension - 2.4 * DT);
          if (armExtension <= 0.05) {
            if (basketCargo.length > 0) {
              deliverPhase = 0;
              deliveryTimer = runtime + 0.2;
            } else {
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

          missionState = 'WAITING_NEXT_BATCH';
          nextBatchTimer = runtime + 1.8;
        }
      }
      break;
    }

    case 'WAITING_NEXT_BATCH': {
      if (runtime > nextBatchTimer) {
        batchIndex++;
        shelfBooks = getBatch(batchIndex);
        missionState = 'NAV_TO_SHELF';
        buildShelfPath();
        setMissionStatus('Deplasare către Raft', 1, `Ciclu continuu activ: robotul preia lotul ${batchIndex + 1}...`);
      }
      break;
    }

    case 'COMPLETE': {
      break;
    }
  }
}

// ─── Physics Integration with Realistic AGV Momentum & Safety Shield ─────────
function pushOutFromRoundedBox(cx, cy, hx, hy, minR) {
  const qx = clamp(drive.X, cx - hx, cx + hx);
  const qy = clamp(drive.Y, cy - hy, cy + hy);
  const dx = drive.X - qx;
  const dy = drive.Y - qy;
  const dist = Math.hypot(dx, dy);
  if (dist < minR) {
    let nx, ny;
    if (dist > 1e-4) {
      nx = dx / dist;
      ny = dy / dist;
    } else {
      nx = 0;
      ny = drive.Y >= cy ? 1 : -1;
    }
    drive.X = qx + nx * minR;
    drive.Y = qy + ny * minR;
    const vn = simVelX * nx + simVelY * ny;
    if (vn < 0) {
      simVelX -= vn * nx;
      simVelY -= vn * ny;
    }
  }
}

function resolveObstacleCollisions() {
  if (currentScenario === 'parking') {
    // 1) Concrete traffic islands safety perimeter
    PARKING_ISLANDS.forEach((isl) => {
      pushOutFromRoundedBox(isl.x, isl.y, 10, 4.5, 17.5);
    });
    // 2) Occupied parking bays safety perimeter
    parkingBays.forEach((bay, idx) => {
      if (!bay.car) return;
      if (departingCar && departingCar.spotIdx === idx) return;
      pushOutFromRoundedBox(bay.spot.x, bay.spot.y, 12, 16, 15);
    });
  } else {
    // Courier room obstacles (lounge table & armchair)
    pushOutFromRoundedBox(-10, -15, 0, 0, 23);
    pushOutFromRoundedBox(-15, 70, 6, 6, 16);
  }
}

function stepPhysics(dt) {
  const FL = drive.FL, FR = drive.FR, BL = drive.BL, BR = drive.BR;
  const forward = (FL + FR + BL + BR) / 4;
  const strafe = (FL - FR - BL + BR) / 4;
  const rotate = (FL - FR + BL - BR) / 4;

  const H = drive.H;
  const worldX = Math.sin(H) * forward + Math.cos(H) * strafe;
  const worldY = Math.cos(H) * forward - Math.sin(H) * strafe;

  const targetVx = worldX * MAX_SPEED;
  const targetVy = worldY * MAX_SPEED;
  const targetOmega = rotate * MAX_ROT;

  const accelRate = 8.5;
  simVelX += (targetVx - simVelX) * Math.min(1.0, accelRate * dt);
  simVelY += (targetVy - simVelY) * Math.min(1.0, accelRate * dt);
  simOmega += (targetOmega - simOmega) * Math.min(1.0, 10.0 * dt);

  drive.X = clamp(drive.X + simVelX * dt, -ROOM_WIDTH / 2 + 16, ROOM_WIDTH / 2 - 16);
  drive.Y = clamp(drive.Y + simVelY * dt, -ROOM_HEIGHT / 2 + 16, ROOM_HEIGHT / 2 - 16);
  resolveObstacleCollisions();
  drive.H = angleWrap(drive.H + simOmega * dt);
  drive.vel = new Point(simVelX, simVelY);

  if (Math.hypot(simVelX, simVelY) > 2.0) {
    trail.push({ x: drive.X, y: drive.Y });
    if (trail.length > MAX_TRAIL) trail.shift();
  }
}

// ─── Rendering: Parking Lot & Office Courier Environments ────────────────────
function render() {
  ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  if (currentScenario === 'parking') {
    drawParkingEnvironment();
    drawMotionTrail();
    drawTrajectorySpline();
    drawParkingCars();
    drawRobot();
  } else {
    drawRoomEnvironment();
    drawMotionTrail();
    drawTrajectorySpline();
    drawDeskAndClient();
    drawBookshelf();
    drawObstacles();
    drawRobot();
  }

  drawJoystickWidget();
  updateTelemetry();
}

// ─── Draw Smart Parking Lot Environment ───────────────────────────────────────
function drawParkingEnvironment() {
  // Deep asphalt surface
  ctx.fillStyle = '#0c1320';
  ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  // Subtle asphalt grid texture
  ctx.strokeStyle = 'rgba(30, 41, 59, 0.55)';
  ctx.lineWidth = 1;
  const stepPx = 30 * SCALE;
  for (let x = 0; x < CANVAS_WIDTH; x += stepPx) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, CANVAS_HEIGHT); ctx.stroke();
  }
  for (let y = 0; y < CANVAS_HEIGHT; y += stepPx) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(CANVAS_WIDTH, y); ctx.stroke();
  }

  // Outer concrete perimeter wall
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 3;
  ctx.strokeRect(10, 10, CANVAS_WIDTH - 20, CANVAS_HEIGHT - 20);

  // Driving lane dashed centerlines (North lane y = +34, South lane y = -34)
  ctx.save();
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.25)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([10, 10]);
  const northLaneY = worldToCanvas(0, 34).y;
  const southLaneY = worldToCanvas(0, -34).y;
  ctx.beginPath();
  ctx.moveTo(24, northLaneY); ctx.lineTo(CANVAS_WIDTH - 24, northLaneY);
  ctx.moveTo(24, southLaneY); ctx.lineTo(CANVAS_WIDTH - 24, southLaneY);
  ctx.stroke();
  ctx.setLineDash([]);

  // Lane directional arrows & labels
  ctx.fillStyle = 'rgba(148, 163, 184, 0.45)';
  ctx.font = 'bold 9px sans-serif';
  ctx.fillText('⬅ CULOAR NORD (SENS ÎNGUST)', CENTER_X - 75, northLaneY - 6);
  ctx.fillText('CULOAR SUD (SENS ÎNGUST) ➔', CENTER_X - 75, southLaneY + 14);
  ctx.restore();

  // Draw the 2 Central Concrete Traffic Islands & Safety Bollards
  PARKING_ISLANDS.forEach(isl => {
    const ic = worldToCanvas(isl.x, isl.y);
    const iw = isl.w * SCALE;
    const ih = isl.h * SCALE;

    ctx.save();
    // Drop shadow
    ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(ic.x - iw / 2 + 3, ic.y - ih / 2 + 3, iw, ih, 10);
    else ctx.rect(ic.x - iw / 2 + 3, ic.y - ih / 2 + 3, iw, ih);
    ctx.fill();

    // Concrete curb
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 2;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(ic.x - iw / 2, ic.y - ih / 2, iw, ih, 10);
    else ctx.rect(ic.x - iw / 2, ic.y - ih / 2, iw, ih);
    ctx.fill();
    ctx.stroke();

    // Inner greenery bed
    ctx.fillStyle = '#064e3b';
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(ic.x - iw / 2 + 6, ic.y - ih / 2 + 5, iw - 12, ih - 10, 6);
    else ctx.rect(ic.x - iw / 2 + 6, ic.y - ih / 2 + 5, iw - 12, ih - 10);
    ctx.fill();

    // Yellow safety bollards at left and right ends of island
    [-iw / 2 + 8, iw / 2 - 8].forEach(bx => {
      ctx.fillStyle = '#f59e0b';
      ctx.beginPath();
      ctx.arc(ic.x + bx, ic.y, 4.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#fef3c7';
      ctx.lineWidth = 1;
      ctx.stroke();
    });

    // Shrub in center of island
    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.arc(ic.x, ic.y, 5.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  });

  // Draw the 6 Parking Bays (P1..P6)
  const bayW = 48 * SCALE;
  const bayH = 52 * SCALE;

  parkingBays.forEach((bay, idx) => {
    const s = bay.spot;
    const sc = worldToCanvas(s.x, s.y);
    const isNorth = s.row === 'N';
    const left = sc.x - bayW / 2;
    const top = sc.y - bayH / 2;

    const isRobotHere = robotCurrentSpotIdx === idx;
    const isTargetFree = (
      (missionState === 'PARKING_FREEING_SPOT' || missionState === 'PARKING_NAV_TO_SPOT') &&
      targetFreeSpotIdx === idx
    );
    const isPrevVacated = (
      (missionState === 'PARKING_NAV_TO_SPOT' || missionState === 'PARKING_FILLING_OLD_SPOT') &&
      robotPrevSpotIdx === idx
    );

    ctx.save();

    // Bay floor highlight based on dynamic state
    if (isTargetFree) {
      ctx.fillStyle = 'rgba(34, 197, 94, 0.16)';
      ctx.strokeStyle = '#22c55e';
    } else if (isPrevVacated) {
      ctx.fillStyle = 'rgba(245, 158, 11, 0.15)';
      ctx.strokeStyle = '#f59e0b';
    } else if (isRobotHere) {
      ctx.fillStyle = 'rgba(56, 189, 248, 0.14)';
      ctx.strokeStyle = '#38bdf8';
    } else {
      ctx.fillStyle = 'rgba(15, 23, 42, 0.65)';
      ctx.strokeStyle = '#475569';
    }

    ctx.fillRect(left, top, bayW, bayH);

    // Painted U-shaped parking bay lines (open toward the driving lane)
    ctx.lineWidth = (isTargetFree || isPrevVacated || isRobotHere) ? 2.5 : 2;
    ctx.beginPath();
    if (isNorth) {
      // Open at bottom (south edge)
      ctx.moveTo(left, top + bayH);
      ctx.lineTo(left, top);
      ctx.lineTo(left + bayW, top);
      ctx.lineTo(left + bayW, top + bayH);
    } else {
      // Open at top (north edge)
      ctx.moveTo(left, top);
      ctx.lineTo(left, top + bayH);
      ctx.lineTo(left + bayW, top + bayH);
      ctx.lineTo(left + bayW, top);
    }
    ctx.stroke();

    // Concrete wheel stop block at the back of the bay
    const curbY = isNorth ? top + 8 : top + bayH - 13;
    ctx.fillStyle = '#334155';
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 1;
    ctx.fillRect(left + 14, curbY, bayW - 28, 5);
    ctx.strokeRect(left + 14, curbY, bayW - 28, 5);

    // Large Bay ID Watermark & Status Badge (placed clear of bottom banner)
    const labelColor = isTargetFree ? '#4ade80' : isPrevVacated ? '#fbbf24' : isRobotHere ? '#38bdf8' : '#cbd5e1';
    ctx.font = 'bold 11px sans-serif';
    let statusTag = s.id;
    if (isTargetFree) statusTag = `${s.id} · LIBER`;
    else if (isPrevVacated) statusTag = `${s.id} · LOC ANTERIOR`;
    else if (isRobotHere) statusTag = `${s.id} · ROARY`;
    else if (bay.car) statusTag = `${s.id} · OCUPAT`;

    const tw = ctx.measureText(statusTag).width;
    const labelY = isNorth ? top - 7 : top - 7;

    ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
    ctx.strokeStyle = labelColor;
    ctx.lineWidth = 1;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(sc.x - tw / 2 - 6, labelY - 10, tw + 12, 14, 4);
    else ctx.rect(sc.x - tw / 2 - 6, labelY - 10, tw + 12, 14);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = labelColor;
    ctx.fillText(statusTag, sc.x - tw / 2, labelY);

    ctx.restore();
  });
}

// ─── Draw Top-Down Cars (Static + Animated Departing/Arriving) ────────────────
function drawSingleCar(worldX, worldY, headingDeg, car, lightsActive = false) {
  const cc = worldToCanvas(worldX, worldY);
  const w = 31 * SCALE; // ~66 px wide
  const l = 40 * SCALE; // ~86 px long

  ctx.save();
  ctx.translate(cc.x, cc.y);
  ctx.rotate(toRadians(headingDeg));

  // Soft shadow
  ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(-w / 2 + 4, -l / 2 + 5, w, l, 10);
  else ctx.rect(-w / 2 + 4, -l / 2 + 5, w, l);
  ctx.fill();

  // 4 Tires
  ctx.fillStyle = '#090d16';
  const tireW = 5, tireL = 14;
  ctx.fillRect(-w / 2 - 2, -l / 2 + 10, tireW, tireL);
  ctx.fillRect(w / 2 - 3,  -l / 2 + 10, tireW, tireL);
  ctx.fillRect(-w / 2 - 2, l / 2 - 24,  tireW, tireL);
  ctx.fillRect(w / 2 - 3,  l / 2 - 24,  tireW, tireL);

  // Main car body
  ctx.fillStyle = car.color;
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(-w / 2, -l / 2, w, l, 10);
  else ctx.rect(-w / 2, -l / 2, w, l);
  ctx.fill();
  ctx.stroke();

  // Tinted Cabin / Windshields
  ctx.fillStyle = '#0f172a';
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(-w / 2 + 5, -l / 2 + 14, w - 10, l - 28, 6);
  else ctx.rect(-w / 2 + 5, -l / 2 + 14, w - 10, l - 28);
  ctx.fill();

  // Roof panel inside cabin
  ctx.fillStyle = car.roof || '#1e293b';
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(-w / 2 + 7, -l / 2 + 23, w - 14, l - 46, 4);
  else ctx.rect(-w / 2 + 7, -l / 2 + 23, w - 14, l - 46);
  ctx.fill();

  // Front Headlights (at -l/2)
  ctx.fillStyle = lightsActive ? '#fef08a' : '#e2e8f0';
  ctx.fillRect(-w / 2 + 6, -l / 2 + 1, 9, 3);
  ctx.fillRect(w / 2 - 15, -l / 2 + 1, 9, 3);

  // Rear Taillights / Reverse Glow (at +l/2)
  ctx.fillStyle = lightsActive ? '#ff4d4d' : '#991b1b';
  ctx.fillRect(-w / 2 + 6, l / 2 - 4, 10, 3);
  ctx.fillRect(w / 2 - 16, l / 2 - 4, 10, 3);

  // Keep license plate badge upright regardless of car orientation
  ctx.rotate(-toRadians(headingDeg));
  ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
  ctx.fillRect(-20, -6, 40, 12);
  ctx.fillStyle = '#f8fafc';
  ctx.font = 'bold 7.5px monospace';
  const plate = car.plate || 'AUTO';
  const pw = ctx.measureText(plate).width;
  ctx.fillText(plate, -pw / 2, 2.5);

  ctx.restore();
}

function drawParkingCars() {
  ctx.save();
  if (ctx.rect && ctx.clip) {
    ctx.beginPath();
    ctx.rect(11, 11, CANVAS_WIDTH - 22, CANVAS_HEIGHT - 22);
    ctx.clip();
  }

  // 1) Draw static parked cars
  parkingBays.forEach((bay, idx) => {
    if (!bay.car) return;
    if (departingCar && departingCar.spotIdx === idx) return; // drawn via departingCar animation
    const s = bay.spot;
    drawSingleCar(s.x, s.y, s.heading, bay.car, false);
  });

  // 2) Draw departing car (smoothly pulling out into the lane and exiting)
  if (departingCar) {
    const s = PARKING_SPOTS[departingCar.spotIdx];
    const p = departingCar.progress;
    const laneY = s.row === 'N' ? 34 : -34;
    const exitX = s.x >= 0 ? 185 : -185;
    let cx, cy, ch;
    if (p < 0.45) {
      // Phase 1: Pull out of bay into driving lane
      const k = p / 0.45;
      cx = s.x;
      cy = s.y + (laneY - s.y) * k;
      ch = s.heading;
    } else {
      // Phase 2: Turn and drive along lane toward exit
      const k = (p - 0.45) / 0.55;
      cx = s.x + (exitX - s.x) * k;
      cy = laneY;
      const targetH = exitX > s.x ? 90 : -90;
      ch = s.heading + (targetH - s.heading) * Math.min(1, k * 2.2);
    }
    drawSingleCar(cx, cy, ch, departingCar.car, true);
  }

  // 3) Draw arriving car (entering from access lane into Roary's previous spot!)
  if (arrivingCar) {
    const s = PARKING_SPOTS[arrivingCar.spotIdx];
    const p = arrivingCar.progress;
    const laneY = s.row === 'N' ? 34 : -34;
    const entryX = s.x <= 0 ? -185 : 185;
    let cx, cy, ch;
    if (p < 0.55) {
      // Phase 1: Drive in from lot entrance along lane
      const k = p / 0.55;
      cx = entryX + (s.x - entryX) * k;
      cy = laneY;
      ch = s.x > entryX ? 90 : -90;
    } else {
      // Phase 2: Turn into bay and park in Roary's old spot
      const k = (p - 0.55) / 0.45;
      cx = s.x;
      cy = laneY + (s.y - laneY) * k;
      const startH = s.x > entryX ? 90 : -90;
      ch = startH + (s.heading - startH) * k;
    }
    drawSingleCar(cx, cy, ch, arrivingCar.car, true);
  }

  ctx.restore();
}

// ─── Draw Floor & Ambient Layout (Courier Scenario) ───────────────────────────
function drawRoomEnvironment() {
  ctx.fillStyle = '#0f1726';
  ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  ctx.strokeStyle = '#182336';
  ctx.lineWidth = 1;
  const tileW = 35 * SCALE;
  for (let x = 0; x < CANVAS_WIDTH; x += tileW) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, CANVAS_HEIGHT); ctx.stroke();
  }
  for (let y = 0; y < CANVAS_HEIGHT; y += tileW) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(CANVAS_WIDTH, y); ctx.stroke();
  }

  ctx.strokeStyle = '#27354d';
  ctx.lineWidth = 3;
  ctx.strokeRect(10, 10, CANVAS_WIDTH - 20, CANVAS_HEIGHT - 20);

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

// ─── Draw Catmull-Rom Trajectory Spline & Waypoint Nodes ─────────────────────
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

  const uniqueWaypoints = [];
  for (let i = 0; i < CURRENT_SPLINE.length; i++) {
    const pt = CURRENT_SPLINE[i].target;
    const isDuplicate = uniqueWaypoints.some(u => Math.hypot(u.x - pt.x, u.y - pt.y) < 1.5);
    if (!isDuplicate) {
      uniqueWaypoints.push(pt);
    }
  }

  const total = uniqueWaypoints.length;
  uniqueWaypoints.forEach((wp, idx) => {
    const c = worldToCanvas(wp.x, wp.y);
    const isStart = idx === 0;
    const isEnd = idx === total - 1;

    const strokeColor = isStart ? '#38bdf8' : isEnd ? '#22c55e' : '#60a5fa';
    const fillColor = isStart ? 'rgba(56, 189, 248, 0.25)' : isEnd ? 'rgba(34, 197, 94, 0.3)' : 'rgba(96, 165, 250, 0.2)';

    ctx.fillStyle = fillColor;
    ctx.beginPath();
    ctx.arc(c.x, c.y, isEnd ? 9 : 7, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = isEnd ? 2.2 : 1.8;
    ctx.beginPath();
    ctx.arc(c.x, c.y, isEnd ? 9 : 7, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(c.x, c.y, 3, 0, Math.PI * 2);
    ctx.fill();

    if (wp.heading !== undefined) {
      const hRad = toRadians(wp.heading);
      const dirX = Math.sin(hRad);
      const dirY = -Math.cos(hRad);
      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(c.x, c.y);
      ctx.lineTo(c.x + dirX * 14, c.y + dirY * 14);
      ctx.stroke();
    }

    const label = isStart ? 'START' : isEnd ? 'DEST' : `W${idx + 1}`;
    ctx.font = 'bold 8px sans-serif';
    const textW = ctx.measureText(label).width;
    const labelX = c.x + 10;
    const labelY = c.y - 6;

    ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 1;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(labelX - 3, labelY - 8, textW + 6, 12, 3);
    else ctx.rect(labelX - 3, labelY - 8, textW + 6, 12);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = isEnd ? '#4ade80' : '#e2e8f0';
    ctx.fillText(label, labelX, labelY + 1);
  });

  ctx.restore();
}

// ─── Draw Bookshelf / Library Storage ─────────────────────────────────────────
function drawBookshelf() {
  const shelfPose = POSES.shelf();
  const sc = worldToCanvas(shelfPose.x, shelfPose.y);

  const shelfLeft = 14;
  const shelfTop = sc.y - 120;
  const shelfW = 94;
  const shelfH = 240;

  ctx.save();

  ctx.fillStyle = 'rgba(0, 0, 0, 0.55)';
  ctx.fillRect(shelfLeft + 5, shelfTop + 5, shelfW, shelfH);

  ctx.fillStyle = '#3a2114';
  ctx.fillRect(shelfLeft, shelfTop, shelfW, shelfH);
  ctx.strokeStyle = '#5c3521';
  ctx.lineWidth = 3;
  ctx.strokeRect(shelfLeft, shelfTop, shelfW, shelfH);

  ctx.fillStyle = '#24140b';
  ctx.fillRect(shelfLeft + 6, shelfTop + 6, shelfW - 12, shelfH - 12);

  const tierCount = 4;
  const tierH = (shelfH - 16) / tierCount;
  ctx.fillStyle = '#4a2b1a';
  ctx.strokeStyle = '#6a3e26';
  ctx.lineWidth = 1.5;
  for (let t = 1; t < tierCount; t++) {
    const sy = shelfTop + 8 + t * tierH;
    ctx.fillRect(shelfLeft + 6, sy, shelfW - 12, 6);
    ctx.strokeRect(shelfLeft + 6, sy, shelfW - 12, 6);
  }

  const topTierY = shelfTop + 8;
  const encColors = ['#1e3a8a', '#1e40af', '#1d4ed8', '#0369a1', '#0e7490', '#155e75', '#0f766e'];
  encColors.forEach((col, i) => {
    ctx.fillStyle = col;
    ctx.fillRect(shelfLeft + 9 + i * 11, topTierY + 4, 9, tierH - 12);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 0.5;
    ctx.strokeRect(shelfLeft + 9 + i * 11, topTierY + 4, 9, tierH - 12);
  });

  shelfBooks.forEach((b, idx) => {
    const bookY = shelfTop + 8 + (idx + 1) * tierH - (tierH - 10);

    ctx.fillStyle = 'rgba(0, 0, 0, 0.4)';
    ctx.fillRect(shelfLeft + 12, bookY + 3, shelfW - 24, tierH - 12);

    ctx.fillStyle = b.color;
    ctx.fillRect(shelfLeft + 10, bookY, shelfW - 24, tierH - 14);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.strokeRect(shelfLeft + 10, bookY, shelfW - 24, tierH - 14);

    ctx.fillStyle = 'rgba(255, 255, 255, 0.25)';
    ctx.fillRect(shelfLeft + 10, bookY + 2, 6, tierH - 18);

    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 9px sans-serif';
    ctx.fillText(`Vol.${b.id} ${b.title || ''}`, shelfLeft + 19, bookY + (tierH - 14) / 2 + 3);
  });

  const padX = shelfLeft + shelfW + 2;
  const padY = sc.y - 35;
  const padW = 34;
  const padH = 70;
  ctx.strokeStyle = 'rgba(245, 158, 11, 0.85)';
  ctx.fillStyle = 'rgba(245, 158, 11, 0.15)';
  ctx.lineWidth = 1.8;
  ctx.setLineDash([4, 3]);
  ctx.strokeRect(padX, padY, padW, padH);
  ctx.fillRect(padX, padY, padW, padH);
  ctx.setLineDash([]);

  ctx.fillStyle = '#fbbf24';
  ctx.font = 'bold 8px sans-serif';
  ctx.fillText('ZONĂ PRELUARE', padX + 2, padY - 5);

  ctx.fillStyle = '#f59e0b';
  ctx.font = 'bold 11px sans-serif';
  ctx.fillText('📚 RAFT CENTRAL CĂRȚI', shelfLeft, shelfTop - 8);

  ctx.restore();
}

// ─── Draw Desk & Client Person ────────────────────────────────────────────────
function drawDeskAndClient() {
  const deskPose = POSES.desk();
  const dc = worldToCanvas(deskPose.x, deskPose.y);

  const deskX = dc.x + 38;
  const deskY = dc.y - 85;
  const deskW = 96;
  const deskH = 170;

  ctx.save();

  ctx.fillStyle = 'rgba(0, 0, 0, 0.55)';
  ctx.fillRect(deskX + 6, deskY + 6, deskW, deskH);

  ctx.fillStyle = '#5c331c';
  ctx.fillRect(deskX, deskY, deskW, deskH);
  ctx.strokeStyle = '#854d2b';
  ctx.lineWidth = 3;
  ctx.strokeRect(deskX, deskY, deskW, deskH);

  ctx.strokeStyle = '#3d2010';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(deskX + 4, deskY + 4, deskW - 8, deskH - 8);

  ctx.fillStyle = '#1c1917';
  ctx.strokeStyle = '#44403c';
  ctx.lineWidth = 1;
  ctx.fillRect(deskX + 16, deskY + 28, deskW - 32, deskH - 56);
  ctx.strokeRect(deskX + 16, deskY + 28, deskW - 32, deskH - 56);

  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = '#475569';
  ctx.lineWidth = 1.2;
  ctx.fillRect(deskX + 32, deskY + 48, 28, 22);
  ctx.strokeRect(deskX + 32, deskY + 48, 28, 22);
  ctx.fillStyle = '#0284c7';
  ctx.fillRect(deskX + 34, deskY + 50, 24, 18);

  ctx.fillStyle = '#0f172a';
  ctx.fillRect(deskX + 32, deskY + 76, 26, 36);
  ctx.strokeStyle = '#475569';
  ctx.strokeRect(deskX + 32, deskY + 76, 26, 36);
  ctx.fillStyle = '#10b981';
  ctx.fillRect(deskX + 34, deskY + 78, 22, 32);

  ctx.fillStyle = '#94a3b8';
  ctx.fillRect(deskX + 22, deskY + 62, 7, 24);
  ctx.beginPath();
  ctx.arc(deskX + 25, deskY + 96, 3, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#f8fafc';
  ctx.beginPath();
  ctx.arc(deskX + 22, deskY + 18, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#94a3b8';
  ctx.stroke();
  ctx.fillStyle = '#78350f';
  ctx.beginPath();
  ctx.arc(deskX + 22, deskY + 18, 3, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#eab308';
  ctx.beginPath();
  ctx.arc(deskX + 75, deskY + 18, 6, 0, Math.PI * 2);
  ctx.fill();
  const lampGlow = ctx.createRadialGradient(deskX + 75, deskY + 18, 4, deskX + 75, deskY + 18, 28);
  lampGlow.addColorStop(0, 'rgba(234, 179, 8, 0.35)');
  lampGlow.addColorStop(1, 'rgba(234, 179, 8, 0)');
  ctx.fillStyle = lampGlow;
  ctx.beginPath();
  ctx.arc(deskX + 75, deskY + 18, 28, 0, Math.PI * 2);
  ctx.fill();

  const trayX = dc.x + 8;
  const trayY = dc.y - 35;
  const trayW = 28;
  const trayH = 70;
  ctx.fillStyle = 'rgba(34, 197, 94, 0.16)';
  ctx.strokeStyle = '#22c55e';
  ctx.lineWidth = 2;
  ctx.fillRect(trayX, trayY, trayW, trayH);
  ctx.strokeRect(trayX, trayY, trayW, trayH);

  ctx.fillStyle = '#22c55e';
  ctx.font = 'bold 9px sans-serif';
  ctx.fillText('📥 INBOX', trayX + 2, trayY - 6);

  const maxVisible = 6;
  const visibleBooks = deskDeliveredBooks.slice(-maxVisible);
  visibleBooks.forEach((b, i) => {
    ctx.fillStyle = b.color;
    ctx.fillRect(trayX + 3, trayY + 5 + i * 10, trayW - 6, 8);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 0.6;
    ctx.strokeRect(trayX + 3, trayY + 5 + i * 10, trayW - 6, 8);
  });

  if (deskDeliveredBooks.length > 0) {
    ctx.fillStyle = '#4ade80';
    ctx.font = 'bold 9px sans-serif';
    ctx.fillText(`${deskDeliveredBooks.length} Cărți`, trayX, trayY + trayH + 12);
  }

  const px = deskX + deskW + 24;
  const py = dc.y;

  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(px, py, 18, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = '#0284c7';
  ctx.beginPath();
  ctx.arc(px, py, 13, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#fed7aa';
  ctx.beginPath();
  ctx.arc(px, py, 8, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#451a03';
  ctx.beginPath();
  ctx.arc(px, py - 3, 8, Math.PI, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#38bdf8';
  ctx.font = 'bold 11px sans-serif';
  ctx.fillText('💼 BIROU CLIENT', deskX + 4, deskY - 8);

  if (clientBubble) {
    const bubbleX = deskX + deskW / 2 - 10;
    const bubbleY = deskY - 28;
    ctx.font = 'bold 9px sans-serif';
    const textW = ctx.measureText(clientBubble).width;

    ctx.fillStyle = 'rgba(15, 23, 42, 0.94)';
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.6)';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(bubbleX - textW / 2, bubbleY - 14, textW + 18, 22, 6);
    else ctx.rect(bubbleX - textW / 2, bubbleY - 14, textW + 18, 22);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#ffffff';
    ctx.fillText(clientBubble, bubbleX - textW / 2 + 9, bubbleY);
  }

  ctx.restore();
}

// ─── Draw Room Obstacles (Lounge Table, Armchair, Plant) ───────────────────────
function drawObstacles() {
  ctx.save();

  const tc = worldToCanvas(-10, -15);
  ctx.fillStyle = 'rgba(0,0,0,0.3)';
  ctx.beginPath(); ctx.arc(tc.x + 3, tc.y + 3, 24, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = '#1e293b';
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(tc.x, tc.y, 24, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

  ctx.fillStyle = '#10b981';
  ctx.beginPath(); ctx.arc(tc.x, tc.y, 7, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#059669';
  ctx.beginPath(); ctx.arc(tc.x + 4, tc.y - 3, 5, 0, Math.PI * 2); ctx.fill();

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

// ─── Draw Robot (High Polish with 4 Mecanum Wheels, Safety Ring & Basket) ─────
function drawRobot() {
  const rc = worldToCanvas(drive.X, drive.Y);
  const H = drive.H;

  const w = 32 * SCALE;
  const l = 32 * SCALE;

  ctx.save();
  ctx.translate(rc.x, rc.y);

  // In Parking Lot scenario, draw 360° Safety Proximity LiDAR Halo around chassis
  if (currentScenario === 'parking') {
    const isMoving = drive.vel.radius() > 3.0;
    ctx.save();
    ctx.strokeStyle = isMoving ? 'rgba(34, 197, 94, 0.45)' : 'rgba(56, 189, 248, 0.3)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.arc(0, 0, w * 0.76, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
  }

  ctx.rotate(H);

  // 1) Soft drop shadow underneath chassis
  ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
  ctx.beginPath();
  ctx.roundRect(-w / 2 + 3, -l / 2 + 5, w, l, 8);
  ctx.fill();

  // 2) Four Mecanum Wheels at the 4 corners with visible 45° angled rollers
  const wheelW = 9;
  const wheelL = 20;
  const wheelOffsets = [
    { x: -w / 2 - wheelW / 2 + 1, y: -l / 2 + wheelL / 2 - 2, angle: Math.PI / 4 },
    { x: w / 2 + wheelW / 2 - 1,  y: -l / 2 + wheelL / 2 - 2, angle: -Math.PI / 4 },
    { x: -w / 2 - wheelW / 2 + 1, y: l / 2 - wheelL / 2 + 2,  angle: -Math.PI / 4 },
    { x: w / 2 + wheelW / 2 - 1,  y: l / 2 - wheelL / 2 + 2,  angle: Math.PI / 4 },
  ];

  wheelOffsets.forEach((wo) => {
    ctx.save();
    ctx.translate(wo.x, wo.y);
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1.2;
    ctx.fillRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);
    ctx.strokeRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);

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
  const armExtPx = armExtension * 45;
  ctx.save();
  ctx.strokeStyle = '#94a3b8';
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  ctx.moveTo(-7, -l / 2);
  ctx.lineTo(-7, -l / 2 - armExtPx);
  ctx.moveTo(7, -l / 2);
  ctx.lineTo(7, -l / 2 - armExtPx);
  ctx.stroke();

  ctx.fillStyle = '#0284c7';
  ctx.fillRect(-11, -l / 2 - armExtPx - 3, 22, 5);

  ctx.strokeStyle = armGrabbing ? '#22c55e' : '#38bdf8';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-11, -l / 2 - armExtPx);
  ctx.lineTo(-11, -l / 2 - armExtPx - 7);
  ctx.moveTo(11, -l / 2 - armExtPx);
  ctx.lineTo(11, -l / 2 - armExtPx - 7);
  ctx.stroke();

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
                    (missionState === 'DELIVER_TO_DESK' || missionState === 'PARKING_FILLING_OLD_SPOT') ? '#22c55e' :
                    (missionState === 'PICKUP_BOOKS' || missionState === 'PARKING_FREEING_SPOT') ? '#f59e0b' : '#38bdf8';
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

  // 5) Top Deck Carrier / LiDAR Dome
  const basketW = w - 16;
  const basketH = l - 22;
  ctx.fillStyle = 'rgba(30, 41, 59, 0.85)';
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.roundRect(-basketW / 2, -basketH / 2 + 4, basketW, basketH, 4);
  ctx.fill();
  ctx.stroke();

  if (currentScenario === 'parking') {
    // Draw RX7 Autonomous Navigation Core badge upright on top deck
    ctx.save();
    ctx.rotate(-H);
    ctx.fillStyle = '#38bdf8';
    ctx.font = 'bold 9px sans-serif';
    ctx.fillText('ROARY X7', -22, 2);
    ctx.fillStyle = '#4ade80';
    ctx.font = 'bold 7px monospace';
    ctx.fillText('SAFE-NAV', -18, 12);
    ctx.restore();
  } else {
    basketCargo.forEach((b, idx) => {
      ctx.fillStyle = b.color;
      const by = -basketH / 2 + 8 + idx * 11;
      ctx.fillRect(-basketW / 2 + 4, by, basketW - 8, 8);
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 0.5;
      ctx.strokeRect(-basketW / 2 + 4, by, basketW - 8, 8);
    });
  }

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

  joyCtx.fillStyle = '#0f172a';
  joyCtx.strokeStyle = '#1e293b';
  joyCtx.lineWidth = 2;
  joyCtx.beginPath();
  joyCtx.arc(cx, cy, maxR, 0, Math.PI * 2);
  joyCtx.fill();
  joyCtx.stroke();

  joyCtx.strokeStyle = '#334155';
  joyCtx.lineWidth = 1;
  joyCtx.beginPath();
  joyCtx.moveTo(cx, cy - maxR + 4); joyCtx.lineTo(cx, cy + maxR - 4);
  joyCtx.moveTo(cx - maxR + 4, cy); joyCtx.lineTo(cx + maxR - 4, cy);
  joyCtx.stroke();

  const tx = cx + joyX * maxR;
  const ty = cy - joyY * maxR;

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
syncScenarioStaticLabels();
updateCargoUI();
requestAnimationFrame(frame);

// Auto-launch the active scenario after 600ms so the sponsor immediately sees the robot in motion!
setTimeout(() => {
  if (!missionActive && missionState === 'STANDBY') {
    startMission();
  }
}, 600);
