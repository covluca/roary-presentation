// RoaryX7 Web Demo — Interactive Sponsor Presentation & 2D Holonomic Showcase.
//
// Emulates the real runOpMode() loop (doritoPower/Roaryx7.java + autonom/Kirby/KirbyAirRiders.java)
// and the non-blocking subsystem managers (Sensors, Intake, OutTake, Farul status LED):
//
//   mainDrive(joyX, -joyY, joyR)                     (always — stick → mainDrivePowers)
//   if (DXEnabled)  splineToDX(TARGET, vel)          (→ sendPower → motors = goTo·goToK + mainDrivePowers)
//   if (splineDone) goToV2(lastTarget, …)            (final approach → same composition)
//   sensors.updateSlots(loopNr % 3)                  (round-robin 3-slot color/distance polling)
//   intake.unloadManager() / launchControl() / Unstuck() / ocProtection()
//   outTake.step() / statusCode()                    (slides, bascula, Farul LED)
//   updatePowerCompensator()

'use strict';

// ─── Arena & Physics Constants ───────────────────────────────────────────────
const FIELD_SIZE = 365;                        // cm (FTC field ≈ 12 ft × 12 ft, -182.5..+182.5)
const ROBOT_SIZE = 28;                         // cm visual chassis footprint
const CANVAS_SIZE = 640;
const FIELD_PADDING = 28;                      // px margin around field walls
const CENTER = CANVAS_SIZE / 2;
const SCALE = (CANVAS_SIZE - FIELD_PADDING * 2) / FIELD_SIZE; // ~1.60 px per cm (fills canvas!)
const DT = 1 / 60;                             // fixed 60 Hz sim timestep (s)
const MAX_SPEED = 92;                          // cm/s at |power| = 1 (responsive 2D traversal)
const MAX_ROT = 3.2;                           // rad/s at |rotate| = 1

// ─── Real KirbyAirRiders waypoints (autonom/Kirby/KirbyAirRiders.java:62-86) ─
const WP_LIBRARY = {
  launch_first:       () => new goToValues(new Pose2d(-35, -124, -70), 0.71, 3),
  launch:             () => new goToValues(new Pose2d(-53, -145, -90), 0.62, 3),
  artifact_3_sliding: () => new goToValues(new Pose2d(-50, -110, -66), 0.71, 3),
  artifact_3_facing:  () => new goToValues(new Pose2d(-120, -90, -90), 0.68, 3),
  artifact_3:         () => new goToValues(new Pose2d(-152, -94, -93), 0.58, 3),
  artifact_2_facing:  () => new goToValues(new Pose2d(-80, -38, -93), 0.71, 3),
  artifact_2:         () => new goToValues(new Pose2d(-155, -38, -93), 0.71, 3),
  launch_close:       () => new goToValues(new Pose2d(-45, 26, -90), 0.68, 6.4),
  park:               () => new goToValues(new Pose2d(-45, 130, 0), 0.58, 3),
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

function circuitSplineTarget() {
  return [
    new goToValues(new Pose2d(-125, -120, -45), 0.78, 3.5),
    new goToValues(new Pose2d(-125, -120, -45), 0.78, 3.5),
    new goToValues(new Pose2d(-110, 45, 0), 0.82, 3.5),
    new goToValues(new Pose2d(0, 115, 55), 0.80, 3.5),
    new goToValues(new Pose2d(125, 35, 110), 0.82, 3.5),
    new goToValues(new Pose2d(95, -115, 160), 0.75, 3.5),
    new goToValues(new Pose2d(95, -115, 160), 0.75, 3.5),
  ];
}

function holonomicSpinTarget() {
  return [
    new goToValues(new Pose2d(-135, -115, -90), 0.72, 3),
    new goToValues(new Pose2d(-135, -115, -90), 0.72, 3),
    new goToValues(new Pose2d(-45, -35, 0), 0.76, 3),
    new goToValues(new Pose2d(45, 45, 90), 0.76, 3),
    new goToValues(new Pose2d(130, 115, 175), 0.68, 3),
    new goToValues(new Pose2d(130, 115, 175), 0.68, 3),
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
let activeChapter = 1;             // 1..5 presentation chapter

// Patrol / continuous presentation timers & visual effects
let parkedSince = null;
let demoNudgeUntil = 0;
let demoNudgeVec = { x: 0, y: 0, r: 0 };
let pushEffects = [];              // [{ x, y, t, label }] visual ripples on canvas
let highlightedElement = null;     // field element targeted by clawToRobotPos
let canvasBannerText = 'Cap. 1 · Șasiu Holonomic: Fuziune live Autonom + Pilot pe toate cele 4 roți';

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
    { x: -136, y: -93, color: inIndexer.PURPLE, collected: false },
    { x: -148, y: -95, color: inIndexer.GREEN,  collected: false },
    { x: -160, y: -91, color: inIndexer.PURPLE, collected: false },
    // Spike 2 cluster (around artifact_2: -155, -38)
    { x: -135, y: -38, color: inIndexer.PURPLE, collected: false },
    { x: -147, y: -35, color: inIndexer.GREEN,  collected: false },
    { x: -159, y: -41, color: inIndexer.PURPLE, collected: false },
    // Spike 1 cluster (top-left)
    { x: -135, y: 29,  color: inIndexer.GREEN,  collected: false },
    { x: -148, y: 32,  color: inIndexer.PURPLE, collected: false },
    // Center-right interactive elements for Offset IK & Click-to-Grab
    { x: 55,   y: -45, color: inIndexer.GREEN,  collected: false },
    { x: 85,   y: -15, color: inIndexer.PURPLE, collected: false },
    { x: 60,   y: 35,  color: inIndexer.GREEN,  collected: false },
  ];
  outTake.scoredDrops = [];
  highlightedElement = null;
}
spawnFieldElements();

// Odometry trail
const trail = [];
const MAX_TRAIL = 280;

// ─── Stick & Keyboard Input ──────────────────────────────────────────────────
let joyX = 0, joyY = 0, joyR = 0, joyActive = false;

const keys = Object.create(null);
window.addEventListener('keydown', (e) => {
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
const ovLoop = document.getElementById('ov-loop');
const ovLookahead = document.getElementById('ov-lookahead');

// ─── Chapter Pitch Content (5 Interactive Presentation Slides) ───────────────
const CHAPTER_PITCHES = {
  1: {
    tag: 'CAPITOLUL 1 DIN 5 · ARHITECTURA HOLONOMICĂ',
    title: 'De ce Roary pe un Șasiu Holonomic?',
    lead: 'Spre deosebire de roboții clasici care trebuie să se oprească pentru a vira, un <b>șasiu holonomic</b> condus de <b>Roary</b> se deplasează omnidirecțional (înainte, lateral și rotație simultan), în timp ce pilotul și algoritmul autonom colaborează în timp real pe aceleași 4 roți.',
    bullets: [
      '<b>Mișcare Omnidirecțională (3 Grade de Libertate):</b> Robotul translatează în orice direcție în timp ce își rotește șasiul independent către țintă.',
      '<b>Fuziune Live (<code>Motor = Autonom + Pilot</code>):</b> Autonomia nu blochează niciodată controlul manual. Pilotul poate interveni cu <b>WASD / Joystick</b> pentru a ocoli un obstacol din mers, iar Roary îl readuce lin pe curbă.',
      '<b>Experimentează acum:</b> Apasă butonul de mai jos sau folosește tastele <b>WASD + Q/E</b> în timp ce robotul urmărește traseul!',
    ],
    banner: 'Cap. 1 · Fuziune Live: Pilotul și autonomia controlează simultan cele 4 roți holonomice',
    actions: [
      { id: 'act-nudge', label: '👋 Simulează Ocolire Obstacol (Intervenție Pilot)', cls: 'cta-btn', fn: () => triggerPilotNudgeDemo() },
      { id: 'act-push',  label: '💥 Simulează Împingere Laterală pe Curbă', cls: 'cta-btn secondary', fn: () => triggerPushPerturbation() },
    ],
  },
  2: {
    tag: 'CAPITOLUL 2 DIN 5 · NAVIGAȚIE CONTINUĂ PE CURBE',
    title: 'Traiectorii Catmull-Rom & Cei 3 Vectori de Control',
    lead: 'Roary elimină mișcările sacadate de tip „oprește-rotește-mergi”. La fiecare cadru (60 Hz), controlerul evaluează o curbă continuă <b>Catmull-Rom</b> și compune <b>3 vectori fizici de ghidare</b>:',
    bullets: [
      '🟢 <b>Propulsie Tangentă (Înaintare):</b> Împinge robotul de-a lungul direcției curbei, adaptând viteza automat.',
      '🔴 <b>Corecție Normală (Readucere pe Traseu):</b> Dacă robotul este împins sau deviat, vectorul roșu îl trage imediat înapoi pe linia ideală.',
      '🟣 <b>Anticipare Centripetă (Viraje de Mare Viteză):</b> Citește curba în avans pentru a compensa forța centrifugă în viraje strânse, urmată de frânare de precizie la destinație.',
    ],
    banner: 'Cap. 2 · Control pe Curbă: Trage de punctele de pe teren sau testează o perturbație laterală!',
    actions: [
      { id: 'act-push2',   label: '💥 Simulează Împingere Laterală (Vezi Vectorul Roșu)', cls: 'cta-btn', fn: () => triggerPushPerturbation() },
      { id: 'act-circuit', label: '∞ Încarcă Circuit Amplu', cls: 'cta-btn secondary', fn: () => loadPathPreset('circuit') },
      { id: 'act-spin',    label: '↻ Translație + Piruetă 180°', cls: 'cta-btn secondary', fn: () => loadPathPreset('spin') },
    ],
  },
  3: {
    tag: 'CAPITOLUL 3 DIN 5 · CINEMATICĂ DE OFFSET 2D',
    title: 'Poziționarea Vârfului de Mecanism, nu doar a Centrului',
    lead: 'În competiție, nu centrul robotului colectează sau plasează obiectele, ci <b>glisiera frontală cu gheară</b> și <b>glisiera spate</b> care se extind în afara șasiului. Roary include cinematică inversă de offset pentru ambele mecanisme.',
    bullets: [
      '<b>Calcul Automat al Țintei Șasiului:</b> Funcția <code>clawToRobotPos</code> calculează exact unde trebuie să se oprească centrul șasiului și sub ce unghi, astfel încât vârful ghearei extinse să cadă milimetric pe obiect.',
      '<b>Click Direct pe Obiectele de pe Teren:</b> Dă click pe orice bilă <b>Verde</b> sau <b>Mov</b> de pe teren sau apasă butonul de mai jos!',
      '<b>Glisiere Live în 2D:</b> Ajustează extensia glisierei față (0–37 cm) și spate (0–18 cm) și urmărește poziția lor pe teren.',
    ],
    banner: 'Cap. 3 · Cinematică Offset: Dă click pe orice obiect verde/mov de pe teren pentru a trimite gheara!',
    actions: [
      { id: 'act-snap',   label: '🎯 Trimite Gheara la Cel Mai Apropiat Obiect', cls: 'cta-btn', fn: () => snapClawToNearestElement() },
      { id: 'act-unload', label: '📦 Transferă & Plasează cu Glisiera Spate', cls: 'cta-btn secondary', fn: () => triggerUnloadManager() },
    ],
  },
  4: {
    tag: 'CAPITOLUL 4 DIN 5 · SUBSISTEME & STATE-MANAGEMENT',
    title: 'Manageri de Subsisteme Non-Blocking & Auto-Protecție',
    lead: 'Toate mecanismele robotului (Colector, Magazie cu 3 Sloturi, Senzori de Culoare, Glisiere și LED-ul de stare „Farul”) sunt conduse de <b>automate cu stări (State Machines)</b> care rulează în paralel la 60 Hz, fără a bloca vreodată șasiul.',
    bullets: [
      '<b>Indexor cu 3 Sloturi & Senzori Optici:</b> Detectează culoarea obiectelor colectate (Verde / Mov), le distribuie în cele 3 sloturi vizibile pe șasiu și respinge automat surplusul când magazia e plină (3/3).',
      '<b>Protecție Automată la Blocaj (>7.1A):</b> La creșterea curentului în motorul de colectare, Roary inversează automat rolele și ciclul paletelor pentru deblocare (<i>Auto-Unstuck</i>).',
      '<b>Secvență de Transfer în 6 Pași:</b> Coordonează retragerea colectorului, extensia glisierei spate și eliberarea elementelor pe teren.',
    ],
    banner: 'Cap. 4 · Subsisteme Active: Colectare în 3 sloturi, detecție culoare și protecție automată la blocaj',
    actions: [
      { id: 'act-jam',     label: '⚡ Simulează Blocaj (>7.1A Auto-Deblocare)', cls: 'cta-btn', fn: () => triggerJamDemo() },
      { id: 'act-unload4', label: '📦 Execută Secvența de Transfer (6 Pași)', cls: 'cta-btn secondary', fn: () => triggerUnloadManager() },
      { id: 'act-sweep',   label: '🟢 Reia Tura de Colectare pe Teren', cls: 'cta-btn secondary', fn: () => selectChapter(4) },
    ],
  },
  5: {
    tag: 'CAPITOLUL 5 DIN 5 · DEMONSTRAȚIE CAP-COADĂ',
    title: 'Misiune Autonomă Completă de Competiție',
    lead: 'Toate straturile arhitecturii Roary lucrează împreună într-un ciclu autonom complet, coordonat de automatul principal de stări al robotului:',
    bullets: [
      '<b>1. Lansare Inițială:</b> Deplasare din start către zona de scor și plasarea primelor 3 elemente în ordinea motivului.',
      '<b>2. Colectare Dinamică + Oscilație „Tremor”:</b> Navigație pe curbă către grupul de obiecte, pornirea automată a colectorului și oscilație fină a șasiului pentru preluarea rapidă în cele 3 sloturi.',
      '<b>3. Revenire la Scor & Parcare:</b> Întoarcere pe spline în zona de lansare, plasarea elementelor colectate și parcare de precizie cu <code>goToV2</code>.',
    ],
    banner: 'Cap. 5 · Misiune Autonomă: START → SCOR → COLECTARE (+tremor) → SCOR → PARCARE',
    actions: [
      { id: 'act-restart-fsm', label: '▶ Reia Misiunea Autonomă de la Început', cls: 'cta-btn', fn: () => startFullAutoMission() },
      { id: 'act-nudge-fsm',   label: '👋 Intervine cu Pilotul în Timpul Misiunii', cls: 'cta-btn secondary', fn: () => triggerPilotNudgeDemo() },
    ],
  },
};

function renderPitchCard(chNum) {
  const data = CHAPTER_PITCHES[chNum] || CHAPTER_PITCHES[1];
  const badgeEl = document.getElementById('pitch-ch-badge');
  const titleEl = document.getElementById('pitch-title');
  const leadEl = document.getElementById('pitch-lead');
  const bulletsEl = document.getElementById('pitch-bullets');
  const actionsEl = document.getElementById('pitch-actions');

  if (badgeEl) badgeEl.textContent = data.tag;
  if (titleEl) titleEl.textContent = data.title;
  if (leadEl) leadEl.innerHTML = data.lead;
  if (bulletsEl) {
    bulletsEl.innerHTML = data.bullets.map(b => `<li>${b}</li>`).join('');
  }
  if (actionsEl) {
    actionsEl.innerHTML = '';
    data.actions.forEach(act => {
      const b = document.createElement('button');
      b.className = act.cls;
      b.id = act.id;
      b.textContent = act.label;
      b.onclick = act.fn;
      actionsEl.appendChild(b);
    });
  }
  canvasBannerText = data.banner;
}

// ─── Helper: aligned world pose for robotToClawPos / robotToOutTakePos ───────
// In Roaryx7.java, robotToClawPos / robotToOutTakePos use (-heading - 90) and (-heading + 90).
// Passing (getH() - 180) aligns +forward with the 2D canvas arrow (sin H, cos H) across all 360°.
function getAlignedMechanismPose() {
  return new Pose2d(drive.X, drive.Y, drive.getH() - 180);
}

// ─── Auto toggle (goToK) & Full FSM toggle ───────────────────────────────────
function refreshAutoButton() {
  const on = drive.goToK === 1;
  btnStartAuto.textContent = on ? '■ Oprește Autonom (B)' : '▶ Pornește Autonom (B)';
  btnStartAuto.classList.toggle('active', on);
  goToKIndicator.textContent = on
    ? 'ACTIVĂ · Autonom + Pilot'
    : 'OPRITĂ · Doar Pilot (WASD)';
  if (chipGoToK) chipGoToK.classList.toggle('on', on);
  if (dotGoToK) dotGoToK.classList.toggle('on', on);
  if (btnFullFsm) btnFullFsm.classList.toggle('active', autoFsmEnabled);
}

function toggleAuto() {
  drive.toggleGoToK();
  if (drive.goToK === 1) {
    DXEnabled = true;
    drive.resetSpline();
    parkedSince = null;
  }
  refreshAutoButton();
}
btnStartAuto.onclick = toggleAuto;
if (chipGoToK) chipGoToK.onclick = toggleAuto;

function syncSlideControls(hCm, outCm) {
  intake.setHExtCm(hCm);
  intake.hExtCm = hCm;
  outTake.targetOutExtCm = outCm;
  outTake.outExtCm = outCm;
  outTake.velocityEnabled = outCm > 0;
  const sh = document.getElementById('slider-hext');
  const vh = document.getElementById('val-hext');
  const so = document.getElementById('slider-outext');
  const vo = document.getElementById('val-outext');
  if (sh) sh.value = hCm;
  if (vh) vh.textContent = `${hCm} cm`;
  if (so) so.value = outCm;
  if (vo) vo.textContent = `${outCm} cm`;
}

function startFullAutoMission() {
  intake.reset();
  outTake.reset();
  syncSlideControls(0, 0);
  autoFsmEnabled = true;
  AUTO = 'START';
  hasStarted = true;
  cycleNR = 0;
  DXEnabled = true;
  parkedSince = null;
  demoNudgeUntil = 0;
  drive.setPosition(0, 0, 0);
  drive.vel = new Point(0, 0);
  drive.setGoToK(true);
  drive.resetSpline();
  spawnFieldElements();
  trail.length = 0;
  refreshAutoButton();
}

function startFullAutoFsm() {
  if (!autoFsmEnabled) {
    selectChapter(5);
  } else {
    startFullAutoMission();
  }
}
if (btnFullFsm) btnFullFsm.onclick = startFullAutoFsm;

function setTargetWaypoints(newList) {
  TARGET.length = 0;
  newList.forEach(wp => TARGET.push(new goToValues(wp)));
  drive.resetSpline();
  parkedSince = null;
  selectedWaypoint = -1;
  rebuildWaypointList();
}

function resetAll() {
  autoFsmEnabled = false;
  AUTO = 'IDLE';
  DXEnabled = true;
  parkedSince = null;
  demoNudgeUntil = 0;
  drive.setPosition(-35, -124, toRadians(-70));
  drive.vel = new Point(0, 0);
  drive.resetSpline();
  drive.mainDrivePowers = { FLPow: 0, FRPow: 0, BLPow: 0, BRPow: 0 };
  drive.FL = drive.FR = drive.BL = drive.BR = 0;
  drive.power = 1;
  intake.reset();
  outTake.reset();
  syncSlideControls(0, 0);
  trail.length = 0;
  refreshAutoButton();
}
btnReset.onclick = resetAll;

document.getElementById('btn-clear-trail').onclick = () => {
  trail.length = 0;
  spawnFieldElements();
};

// ─── Interactive Demo Actions for Sponsor Pitch ──────────────────────────────
function triggerPilotNudgeDemo() {
  if (drive.goToK === 0) {
    drive.setGoToK(true);
    drive.resetSpline();
    refreshAutoButton();
  }
  const nearEnd = drive.splineDone || (drive.i >= TARGET.length - 4 && drive.localT > 0.55);
  if (nearEnd && !autoFsmEnabled) {
    TARGET.reverse();
    if (selectedWaypoint >= 0) selectedWaypoint = TARGET.length - 1 - selectedWaypoint;
    drive.resetSpline();
    rebuildWaypointList();
  }
  // Compute a clean lateral vector perpendicular to the robot's current velocity/heading
  const h = drive.H + Math.PI / 2;
  demoNudgeVec = {
    x: Math.sin(h) * 0.72,
    y: Math.cos(h) * 0.72,
    r: 0.35,
  };
  demoNudgeUntil = runtime + 0.85;
  pushEffects.push({ x: drive.X, y: drive.Y, t: runtime, color: '#38bdf8', label: '👋 Intervenție Pilot (Ocolire)' });
}

function triggerPushPerturbation() {
  if (drive.goToK === 0) {
    drive.setGoToK(true);
    drive.resetSpline();
    refreshAutoButton();
  }
  const nearEnd = drive.splineDone || (drive.i >= TARGET.length - 4 && drive.localT > 0.55);
  if (nearEnd && !autoFsmEnabled) {
    TARGET.reverse();
    if (selectedWaypoint >= 0) selectedWaypoint = TARGET.length - 1 - selectedWaypoint;
    drive.resetSpline();
    rebuildWaypointList();
  }
  // Displace robot 42 cm perpendicular to its current path so normalPower visibly pulls it back
  const d = drive.derivative || new Point(1, 1);
  const len = Math.hypot(d.x, d.y) || 1;
  const nx = -d.y / len;
  const ny = d.x / len;
  const sideSign = (Math.abs(drive.X) > 110 || Math.abs(drive.Y) > 110) ? -Math.sign(drive.X * nx + drive.Y * ny || 1) : 1;
  const oldX = drive.X, oldY = drive.Y;
  drive.X = clamp(drive.X + nx * 44 * sideSign, -FIELD_SIZE / 2 + 15, FIELD_SIZE / 2 - 15);
  drive.Y = clamp(drive.Y + ny * 44 * sideSign, -FIELD_SIZE / 2 + 15, FIELD_SIZE / 2 - 15);
  pushEffects.push({ x: oldX, y: oldY, tx: drive.X, ty: drive.Y, t: runtime, color: '#ff7b72', label: '💥 Perturbație → Corecție Normală!' });
}

function loadPathPreset(kind) {
  autoFsmEnabled = false;
  AUTO = 'IDLE';
  DXEnabled = true;
  if (kind === 'circuit') {
    setTargetWaypoints(circuitSplineTarget());
    drive.setPosition(-125, -120, toRadians(-45));
  } else if (kind === 'spin') {
    setTargetWaypoints(holonomicSpinTarget());
    drive.setPosition(-135, -115, toRadians(-90));
  } else {
    setTargetWaypoints(defaultSplineTarget());
    drive.setPosition(-35, -124, toRadians(-70));
  }
  trail.length = 0;
  drive.setGoToK(true);
  drive.resetSpline();
  refreshAutoButton();
}

const btnPathSlalom = document.getElementById('btn-path-slalom');
const btnPathCircuit = document.getElementById('btn-path-circuit');
const btnPathSpin = document.getElementById('btn-path-spin');
if (btnPathSlalom) btnPathSlalom.onclick = () => loadPathPreset('slalom');
if (btnPathCircuit) btnPathCircuit.onclick = () => loadPathPreset('circuit');
if (btnPathSpin) btnPathSpin.onclick = () => loadPathPreset('spin');

// ─── Tabs & 5-Chapter Navigation ─────────────────────────────────────────────
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

function selectChapter(chNum) {
  activeChapter = ((chNum - 1 + 5) % 5) + 1;
  document.querySelectorAll('.preset-btn').forEach(b => {
    b.classList.toggle('active', +b.dataset.ch === activeChapter);
  });
  renderPitchCard(activeChapter);

  if (activeChapter === 1) {
    resetAll();
    setTargetWaypoints(defaultSplineTarget());
    drive.setPosition(-35, -124, toRadians(-70));
    if (ovLoop) ovLoop.checked = true;
    if (ovVectors) ovVectors.checked = true;
    drive.setGoToK(true);
    drive.resetSpline();
    refreshAutoButton();
    switchTab('tab-drive');
  } else if (activeChapter === 2) {
    resetAll();
    setTargetWaypoints(circuitSplineTarget());
    drive.setPosition(-125, -120, toRadians(-45));
    if (ovLoop) ovLoop.checked = true;
    if (ovVectors) ovVectors.checked = true;
    if (ovLookahead) ovLookahead.checked = true;
    drive.setGoToK(true);
    drive.resetSpline();
    refreshAutoButton();
    switchTab('tab-drive');
  } else if (activeChapter === 3) {
    resetAll();
    spawnFieldElements();
    sensors.wipeSlots();
    if (ovOffsets) ovOffsets.checked = true;
    syncSlideControls(16, 14);
    drive.setPosition(0, -60, toRadians(45));
    switchTab('tab-drive');
    snapClawToNearestElement();
  } else if (activeChapter === 4) {
    resetAll();
    spawnFieldElements();
    sensors.wipeSlots();
    if (ovLoop) ovLoop.checked = false;
    syncSlideControls(14, 0);
    intake.setPowerIntakeAuto(-1);
    setTargetWaypoints([
      new goToValues(new Pose2d(-50, -110, -90), 0.72, 3),
      new goToValues(new Pose2d(-50, -110, -90), 0.72, 3),
      new goToValues(new Pose2d(-136, -93, -92), 0.62, 3),
      new goToValues(new Pose2d(-158, -92, -90), 0.58, 3),
      new goToValues(new Pose2d(-148, -38, -90), 0.65, 3),
      new goToValues(new Pose2d(-148, -38, -90), 0.65, 3),
    ]);
    drive.setPosition(-50, -110, toRadians(-90));
    drive.setGoToK(true);
    drive.resetSpline();
    refreshAutoButton();
    switchTab('tab-subsystems');
  } else if (activeChapter === 5) {
    resetAll();
    switchTab('tab-subsystems');
    startFullAutoMission();
  }
}

document.querySelectorAll('.preset-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    selectChapter(+btn.dataset.ch || 1);
  });
});

const btnPrevCh = document.getElementById('btn-prev-chapter');
const btnNextCh = document.getElementById('btn-next-chapter');
if (btnPrevCh) btnPrevCh.onclick = () => selectChapter(activeChapter - 1);
if (btnNextCh) btnNextCh.onclick = () => selectChapter(activeChapter + 1);

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
  pushEffects.push({ x: drive.X, y: drive.Y, t: runtime, color: '#ef4444', label: '⚡ Blocaj >7.1A → Auto-Unstuck!' });
}

document.getElementById('btn-fill-slots').onclick = () => sensors.fillSlotsRobot();
document.getElementById('btn-wipe-slots').onclick = () => sensors.wipeSlots();
document.getElementById('btn-cycle-motif').onclick = () => {
  motif = (motif % 3) + 1;
  const labels = { 1: '1 (V·M·M)', 2: '2 (M·V·M)', 3: '3 (M·M·V)' };
  document.getElementById('motif-val').textContent = labels[motif];
};

document.getElementById('btn-intake-eat').onclick = () => intake.setPowerIntakeAuto(-1);
document.getElementById('btn-intake-stop').onclick = () => intake.setPowerIntakeAuto(0);
document.getElementById('btn-intake-spit').onclick = () => intake.setPowerIntakeAuto(1);
document.getElementById('btn-jam').onclick = triggerJamDemo;

document.getElementById('btn-launch-seq').onclick = triggerLaunchControl;
document.getElementById('btn-unload-seq').onclick = triggerUnloadManager;

// ─── Offset IK: Send Extended Front Claw to a Specific or Nearest Element ───
function snapClawToElement(el) {
  if (!el) return;
  autoFsmEnabled = false;
  AUTO = 'IDLE';
  DXEnabled = true;
  if (ovLoop) ovLoop.checked = false;
  if (ovOffsets) ovOffsets.checked = true;
  highlightedElement = el;

  if (intake.targetHExtCm < 12) {
    syncSlideControls(16, outTake.targetOutExtCm);
  }
  const pos = drive.getPoint();
  const headingDeg = toDegrees(Math.atan2(el.x - pos.x, el.y - pos.y));
  // Using (headingDeg - 180) aligns clawToRobotPos with the 2D canvas forward arrow (+sin H, +cos H)
  const desiredClaw = new goToValues(new Pose2d(el.x, el.y, headingDeg - 180), 0.72, 2.5);
  const robotGoal = drive.clawToRobotPos(desiredClaw, intake.targetHExtCm);
  robotGoal.target.heading = headingDeg;

  const startPose = new goToValues(drive.getPose(), 0.75, 3);
  const midPose = new goToValues(
    new Pose2d((pos.x + robotGoal.target.x) / 2, (pos.y + robotGoal.target.y) / 2, headingDeg),
    0.75, 3
  );
  setTargetWaypoints([startPose, startPose, midPose, robotGoal, robotGoal]);
  intake.setPowerIntakeAuto(-1);
  drive.setGoToK(true);
  drive.resetSpline();
  refreshAutoButton();
}

function snapClawToNearestElement() {
  let pool = fieldElements.filter(e => !e.collected);
  if (pool.length === 0) {
    spawnFieldElements();
    pool = fieldElements.filter(e => !e.collected);
  }
  const pos = drive.getPoint();
  let best = pool[0], bestD = Infinity;
  pool.forEach(el => {
    const d = Math.hypot(el.x - pos.x, el.y - pos.y);
    if (d < bestD && d > 8) { bestD = d; best = el; }
  });
  snapClawToElement(best || pool[0]);
}

document.getElementById('btn-snap-claw').onclick = snapClawToNearestElement;

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

// ─── Keyboard & Simulated Pilot Nudge → stick ────────────────────────────────
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

  // If the sponsor clicked "Simulează Ocolire Obstacol", blend in the demo pilot impulse
  if (runtime < demoNudgeUntil && kx === 0 && ky === 0 && kr === 0) {
    kx = demoNudgeVec.x;
    ky = demoNudgeVec.y;
    kr = demoNudgeVec.r;
  }

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
let dragTwinWaypoint = -1; // keeps colocated start/end phantom pairs together when dragged on canvas!
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

function waypointAtCanvas(px, py, maxDist = 16) {
  let best = -1, bestD = maxDist;
  TARGET.forEach((wp, idx) => {
    const c = worldToCanvas(wp.target.x, wp.target.y);
    const d = Math.hypot(px - c.x, py - c.y);
    if (d <= bestD) { bestD = d; best = idx; }
  });
  return best;
}

function elementAtCanvas(px, py, maxDist = 18) {
  let best = null, bestD = maxDist;
  fieldElements.forEach((el) => {
    if (el.collected) return;
    const c = worldToCanvas(el.x, el.y);
    const d = Math.hypot(px - c.x, py - c.y);
    if (d <= bestD) { bestD = d; best = el; }
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

// Drag a waypoint on the field OR click a field element to grab it with the claw!
fieldCanvas.addEventListener('pointerdown', (e) => {
  const p = fieldPointerPos(e);
  const idx = waypointAtCanvas(p.x, p.y);
  if (idx >= 0) {
    selectedWaypoint = idx;
    dragWaypoint = idx;
    dragTwinWaypoint = -1;
    // If dragging a start/end waypoint that has a colocated phantom twin, move both together!
    const last = TARGET.length - 1;
    if ((idx === 0 || idx === 1) && TARGET.length >= 2) {
      const d01 = Math.hypot(TARGET[0].target.x - TARGET[1].target.x, TARGET[0].target.y - TARGET[1].target.y);
      if (d01 < 4) dragTwinWaypoint = idx === 0 ? 1 : 0;
    } else if ((idx === last || idx === last - 1) && TARGET.length >= 2) {
      const dEnd = Math.hypot(TARGET[last].target.x - TARGET[last - 1].target.x, TARGET[last].target.y - TARGET[last - 1].target.y);
      if (dEnd < 4) dragTwinWaypoint = idx === last ? last - 1 : last;
    }
    try { fieldCanvas.setPointerCapture(e.pointerId); } catch (_) {}
    refreshWpSelection();
    return;
  }

  // Check if user clicked on a collectible field element (Green/Purple artifact)
  const el = elementAtCanvas(p.x, p.y);
  if (el) {
    snapClawToElement(el);
    return;
  }

  selectedWaypoint = -1;
  refreshWpSelection();
});

fieldCanvas.addEventListener('pointermove', (e) => {
  const p = fieldPointerPos(e);
  if (dragWaypoint >= 0) {
    const w = canvasToWorld(p.x, p.y);
    const F = FIELD_SIZE / 2 - 8;
    const cx = clamp(w.x, -F, F);
    const cy = clamp(w.y, -F, F);
    TARGET[dragWaypoint].target.x = cx;
    TARGET[dragWaypoint].target.y = cy;
    updateWpListValues(dragWaypoint);
    if (dragTwinWaypoint >= 0 && TARGET[dragTwinWaypoint]) {
      TARGET[dragTwinWaypoint].target.x = cx;
      TARGET[dragTwinWaypoint].target.y = cy;
      updateWpListValues(dragTwinWaypoint);
    }
    if (drive.splineDone || dragWaypoint <= drive.i + 2) {
      drive.resetSpline();
    }
    fieldCanvas.style.cursor = 'grabbing';
  } else {
    const onWp = waypointAtCanvas(p.x, p.y) >= 0;
    const onEl = elementAtCanvas(p.x, p.y) !== null;
    fieldCanvas.style.cursor = onWp ? 'grab' : onEl ? 'pointer' : 'crosshair';
  }
});

const endWpDrag = () => { dragWaypoint = -1; dragTwinWaypoint = -1; };
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
  if (TARGET.length >= MAX_WAYPOINTS) { flashWpMsg(`Maxim ${MAX_WAYPOINTS} puncte`); return; }
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
  drive.resetSpline();
  selectedWaypoint = insertAt;
  rebuildWaypointList();
  flashWpMsg(`Punctul ${insertAt} a fost adăugat`);
}

function deleteWaypoint(idx) {
  if (idx < 0 || idx >= TARGET.length) return;
  if (TARGET.length <= MIN_WAYPOINTS) { flashWpMsg(`Curba necesită minim ${MIN_WAYPOINTS} puncte`); return; }
  TARGET.splice(idx, 1);
  drive.resetSpline();
  selectedWaypoint = -1;
  rebuildWaypointList();
  flashWpMsg(`Punctul ${idx} a fost șters`);
}

// ─── Waypoint list (per-point x / y / heading / speed / error) ───────────────
function rebuildWaypointList() {
  const list = document.getElementById('wp-list');
  const badge = document.getElementById('wp-count-badge');
  if (badge) badge.textContent = `${TARGET.length} puncte`;
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
      `<button class="wp-del" title="Șterge punctul ${idx}">✕</button>`;
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
  const ix = row.querySelector('[data-f="x"]');
  const iy = row.querySelector('[data-f="y"]');
  const ih = row.querySelector('[data-f="h"]');
  const is = row.querySelector('[data-f="s"]');
  const ie = row.querySelector('[data-f="e"]');
  if (ix && document.activeElement !== ix) ix.value = Math.round(wp.target.x * 10) / 10;
  if (iy && document.activeElement !== iy) iy.value = Math.round(wp.target.y * 10) / 10;
  if (ih && document.activeElement !== ih) ih.value = Math.round(wp.target.heading * 10) / 10;
  if (is && document.activeElement !== is) is.value = wp.speed;
  if (ie && document.activeElement !== ie) ie.value = wp.error;
}

function applyWaypointInput(idx, field, value, el) {
  const v = parseFloat(value);
  if (!isFinite(v)) return;
  const wp = TARGET[idx];
  if (!wp) return;

  // Check if this start/end waypoint has a colocated phantom twin before mutating
  let twinIdx = -1;
  const last = TARGET.length - 1;
  if ((idx === 0 || idx === 1) && TARGET.length >= 2) {
    const d01 = Math.hypot(TARGET[0].target.x - TARGET[1].target.x, TARGET[0].target.y - TARGET[1].target.y);
    if (d01 < 4) twinIdx = idx === 0 ? 1 : 0;
  } else if ((idx === last || idx === last - 1) && TARGET.length >= 2) {
    const dEnd = Math.hypot(TARGET[last].target.x - TARGET[last - 1].target.x, TARGET[last].target.y - TARGET[last - 1].target.y);
    if (dEnd < 4) twinIdx = idx === last ? last - 1 : last;
  }

  const F = FIELD_SIZE / 2;
  let out = v;
  if (field === 'x') { out = clamp(v, -F, F); wp.target.x = out; if (twinIdx >= 0) TARGET[twinIdx].target.x = out; }
  else if (field === 'y') { out = clamp(v, -F, F); wp.target.y = out; if (twinIdx >= 0) TARGET[twinIdx].target.y = out; }
  else if (field === 'h') { wp.target.heading = v; if (twinIdx >= 0) TARGET[twinIdx].target.heading = v; }
  else if (field === 's') { out = clamp(v, 0, 1); wp.speed = out; if (twinIdx >= 0) TARGET[twinIdx].speed = out; }
  else if (field === 'e') { out = Math.max(0.5, v); wp.error = out; if (twinIdx >= 0) TARGET[twinIdx].error = out; }

  if (twinIdx >= 0) updateWpListValues(twinIdx);
  if (drive.splineDone || idx <= drive.i + 2) drive.resetSpline();
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

  if (loopNr % 2 === 0 && (Math.abs(worldX) + Math.abs(worldY) > 0.01)) {
    trail.push({ x: drive.X, y: drive.Y });
    if (trail.length > MAX_TRAIL) trail.shift();
  }
}

// ─── OpMode AUTO_STATE Machine Step (KirbyAirRiders.java:338-807) ────────────
function stepAutoStateMachine() {
  if (!autoFsmEnabled) return;

  const endPt = TARGET[TARGET.length - 1].target;
  const parked = drive.splineDone && drive.doneGoTo && drive.getPoint().distanceTo(endPt) < 7.5;

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
            new goToValues(new Pose2d(0, 0, 0), 0.75, 3),
            new goToValues(new Pose2d(0, 0, 0), 0.75, 3),
            WP_LIBRARY.launch_first(),
            WP_LIBRARY.launch_first(),
          ]);
        } else {
          const cur = new goToValues(drive.getPose(), 0.72, 3);
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
        safetyTime = runtime + 6.5;
        sensors.wipeSlots();
        intake.setHExtCm(18);
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
          delayTime = runtime + 1.15;
          tremorTime = runtime + 0.3;
          currentAction = 2;
        }
        // KirbyAirRiders.java:622 — tremor oscillation in Roary space to shake artifacts into the intake
        if (!DXEnabled) {
          drive.sendPower(new Pose2d(-0.13 * side, 0.17 * tremor, -90 * side + 8 * tremor));
        }
        if (runtime > tremorTime) {
          tremorTime = runtime + 0.3;
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
        const cur = new goToValues(drive.getPose(), 0.72, 3);
        setTargetWaypoints([cur, cur, WP_LIBRARY.launch_close(), WP_LIBRARY.park(), WP_LIBRARY.park()]);
        DXEnabled = true;
        currentAction = 1;
        hasStarted = false;
      }
      if (parked) {
        intake.setPowerIntakeAuto(0);
        outTake.stopLauncher();
        // If continuous presentation mode is checked, restart full auto after a brief pause
        if (ovLoop && ovLoop.checked) {
          if (parkedSince === null) parkedSince = runtime;
          else if (runtime - parkedSince > 1.8) startFullAutoMission();
        }
      }
      break;
    }
  }
}

// ─── Field Element Collection (2D top-down intake pickup) ────────────────────
function stepFieldCollection() {
  if (intake.motorOutput >= -0.05 || intake.reserveIntake || intake.unloadBusy || intake.launchBusy || sensors.areSlotsFull()) return;
  const clawPose = drive.robotToClawPos(getAlignedMechanismPose(), intake.hExtCm);
  const robotPos = drive.getPoint();

  for (const el of fieldElements) {
    if (el.collected) continue;
    const dClaw = Math.hypot(el.x - clawPose.x, el.y - clawPose.y);
    const dRobot = Math.hypot(el.x - robotPos.x, el.y - robotPos.y);
    if (dClaw < 13 || dRobot < 15) {
      const slot = sensors.senseElement(el.color);
      if (slot >= 0) {
        el.collected = true;
        if (highlightedElement === el) highlightedElement = null;
        pushEffects.push({
          x: el.x,
          y: el.y,
          t: runtime,
          color: el.color === inIndexer.GREEN ? '#22c55e' : '#a855f7',
          label: `✓ Colectat în Slot #${slot + 1}`,
        });
        break;
      }
    }
  }
}

// ─── Continuous Presentation Patrol (keeps robot moving smoothly in Ch 1/2) ──
function stepPatrolLoop() {
  const editingWpInput = wpListEl && document.activeElement && wpListEl.contains(document.activeElement);
  if (autoFsmEnabled || drive.goToK === 0 || dragWaypoint >= 0 || editingWpInput) {
    parkedSince = null;
    return;
  }
  const endTarget = TARGET[TARGET.length - 1].target;
  const isParked = drive.splineDone && drive.doneGoTo && drive.getPoint().distanceTo(endTarget) < 7;
  if (!isParked) {
    parkedSince = null;
    return;
  }
  if (ovLoop && ovLoop.checked) {
    if (parkedSince === null) {
      parkedSince = runtime;
    } else if (runtime - parkedSince > 0.75) {
      // Seamlessly reverse the spline waypoints so the robot glides back and forth continuously!
      TARGET.reverse();
      if (selectedWaypoint >= 0) selectedWaypoint = TARGET.length - 1 - selectedWaypoint;
      drive.resetSpline();
      parkedSince = null;
      rebuildWaypointList();
    }
  }
}

// ─── runOpMode() loop ────────────────────────────────────────────────────────
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

  stepPatrolLoop();

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
    intake.launchControl(runtime, inZone && drive.vel.radius() < 35, motif, drive);
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
  drawEffectsAndHud();
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

  // Arena floor background
  ctx.fillStyle = '#111826';
  ctx.fillRect(CENTER - half, CENTER - half, FIELD_SIZE * SCALE, FIELD_SIZE * SCALE);

  // 6x6 FTC foam tile grid (60.8 cm tiles)
  ctx.strokeStyle = '#1c2638';
  ctx.lineWidth = 1;
  const tileStep = FIELD_SIZE / 6;
  for (let k = -3; k <= 3; k++) {
    const coord = k * tileStep;
    let a = worldToCanvas(coord, FIELD_SIZE / 2), b = worldToCanvas(coord, -FIELD_SIZE / 2);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    a = worldToCanvas(-FIELD_SIZE / 2, coord); b = worldToCanvas(FIELD_SIZE / 2, coord);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
  }

  // Center axes + circle
  ctx.strokeStyle = 'rgba(56, 189, 248, 0.16)';
  ctx.lineWidth = 1.2;
  ctx.beginPath(); ctx.moveTo(CENTER, CENTER - half); ctx.lineTo(CENTER, CENTER + half); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(CENTER - half, CENTER); ctx.lineTo(CENTER + half, CENTER); ctx.stroke();
  ctx.beginPath(); ctx.arc(CENTER, CENTER, 34, 0, Math.PI * 2); ctx.stroke();

  // Outer perimeter walls
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 2.5;
  ctx.strokeRect(CENTER - half, CENTER - half, FIELD_SIZE * SCALE, FIELD_SIZE * SCALE);

  // Odometry trail
  if (ovTrail && ovTrail.checked && trail.length > 1) {
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.34)';
    ctx.lineWidth = 2.5;
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
    ctx.fillStyle = inBig ? 'rgba(34, 197, 94, 0.13)' : 'rgba(56, 189, 248, 0.045)';
    ctx.strokeStyle = inBig ? 'rgba(34, 197, 94, 0.65)' : 'rgba(56, 189, 248, 0.22)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(b1.x, b1.y); ctx.lineTo(b2.x, b2.y); ctx.lineTo(b3.x, b3.y); ctx.closePath();
    ctx.fill(); ctx.stroke();

    // Small Triangle: (-60,-180), (60,-180), (0,-120)
    const s1 = worldToCanvas(-60, -180), s2 = worldToCanvas(60, -180), s3 = worldToCanvas(0, -120);
    ctx.fillStyle = inSmall ? 'rgba(34, 197, 94, 0.18)' : 'rgba(245, 158, 11, 0.08)';
    ctx.strokeStyle = inSmall ? 'rgba(34, 197, 94, 0.75)' : 'rgba(245, 158, 11, 0.35)';
    ctx.beginPath();
    ctx.moveTo(s1.x, s1.y); ctx.lineTo(s2.x, s2.y); ctx.lineTo(s3.x, s3.y); ctx.closePath();
    ctx.fill(); ctx.stroke();

    ctx.fillStyle = 'rgba(148, 163, 184, 0.78)';
    ctx.font = '600 11px -apple-system, Inter, sans-serif';
    ctx.fillText(inBig ? '✓ ÎN ZONA SUPERIOARĂ DE SCOR' : 'Zonă Superioară de Scor', b1.x + 12, b1.y + 52);
    ctx.fillText(inSmall ? '✓ ÎN ZONA DE LANSARE' : 'Zonă Lansare / Scor', s3.x - 52, s1.y - 8);
  }

  // 2) Field elements (collectible PURPLE / GREEN artifacts — clickable!)
  fieldElements.forEach((el) => {
    if (el.collected) return;
    const c = worldToCanvas(el.x, el.y);
    const isHl = highlightedElement === el;
    ctx.save();
    if (isHl) {
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.arc(c.x, c.y, 14, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.fillStyle = el.color === inIndexer.GREEN ? '#22c55e' : '#a855f7';
    ctx.strokeStyle = el.color === inIndexer.GREEN ? '#86efac' : '#d8b4fe';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(c.x, c.y, 7.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  });

  // 3) Scored / unloaded elements placed on the field by unloadManager or launchControl
  outTake.scoredDrops.forEach((drop) => {
    const c = worldToCanvas(drop.x, drop.y);
    ctx.save();
    ctx.fillStyle = drop.color === inIndexer.GREEN ? 'rgba(34,197,94,0.9)' : 'rgba(168,85,247,0.9)';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(c.x, c.y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 10px sans-serif';
    ctx.fillText('★', c.x - 4, c.y + 3.5);
    ctx.restore();
  });
}

function drawWaypoints() {
  const ctx = fieldCtx;

  // Spline path (the whole route, plus the active segment i highlighted)
  if (TARGET.length > 3) {
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
    ctx.lineWidth = 3;
    ctx.setLineDash([6, 5]);
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
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.92)';
      ctx.lineWidth = 3.5;
      ctx.beginPath();
      for (let t = 0; t <= 1; t += 0.02) {
        const p = drive.catmullRomSpline(pts, t);
        const c = worldToCanvas(p.x, p.y);
        if (t === 0) ctx.moveTo(c.x, c.y); else ctx.lineTo(c.x, c.y);
      }
      ctx.stroke();
    }
  }

  // Optional lookAhead / futurePos anticipation markers
  if (ovLookahead && ovLookahead.checked && !drive.splineDone) {
    if (drive.lookAhead) {
      const la = worldToCanvas(drive.lookAhead.x, drive.lookAhead.y);
      ctx.fillStyle = 'rgba(210, 168, 255, 0.85)';
      ctx.beginPath(); ctx.arc(la.x, la.y, 4.5, 0, Math.PI * 2); ctx.fill();
      ctx.font = '600 10px -apple-system, Inter, sans-serif';
      ctx.fillText('Anticipare Curbură', la.x + 7, la.y - 4);
    }
    if (drive.futurePos) {
      const fp = worldToCanvas(drive.futurePos.x, drive.futurePos.y);
      ctx.fillStyle = '#38bdf8';
      ctx.beginPath(); ctx.arc(fp.x, fp.y, 4, 0, Math.PI * 2); ctx.fill();
    }
  }

  // Current spline target + true tangent (Catmull-Rom derivative at localT)
  if (drive.target && drive.goToK === 1) {
    const c = worldToCanvas(drive.target.x, drive.target.y);
    ctx.fillStyle = '#f0b429';
    ctx.beginPath(); ctx.arc(c.x, c.y, 6, 0, Math.PI * 2); ctx.fill();
    const d = drive.derivative;
    const dl = d ? Math.hypot(d.x, d.y) : 0;
    if (dl > 1e-6 && !drive.splineDone) {
      const L = 34; // px
      const ex = c.x + (d.x / dl) * L, ey = c.y - (d.y / dl) * L;
      drawArrow(ctx, c.x, c.y, ex, ey, 'rgba(240,180,41,0.95)', 2.5, 8);
    }
  }

  // Waypoint handles + desired heading ticks (skip duplicate labels on colocated phantom twins)
  TARGET.forEach((wp, idx) => {
    const isLast = idx === TARGET.length - 1;
    // Skip drawing duplicate overlapping phantom dot if colocated with neighbor
    if (idx === 0 && TARGET.length >= 2) {
      const d01 = Math.hypot(TARGET[0].target.x - TARGET[1].target.x, TARGET[0].target.y - TARGET[1].target.y);
      if (d01 < 2 && selectedWaypoint !== 0) return;
    }
    if (idx === TARGET.length - 2 && TARGET.length >= 2) {
      const dEnd = Math.hypot(TARGET[idx].target.x - TARGET[idx + 1].target.x, TARGET[idx].target.y - TARGET[idx + 1].target.y);
      if (dEnd < 2 && selectedWaypoint !== idx) return;
    }

    const c = worldToCanvas(wp.target.x, wp.target.y);
    if (idx === selectedWaypoint) {
      ctx.strokeStyle = '#f0b429';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.arc(c.x, c.y, 12, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
    }
    const hRad = toRadians(wp.target.heading);
    const hx = c.x + Math.sin(hRad) * 14;
    const hy = c.y - Math.cos(hRad) * 14;
    ctx.strokeStyle = isLast ? 'rgba(34,197,94,0.8)' : 'rgba(56,189,248,0.7)';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(hx, hy); ctx.stroke();

    ctx.fillStyle = isLast ? '#22c55e' : idx <= 1 ? '#f59e0b' : '#38bdf8';
    ctx.beginPath(); ctx.arc(c.x, c.y, 5.5, 0, Math.PI * 2); ctx.fill();

    const label = isLast ? 'FINAL' : idx <= 1 ? 'START' : `P${idx}`;
    ctx.font = '700 10px -apple-system, Inter, sans-serif';
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = 'rgba(11, 15, 23, 0.82)';
    ctx.fillRect(c.x + 7, c.y - 14, tw + 6, 13);
    ctx.fillStyle = isLast ? '#86efac' : idx <= 1 ? '#fde68a' : '#cbd5e1';
    ctx.fillText(label, c.x + 10, c.y - 4);
  });
}

// ─── Force Vector Decomposition Overlay (normalPower + derivativePower + projectionPower) ──
function drawForceVectors() {
  if (!ovVectors || !ovVectors.checked) return;
  if (drive.splineDone || !drive.totalPower) return;

  const ctx = fieldCtx;
  const pos = drive.getPoint();
  const c = worldToCanvas(pos.x, pos.y);
  const VSCALE = 75; // px per unit power

  const drawVec = (pt, color, width = 2.5) => {
    if (!pt) return;
    const mag = pt.radius();
    if (mag < 0.015) return;
    const wx = -pt.x, wy = -pt.y;
    const ex = c.x + wx * VSCALE;
    const ey = c.y - wy * VSCALE;
    drawArrow(ctx, c.x, c.y, ex, ey, color, width, 7);
  };

  drawVec(drive.normalPower, '#ff7b72', 2.5);     // normal correction (onto spline)
  drawVec(drive.derivativePower, '#3fb950', 2.5); // tangent cruise power
  drawVec(drive.projectionPower, '#d2a8ff', 2.5); // centripetal lookAhead projection
  drawVec(drive.totalPower, '#58a6ff', 3);        // resultant totalPower
}

function drawRobot() {
  const ctx = fieldCtx;
  const pos = drive.getPoint();
  const c = worldToCanvas(pos.x, pos.y);
  const H = drive.H;
  const s = ROBOT_SIZE * SCALE;

  // 1) Draw front Claw (`robotToClawPos`) and rear OutTake (`robotToOutTakePos`) slides in world space
  //    Starting from the front/rear chassis edges so the rails never cover the internal 3-slot magazine!
  if (ovOffsets && ovOffsets.checked) {
    const mechPose = getAlignedMechanismPose();
    const fwdX = Math.sin(H), fwdY = -Math.cos(H);
    const rightX = Math.cos(H), rightY = Math.sin(H);

    // Front horizontal slide (hExt) + 2-finger Claw Gripper
    if (intake.hExtCm > 0.5 || intake.motorOutput !== 0 || activeChapter === 3) {
      const clawWorld = drive.robotToClawPos(mechPose, intake.hExtCm);
      const cc = worldToCanvas(clawWorld.x, clawWorld.y);
      const frontEdgeX = c.x + fwdX * (s / 2);
      const frontEdgeY = c.y + fwdY * (s / 2);
      const col = intake.motorOutput < 0 ? '#22c55e' : intake.motorOutput > 0 ? '#ef4444' : '#38bdf8';

      // Twin parallel slide rails from front chassis edge to claw base
      ctx.strokeStyle = col;
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(frontEdgeX - rightX * 4, frontEdgeY - rightY * 4);
      ctx.lineTo(cc.x - rightX * 4, cc.y - rightY * 4);
      ctx.moveTo(frontEdgeX + rightX * 4, frontEdgeY + rightY * 4);
      ctx.lineTo(cc.x + rightX * 4, cc.y + rightY * 4);
      ctx.stroke();

      // Claw gripper jaws at cc
      ctx.fillStyle = '#0d131d';
      ctx.beginPath();
      ctx.arc(cc.x, cc.y, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.lineWidth = 2.4;
      ctx.beginPath();
      ctx.moveTo(cc.x - rightX * 7, cc.y - rightY * 7);
      ctx.lineTo(cc.x - rightX * 5 + fwdX * 8, cc.y - rightY * 5 + fwdY * 8);
      ctx.moveTo(cc.x + rightX * 7, cc.y + rightY * 7);
      ctx.lineTo(cc.x + rightX * 5 + fwdX * 8, cc.y + rightY * 5 + fwdY * 8);
      ctx.stroke();
    }

    // Rear outTake slide (outExt) + Bascula Basket
    if (outTake.outExtCm > 0.5 || activeChapter === 3) {
      const outWorld = drive.robotToOutTakePos(mechPose, outTake.outExtCm);
      const oc = worldToCanvas(outWorld.x, outWorld.y);
      const rearEdgeX = c.x - fwdX * (s / 2);
      const rearEdgeY = c.y - fwdY * (s / 2);
      ctx.strokeStyle = '#a855f7';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(rearEdgeX, rearEdgeY);
      ctx.lineTo(oc.x, oc.y);
      ctx.stroke();
      ctx.fillStyle = outTake.clawState === 'OPEN_BASKET' ? '#22c55e' : '#a855f7';
      ctx.beginPath();
      ctx.arc(oc.x, oc.y, 6, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // 2) Robot chassis in local frame (rotated by H)
  ctx.save();
  ctx.translate(c.x, c.y);
  ctx.rotate(H); // forward edge points where +forward motion goes

  // Chassis body
  ctx.fillStyle = 'rgba(19, 28, 46, 0.9)';
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2.2;
  ctx.fillRect(-s / 2, -s / 2, s, s);
  ctx.strokeRect(-s / 2, -s / 2, s, s);

  // Active intake roller bar on front edge
  if (Math.abs(intake.motorOutput) > 0.05) {
    ctx.strokeStyle = intake.motorOutput < 0 ? '#22c55e' : '#ef4444';
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    ctx.moveTo(-s / 2 + 4, -s / 2 - 3);
    ctx.lineTo(s / 2 - 4, -s / 2 - 3);
    ctx.stroke();
  }

  // Forward heading arrow (top edge)
  ctx.fillStyle = '#22c55e';
  ctx.beginPath();
  ctx.moveTo(0, -s / 2 - 10);
  ctx.lineTo(-7, -s / 2);
  ctx.lineTo(7, -s / 2);
  ctx.closePath();
  ctx.fill();

  // 3 internal indexer slots [Mid, Left, Right] drawn inside the chassis!
  const slotCoords = [
    { idx: 0, x: 0,         y: -1 }, // Mid
    { idx: 1, x: -s * 0.24, y: 2 },  // Left
    { idx: 2, x:  s * 0.24, y: 2 },  // Right
  ];
  slotCoords.forEach(sc => {
    const val = sensors.slots[sc.idx];
    ctx.fillStyle = val === inIndexer.GREEN ? '#22c55e'
                  : val === inIndexer.PURPLE ? '#a855f7'
                  : 'rgba(11, 15, 23, 0.85)';
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, 4.2, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });

  // Farul LED beacon at rear of chassis
  ctx.fillStyle = outTake.farul.hex;
  ctx.beginPath();
  ctx.arc(0, s / 2 - 4, 3.2, 0, Math.PI * 2);
  ctx.fill();

  // 4 Holonomic wheels (FL, FR, BL, BR)
  const wheelR = 5.2;
  const wp = [
    { x: -s / 2 * 0.72, y: -s / 2 * 0.72, p: drive.FL }, // FL
    { x:  s / 2 * 0.72, y: -s / 2 * 0.72, p: drive.FR }, // FR
    { x: -s / 2 * 0.72, y:  s / 2 * 0.72, p: drive.BL }, // BL
    { x:  s / 2 * 0.72, y:  s / 2 * 0.72, p: drive.BR }, // BR
  ];
  const spin = performance.now() / 85;
  wp.forEach(w => {
    const p = w.p;
    const color = p > 0.05 ? '#22c55e' : p < -0.05 ? '#ef4444' : '#64748b';
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(w.x, w.y, wheelR, 0, Math.PI * 2); ctx.fill();
    if (Math.abs(p) > 0.01) {
      const a = spin * p;
      ctx.strokeStyle = '#0b0f17';
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.moveTo(w.x - Math.cos(a) * wheelR * 0.7, w.y - Math.sin(a) * wheelR * 0.7);
      ctx.lineTo(w.x + Math.cos(a) * wheelR * 0.7, w.y + Math.sin(a) * wheelR * 0.7);
      ctx.stroke();
    }
  });

  ctx.restore();
}

function drawEffectsAndHud() {
  const ctx = fieldCtx;

  // 1) Transient visual callouts (push perturbation, pilot nudge, element pickup)
  pushEffects = pushEffects.filter(ef => runtime - ef.t < 1.6);
  pushEffects.forEach(ef => {
    const age = (runtime - ef.t) / 1.6;
    const alpha = Math.max(0, 1 - age);
    const c = worldToCanvas(ef.x, ef.y);
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = ef.color || '#38bdf8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(c.x, c.y, 12 + age * 28, 0, Math.PI * 2);
    ctx.stroke();
    if (ef.tx !== undefined && ef.ty !== undefined) {
      const tc = worldToCanvas(ef.tx, ef.ty);
      drawArrow(ctx, c.x, c.y, tc.x, tc.y, ef.color, 2.5, 8);
    }
    if (ef.label) {
      ctx.fillStyle = ef.color || '#38bdf8';
      ctx.font = 'bold 11px -apple-system, Inter, sans-serif';
      ctx.fillText(ef.label, c.x - 45, c.y - 16 - age * 12);
    }
    ctx.restore();
  });

  // 2) Top on-canvas HUD pill showing live presentation status
  ctx.save();
  ctx.fillStyle = 'rgba(13, 19, 29, 0.84)';
  ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
  ctx.lineWidth = 1;
  const bx = 34, by = 34, bw = CANVAS_SIZE - 68, bh = 28;
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(bx, by, bw, bh, 7);
  else ctx.rect(bx, by, bw, bh);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = '#e2e8f0';
  ctx.font = '600 11.5px -apple-system, Inter, sans-serif';
  let liveBanner = canvasBannerText;
  if (runtime < demoNudgeUntil) {
    liveBanner = '👋 Intervenție Pilot Activă: Robotul ocolește obstacolul și revine automat pe curbă!';
  } else if (intake.ocTime > runtime || !intake.doneUnstuck) {
    liveBanner = '⚡ Protecție Supracurent (>7.1A) Activată: Inversare automată a rolelor și deblocare!';
  } else if (!intake.unloadDone) {
    liveBanner = `📦 Secvență Automată de Transfer & Plasare: Pasul ${intake.currentActionUnload} din 6`;
  } else if (autoFsmEnabled) {
    const fsmNames = {
      START: 'Etapa 1: Inițializare misiune autonomă',
      LAUNCH: 'Etapa Scor: Deplasare la zona de lansare & descărcare magazie',
      LOAD_SPIKE: DXEnabled ? 'Etapa Colectare: Navigație pe curbă către elemente' : 'Etapa Colectare: Oscilație „Tremor” în spațiul Roary pentru preluare',
      PARK: 'Etapa Finală: Parcare de precizie cu goToV2',
    };
    liveBanner = `⚡ Misiune Autonomă · ${fsmNames[AUTO] || AUTO}`;
  }
  ctx.fillText(liveBanner, bx + 12, by + 18);
  ctx.restore();
}

function drawJoystick() {
  const ctx = joyCtx;
  const size = 120, cx = size / 2, cy = size / 2, r = size / 2 - 10;
  ctx.clearRect(0, 0, size, size);

  ctx.fillStyle = '#0d131d';
  ctx.beginPath(); ctx.arc(cx, cy, r + 2, 0, Math.PI * 2); ctx.fill();

  // 0.89 power-ramp threshold ring
  ctx.strokeStyle = 'rgba(245, 158, 11, 0.25)';
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 3]);
  ctx.beginPath(); ctx.arc(cx, cy, r * 0.7 * 0.89, 0, Math.PI * 2); ctx.stroke();
  ctx.setLineDash([]);

  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r);
  ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
  ctx.stroke();

  const kx = cx + joyX * r * 0.7;
  const ky = cy - joyY * r * 0.7;
  ctx.strokeStyle = 'rgba(56, 189, 248, 0.55)';
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(kx, ky); ctx.stroke();

  ctx.fillStyle = '#38bdf8';
  ctx.beginPath(); ctx.arc(kx, ky, 10, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = '#334155';
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, Math.PI * 2); ctx.fill();

  if (Math.abs(joyR) > 0.01) {
    ctx.strokeStyle = joyR > 0 ? '#22c55e' : '#ef4444';
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
    const autoVal = (goToPow * k).toFixed(2);
    const pilVal = stickPow.toFixed(2);
    txtEl.textContent = `Auto ${autoVal} + Pilot ${pilVal}`;
  }
}

function updateTelemetry() {
  const endTarget = TARGET[TARGET.length - 1].target;
  const distToEnd = drive.getPoint().distanceTo(endTarget);
  const isParked = drive.splineDone && drive.doneGoTo && distToEnd < 7;
  const stageStr = !DXEnabled
    ? 'Oscilație Colectare (Tremor)'
    : drive.splineDone
      ? (isParked ? '✓ Aliniat la Țintă' : '→ Aliniere Finală (goToV2)')
      : `În mișcare pe curbă (seg. ${drive.i + 1})`;

  // Topbar chips
  document.getElementById('tel-hz').textContent = currentHz;
  document.getElementById('tel-runtime').textContent = runtime.toFixed(1);
  document.getElementById('chip-autostate').textContent = autoFsmEnabled
    ? `Misiune: ${AUTO}`
    : (drive.goToK === 1 ? 'Urmărire Curbă + Pilot' : 'Pilotare Manuală');

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

  // Tab 2: Subsystems & FSM
  const activeState = autoFsmEnabled ? AUTO : 'IDLE';
  document.querySelectorAll('#fsm-auto-nodes .fsm-node').forEach(n => {
    n.classList.toggle('active', n.dataset.st === activeState);
  });
  document.getElementById('fsm-cycle-badge').textContent = autoFsmEnabled
    ? `Ciclu #${cycleNR + 1} · Pas ${currentAction}`
    : 'Traseu Liber';
  const autoDetail = document.getElementById('fsm-auto-detail');
  if (autoDetail) {
    autoDetail.textContent = autoFsmEnabled
      ? `Stare: ${AUTO} · Urmărire curbă: ${DXEnabled ? 'ACTIVĂ' : 'OSCILAȚIE TREMOR'} · Timp siguranță: ${Math.max(0, safetyTime - runtime).toFixed(1)}s`
      : `Mod curent: Urmărire continuă a curbei Catmull-Rom. Apasă „⚡ Misiune Completă” pentru ciclul autonom cap-coadă.`;
  }

  // 3-Slot Indexer cards
  document.getElementById('sensor-poll-badge').textContent = `Senzor activ: #${sensors.lastPolledSlot + 1} / 3`;
  const paddles = [intake.MID, intake.LEFT, intake.RIGHT];
  const colorLabels = {
    [inIndexer.NOTHING]: 'GOL',
    [inIndexer.GREEN]: 'VERDE',
    [inIndexer.PURPLE]: 'MOV',
    [inIndexer.ROBOT]: 'PREÎNCĂRCAT',
  };
  const paddleLabels = {
    [PaddleState.LOAD]: 'ÎNCĂRCARE',
    [PaddleState.FORCED_LOAD]: 'ÎNCĂRCARE FORȚATĂ',
    [PaddleState.UNLOAD]: 'TRANSFER',
    [PaddleState.FORCED_UNLOAD]: 'LANSARE',
    [PaddleState.RESERVED]: 'REZERVAT',
  };
  for (let i = 0; i < 3; i++) {
    const val = sensors.slots[i];
    const card = document.getElementById(`slot-${i}`);
    const orb = document.getElementById(`slot-orb-${i}`);
    const valEl = document.getElementById(`slot-val-${i}`);
    const padEl = document.getElementById(`slot-pad-${i}`);
    if (card) card.classList.toggle('polled', sensors.lastPolledSlot === i);
    if (orb) orb.className = 'slot-orb ' + (val === inIndexer.GREEN || val === inIndexer.PURPLE ? val : '');
    if (valEl) valEl.textContent = colorLabels[val] || val;
    if (padEl) padEl.textContent = `Paletă: ${paddleLabels[paddles[i]] || paddles[i]}`;
  }

  // Intake status
  const ocActive = intake.ocTime > runtime;
  const ampsBadge = document.getElementById('intake-amps-badge');
  if (ampsBadge) {
    ampsBadge.textContent = `${intake.currentAmps.toFixed(1)} A` + (ocActive ? ' (INVERSARE!)' : '');
    ampsBadge.className = 'badge' + (ocActive ? ' danger' : intake.motorOutput < 0 ? ' on' : '');
  }
  document.getElementById('btn-intake-eat').classList.toggle('active', intake.intakePower < 0);
  document.getElementById('btn-intake-spit').classList.toggle('active', intake.intakePower > 0);
  document.getElementById('intake-status-bar').textContent =
    `Role: ${intake.motorOutput < 0 ? 'COLECTARE (-1)' : intake.motorOutput > 0 ? 'EVACUARE (+1)' : 'OPRITE'} · ` +
    `Glisieră față: ${intake.hExtCm.toFixed(0)} cm · ` +
    `Anti-blocaj: ${!intake.doneUnstuck ? 'DEBLOCARE PAS ' + intake.stuckAction + '/4' : 'NORMAL (<7.1A)'}`;

  // OutTake & UnloadManager steps
  const uStep = intake.unloadDone && intake.currentActionUnload > 6 ? 0 : intake.currentActionUnload;
  document.querySelectorAll('#fsm-unload-steps .fsm-node').forEach(n => {
    n.classList.toggle('active', +n.dataset.ustep === uStep);
  });
  const outBadge = document.getElementById('outtake-ready-badge');
  if (outBadge) {
    const rdy = outTake.velocityDone && outTake.turretDone;
    outBadge.textContent = rdy ? 'PREGĂTIT DE SCOR' : outTake.velocityEnabled ? 'EXTINDERE GLISIERĂ…' : 'PREGĂTIT';
    outBadge.className = 'badge' + (rdy ? ' on' : outTake.velocityEnabled ? ' warn' : '');
  }
  document.getElementById('outtake-status-bar').textContent =
    `Transfer 6 pași: ${intake.unloadDone ? 'FINALIZAT' : 'În curs (Pasul ' + intake.currentActionUnload + '/6)'} · ` +
    `Lansare motiv: ${intake.launchDone ? 'FINALIZAT' : 'În curs (Pasul ' + intake.currentActionLaunch + '/5)'} · ` +
    `Obiecte plasate: ${outTake.scoredDrops.length}`;

  // Tab 1: Stick
  document.getElementById('joy-x').textContent = `X: ${joyX.toFixed(2)}`;
  document.getElementById('joy-y').textContent = `Y: ${joyY.toFixed(2)}`;
  document.getElementById('joy-r').textContent = `R: ${joyR.toFixed(2)}`;
  const pBadge = document.getElementById('power-ramp-badge');
  if (pBadge) {
    pBadge.textContent = `Amplificare: ${(drive.power * 100).toFixed(0)}%`;
    pBadge.className = 'badge' + (drive.power > 1.02 ? ' warn' : '');
  }

  // Tab 3: Technical Telemetry
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

  const mechPose = getAlignedMechanismPose();
  const clawP = drive.robotToClawPos(mechPose, intake.hExtCm);
  const outP = drive.robotToOutTakePos(mechPose, outTake.outExtCm);
  document.getElementById('tel-claw-pose').textContent = `${clawP.x.toFixed(1)}, ${clawP.y.toFixed(1)}`;
  document.getElementById('tel-out-pose').textContent = `${outP.x.toFixed(1)}, ${outP.y.toFixed(1)}`;
}

// ─── Robust 60 Hz Loop ───────────────────────────────────────────────────────
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

// ─── Init: Start immediately in Chapter 1 (or URL #chN) with active motion ───
const hashMatch = (window.location && window.location.hash || '').match(/ch(\d)/i);
const initialCh = hashMatch ? clamp(+hashMatch[1], 1, 5) : 1;
selectChapter(initialCh);
control();
render();
fieldCanvas.style.cursor = 'crosshair';

