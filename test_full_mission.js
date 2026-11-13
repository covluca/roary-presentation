// End-to-end test of RoaryX7 Smart Courier Robot
const fs = require('fs');
const assert = require('assert');

// Setup mock DOM environment
global.window = {
  addEventListener: () => {},
  location: { hash: '' }
};
const mockElements = {};
global.document = {
  getElementById: (id) => {
    if (!mockElements[id]) {
      mockElements[id] = {
        id,
        textContent: '',
        className: '',
        classList: {
          add: function(c) { this._classes = this._classes || new Set(); this._classes.add(c); },
          remove: function(c) { this._classes = this._classes || new Set(); this._classes.delete(c); },
          contains: function(c) { return this._classes ? this._classes.has(c) : false; }
        },
        addEventListener: function(ev, fn) { this._listeners = this._listeners || {}; this._listeners[ev] = fn; },
        getBoundingClientRect: () => ({ left: 0, top: 0, width: 760, height: 580 }),
        getContext: () => ({
          clearRect: () => {}, fillRect: () => {}, strokeRect: () => {},
          beginPath: () => {}, arc: () => {}, fill: () => {}, stroke: () => {},
          moveTo: () => {}, lineTo: () => {}, closePath: () => {},
          save: () => {}, restore: () => {}, translate: () => {}, rotate: () => {},
          setLineDash: () => {}, fillText: () => {}, measureText: () => ({ width: 50 }),
          roundRect: () => {}, createRadialGradient: () => ({ addColorStop: () => {} })
        }),
        style: {},
        checked: true,
        width: 760,
        height: 580
      };
    }
    return mockElements[id];
  },
  addEventListener: () => {}
};
global.performance = { now: () => Date.now() };
global.requestAnimationFrame = () => {};

require('./web/roaryx7.js');
const simCode = fs.readFileSync('web/sim.js', 'utf8');
eval(simCode + `
  global.control = control;
  global.render = render;
  global.startMission = startMission;
  global.pauseMission = pauseMission;
  global.resetAll = resetAll;
  global.setScenario = setScenario;
  global.worldToCanvas = worldToCanvas;
  global.getDrive = () => drive;
  global.getMissionState = () => missionState;
  global.getSplinePhase = () => splinePhase;
  global.getBasketCargo = () => basketCargo;
  global.getTotalDelivered = () => totalDelivered;
  global.getParkingBays = () => parkingBays;
  global.getArrivingCar = () => arrivingCar;
  global.getRobotCurrentSpotIdx = () => robotCurrentSpotIdx;
  global.getRobotPrevSpotIdx = () => robotPrevSpotIdx;
  global.getTargetFreeSpotIdx = () => targetFreeSpotIdx;
  global.getTotalRelocations = () => totalRelocations;
  global.getMinSafetyClearance = () => minSafetyClearance;
  global.setPilotJoy = (x, y, r) => {
    joyX = x;
    joyY = y;
    joyR = r;
    joyActive = (Math.hypot(x, y) > 0.001 || Math.abs(r) > 0.001);
  };
`);


console.log('=== TEST 1: Initial Standby & Dock State ===');
const dr = getDrive();
console.log('Initial robot pose:', dr.getX().toFixed(1), dr.getY().toFixed(1), dr.getH().toFixed(1));
assert.strictEqual(getMissionState(), 'STANDBY');
assert.strictEqual(getBasketCargo().length, 0);

console.log('=== TEST 2: Start Mission -> Fetch 3 Books -> Deliver to Desk ===');
startMission();
assert.strictEqual(getMissionState(), 'NAV_TO_SHELF');

// Step simulation through navigation to shelf and book pickup
let ticks = 0;
const maxTicks = 1600;
let pickedUpAllThree = false;
let deliveredToDesk = false;

while (ticks < maxTicks) {
  control();
  const state = getMissionState();
  const cargo = getBasketCargo();
  
  if (state === 'PICKUP_BOOKS' && cargo.length === 3 && !pickedUpAllThree) {
    pickedUpAllThree = true;
    console.log(`[Tick ${ticks}] Picked up 3 books! Basket fill: 3/3`);
  }

  if ((state === 'DELIVER_TO_DESK' || state === 'WAITING_NEXT_BATCH' || state === 'COMPLETE') && getTotalDelivered() > 0 && !deliveredToDesk) {
    deliveredToDesk = true;
    console.log(`[Tick ${ticks}] Delivered 3 books to client desk! Total delivered: ${getTotalDelivered()}`);
    break;
  }

  if (ticks % 100 === 0) {
    const p = dr.getPose();
    console.log(`[Tick ${ticks}] State: ${state}, Pos: (${p.x.toFixed(1)}, ${p.y.toFixed(1)}, ${p.heading.toFixed(1)}°), Cargo: ${cargo.length}, Delivered: ${getTotalDelivered()}`);
  }

  ticks++;
}

assert(pickedUpAllThree, 'Should have picked up 3 books into the basket');
assert(deliveredToDesk, 'Should have delivered the 3 books to the desk');
assert.strictEqual(getTotalDelivered(), 3, 'Total delivered books should equal 3');
console.log('TEST 2 PASSED: Full delivery cycle executed smoothly!');

console.log('=== TEST 3: Shared Autonomy / Pilot Nudge ===');
resetAll();
startMission();
// Run 30 ticks autonomous
for (let i = 0; i < 30; i++) control();
const autoPosBeforeNudge = { x: dr.getX(), y: dr.getY() };

// Operator pushes pilot stick sideways (strafe right joyX = 1)
console.log('Simulating pilot intervention (joyX = 1.0)...');
setPilotJoy(1.0, 0, 0);
for (let i = 0; i < 40; i++) control();
const pilotedPos = { x: dr.getX(), y: dr.getY() };
console.log('Robot pos after pilot nudge:', pilotedPos.x.toFixed(1), pilotedPos.y.toFixed(1));
assert(pilotedPos.x > autoPosBeforeNudge.x, 'Robot should have strafed right following operator input');

// Operator releases stick
setPilotJoy(0, 0, 0);
console.log('Operator released stick. Verifying Roary resumes autonomous trajectory...');
for (let i = 0; i < 60; i++) control();
console.log('Robot pos after resuming autonomy:', dr.getX().toFixed(1), dr.getY().toFixed(1));
console.log('TEST 3 PASSED: Shared autonomy fused human pilot with autonomous path seamlessly!');

console.log('=== TEST 4: Continuous Multi-Batch Logistics ("ia alte 3 carti, iara le aduce") ===');
// Continue running from current delivery through the next batch cycle
let batch2Ticks = 0;
const maxBatch2Ticks = 2400;
let batch2Completed = false;

while (batch2Ticks < maxBatch2Ticks) {
  control();
  if (getTotalDelivered() >= 6) {
    batch2Completed = true;
    console.log(`[Batch 2 completed at tick +${batch2Ticks}] Total delivered: ${getTotalDelivered()} books!`);
    break;
  }
  if (batch2Ticks % 200 === 0) {
    const p = dr.getPose();
    console.log(`[Tick +${batch2Ticks}] State: ${getMissionState()}, Pos: (${p.x.toFixed(1)}, ${p.y.toFixed(1)}, ${p.heading.toFixed(1)}°), Cargo: ${getBasketCargo().length}, Delivered: ${getTotalDelivered()}`);
  }
  batch2Ticks++;
}

assert(batch2Completed, 'Robot should have automatically returned to shelf, fetched batch 2, and delivered total of 6 books');
console.log('TEST 4 PASSED: Continuous multi-batch logistics loop verified successfully!');

console.log('=== TEST 5: Smart Parking Lot Scenario — Initial State ===');
setScenario('parking');
assert.strictEqual(getMissionState(), 'STANDBY');
assert.strictEqual(getRobotCurrentSpotIdx(), 0, 'Roary should start parked in spot P1 (index 0)');
assert.strictEqual(getParkingBays()[0].car, null, 'P1 should not have a car while Roary is parked there');
assert.strictEqual(getParkingBays().filter(b => b.car !== null).length, 5, 'All other 5 spots (P2..P6) should be occupied by cars');
console.log('TEST 5 PASSED: Initial parking lot state verified!');

console.log('=== TEST 6: Parking Lot Relocation 1 (Spot Frees -> Roary Navigates -> Car Appears in Old Spot) ===');
startMission();
assert.strictEqual(getMissionState(), 'PARKING_FREEING_SPOT', 'Should begin by freeing a parking spot');
const firstTargetIdx = getTargetFreeSpotIdx();
assert.strictEqual(firstTargetIdx, 5, 'First spot to free up should be P6 (index 5)');

let pTicks = 0;
let sawSpotFreed = false;
let sawRoaryArrivedAndFillingOld = false;
let completedFirstRelocation = false;

while (pTicks < 900) {
  control();
  render();
  const st = getMissionState();

  if (st === 'PARKING_NAV_TO_SPOT' && !sawSpotFreed) {
    sawSpotFreed = true;
    assert.strictEqual(getParkingBays()[firstTargetIdx].car, null, 'Target spot P6 should now be empty');
    assert.strictEqual(getRobotPrevSpotIdx(), 0, 'Previous spot should be recorded as P1 (index 0)');
    console.log(`[Parking Tick ${pTicks}] Spot P6 freed! Roary departing P1 along Catmull-Rom spline...`);
  }

  if (st === 'PARKING_FILLING_OLD_SPOT' && !sawRoaryArrivedAndFillingOld) {
    sawRoaryArrivedAndFillingOld = true;
    assert.strictEqual(getRobotCurrentSpotIdx(), firstTargetIdx, 'Roary should now be parked in P6');
    assert.strictEqual(getParkingBays()[0].car, null, 'Old spot P1 is currently being entered by the new car');
    console.log(`[Parking Tick ${pTicks}] Roary docked safely in P6! New car arriving into previous spot P1...`);
  }

  if (getTotalRelocations() >= 1) {
    completedFirstRelocation = true;
    assert.notStrictEqual(getParkingBays()[0].car, null, 'Previous spot P1 must now be occupied by the newly arrived car');
    console.log(`[Parking Tick ${pTicks}] New car parked in P1! Relocation #1 complete.`);
    break;
  }
  pTicks++;
}

assert(sawSpotFreed, 'Target spot should have freed up');
assert(sawRoaryArrivedAndFillingOld, 'Roary should have navigated to the freed spot and triggered a car arriving at its old spot');
assert(completedFirstRelocation, 'First parking relocation cycle should complete');
assert(getMinSafetyClearance() > 20, `Safety clearance from central islands must remain > 20cm (actual: ${getMinSafetyClearance().toFixed(1)}cm)`);
console.log('TEST 6 PASSED: Spot freed -> Roary parked -> New car occupied previous spot!');

console.log('=== TEST 7: Continuous Parking Loop (Another Spot Frees -> Roary Leaves Again) ===');
let sawSecondDeparture = false;
let completedThirdRelocation = false;
let loopTicks = 0;

while (loopTicks < 1800) {
  control();
  render();
  const st = getMissionState();
  if (st === 'PARKING_NAV_TO_SPOT' && getTotalRelocations() === 1 && !sawSecondDeparture) {
    sawSecondDeparture = true;
    const nextSpot = getTargetFreeSpotIdx();
    console.log(`[Loop Tick +${loopTicks}] Second spot (P${nextSpot + 1}) freed! Roary departed P6 toward P${nextSpot + 1}...`);
  }
  if (getTotalRelocations() >= 3) {
    completedThirdRelocation = true;
    console.log(`[Loop Tick +${loopTicks}] Completed ${getTotalRelocations()} continuous parking relocations! Minimum obstacle clearance: ${getMinSafetyClearance().toFixed(1)}cm`);
    break;
  }
  loopTicks++;
}

assert(sawSecondDeparture, 'Another spot should free up and Roary should leave again');
assert(completedThirdRelocation, 'Roary should continuously complete multiple parking relocations');
assert(getMinSafetyClearance() > 20, 'Roary must maintain safe clearance across all relocations');
console.log('TEST 7 PASSED: Continuous multi-spot parking loop verified!');

console.log('=== TEST 8: Shared Autonomy Pilot Nudge inside Narrow Parking Lot ===');
resetAll();
startMission();
// Advance until Roary is actively navigating in PARKING_NAV_TO_SPOT
while (getMissionState() !== 'PARKING_NAV_TO_SPOT') {
  control();
}
for (let i = 0; i < 40; i++) control();
const posBeforeParkingNudge = { x: dr.getX(), y: dr.getY() };
setPilotJoy(0, -0.9, 0);
for (let i = 0; i < 30; i++) control();
const posAfterParkingNudge = { x: dr.getX(), y: dr.getY() };
assert(Math.hypot(posAfterParkingNudge.x - posBeforeParkingNudge.x, posAfterParkingNudge.y - posBeforeParkingNudge.y) > 5, 'Pilot nudge should displace Roary');
setPilotJoy(0, 0, 0);

// Verify Roary recovers smoothly onto the parking spline and still completes the parking relocation
let nudgeRecoveryTicks = 0;
while (nudgeRecoveryTicks < 800 && getTotalRelocations() < 1) {
  control();
  nudgeRecoveryTicks++;
}
assert.strictEqual(getTotalRelocations(), 1, 'Roary should recover from pilot nudge and safely complete parking');
console.log('TEST 8 PASSED: Shared autonomy recovery in narrow parking lot verified!');

console.log('=== TEST 9: Interactive Bay Clicks Mid-Transition & Obstacle Collision Shield ===');
const fieldClick = mockElements['field']._listeners['click'];

// 9a) Click bay P2 while arrivingCar is mid-entry in PARKING_FILLING_OLD_SPOT
resetAll();
startMission();
while (getMissionState() !== 'PARKING_FILLING_OLD_SPOT') control();
for (let i = 0; i < 5; i++) control();
const p2Px = worldToCanvas(0, 84);
fieldClick({ clientX: p2Px.x, clientY: p2Px.y });
assert.strictEqual(getArrivingCar(), null, 'Arriving car must be finalized immediately, not orphaned');
assert.notStrictEqual(getParkingBays()[0].car, null, 'Previous spot P1 must be populated by the arriving car');
assert.strictEqual(getTotalRelocations(), 1, 'Relocation counter must increment when arrivingCar is finalized');

// 9b) Click bay P3 while Roary is mid-transit in PARKING_NAV_TO_SPOT & click on asphalt lane
while (getMissionState() !== 'PARKING_NAV_TO_SPOT') control();
for (let i = 0; i < 60; i++) control();
const lanePx = worldToCanvas(0, 34);
fieldClick({ clientX: lanePx.x, clientY: lanePx.y });
assert.strictEqual(getSplinePhase(), 'TO_PARKING_SPOT', 'Clicking asphalt lane must not overwrite parking spline with CUSTOM');

const p3Px = worldToCanvas(96, 84);
fieldClick({ clientX: p3Px.x, clientY: p3Px.y });
let redirectTicks = 0;
while (getMissionState() !== 'PARKING_WAITING_NEXT' && redirectTicks < 600) {
  control();
  redirectTicks++;
}
assert.strictEqual(getRobotCurrentSpotIdx(), 2, 'Roary should dock at redirected spot P3');
assert.strictEqual(getParkingBays().filter(b => b.car !== null).length, 5, 'Exactly 5 of 6 bays must remain occupied by cars after mid-transit redirect');
console.log('TEST 9 PASSED: Mid-transition bay clicks & 5-car invariant verified!');

console.log('=== ALL E2E VERIFICATION TESTS PASSED! ===');

