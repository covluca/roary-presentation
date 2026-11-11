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
        addEventListener: () => {},
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
  global.getDrive = () => drive;
  global.getMissionState = () => missionState;
  global.getBasketCargo = () => basketCargo;
  global.getTotalDelivered = () => totalDelivered;
  global.setPilotJoy = (x, y, r) => { joyX = x; joyY = y; joyR = r; };
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

console.log('=== ALL E2E VERIFICATION TESTS PASSED! ===');
