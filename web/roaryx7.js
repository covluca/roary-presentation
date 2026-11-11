// Roaryx7 Drive Controller + Roary Subsystems — faithful JavaScript port.
//
// Source of truth (read-only reference):
//   doritoPower/Roaryx7.java          → splineToDX / sendPower / goToV2 / catmullRom* / IK / claw-outTake offsets
//   autonom/Kirby/KirbyAirRiders.java → mainDrive / AUTO_STATE / tremor / waypoints
//   hardware/math/Point.java          → Point (note: atan() = atan2(x, y)  — x FIRST)
//   hardware/math/Pose2d.java         → Pose2d
//   hardware/math/Meth.java           → inRange / FSqrt / isRobotInBigTriangle / isRobotInSmallTriangle
//   doritoPower/goToValues.java       → goToValues
//   hardware/Globals.java             → Octav / Macara / Kirby constants (HEXT, RAIL, BASC, CLAW, Codes)
//   hardware/subsystems/Decode/Kirby/IntakeKirby.java → unloadManager / ocProtection / spit-on-full / Unstuck
//
// This file ports the DRIVE LOGIC 1:1. It does NOT contain the motor/encoder
// hardware: the physics (wheel power → world motion) lives in sim.js and uses
// the odometry's own world transform so that this code drives the robot the
// same way it does on the real robot.

'use strict';

// ─── Math helpers (Roaryx7.java + Meth.java) ────────────────────────────────

/** Roaryx7.java:AngleWrap — wrap an angle (radians) into [-π, π]. */
function angleWrap(angle) {
  while (angle < -Math.PI) angle += 2.0 * Math.PI;
  while (angle > Math.PI) angle -= 2.0 * Math.PI;
  return angle;
}

/**
 * Roaryx7.java:twoXSqrd(x, vMin)
 * Quadratic "better-P" interpolator. |x|<... → vMin*sign, clamps to ±1.
 */
function twoXSqrd(x, vMin) {
  if (x === 0) return 0;
  const speed = 1.55 * x * x;
  const sign = x / Math.abs(x);
  if (vMin > Math.abs(speed)) return vMin * sign;
  else if (Math.abs(speed) > 1) return sign;
  else return speed * sign;
}

/**
 * Roaryx7.java:twoXSqrd(x, vMin, error)
 * Same as above but deadbands |x| < error → 0.
 */
function twoXSqrdErr(x, vMin, error) {
  if (Math.abs(x) < error) return 0;
  const speed = 1.55 * x * x;
  const sign = x / Math.abs(x);
  if (vMin > Math.abs(speed)) return vMin * sign;
  else if (Math.abs(speed) > 1) return sign;
  else return speed * sign;
}

/** Java Math.toRadians / Math.toDegrees (no JS equivalent). */
const toRadians = (deg) => deg * Math.PI / 180;
const toDegrees = (rad) => rad * 180 / Math.PI;

/** Meth.java:inRange */
function inRange(x, min, max) {
  if (min > max) { const t = min; min = max; max = t; }
  return Math.max(Math.min(x, max), min);
}

/** Meth.java:isInRange */
function isInRange(x, min, max) {
  if (min > max) { const t = min; min = max; max = t; }
  return x >= min && x <= max;
}

/** Meth.java:FSqrt (Newton sqrt, ported for fidelity) */
function FSqrt(a) {
  if (a === 0) return 0;
  let x = a / 2;
  const epsilon = 1e-9;
  for (let i = 0; i < 8; ++i) {
    const nx = (x + a / x) / 2;
    if (Math.abs(nx - x) < epsilon) break;
    x = nx;
  }
  return x;
}

/** Meth.java:61-114 — Triangle zone checks (area-sum test on all 4 robot corners). */
function triangleArea(first, second, third) {
  return Math.abs((first.x * (second.y - third.y) + second.x * (third.y - first.y) + third.x * (first.y - second.y)) / 2.0);
}

function isInsideTriangle(first, second, third, x) {
  const A = triangleArea(first, second, third);
  const A1 = triangleArea(x, second, third);
  const A2 = triangleArea(first, x, third);
  const A3 = triangleArea(first, second, x);
  return Math.abs(A - (A1 + A2 + A3)) < 1e-4;
}

function isRobotInTriangle(f, s, t, robot, width, length) {
  const rad = toRadians(-robot.heading);
  const FL = new Point(-width / 2, length / 2).rotate(rad).add(robot);
  const FR = new Point(width / 2, length / 2).rotate(rad).add(robot);
  const BL = new Point(-width / 2, -length / 2).rotate(rad).add(robot);
  const BR = new Point(width / 2, -length / 2).rotate(rad).add(robot);
  return isInsideTriangle(f, s, t, FL) || isInsideTriangle(f, s, t, FR) ||
         isInsideTriangle(f, s, t, BL) || isInsideTriangle(f, s, t, BR);
}

/** Meth.java:97 — Top half triangle: (-180,180), (180,180), (0,0) */
function isRobotInBigTriangle(robot, width = 35, length = 38) {
  return isRobotInTriangle(new Point(-180, 180), new Point(180, 180), new Point(0, 0), robot, width, length);
}

/** Meth.java:106 — Bottom small triangle: (-60,-180), (60,-180), (0,-120) */
function isRobotInSmallTriangle(robot, width = 35, length = 38) {
  return isRobotInTriangle(new Point(-60, -180), new Point(60, -180), new Point(0, -120), robot, width, length);
}

// ─── Point / Pose2d (Point.java / Pose2d.java) ──────────────────────────────

class Point {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  /** Point.java:polar  →  (cos(a)·r, sin(a)·r) */
  static polar(r, angle) {
    return new Point(Math.cos(angle) * r, Math.sin(angle) * r);
  }

  subtract(other) { return new Point(this.x - other.x, this.y - other.y); }
  add(other)      { return new Point(this.x + other.x, this.y + other.y); }
  divide(d)       { return new Point(this.x / d, this.y / d); }
  multiply(m)     { return new Point(this.x * m, this.y * m); }

  distanceTo(other) { return other.subtract(this).radius(); }

  /** Point.java:atan  →  atan2(x, y)   (x FIRST — non-standard, must match Java) */
  atan() { return Math.atan2(this.x, this.y); }

  /** Point.java:atan2(other) → atan2(other.x - x, other.y - y) */
  atan2(other) { return Math.atan2(other.x - this.x, other.y - this.y); }

  radius() { return Math.hypot(this.x, this.y); }

  /** Point.java:rotate → polar(radius(), atan() + amount) */
  rotate(amount) { return Point.polar(this.radius(), this.atan() + amount); }

  toString() { return 'x=' + this.x + ', y=' + this.y; }
}

class Pose2d extends Point {
  // Matches the Java overloads:
  //   Pose2d(double x, double y, double heading)
  //   Pose2d(Point p, double heading)
  constructor(a, b, c) {
    if (a instanceof Point) {
      super(a.x, a.y);
      this.heading = (typeof b === 'number') ? b : (a instanceof Pose2d ? a.heading : 0);
    } else {
      super((typeof a === 'number') ? a : 0, (typeof b === 'number') ? b : 0);
      this.heading = (typeof c === 'number') ? c : 0;
    }
  }
  distanceToPose(other) { return other.subtract(this).radius(); }
  toString() { return `${this.x.toFixed(2)} ${this.y.toFixed(2)} ${this.heading.toFixed(2)}`; }
}

// ─── goToValues (goToValues.java) ───────────────────────────────────────────

class goToValues {
  constructor(target, speed, error, init = null) {
    if (target instanceof goToValues) {
      this.target = new Pose2d(target.target.x, target.target.y, target.target.heading);
      this.speed = target.speed;
      this.error = target.error;
      this.init = target.init;
    } else {
      this.target = target;   // Pose2d
      this.speed = speed;
      this.error = error;
      this.init = init;
    }
  }
}

// ─── Roaryx7 drive controller ───────────────────────────────────────────────

class Roaryx7Drive {
  constructor(robotName = 'Kirby') {
    this.robotName = robotName;

    // Position / heading (cm, radians)
    this.X = 0;
    this.Y = 0;
    this.H = 0;

    // Robot velocity in the world frame (Point) — fed to splineToDX as `vel`
    this.vel = new Point(0, 0);

    // Roary core
    this.goToK = 0; // 1 = auto power enabled, 0 = stick only  (toggled by B / button)
    this.powerCompensator = 1;
    this.batteryVoltage = 14.0;
    this.TESTINGROARY = false;

    // mainDrive "power" ramp (KirbyAirRiders.mainDrive)
    this.power = 1;

    // Motor powers
    this.goTo = { FLPow: 0, FRPow: 0, BLPow: 0, BRPow: 0 };
    this.mainDrivePowers = { FLPow: 0, FRPow: 0, BLPow: 0, BRPow: 0 };
    this.FL = 0; this.FR = 0; this.BL = 0; this.BR = 0;

    // goTo / sendPower state
    this.target = new Pose2d(0, 0, 0);
    this.goToDelta = new Point(0, 0);
    this.lastDirection = new Point(0, 0);
    this.deltaH = 0;
    this.pathH = 0;
    this.interpolator = 0;
    this.headingPower = 0;
    this.transInter = 1;
    this.tVR = 0;
    this.rotX = 0;
    this.rotY = 0;
    this.doneHeading = false;
    this.doneTranslation = false;
    this.doneHeadingGoTo = false;
    this.doneGoTo = false;

    // spline state
    this.splineFirstTime = true;
    this.splineDone = false;
    this.doneTranslationSpline = false;
    this.firstTime = true;
    this.useInterpolator = false;
    this.BRAKEPLS = true;
    this.futurePos = new Point(0, 0);
    this.t = 0;
    this.i = 0;
    this.localT = 0;
    this.length = 0;
    this.startPoint = null;
    this.endPoint = null;
    this.splinePoints = null;
    this.lookBehind = null;
    this.lookAhead = null;
    this.derivative = null;
    this.normalLeft = null;
    this.normalRight = null;
    this.derivativeGlobal = null;
    this.normalGlobalLeft = null;
    this.normalGlobalRight = null;
    this.derivativePower = null;
    this.normalPower = null;
    this.projectionPower = null;
    this.totalPower = null;
    this.projectedP = null;
    this.curveBreak = 0;
    this.normalInterpolator = 0;
    this.projectionInterpolator = 0;
    this.powerMixer = 0;
    this.breakInterpolator = 0;
    this.breakPower = 0;
    this.distanceToEnd = 0;
    this.directionToEnd = 0;
    this.speed = 0;
    this.extTelemetry = 0;
  }

  // ── Accessors ──────────────────────────────────────────────────────────
  getX() { return this.X; }
  getY() { return this.Y; }
  getH() { return this.H * 180 / Math.PI; }          // degrees (Roaryx7.getH)
  getHRadians() { return this.H; }
  getPoint() { return new Point(this.X, this.Y); }
  getPose() { return new Pose2d(this.X, this.Y, this.H * 180 / Math.PI); }

  setPosition(x, y, hRadians) {
    this.X = x; this.Y = y; this.H = hRadians;
  }

  // ── Roary core ─────────────────────────────────────────────────────────
  toggleGoToK() { this.goToK = this.goToK === 0 ? 1 : 0; }
  setGoToK(enable) { this.goToK = enable ? 1 : 0; }

  updatePowerCompensator() {
    this.powerCompensator = Math.max(1, 14.0 / this.batteryVoltage);
  }

  stopMotors() { this.FL = 0; this.FR = 0; this.BL = 0; this.BR = 0; }

  /** Reset the spline follower so the next splineToDX re-initializes. */
  resetSpline() {
    this.splineFirstTime = true;
    this.splineDone = false;
    this.doneTranslationSpline = false;
    this.doneGoTo = false;
    this.doneTranslation = false;
    this.doneHeading = false;
    this.doneHeadingGoTo = false;
    this.t = 0; this.i = 0; this.localT = 0;
    this.futurePos = new Point(this.X, this.Y);
  }

  // ── Catmull-Rom (Roaryx7.java:2207-2223, 2153-2205) ───────────────────
  catmullRomSpline(points, t) {
    const a1 = points[0], p1 = points[1], p2 = points[2], a2 = points[3];
    const q0 = (-1 * t ** 3) + (2 * t ** 2) + (-1 * t);
    const q1 = (3 * t ** 3) + (-5 * t ** 2) + 2;
    const q2 = (-3 * t ** 3) + (4 * t ** 2) + t;
    const q3 = (t ** 3) - (t ** 2);
    return new Point(
      0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3)),
      0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3))
    );
  }

  catmullRomDerivative(points, t) {
    const a1 = points[0], p1 = points[1], p2 = points[2], a2 = points[3];
    const q0 = (-3 * t * t) + (4 * t) - 1;
    const q1 = (9 * t * t) + (-10 * t);
    const q2 = (-9 * t * t) + (8 * t) + 1;
    const q3 = (3 * t * t) - (2 * t);
    return new Point(
      0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3)),
      0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3))
    );
  }

  catmullRomNormalLeft(points, t) {
    const a1 = points[0], p1 = points[1], p2 = points[2], a2 = points[3];
    const q0 = (-3 * t * t) + (4 * t) - 1;
    const q1 = (9 * t * t) + (-10 * t);
    const q2 = (-9 * t * t) + (8 * t) + 1;
    const q3 = (3 * t * t) - (2 * t);
    return new Point(
      -0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3)),
      0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3))
    );
  }

  catmullRomNormalRight(points, t) {
    const a1 = points[0], p1 = points[1], p2 = points[2], a2 = points[3];
    const q0 = (-3 * t * t) + (4 * t) - 1;
    const q1 = (9 * t * t) + (-10 * t);
    const q2 = (-9 * t * t) + (8 * t) + 1;
    const q3 = (3 * t * t) - (2 * t);
    return new Point(
      0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3)),
      -0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3))
    );
  }

  // ── Geometry helpers (Roaryx7.java:345-372, 1289-1301) ────────────────
  dotProduct(v1, v2) { return v1.x * v2.x + v1.y * v2.y; }

  projectionOnLine(v1, v2, p) {
    const e1 = new Point(v2.x - v1.x, v2.y - v1.y);
    const e2 = new Point(p.x - v1.x, p.y - v1.y);
    const valDp = this.dotProduct(e1, e2);
    const lenLineE1 = Math.sqrt(e1.x * e1.x + e1.y * e1.y);
    const lenLineE2 = Math.sqrt(e2.x * e2.x + e2.y * e2.y);
    const cos = valDp / (lenLineE1 * lenLineE2);
    const projLenOfLine = cos * lenLineE2;
    return new Point(
      Math.trunc(v1.x + (projLenOfLine * e1.x) / lenLineE1),
      Math.trunc(v1.y + (projLenOfLine * e1.y) / lenLineE1)
    );
  }

  closestT(splinePoints) {
    let closest = 100;
    let closestT = 0;
    for (let t = 0; t <= 1.05; t += 0.05) {
      if (this.getPoint().distanceTo(this.catmullRomSpline(splinePoints, t)) <= closest) {
        closest = this.getPoint().distanceTo(this.catmullRomSpline(splinePoints, t));
        closestT = t;
      }
      if (closest <= 3) break;
    }
    return closestT;
  }

  // ── splineToDX (Roaryx7.java:1490-1642) — the real spline follower ────
  splineToDX(goToValuesArr, vel) {
    if (this.splineFirstTime) {
      this.t = 0;
      this.i = 0;
      this.localT = 0;
      this.firstTime = false;
      this.doneGoTo = true;
      this.splineDone = false;
      this.splineFirstTime = false;
      this.useInterpolator = false;
      this.doneTranslationSpline = false;

      if (goToValuesArr.length === 1) {
        this.length = 4;
        this.startPoint = new Point(this.getX(), this.getY());
        this.endPoint = new Point(goToValuesArr[0].target.x, goToValuesArr[0].target.y);
        this.target = goToValuesArr[0].target;
        this.speed = goToValuesArr[0].speed;
      } else {
        this.length = goToValuesArr.length;
      }
    }

    this.t = this.i + this.localT;
    if (this.localT >= 1 && this.t <= this.length - 3.3) this.i++;

    if (goToValuesArr.length > 1) {
      this.splinePoints = [
        new Point(goToValuesArr[this.i].target.x, goToValuesArr[this.i].target.y),
        new Point(goToValuesArr[this.i + 1].target.x, goToValuesArr[this.i + 1].target.y),
        new Point(goToValuesArr[this.i + 2].target.x, goToValuesArr[this.i + 2].target.y),
        new Point(goToValuesArr[this.i + 3].target.x, goToValuesArr[this.i + 3].target.y),
      ];
      const headingPoints = [
        new Point(goToValuesArr[this.i].target.heading, 0),
        new Point(goToValuesArr[this.i + 1].target.heading, 0),
        new Point(goToValuesArr[this.i + 2].target.heading, 0),
        new Point(goToValuesArr[this.i + 3].target.heading, 0),
      ];
      const speedPoints = [
        new Point(goToValuesArr[this.i].speed, 0),
        new Point(goToValuesArr[this.i + 1].speed, 0),
        new Point(goToValuesArr[this.i + 2].speed, 0),
        new Point(goToValuesArr[this.i + 3].speed, 0),
      ];
      this.localT = this.closestT(this.splinePoints);

      const interH = this.catmullRomSpline(headingPoints, this.localT);
      const interS = this.catmullRomSpline(speedPoints, this.localT);
      this.speed = interS.x;

      this.target = new Pose2d(this.catmullRomSpline(this.splinePoints, this.localT), interH.x);
    } else {
      this.splinePoints = [this.startPoint, this.startPoint, this.endPoint, this.endPoint];
      this.localT = this.closestT(this.splinePoints);
      this.target = new Pose2d(this.catmullRomSpline(this.splinePoints, this.localT), goToValuesArr[0].target.heading);
      this.speed = goToValuesArr[0].speed;
    }

    this.distanceToEnd = goToValuesArr[goToValuesArr.length - 1].target.distanceToPose(this.target);

    if (this.doneTranslationSpline) {
      this.splineDone = true;
      return;
    } else {
      this.splineDone = false;
    }

    this.lookBehind = this.catmullRomSpline(this.splinePoints, this.localT - 0.2 * (this.length - 3));
    this.lookAhead = this.catmullRomSpline(this.splinePoints, this.localT + 0.2 * (this.length - 3));

    this.derivative = this.catmullRomDerivative(this.splinePoints, this.localT);
    this.normalLeft = this.catmullRomNormalLeft(this.splinePoints, this.localT);
    this.normalRight = this.catmullRomNormalRight(this.splinePoints, this.localT);

    this.derivativeGlobal = this.catmullRomSpline(this.splinePoints, this.localT).add(this.derivative);
    this.normalGlobalLeft = this.catmullRomSpline(this.splinePoints, this.localT).add(this.normalLeft);
    this.normalGlobalRight = this.catmullRomSpline(this.splinePoints, this.localT).add(this.normalRight);

    if (this.getPoint().distanceTo(this.target) <= 4.25) this.normalInterpolator = 0;
    else this.normalInterpolator = this.getPoint().distanceTo(this.target) / 48;

    /// PUTEREA NORMALA — pushes the robot back onto the spline
    if (this.getPoint().distanceTo(this.normalGlobalLeft) > this.getPoint().distanceTo(this.normalGlobalRight)) {
      this.normalPower = Point.polar(-Math.min(twoXSqrdErr(this.normalInterpolator, 0.17, 0.05), this.speed), this.normalLeft.atan());
      this.isLeftNormal = true;
    } else {
      this.normalPower = Point.polar(-Math.min(twoXSqrdErr(this.normalInterpolator, 0.17, 0.05), this.speed), this.normalRight.atan());
      this.isLeftNormal = false;
    }

    /// PUTEREA CENTRIPETA
    const projectedP = this.projectionOnLine(this.normalGlobalLeft, this.normalGlobalRight, this.lookAhead);
    this.projectedP = projectedP;
    if (projectedP.distanceTo(this.target) <= 5) this.projectionInterpolator = 0;
    else this.projectionInterpolator = inRange(projectedP.distanceTo(this.target) / 33, -0.64, 0.64);

    if (projectedP.distanceTo(this.normalGlobalLeft) > projectedP.distanceTo(this.normalGlobalRight)) {
      this.projectionPower = Point.polar(-Math.min(twoXSqrdErr(this.projectionInterpolator, 0.13, 0.05), this.speed - this.normalPower.radius()), this.normalRight.atan());
      this.isLeftProjection = false;
    } else {
      this.projectionPower = Point.polar(-Math.min(twoXSqrdErr(this.projectionInterpolator, 0.13, 0.05), this.speed - this.normalPower.radius()), this.normalLeft.atan());
      this.isLeftProjection = true;
    }

    this.breakInterpolator = 1;
    this.breakPower = 1;
    this.distanceToEnd = goToValuesArr[goToValuesArr.length - 1].target.distanceToPose(this.target);
    this.directionToEnd = this.getPoint().distanceTo(this.lookAhead) - this.getPoint().distanceTo(this.lookBehind);

    this.breakInterpolator = Math.min(this.distanceToEnd / 71, 1);
    this.breakPower = twoXSqrd(this.breakInterpolator, 0.8);

    this.powerMixer = 0;
    const endTarget = goToValuesArr[goToValuesArr.length - 1].target;
    if (this.distanceToEnd <= 6.4 || (endTarget.distanceTo(this.futurePos) < 33 && vel.radius() < 33)) {
      this.doneTranslationSpline = true;
    } else if (!this.doneTranslationSpline) {
      this.breakPower = twoXSqrd(this.breakInterpolator, 0.8);
      this.futurePos = this.getPoint().add(vel.divide(8));

      if (endTarget.distanceTo(this.futurePos) < 8) {
        this.BRAKEPLS = true;
        this.breakPower = 0;
      }
      if (endTarget.distanceTo(this.futurePos) < 35) {
        this.breakPower = Math.min(this.breakPower, 0.35);
      } else {
        this.BRAKEPLS = false;
      }

      this.powerMixer = Math.max(Math.sqrt(Math.pow(this.speed, 2) - Math.pow(this.normalPower.radius() - 0.0000001, 2)) * this.breakPower, this.BRAKEPLS ? 0 : 0.24);
    }

    const curveBreak = Math.min(twoXSqrdErr(Math.abs(this.projectionInterpolator), 0.08, 0.02), 0.17);
    this.curveBreak = curveBreak;
    this.derivativePower = Point.polar(-this.powerMixer + Math.min(curveBreak, this.powerMixer), this.derivative.atan());

    if (!this.doneTranslationSpline)
      this.totalPower = this.normalPower.add(this.derivativePower).add(this.projectionPower);
    else
      this.totalPower = new Point(0, 0);

    if (!this.splineDone) this.sendPower(this.target, this.totalPower);
  }

  // ── sendPower (Roaryx7.java:1644-1788) ────────────────────────────────
  // Two Java overloads:
  //   sendPower(Pose2d target, Point direction)   — splineToDX uses this
  //   sendPower(Pose2d roaryTarget)               — direct "roary space" vector + heading (e.g. tremor)
  sendPower(target, direction) {
    if (direction === undefined) {
      return this.sendPowerRoarySpace(target);
    }
    this.lastDirection = direction;

    this.deltaH = angleWrap(toRadians(target.heading) - this.H);

    this.interpolator = this.deltaH / (Math.PI / 2.4);

    if (Math.abs(toDegrees(this.deltaH)) > 2.4 && Math.abs(target.heading) <= 180) {
      this.doneHeading = false;
      this.headingPower = twoXSqrd(this.interpolator,
        ((this.robotName === 'Clippy' || this.robotName === 'Kathy' || this.robotName === 'Kirby' || this.robotName === 'Sasha') ? 0.17 : 0.24) *
        (this.robotName === 'Macara' ? -1 : 1));
    } else {
      this.headingPower = 0;
      this.doneHeading = true;
    }

    const tVR = -(-direction.atan() - Math.PI / 2 - this.H) + Math.PI / 4;

    if (this.robotName === 'Echo' || this.robotName === 'Clippy' || this.robotName === 'Kathy' || this.robotName === 'Kirby') {
      this.rotX = direction.radius() * Math.cos(tVR) - direction.radius() * Math.sin(tVR);
      this.rotY = direction.radius() * Math.sin(tVR) + direction.radius() * Math.cos(tVR);
    } else {
      this.rotX = -direction.radius() * Math.cos(tVR) + direction.radius() * Math.sin(tVR);
      this.rotY = -direction.radius() * Math.sin(tVR) - direction.radius() * Math.cos(tVR);
    }

    const denominator = Math.max(Math.abs(this.rotY) + Math.abs(this.rotX) + Math.abs(this.headingPower), 1);
    this.goTo.FLPow = (this.rotY + this.rotX + this.headingPower) * this.powerCompensator / denominator;
    this.goTo.FRPow = (this.rotY - this.rotX - this.headingPower) * this.powerCompensator / denominator;
    this.goTo.BLPow = (this.rotY - this.rotX + this.headingPower) * this.powerCompensator / denominator;
    this.goTo.BRPow = (this.rotY + this.rotX - this.headingPower) * this.powerCompensator / denominator;

    this._composeAndSend();
  }

  /** Roaryx7.java:1716-1788 — sendPower(Pose2d roaryTarget) overload ("roary space", used in tremor / nudge). */
  sendPowerRoarySpace(roaryTarget) {
    const target = new Point(roaryTarget.x, roaryTarget.y).rotate(toRadians(180));
    this.lastDirection = target;

    const deltaHDeg = toDegrees(angleWrap(toRadians(roaryTarget.heading) - this.H));
    this.deltaH = toRadians(deltaHDeg);

    this.interpolator = inRange(deltaHDeg / 89, -0.89, 0.89);

    if (Math.abs(deltaHDeg) > 1.3 && Math.abs(roaryTarget.heading) <= 180) {
      this.doneHeading = false;
      this.headingPower = twoXSqrd(this.interpolator, 0.17) * (this.robotName === 'Macara' ? -1 : 1);
    } else {
      this.headingPower = 0;
      this.doneHeading = true;
    }

    const tVR = -(-target.atan() - Math.PI / 2 - this.H) + Math.PI / 4;
    if (this.robotName === 'Echo' || this.robotName === 'Clippy' || this.robotName === 'Kathy' || this.robotName === 'Kirby') {
      this.rotX = target.radius() * Math.cos(tVR) - target.radius() * Math.sin(tVR);
      this.rotY = target.radius() * Math.sin(tVR) + target.radius() * Math.cos(tVR);
    } else {
      this.rotX = -target.radius() * Math.cos(tVR) + target.radius() * Math.sin(tVR);
      this.rotY = -target.radius() * Math.sin(tVR) - target.radius() * Math.cos(tVR);
    }

    const denominator = Math.max(Math.abs(this.rotY) + Math.abs(this.rotX) + Math.abs(this.headingPower), 1);
    this.goTo.FLPow = (this.rotY + this.rotX + this.headingPower) * this.powerCompensator / denominator;
    this.goTo.FRPow = (this.rotY - this.rotX - this.headingPower) * this.powerCompensator / denominator;
    this.goTo.BLPow = (this.rotY - this.rotX + this.headingPower) * this.powerCompensator / denominator;
    this.goTo.BRPow = (this.rotY + this.rotX - this.headingPower) * this.powerCompensator / denominator;

    this._composeAndSend();
  }

  // ── goToV2 (Roaryx7.java:1841-1947) — final approach / parking ─────────
  goToV2(target, speed, error) {
    const min = Math.min, abs = Math.abs;

    this.goToDelta = target.subtract(this.getPoint());

    this.deltaH = angleWrap(toRadians(target.heading) - this.H);
    this.pathH = this.goToDelta.atan() - this.H;

    this.interpolator = this.deltaH / (Math.PI * 2.0 / 3.0);

    if (Math.abs(toDegrees(this.deltaH)) > 0.8 * error) {
      this.doneHeadingGoTo = false;
      const ultimateMinH = 0.134;
      this.headingPower = twoXSqrd(this.interpolator, 0.134) * (this.robotName === 'Macara' ? -1 : 1);
      if (abs(this.headingPower) < ultimateMinH)
        this.headingPower = ultimateMinH * this.headingPower / abs(this.headingPower);
    } else {
      this.headingPower = 0;
      this.doneHeadingGoTo = true;
    }

    const SAFE_ZONE_RADIUS = 24;
    const transInterp = min(this.goToDelta.radius() / SAFE_ZONE_RADIUS, 1);
    this.transInter = twoXSqrd(transInterp, 0.22);

    this.tVR = -this.pathH + Math.PI / 4;

    if (this.robotName === 'Echo' || this.robotName === 'Clippy' || this.robotName === 'Kathy' || this.robotName === 'Kirby') {
      this.globalSpeed = -speed * this.transInter;
    } else {
      this.globalSpeed = speed * this.transInter;
    }

    const errX = -this.goToDelta.radius() * Math.cos(this.tVR) + this.goToDelta.radius() * Math.sin(this.tVR);
    const errY = -this.goToDelta.radius() * Math.sin(this.tVR) - this.goToDelta.radius() * Math.cos(this.tVR);

    this.rotX = -this.globalSpeed * Math.cos(this.tVR) + this.globalSpeed * Math.sin(this.tVR);
    this.rotY = -this.globalSpeed * Math.sin(this.tVR) - this.globalSpeed * Math.cos(this.tVR);

    const ultimateMinX = 0.22;
    const ultimateMinY = 0.17;
    if (abs(this.rotX) < ultimateMinX && this.rotX !== 0) this.rotX = ultimateMinX * this.rotX / abs(this.rotX);
    if (abs(this.rotY) < ultimateMinY && this.rotY !== 0) this.rotY = ultimateMinY * this.rotY / abs(this.rotY);

    if (this.goToDelta.radius() < error) {
      this.rotX = 0;
      this.rotY = 0;
      this.doneTranslation = true;
    } else {
      this.doneTranslation = false;
    }

    if (this.doneTranslation && this.doneHeadingGoTo && this.doneGoTo) {
      this.fastSendPower();
      return;
    }

    if (abs(errX) < error) this.rotX = 0;
    if (abs(errY) < error) this.rotY = 0;

    const denominator = Math.max(Math.abs(this.rotY) + Math.abs(this.rotX) + Math.abs(this.headingPower), 1);
    this.goTo.FLPow = (this.rotY + this.rotX + this.headingPower) * this.powerCompensator / denominator;
    this.goTo.FRPow = (this.rotY - this.rotX - this.headingPower) * this.powerCompensator / denominator;
    this.goTo.BLPow = (this.rotY - this.rotX + this.headingPower) * this.powerCompensator / denominator;
    this.goTo.BRPow = (this.rotY + this.rotX - this.headingPower) * this.powerCompensator / denominator;

    this._composeAndSend();

    this.doneGoTo = this.doneTranslation && this.doneHeadingGoTo;
  }

  // ── fastSendPower (Roaryx7.java:1790-1816) ────────────────────────────
  fastSendPower() {
    let FLPower = 0, FRPower = 0, BLPower = 0, BRPower = 0;
    if (this.mainDrivePowers.FLPow !== null) {
      FLPower = this.mainDrivePowers.FLPow;
      FRPower = this.mainDrivePowers.FRPow;
      BLPower = this.mainDrivePowers.BLPow;
      BRPower = this.mainDrivePowers.BRPow;
    }

    const Normalizer = Math.max(Math.max(Math.abs(FLPower), Math.abs(FRPower)), Math.max(Math.abs(BLPower), Math.abs(BRPower)));
    if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
      FLPower /= Normalizer; FRPower /= Normalizer; BLPower /= Normalizer; BRPower /= Normalizer;
    }

    if (!this.TESTINGROARY) {
      this.FL = FLPower; this.FR = FRPower; this.BL = BLPower; this.BR = BRPower;
    }
  }

  // ── Shared: compose goTo·goToK + mainDrivePowers → motors (sendPower:1690-1711) ──
  _composeAndSend() {
    if (this.mainDrivePowers.FLPow === null) {
      this.mainDrivePowers.FLPow = 0;
      this.mainDrivePowers.FRPow = 0;
      this.mainDrivePowers.BLPow = 0;
      this.mainDrivePowers.BRPow = 0;
    }

    let FLPower = this.goTo.FLPow * this.goToK + this.mainDrivePowers.FLPow;
    let FRPower = this.goTo.FRPow * this.goToK + this.mainDrivePowers.FRPow;
    let BLPower = this.goTo.BLPow * this.goToK + this.mainDrivePowers.BLPow;
    let BRPower = this.goTo.BRPow * this.goToK + this.mainDrivePowers.BRPow;

    const Normalizer = Math.max(Math.max(Math.abs(FLPower), Math.abs(FRPower)), Math.max(Math.abs(BLPower), Math.abs(BRPower)));
    if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
      FLPower /= Normalizer; FRPower /= Normalizer; BLPower /= Normalizer; BRPower /= Normalizer;
    }

    if (!this.TESTINGROARY) {
      this.FL = FLPower; this.FR = FRPower; this.BL = BLPower; this.BR = BRPower;
    }
  }

  // ── mainDrive (KirbyAirRiders.java:1059-1107) ─────────────────────────
  // stickX/stickY = left stick, rotate = net trigger rotation.
  mainDrive(stickX, stickY, rotate) {
    const abs = Math.abs;

    const speed = Math.hypot(stickX, stickY);
    const robotAngle = Math.atan2(stickY, stickX) - Math.PI / 4;
    const TrigRotate = rotate * 0.98;

    if (abs(stickX) < 0.02 && abs(stickY) < 0.02 && abs(TrigRotate) < 0.02) {
      this.mainDrivePowers.FLPow = 0;
      this.mainDrivePowers.FRPow = 0;
      this.mainDrivePowers.BLPow = 0;
      this.mainDrivePowers.BRPow = 0;
      return;
    }

    if (abs(speed) > 0.89 && this.power < 1.7) {
      const delta = abs(speed) * 0.03;
      this.power = this.power + delta;
    } else if (abs(speed) < 0.89) {
      this.power = 1;
    }

    let FLPow = -speed * Math.sin(robotAngle) * this.power + TrigRotate;
    let FRPow = -speed * Math.cos(robotAngle) * this.power - TrigRotate;
    let BLPow = -speed * Math.cos(robotAngle) * this.power + TrigRotate;
    let BRPow = -speed * Math.sin(robotAngle) * this.power - TrigRotate;

    let Normalizer = Math.max(Math.max(abs(FLPow), abs(FRPow)), Math.max(abs(BLPow), abs(BRPow)));
    if (Normalizer > 1) {
      Normalizer = 1 / Normalizer;
      FLPow *= Normalizer; FRPow *= Normalizer; BLPow *= Normalizer; BRPow *= Normalizer;
    }

    this.mainDrivePowers.FLPow = FLPow;
    this.mainDrivePowers.FRPow = FRPow;
    this.mainDrivePowers.BLPow = BLPow;
    this.mainDrivePowers.BRPow = BRPow;
  }

  // ── End-effector offset kinematics (Roaryx7.java:2268-2456) ──────────
  // These compute where the robot's front claw (intake) or back outTake / specimen
  // sits in world coordinates given slide extension `ext` (cm), or inversely
  // where the robot center must go so the claw/outTake lands on a target.

  /** Roaryx7.java:2335 — given desired claw Pose2d + extension, return robot center goToValues */
  clawToRobotPos(claw, ext) {
    const goToVals = new goToValues(new Pose2d(claw.target.x, claw.target.y, claw.target.heading), claw.speed, claw.error);
    const relRobotPos = new Point(0, -(30 + ext));
    const radHeading = angleWrap(toRadians(-goToVals.target.heading - 90));
    const rot = relRobotPos.rotate(radHeading);
    goToVals.target.x += rot.x;
    goToVals.target.y += rot.y;
    return goToVals;
  }

  /** Roaryx7.java:2351 — given robot Pose2d + extension, return world Pose2d of the front claw */
  robotToClawPos(robot, ext) {
    const claw = new Pose2d(robot.x, robot.y, robot.heading);
    const relRobotPos = new Point(0, (30 + ext));
    const radHeading = angleWrap(toRadians(-claw.heading - 90));
    const rot = relRobotPos.rotate(radHeading);
    claw.x += rot.x;
    claw.y += rot.y;
    return claw;
  }

  /** Roaryx7.java:2368 — specimen hook offset (14.5, -21.5) → robot target */
  specimenToRobotPos(specimen) {
    const goToVals = new goToValues(new Pose2d(specimen.target.x, specimen.target.y, specimen.target.heading), specimen.speed, specimen.error);
    const relSpecPos = new Point(14.5, -21.5);
    const radHeading = angleWrap(toRadians(-goToVals.target.heading - 90));
    const rot = relSpecPos.rotate(radHeading);
    goToVals.target.x += rot.x;
    goToVals.target.y += rot.y;
    return goToVals;
  }

  /** Roaryx7.java:2385 — robot Pose2d → world Pose2d of the specimen hook */
  robotToSpecimenPos(robot) {
    const specimen = new Pose2d(robot.x, robot.y, robot.heading);
    const relSpecPos = new Point(-14.5, 21.5);
    const radHeading = angleWrap(toRadians(-specimen.heading - 90));
    const rot = relSpecPos.rotate(radHeading);
    specimen.x += rot.x;
    specimen.y += rot.y;
    return specimen;
  }

  /** Roaryx7.java:2411 — back outTake target (with 70.1° slide angle) → robot center target */
  outTakeToRobotPos(specimen, ext) {
    const goToVals = new goToValues(new Pose2d(specimen.target.x, specimen.target.y, specimen.target.heading), specimen.speed, specimen.error);
    const relSpecPos = new Point(0, -(ext + 17) * Math.cos(toRadians(70.1)));
    const radHeading = angleWrap(toRadians(-goToVals.target.heading + 90));
    const rot = relSpecPos.rotate(radHeading);
    goToVals.target.x += rot.x;
    goToVals.target.y += rot.y;
    return goToVals;
  }

  /** Roaryx7.java:2427 — robot Pose2d + extension → world Pose2d of the back outTake */
  robotToOutTakePos(robot, ext) {
    const outTake = new Pose2d(robot.x, robot.y, robot.heading);
    const relSpecPos = new Point(0, (ext + 17));
    const radHeading = angleWrap(toRadians(-outTake.heading + 90));
    const rot = relSpecPos.rotate(radHeading);
    outTake.x += rot.x;
    outTake.y += rot.y;
    return outTake;
  }

  /** Roaryx7.java:2300 — polar extension finder from robot to a field element */
  polarFinder(robot, element) {
    const robotClone = new Point(robot.x, robot.y);
    const elemClone = new Point(element.x, element.y);
    return Point.polar(Math.abs(robotClone.distanceTo(elemClone) - 31.9), angleWrap(-robotClone.atan2(elemClone) - toRadians(90)));
  }
}

// ─── Roary Subsystem Architecture (2D-visualizable managers) ────────────────
//
// In the Java codebase, a robot (Kirby.java) composes 4 subsystem objects:
//   • drive   (Roaryx7)
//   • sensors (Sensors — 3 color/distance slots, odometry, limelight)
//   • intake  (Intake* — intake rollers, paddles/indexer, horizontal slide hExt, claw)
//   • outTake (OutTake* — transfer rail, bascula/pivot, outtake claw, Farul status LED)
//
// Every subsystem method is NON-BLOCKING and called EVERY TICK from runOpMode().
// Multi-step sequences use the Roary state-machine idiom:
//   • `firstTime` boolean (arm the sequence: set currentAction = 1, done = false, firstTime = false)
//   • `currentAction` int (1 → 2 → 3 → …)
//   • `delayTime = runtime + Δt` and `safetyTime = runtime + T_max` (non-blocking timers)
//   • `done` boolean (signals completion to the parent opMode or sibling manager)
//   • `reserveIntake` / `hasSpit` / `ocTime` (inter-subsystem coordination & protections)

const inIndexer = Object.freeze({
  NOTHING: 'NOTHING',
  GREEN: 'GREEN',
  PURPLE: 'PURPLE',
  ROBOT: 'ROBOT',
});

const PaddleState = Object.freeze({
  FORCED_LOAD: 'FORCED_LOAD',
  LOAD: 'LOAD',
  UNLOAD: 'UNLOAD',
  FORCED_UNLOAD: 'FORCED_UNLOAD',
  RESERVED: 'RESERVED',
});

// Globals.Macara / Globals.Octav constants for the 2D-visible mechanisms
const MechGlobals = Object.freeze({
  HEXT_UNLOAD_CM: 0,
  HEXT_LOAD_CM: 37,         // Globals.Macara.HEXT_LOAD_CM (front slide max extension in cm)
  RAIL_MIN_CM: 0,
  RAIL_LOAD_CM: 6,
  RAIL_MAX_CM: 18,          // rear outtake extension in cm for top-down visibility
  BASC_LOAD: 0.25,
  BASC_UNLOAD: 0.65,
  CLAW_OPEN: 0.55,
  CLAW_CLOSED: 0.95,
});

// Globals.Kirby.Codes — "Farul" status indicator LED
const FarulCodes = Object.freeze({
  OFF:    { code: 0,     name: 'OFF',    hex: '#484f58', reason: 'Idle' },
  RED:    { code: 0.283, name: 'RED',    hex: '#f85149', reason: 'Not ready / out of zone' },
  YELLOW: { code: 0.388, name: 'YELLOW', hex: '#d29922', reason: 'Partial ready (1 condition met)' },
  GREEN:  { code: 0.5,   name: 'GREEN',  hex: '#3fb950', reason: 'READY (in zone + mechanism ready)' },
  INDIGO: { code: 0.66,  name: 'INDIGO', hex: '#6e7681', reason: 'Cycle complete!' },
  PURPLE: { code: 0.71,  name: 'PURPLE', hex: '#bc8cff', reason: 'New element detected!' },
  WHITE:  { code: 1.0,   name: 'WHITE',  hex: '#f0f6fc', reason: 'SLOTS FULL — go score!' },
});

/**
 * Sensors.java (Kirby/Kathy) — 3-slot indexer + round-robin slot polling (loopNr % 3).
 */
class RoarySensors {
  constructor() {
    this.slots = [inIndexer.NOTHING, inIndexer.NOTHING, inIndexer.NOTHING]; // [Mid, Left, Right]
    this.whatIsInThisSlot = inIndexer.NOTHING;
    this.newElement = false;
    this.pendingDetect = [inIndexer.NOTHING, inIndexer.NOTHING, inIndexer.NOTHING];
    this.lastPolledSlot = 0;
  }

  /** Sensors.java:87 — called once per tick with `loopNr % 3` (round-robin sensor read) */
  updateSlots(slot) {
    this.newElement = false;
    this.lastPolledSlot = slot;
    const check = (idx) => {
      const detected = this.pendingDetect[idx];
      this.whatIsInThisSlot = detected;
      if (detected !== inIndexer.NOTHING && detected !== this.slots[idx]) {
        this.newElement = true;
      }
      if (detected !== inIndexer.NOTHING) {
        this.slots[idx] = detected;
        this.pendingDetect[idx] = inIndexer.NOTHING;
      }
    };
    if (slot >= 0 && slot <= 2) check(slot);
    else if (slot === -1) { check(0); check(1); check(2); }
  }

  /** Feed an element from the field into the first empty slot (sensed on next round-robin tick). */
  senseElement(color) {
    const order = [0, 1, 2]; // Mid, Left, Right
    for (const idx of order) {
      if (this.slots[idx] === inIndexer.NOTHING && this.pendingDetect[idx] === inIndexer.NOTHING) {
        this.pendingDetect[idx] = color;
        return idx;
      }
    }
    return -1;
  }

  areSlotsEmpty() {
    return this.slots[0] === inIndexer.NOTHING &&
           this.slots[1] === inIndexer.NOTHING &&
           this.slots[2] === inIndexer.NOTHING &&
           this.pendingDetect[0] === inIndexer.NOTHING &&
           this.pendingDetect[1] === inIndexer.NOTHING &&
           this.pendingDetect[2] === inIndexer.NOTHING;
  }

  areSlotsFull() {
    return this.slots[0] !== inIndexer.NOTHING &&
           this.slots[1] !== inIndexer.NOTHING &&
           this.slots[2] !== inIndexer.NOTHING;
  }

  howManyBalls() {
    return (this.slots[0] !== inIndexer.NOTHING ? 1 : 0) +
           (this.slots[1] !== inIndexer.NOTHING ? 1 : 0) +
           (this.slots[2] !== inIndexer.NOTHING ? 1 : 0);
  }

  firstOccupiedSlot() {
    for (let i = 0; i < 3; i++) if (this.slots[i] !== inIndexer.NOTHING) return i;
    return -1;
  }

  lastEmptySlot() {
    for (let i = 2; i >= 0; i--) if (this.slots[i] === inIndexer.NOTHING) return i;
    return -1;
  }

  wipeSlots() {
    this.slots[0] = this.slots[1] = this.slots[2] = inIndexer.NOTHING;
    this.pendingDetect[0] = this.pendingDetect[1] = this.pendingDetect[2] = inIndexer.NOTHING;
  }

  fillSlotsRobot() {
    for (let i = 0; i < 3; i++) {
      if (this.slots[i] === inIndexer.NOTHING || this.slots[i] === inIndexer.ROBOT) {
        this.slots[i] = (i === 1) ? inIndexer.GREEN : inIndexer.PURPLE;
      }
    }
  }
}

/**
 * OutTake subsystem (OutTakeKathy / OutTakeKirby + Globals.Macara/Octav).
 *
 * Instead of the 3D ballistics shooter (which cannot be seen in 2D top-down),
 * this exposes the 2D-visible mechanisms present in Roaryx7.java & Globals.java:
 *   • Rear outTake extension slide / rail (`outExtCm`, rendered via `robotToOutTakePos`)
 *   • Bascula pivot (`bascPos`: LOAD → HOLD → UNLOAD) & outtake claw (`clawOpen`)
 *   • OutTake readiness (`velocityDone` / `turretDone` equivalents: slide extended + aligned)
 *   • `Farul` status LED (`statusCode(pose)` — exact port of OutTakeKirby.java:211-228)
 */
class RoaryOutTake {
  constructor(sensors) {
    this.sensors = sensors;
    this.outExtCm = 0;            // current rear slide extension (0 .. RAIL_MAX_CM)
    this.targetOutExtCm = 0;
    this.bascState = 'LOAD';      // LOAD | TRANSFER | UNLOAD
    this.clawState = 'OPEN';      // OPEN | CLOSED | OPEN_BASKET
    this.holdingColors = [];      // elements transferred into the outtake basket/carrier
    this.velocityEnabled = false; // in 2D: outtake arming enabled
    this.velocityDone = false;    // in 2D: outtake slide reached target extension
    this.turretDone = true;       // in 2D: aligned with scoring zone
    this.flashTime = 0;
    this.farul = FarulCodes.RED;
    this.scoredDrops = [];        // [{ x, y, color, t }] placed on field
  }

  startLauncher() {
    this.velocityEnabled = true;
    this.targetOutExtCm = MechGlobals.RAIL_MAX_CM;
    this.bascState = 'UNLOAD';
  }

  stopLauncher() {
    this.velocityEnabled = false;
    this.targetOutExtCm = 0;
    this.bascState = 'LOAD';
    this.clawState = 'OPEN';
  }

  /** Step physical slide extension + readiness flags every tick (non-blocking iterative control). */
  step(dt, drive) {
    const target = this.velocityEnabled ? MechGlobals.RAIL_MAX_CM : this.targetOutExtCm;
    const err = target - this.outExtCm;
    const maxStep = 55 * dt; // cm/s slide speed
    if (Math.abs(err) <= maxStep) this.outExtCm = target;
    else this.outExtCm += Math.sign(err) * maxStep;

    this.velocityDone = this.velocityEnabled && Math.abs(this.outExtCm - MechGlobals.RAIL_MAX_CM) < 1.2;
    const pose = drive.getPose();
    this.turretDone = isRobotInBigTriangle(pose, 35, 38) || isRobotInSmallTriangle(pose, 35, 38) || this.velocityEnabled;
  }

  /** OutTakeKirby.java:211-228 — exact status LED priority logic ("Farul"). */
  statusCode(robotPose, runtime) {
    const inZone = isRobotInBigTriangle(robotPose, 35, 35) || isRobotInSmallTriangle(robotPose, 35, 35);
    if (this.sensors.areSlotsFull() && runtime > this.flashTime + 0.48 && !inZone) {
      this.farul = FarulCodes.WHITE;
      this.flashTime = runtime + 0.48;
    }
    if (this.sensors.newElement) {
      this.farul = FarulCodes.PURPLE;
      this.flashTime = runtime + 0.3;
    }
    if (runtime < this.flashTime) {
      // keep flashed color (PURPLE / WHITE / INDIGO) until flashTime expires
    } else if (this.velocityDone && this.turretDone && inZone) {
      this.farul = FarulCodes.GREEN;
    } else if (this.velocityDone || inZone) {
      this.farul = FarulCodes.YELLOW;
    } else {
      this.farul = FarulCodes.RED;
    }
  }
}

/**
 * Intake subsystem (IntakeKirby.java + IntakeKathy.java + Globals.Macara/Octav).
 *
 * Implements the real Roary non-blocking managers & protections:
 *   1. `hExtCm` — front horizontal slide (0 .. 37 cm, rendered via `robotToClawPos`)
 *   2. `MID`, `LEFT`, `RIGHT` PaddleState (LOAD, UNLOAD, FORCED_UNLOAD, FORCED_LOAD, RESERVED)
 *   3. `ocProtection(runtime)` — IntakeKirby.java:146-154 over-current > 7.1A auto-reverse for 0.08s
 *   4. Spit-on-full — KirbyTester.java:408-421 auto-reverse (+1) for 0.24s when 3/3 slots fill
 *   5. `Unstuck(runtime)` — IntakeKathy.java:114-153 4-step non-blocking unstuck state machine
 *   6. `unloadManager(runtime)` — IntakeKathy.java:499-594 6-step transfer & drop state machine
 *   7. `launchControl(runtime, additional, motif, drive)` — IntakeKirby.java:416-507 5-step paddle sequence
 */
class RoaryIntake {
  constructor(sensors, outtake) {
    this.sensors = sensors;
    this.outtake = outtake;

    // Rollers & front extension slide
    this.intakePower = 0;         // commanded auto power (-1 = intake/eat, +1 = spit, 0 = off)
    this.motorOutput = 0;         // actual power sent to Intake motor after OC protection
    this.currentAmps = 1.2;       // simulated motor current (spikes on jam)
    this.hExtCm = 0;              // front slide extension (0 .. 37 cm)
    this.targetHExtCm = 0;
    this.clawClosed = false;

    // 3 Paddles (Mid=0, Left=1, Right=2)
    this.MID = PaddleState.LOAD;
    this.LEFT = PaddleState.LOAD;
    this.RIGHT = PaddleState.LOAD;

    // Over-current protection (IntakeKirby.java:146)
    this.ocTime = 0;
    this.jamSimulated = false;

    // Spit-on-full (KirbyTester.java:408)
    this.spitTime = 0;
    this.hasSpit = false;

    // Unstuck manager (IntakeKathy.java:114-153)
    this.unstuckFirstTime = false;
    this.doneUnstuck = true;
    this.stuckAction = 0;
    this.stuckTime = 0;
    this.unstucker = false;

    // Unload / Transfer manager (IntakeKathy.java:499-594)
    this.unloadFirstTime = false;
    this.unloadDone = true;
    this.currentActionUnload = 0;
    this.delayTimeUnload = 0;
    this.safetyTime = 0;
    this.reserveIntake = false;

    // Launch / Score paddle sequence (IntakeKirby.java:416-507)
    this.launchFirstTime = true;
    this.launchDone = true;
    this.currentActionLaunch = 0;
    this.delayTimeLaunch = 0;
    this.currentPaddle = 0;
    this.launchCondition = false;
    this.shotNr = 0;
  }

  /** IntakeKirby.java:135 — manual stick only drives intake if auto intakePower == 0 */
  setPowerIntake(speed) {
    if (this.intakePower === 0) this.motorOutput = speed;
  }

  /** IntakeKirby.java:140 — MINUS MANANCA (-1 = intake in, +1 = spit out) */
  setPowerIntakeAuto(speed) {
    this.intakePower = speed;
    this.motorOutput = speed;
  }

  setHExtCm(cm) {
    this.targetHExtCm = inRange(cm, MechGlobals.HEXT_UNLOAD_CM, MechGlobals.HEXT_LOAD_CM);
  }

  updateMid(state)   { this.MID = state; }
  updateLeft(state)  { this.LEFT = state; }
  updateRight(state) { this.RIGHT = state; }

  updateIntake(paddleIdx, state) {
    if (paddleIdx === 0) this.updateMid(state);
    else if (paddleIdx === 1) this.updateLeft(state);
    else if (paddleIdx === 2) this.updateRight(state);
  }

  /** Trigger a simulated jam (current > 7.1A) to showcase ocProtection() + Unstuck(). */
  triggerJam() {
    this.jamSimulated = true;
    this.currentAmps = 9.4; // > 7.1A threshold in IntakeKirby.java:147
  }

  /** IntakeKirby.java:146-154 — Overcurrent protection (> 7.1A → reverse at +1 for 0.08s). */
  ocProtection(runtime) {
    if (this.currentAmps > 7.1) {
      this.ocTime = runtime + 0.08;
      this.jamSimulated = false;
    }
    if (this.ocTime > runtime) {
      this.motorOutput = 1; // reverse to clear jam!
      this.currentAmps = 4.2;
    } else {
      if (this.intakePower !== 0) this.motorOutput = this.intakePower;
      this.currentAmps = Math.abs(this.motorOutput) * 3.8 + (this.motorOutput !== 0 ? 0.6 : 0.1);
    }
  }

  /** IntakeKathy.java:114-153 — 4-step non-blocking Unstuck state machine. */
  startUnstuck() {
    this.unstuckFirstTime = true;
    this.doneUnstuck = false;
  }

  Unstuck(runtime) {
    if (this.unstuckFirstTime) {
      this.stuckAction = 1;
      this.doneUnstuck = false;
      this.unstucker = !this.unstucker;
      this.unstuckFirstTime = false;
    }
    if (this.doneUnstuck || this.stuckAction === 0) return;

    if (this.stuckAction === 1) {
      this.setPowerIntakeAuto(0.8); // reverse rollers
      this.updateMid(PaddleState.RESERVED);
      this.updateLeft(this.unstucker ? PaddleState.FORCED_UNLOAD : PaddleState.LOAD);
      this.updateRight(this.unstucker ? PaddleState.LOAD : PaddleState.FORCED_UNLOAD);
      this.stuckTime = runtime + 0.18;
      this.stuckAction++;
    }
    if (this.stuckAction === 2 && runtime > this.stuckTime) {
      this.setPowerIntakeAuto(-0.8); // pull back in
      this.updateLeft(this.unstucker ? PaddleState.LOAD : PaddleState.FORCED_UNLOAD);
      this.updateRight(this.unstucker ? PaddleState.FORCED_UNLOAD : PaddleState.LOAD);
      this.stuckTime = runtime + 0.18;
      this.stuckAction++;
    }
    if (this.stuckAction === 3 && runtime > this.stuckTime) {
      this.updateMid(PaddleState.LOAD);
      this.updateLeft(PaddleState.LOAD);
      this.updateRight(PaddleState.LOAD);
      this.stuckTime = runtime + 0.13;
      this.stuckAction++;
    }
    if (this.stuckAction === 4 && runtime > this.stuckTime) {
      this.setPowerIntakeAuto(0);
      this.doneUnstuck = true;
      this.stuckAction++;
    }
  }

  /** IntakeKathy.java:482-594 — 6-step non-blocking unloadManager() (Transfer → Extend → Place → Retract). */
  startUnload() {
    if (this.sensors.firstOccupiedSlot() !== -1) {
      this.unloadFirstTime = true;
      this.unloadDone = false;
    }
  }

  unloadManager(runtime, drive) {
    if (this.unloadFirstTime) {
      this.unloadDone = false;
      this.currentActionUnload = 1;
      this.unloadFirstTime = false;
    }
    if (this.currentActionUnload === 0 || this.currentActionUnload > 6) return;

    // Action 1: Reserve intake, retract front slide, start transfer rollers
    if (this.currentActionUnload === 1 && this.sensors.firstOccupiedSlot() !== -1) {
      this.reserveIntake = true;
      this.setHExtCm(0);
      this.setPowerIntakeAuto(-0.64);
      this.outtake.bascState = 'TRANSFER';
      this.outtake.clawState = 'CLOSED';
      this.safetyTime = runtime + 0.8;
      this.delayTimeUnload = runtime + 0.22;
      this.currentActionUnload++;
    } else if (this.currentActionUnload === 1) {
      this.currentActionUnload = 6;
    }

    // Action 2: Extend rear outtake rail + move slot into carrier
    if (this.currentActionUnload === 2 && runtime > this.delayTimeUnload) {
      this.outtake.startLauncher();
      this.setPowerIntakeAuto(-0.71);
      const slot = this.sensors.firstOccupiedSlot();
      if (slot !== -1) {
        this.updateIntake(slot, PaddleState.UNLOAD);
        this.outtake.holdingColors.push(this.sensors.slots[slot]);
        this.sensors.slots[slot] = inIndexer.NOTHING;
      }
      this.safetyTime = runtime + 0.8;
      this.delayTimeUnload = runtime + 0.24;
      this.currentActionUnload++;
    }

    // Action 3: Wait for transfer clearance
    if (this.currentActionUnload === 3 && runtime > this.delayTimeUnload) {
      this.setPowerIntakeAuto(-0.64);
      this.delayTimeUnload = runtime + 0.24;
      this.safetyTime = runtime + 1.7;
      this.currentActionUnload++;
    }

    // Action 4: Wait for outtake extension (`velocityDone`) or safetyTime, then release into basket!
    if (this.currentActionUnload === 4 && runtime > this.delayTimeUnload && (this.outtake.velocityDone || runtime > this.safetyTime)) {
      this.outtake.clawState = 'OPEN_BASKET';
      const outPose = drive.robotToOutTakePos(drive.getPose(), this.outtake.outExtCm);
      while (this.outtake.holdingColors.length > 0) {
        const col = this.outtake.holdingColors.shift();
        const jitter = (Math.random() - 0.5) * 6;
        this.outtake.scoredDrops.push({ x: outPose.x + jitter, y: outPose.y + jitter, color: col, t: runtime });
      }
      this.delayTimeUnload = runtime + 0.3;
      this.safetyTime = runtime + 0.8;
      this.motorOutput = -0.53;
      this.currentActionUnload++;
    }

    // Action 5: Mark unloadDone = true (allows auto state machine to start driving early!) while finishing retract timer
    if (this.currentActionUnload === 5 && runtime > this.delayTimeUnload) {
      this.motorOutput = -0.53;
      this.updateMid(PaddleState.LOAD);
      this.updateLeft(PaddleState.LOAD);
      this.updateRight(PaddleState.LOAD);
      this.currentActionUnload++;
      this.delayTimeUnload = runtime + 0.45;
      this.unloadDone = true;
    }

    // Action 6: Retract outtake rail, stop rollers, release reservation
    if (this.currentActionUnload === 6 && runtime >= this.delayTimeUnload) {
      this.outtake.stopLauncher();
      this.setPowerIntakeAuto(0);
      this.reserveIntake = false;
      this.currentActionUnload++;
    }
  }

  /**
   * IntakeKirby.java:416-507 — 5-step non-blocking launchControl(additional, motif).
   * Selects the slot matching the motif order (PURPLE/GREEN), then cycles the paddle:
   *   Action 1 (when launchCondition) → FORCED_UNLOAD (+0.12s)
   *   Action 2 → UNLOAD (+0.08s)
   *   Action 3 → FORCED_LOAD (+0.08s)
   *   Action 4 → LOAD (+0.05s)
   *   Action 5 → clear slot, place element at outTake world pose, re-arm for next slot!
   */
  launchControl(runtime, additional, motif, drive) {
    if (this.launchFirstTime) {
      this.launchDone = false;

      if (this.sensors.slots[0] !== inIndexer.NOTHING) this.currentPaddle = 0;
      else if (this.sensors.slots[1] !== inIndexer.NOTHING) this.currentPaddle = 1;
      else if (this.sensors.slots[2] !== inIndexer.NOTHING) this.currentPaddle = 2;
      else {
        this.shotNr = 0;
        this.launchDone = true;
        this.outtake.stopLauncher();
        return;
      }

      // Motif color ordering (IntakeKirby.java:432-444)
      if (motif !== -1 && this.shotNr === motif - 1) {
        for (let i = 0; i <= 2; i++) {
          if (this.sensors.slots[i] === inIndexer.GREEN) this.currentPaddle = i;
        }
      } else if (motif !== -1) {
        for (let i = 0; i <= 2; i++) {
          if (this.sensors.slots[i] === inIndexer.PURPLE) this.currentPaddle = i;
        }
      }

      this.outtake.startLauncher();
      this.launchCondition = this.outtake.velocityDone && this.outtake.turretDone;
      this.currentActionLaunch = 1;
      this.launchFirstTime = false;
    }

    this.launchCondition = this.outtake.velocityDone && this.outtake.turretDone && additional;

    if (this.currentActionLaunch === 1 && this.launchCondition) {
      this.updateIntake(this.currentPaddle, PaddleState.FORCED_UNLOAD);
      this.outtake.bascState = 'UNLOAD';
      this.outtake.clawState = 'CLOSED';
      this.delayTimeLaunch = runtime + 0.14;
      this.currentActionLaunch++;
    }
    if (this.currentActionLaunch === 2 && runtime > this.delayTimeLaunch) {
      this.updateIntake(this.currentPaddle, PaddleState.UNLOAD);
      this.outtake.clawState = 'OPEN_BASKET';
      this.delayTimeLaunch = runtime + 0.09;
      this.currentActionLaunch++;
    }
    if (this.currentActionLaunch === 3 && runtime > this.delayTimeLaunch) {
      this.updateIntake(this.currentPaddle, PaddleState.FORCED_LOAD);
      this.delayTimeLaunch = runtime + 0.09;
      this.currentActionLaunch++;
    }
    if (this.currentActionLaunch === 4 && runtime > this.delayTimeLaunch) {
      this.updateIntake(this.currentPaddle, PaddleState.LOAD);
      this.delayTimeLaunch = runtime + 0.06;
      this.currentActionLaunch++;
    }
    if (this.currentActionLaunch === 5 && runtime > this.delayTimeLaunch) {
      const col = this.sensors.slots[this.currentPaddle];
      this.sensors.slots[this.currentPaddle] = inIndexer.NOTHING;
      const outPose = drive.robotToOutTakePos(drive.getPose(), this.outtake.outExtCm);
      const offset = (this.shotNr - 1) * 5.5;
      this.outtake.scoredDrops.push({
        x: outPose.x + offset,
        y: outPose.y - offset * 0.5,
        color: col,
        t: runtime,
      });
      this.launchFirstTime = true;
      this.shotNr++;
      this.currentActionLaunch++;
    }
  }

  /** Advance front slide hExt physical position smoothly each tick. */
  stepSlide(dt) {
    // Auto-extend front slide slightly when intaking, or track targetHExtCm
    const desired = this.targetHExtCm;
    const err = desired - this.hExtCm;
    const maxStep = 65 * dt;
    if (Math.abs(err) <= maxStep) this.hExtCm = desired;
    else this.hExtCm += Math.sign(err) * maxStep;
  }
}

// ── Exports (plain scripts, shared global scope) ────────────────────────────
window.Point = Point;
window.Pose2d = Pose2d;
window.goToValues = goToValues;
window.Roaryx7Drive = Roaryx7Drive;
window.angleWrap = angleWrap;
window.twoXSqrd = twoXSqrd;
window.twoXSqrdErr = twoXSqrdErr;
window.inRange = inRange;
window.isInRange = isInRange;
window.FSqrt = FSqrt;
window.toRadians = toRadians;
window.toDegrees = toDegrees;
window.isRobotInBigTriangle = isRobotInBigTriangle;
window.isRobotInSmallTriangle = isRobotInSmallTriangle;
window.inIndexer = inIndexer;
window.PaddleState = PaddleState;
window.MechGlobals = MechGlobals;
window.FarulCodes = FarulCodes;
window.RoarySensors = RoarySensors;
window.RoaryOutTake = RoaryOutTake;
window.RoaryIntake = RoaryIntake;
