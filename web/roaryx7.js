// Roaryx7 Drive Controller — faithful JavaScript port of the Java holonomic drive.
//
// Source of truth (read-only reference):
//   doritoPower/Roaryx7.java          → splineToDX / sendPower / goToV2 / catmullRom* / helpers
//   autonom/Kirby/KirbyAirRiders.java → mainDrive (stick → mainDrivePowers)
//   hardware/math/Point.java          → Point (note: atan() = atan2(x, y)  — x FIRST)
//   hardware/math/Pose2d.java         → Pose2d
//   hardware/math/Meth.java           → inRange / FSqrt
//   doritoPower/goToValues.java       → goToValues
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
      this.heading = (typeof b === 'number') ? b : 0;
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
    this.target = target;   // Pose2d
    this.speed = speed;
    this.error = error;
    this.init = init;
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
    this.normalInterpolator = 0;
    this.projectionInterpolator = 0;
    this.powerMixer = 0;
    this.breakInterpolator = 0;
    this.breakPower = 0;
    this.distanceToEnd = 0;
    this.directionToEnd = 0;
    this.speed = 0;
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
    this.futurePos = new Point(0, 0);
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
    const min = Math.min, max = Math.max, sqrt = Math.sqrt, pow = Math.pow;

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
    this.derivativePower = Point.polar(-this.powerMixer + Math.min(curveBreak, this.powerMixer), this.derivative.atan());

    if (!this.doneTranslationSpline)
      this.totalPower = this.normalPower.add(this.derivativePower).add(this.projectionPower);
    else
      this.totalPower = new Point(0, 0);

    if (!this.splineDone) this.sendPower(this.target, this.totalPower);
  }

  // ── sendPower (Roaryx7.java:1644-1714) ────────────────────────────────
  sendPower(target, direction) {
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

  // ── goToV2 (Roaryx7.java:1841-1947) — final approach / parking ─────────
  goToV2(target, speed, error) {
    const min = Math.min, max = Math.max, abs = Math.abs;

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
    const min = Math.min, max = Math.max, abs = Math.abs;

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
window.FSqrt = FSqrt;
