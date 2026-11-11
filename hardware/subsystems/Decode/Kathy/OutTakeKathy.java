package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kathy;


import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.Globals.Kathy.*;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isInRange;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Point;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;


public class OutTakeKathy {

    //VAL 1620
    //public static double TICKS_PER_CM_LIFT = 8.3;

    //VAL 1150
    public static double TICKS_PER_ROTATION_PER_MINUTE = 28.0 / 60.0;
    public static double TICKS_PER_ROTATION_PER_MINUTE_SECONDARY = 103.8 / 60.0;

    public LinearOpMode opmode;

    public Sensors sensors;

    public Telemetry telemetry;


    public double initValBasc = 0;

    public DcMotorEx PLauncher;
    public DcMotorEx SLauncher;
    public Servo Turela = null;
    public Servo Elevatie = null;


    public TurretPos turretPos = TurretPos.STRAIGHT;
    public ElevationPos elevationPos = ElevationPos.MIN;
    public double stopTime = 0;
    public double attemptToTarget = 0;
    public double bestAttemptToTarget = 0;
    public Point bestDelta = new Point();
    public double railCM = 0;
    public int loopNr = 0;

    public double setRPM = 0;
    public double setPower = 0;

    public Point e1 = new Point();
    public Point e2 = new Point();
    public Point projectedOnBasket = new Point();
    public Point bestAttempt = new Point();
    public double u = 0;
    public double angle = 0;
    public double range = 0;
    public double relTurretAngle = 0;
    public Point turret = new Point();
    public double calcutedRPM = 0;
    public double calculatedPower = 0;
    public double safety = 0;
    public double speedTimer = 0;
    public double deltaRPM = 0;
    public double speedStabilizer = 0.08;
    public double interpolatorRPM = 0;
    public double speedLauncher = 0;
    public boolean velocityDone = false;
    public double batteryVoltage = 0;
    public double turret_compensator = 0;

    public boolean velocityEnabled = false;

    public OutTakeKathy(HardwareMap hardwareMap, LinearOpMode opmode, Sensors sensors) {

        PLauncher = hardwareMap.get(DcMotorEx.class, "PLauncher");
        SLauncher = hardwareMap.get(DcMotorEx.class, "SLauncher");

        Turela = hardwareMap.get(Servo.class, "Turela");
        Elevatie = hardwareMap.get(Servo.class, "Elevatie");

        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        SLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.opmode = opmode;
        this.telemetry = opmode.telemetry;

        this.sensors = sensors;

    }

    public static double twoXSqrd(double x, double vMin) {
        //functia asta e facuta pt interpolatoare cu val intre -1 si 1
        // e un P din Pid mai bun, tinde mereu spre eroare 0
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;

        //1.55 means that when x is 0.8 speed is 1, it's better than both 1 or 2
        if (x == 0) return 0;
        double speed = 1.55 * x * x;
        double sign = x / abs(x);
        if (vMin > Math.abs(speed)) return vMin * sign;
        else if (Math.abs(speed) > 1) return sign;
        else return speed * sign;
    }

    /*
    public void setVelocityIterative() {

        deltaRPM = setRPM - getRPM();
        double BREAK_RPM = 130;
        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, -0.8, 1);
        speedLauncher = twoXSqrd(interpolatorRPM, 0.12);
        if (isInRange(speedLauncher, 0.08, 0.53)) speedLauncher = 0.53;
        speedLauncher = max(speedLauncher, -0.8);

        telemetry.addData("deltaRPM", deltaRPM);
        telemetry.addData("interpolatorRPM", interpolatorRPM);
        telemetry.addData("speedLauncher", speedLauncher);

        if (abs(deltaRPM) < 64) {
            telemetry.addData("isAtRPM", true);
            velocityDone = true;
            PLauncher.setPower(0);
            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
        } else if (setRPM > 100) {
            telemetry.addData("isAtRPM", false);
            velocityDone = false;
            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PLauncher.setPower(speedLauncher);
        } else {
            velocityDone = false;
        }

    }
*/
    public void setVelocityIterative() {


        if (!velocityEnabled) {
            SLauncher.setPower(0);
            PLauncher.setPower(0);
            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            velocityDone = false;
            return;
        }


        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SLauncher.setPower(calculatedPower);


        deltaRPM = calcutedRPM - getRPM();
        double BREAK_RPM = 240;
        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, -0.8, 1);
        speedLauncher = twoXSqrd(interpolatorRPM, 0.24);
        if (isInRange(speedLauncher, -0.08, 1.2)) speedLauncher = 1.2;
        speedLauncher = max(speedLauncher, -0.8);

        telemetry.addData("deltaRPM", deltaRPM);
        telemetry.addData("interpolatorRPM", interpolatorRPM);
        telemetry.addData("speedLauncher", speedLauncher);

        if (abs(deltaRPM) < 80) { //isInRange(deltaDeltaRPM, -89,60)
            telemetry.addData("isAtRPM", true);
            velocityDone = true;
            PLauncher.setPower(0);
            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
        } else if (calcutedRPM > 100) {
            telemetry.addData("isAtRPM", false);
            velocityDone = false;
            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PLauncher.setPower(speedLauncher);
        } else {
            velocityDone = false;
        }
    }

    public void setVelocityIterativeEDX() {

        if (!velocityEnabled) {
            SLauncher.setPower(0);
            PLauncher.setPower(0);
            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            velocityDone = false;
            return;
        }


        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SLauncher.setPower(calculatedPower);


        deltaRPM = calcutedRPM - getRPM();
        double BREAK_RPM = 130;
        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, -0.8, 1);
        speedLauncher = twoXSqrd(interpolatorRPM, 0.12);
        if (isInRange(speedLauncher, 0.08, 0.53)) speedLauncher = 0.53;
        speedLauncher = max(speedLauncher, -0.8);

        telemetry.addData("deltaRPM", deltaRPM);
        telemetry.addData("interpolatorRPM", interpolatorRPM);
        telemetry.addData("speedLauncher", speedLauncher);

        if (abs(deltaRPM) < 64) {
            telemetry.addData("isAtRPM", true);
            velocityDone = true;
            PLauncher.setPower(0);
            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
        } else if (calcutedRPM > 100) {
            telemetry.addData("isAtRPM", false);
            velocityDone = false;
            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PLauncher.setPower(speedLauncher);
        } else {
            velocityDone = false;
        }

    }


    public void initOuttake() {
        Turela.setPosition(TURRET_STRAIGHT);
        Elevatie.setPosition(ELEVATION_MIN);

    }

    public void updateOutake(TurretPos turretPos) {
        this.turretPos = turretPos;
        switch (turretPos) {
            case STRAIGHT:
                Turela.setPosition(TURRET_STRAIGHT);
                break;
            case LEFT:
                Turela.setPosition(TURRET_LEFT);
                break;
            case RIGHT:
                Turela.setPosition(TURRET_RIGHT);
                break;

        }
    }

    public void updateOutake(ElevationPos elevationPos) {
        this.elevationPos = elevationPos;
        switch (elevationPos) {
            case MIN:
                Elevatie.setPosition(ELEVATION_MIN);
                break;
            case SHOOT_NEAR:
                Elevatie.setPosition(ELEVATION_MIN);
                break;
            case SHOOT_FAR:
                Elevatie.setPosition(ELEVATION_MAX);
                break;
            case MAX:
                Elevatie.setPosition(ELEVATION_MAX);
                break;
        }
    }

    public void toggleElevation() {
        switch (elevationPos) {
            case MIN:
            case SHOOT_NEAR:
                Elevatie.setPosition(ELEVATION_MAX);
                elevationPos = ElevationPos.SHOOT_FAR;
                break;
            case SHOOT_FAR:
            case MAX:
                Elevatie.setPosition(ELEVATION_MIN);
                elevationPos = ElevationPos.SHOOT_NEAR;
                break;
        }
    }

    public double degreesCompensator(double degrees) {
        //precalculated conversion function
        /// This has huge chances of eating my days
        return degrees * 1.1 - 3.3;
    }

/*
    public double calculateRailPoint(Point target, Pose2d robot, double targetDistance) {
        //this figures out how much the rail needs to be extended to be at the targetDistance from the target
        Point localTarget = Point.polar(target.distanceTo(robot), robot.atan2(target));

        ///Daca nu gasesti nimic sub un metru, trimite pozitia de load
        double bestMatch = 100;
        double bestRailDistance = RAIL_LOAD_CM;
        for (double i = RAIL_MIN_CM; i <= RAIL_MAX_CM; i += 0.5) {
            Point attempt = Point.polar(-i, toRadians(-robot.heading + 90));
            attemptToTarget = attempt.distanceTo(localTarget);
            if (attempt.distanceTo(localTarget) < bestMatch && attempt.distanceTo(localTarget) > targetDistance) {
                bestMatch = attempt.distanceTo(localTarget);
                bestRailDistance = i;
            }
        }
        bestAttemptToTarget = bestMatch;
        return bestRailDistance;
    }

    public double calculateRailBasket(Point target, Pose2d robot, double targetDistance) {
        //this figures out how much the rail needs to be extended to be at the targetDistance from the target
        ///Daca nu gasesti nimic sub un metru, trimite pozitia de load
        loopNr = 0;
        boolean stopShip = false;
        double bestMatch = 100;
        double bestRailDistance = RAIL_LOAD_CM;

        for (double b = 0; b <= 8.5; b++) {
            Point delta = new Point(b, b);
            Point localTarget = Point.polar(target.add(delta).distanceTo(robot), robot.atan2(target.add(delta)));

            for (double i = RAIL_MAX_CM; i >= RAIL_MIN_CM && !stopShip; i -= 0.5) {
                loopNr++;
                Point attempt = Point.polar(-i, toRadians(-robot.heading + 90));
                attemptToTarget = attempt.distanceTo(localTarget);
                //if (i == RAIL_MAX_CM)
                //telemetry.addData("topRailDistance A", attempt.distanceTo(localTarget));

                if (attempt.distanceTo(localTarget) < bestMatch && attempt.distanceTo(localTarget) >= targetDistance) {
                    bestDelta = target.add(delta);
                    bestMatch = attempt.distanceTo(localTarget);
                    bestRailDistance = i;
                }
                //if (attempt.distanceTo(localTarget) >= targetDistance && attempt.distanceTo(localTarget) < targetDistance + 0.8)
                //    break;
                if (i == RAIL_MAX_CM && attempt.distanceTo(localTarget) >= targetDistance && attempt.distanceTo(localTarget) <= targetDistance + 3) {
                    //telemetry.addLine("works");
                    stopShip = true;
                    bestDelta = target.add(delta);
                    bestMatch = attempt.distanceTo(localTarget);
                    bestRailDistance = i;
                    break;
                }
            }

            //if (bestMatch < targetDistance + 1) break;
        }

        //telemetry.addData("halfLoopNr", loopNr);
        //telemetry.addData("halfBestDelta", bestDelta.subtract(target).radius());
        //telemetry.addData("halfBestMatch", bestMatch);
        //telemetry.addData("halfBestRail", bestRailDistance);

        for (double b = 0; b >= -8.5 && !stopShip; b--) {
            Point delta = new Point(b, b);
            Point localTarget = Point.polar(target.add(delta).distanceTo(robot), robot.atan2(target.add(delta)));

            for (double i = RAIL_MAX_CM; i >= RAIL_MIN_CM; i -= 0.5) {
                loopNr++;
                Point attempt = Point.polar(-i, toRadians(-robot.heading + 90));
                attemptToTarget = attempt.distanceTo(localTarget);
                if (i == RAIL_MAX_CM)
                    //telemetry.addData("topRailDistance B", attempt.distanceTo(localTarget));
                    if (attempt.distanceTo(localTarget) < bestMatch && attempt.distanceTo(localTarget) >= targetDistance) {
                        bestDelta = target.add(delta);
                        bestMatch = attempt.distanceTo(localTarget);
                        bestRailDistance = i;
                    }
                //if (attempt.distanceTo(localTarget) < targetDistance - 3)
                //  break;
                if (i == RAIL_MAX_CM && attempt.distanceTo(localTarget) >= targetDistance && attempt.distanceTo(localTarget) <= targetDistance + 3) {
                    //telemetry.addLine("works");

                    stopShip = true;
                    bestDelta = target.add(delta);
                    bestMatch = attempt.distanceTo(localTarget);
                    bestRailDistance = i;
                    break;
                }
            }
            if (bestMatch < targetDistance + 1) break;

        }

        //telemetry.addData("innerLoopNr", loopNr);
        //telemetry.addData("stopShip", stopShip);
        //telemetry.addData("BestDelta", bestDelta.subtract(target).radius());
        //telemetry.addData("BestMatch", bestMatch);
        //telemetry.addData("BestRail", bestRailDistance);


        //telemetry.update();

        bestAttemptToTarget = bestMatch;
        return bestRailDistance;
    }

    public double calculateRailChamber(Pose2d robot, double targetDistance) {
        //this figures out how much the rail needs to be extended to be at the targetDistance from the chamber

        //distance from robot to chamber
        double deltaChamber = abs(robot.getX()) - 64;

        //angle of the turret
        double turretDeg = 90 - robot.getHeading();

        //how much the rail needs to move us on the X axis
        double deltaCm = deltaChamber - targetDistance;
        //double deltaCm = cos(toRadians(turretDeg))*deltaChamber-targetDistance;

        return deltaCm / cos(toRadians(robot.getHeading() - 90)) + RAIL_LOAD_CM;
    }

    public double calculateRailChamberDeviant(Pose2d robot, double targetDistance) {
        //this figures out how much the rail needs to be extended to be at the targetDistance from the chamber

        //distance from robot to chamber
        double deltaChamber = abs(robot.getX()) - 64;

        //angle of the turret
        double turretDeg = 90 - robot.getHeading();

        //how much the rail needs to move us on the X axis
        double deltaCm = deltaChamber - targetDistance;
        //double deltaCm = cos(toRadians(turretDeg))*deltaChamber-targetDistance;

        return -deltaCm / cos(toRadians(robot.getHeading() + 90)) + RAIL_LOAD_CM;
    }

    public double calculateTurretAngle(Point target, Pose2d robot, double railDistance) {
        //this figures out how much the rail needs to be extended to be at the targetDistance from the target
        Point localTarget = Point.polar(target.distanceTo(robot), robot.atan2(target));

        Point turret = Point.polar(railDistance, toRadians(-robot.heading + 90));

        double turretGlobal = -toDegrees(turret.atan2(localTarget)) + 90;
        double turretLocal = robot.heading - turretGlobal;
        return turretLocal;
    }

    public double calculateTurretAngle(Pose2d robot, double railDistance) {
        //this figures out how much the rail needs to be extended to be at the targetDistance from the target
        Point localTarget = Point.polar(bestDelta.distanceTo(robot), robot.atan2(bestDelta));

        Point turret = Point.polar(railDistance, toRadians(-robot.heading + 90));

        double turretGlobal = -toDegrees(turret.atan2(localTarget)) + 90;
        double turretLocal = robot.heading - turretGlobal;
        return turretLocal;
    }

    public Point projectAttemptOnBasket(Point basket, Point attempt) {
        ///Because basket is at 45 DEGREES and perpendicular to that is -45 DEGREES the factors simplify nicely
        ///don't forget that it's way more likely for you to be silly than the math to be fundamentally wrong
        return new Point((basket.x - basket.y + attempt.x + attempt.y) / 2, (-basket.x + basket.y + attempt.x + attempt.y) / 2);
    }

    public double calculateRailBasketWithMath(Point basket, Pose2d robot, double target) {
        //this figures out how much the rail needs to be extended to be at the targetDistance from the target
        ///Daca nu gasesti nimic sub un metru, trimite pozitia de load

        double bestMatch = 100;
        double bestRailDistance = RAIL_MIN_CM;
        Point edge1 = basket.add(Point.polar(8.5, PI / 4));
        Point edge2 = basket.add(Point.polar(-8.5, PI / 4));
        e1 = edge1;
        e2 = edge2;

        for (double i = RAIL_MIN_CM; i <= RAIL_MAX_CM; i++) {
            loopNr++;
            Point attempt = robot.add(Point.polar(i, toRadians(-robot.heading + 90)));

            Point projectedAttemptOnBasket = projectAttemptOnBasket(basket, attempt);


            if (projectedAttemptOnBasket.distanceTo(basket) < 8.5) {
                if (isInRange(projectedAttemptOnBasket.distanceTo(attempt), target, bestMatch)) {
                    bestRailDistance = i;
                    bestAttempt = attempt;
                    projectedOnBasket = projectedAttemptOnBasket;
                }
                bestMatch = inRange(projectedAttemptOnBasket.distanceTo(attempt), target, bestMatch);
            } else {
                if (isInRange(min(attempt.distanceTo(edge1), attempt.distanceTo(edge2)), target, bestMatch)) {
                    bestRailDistance = i;
                    bestAttempt = attempt;
                    projectedOnBasket = projectedAttemptOnBasket;
                }
                bestMatch = inRange(min(attempt.distanceTo(edge1), attempt.distanceTo(edge2)), target, bestMatch);
            }
        }

        //telemetry.addData("BEST ATTEMPT TO Basket", bestMatch);

        //telemetry.update();

        bestAttemptToTarget = bestMatch;

        return bestRailDistance;
    }

    public double calculateTurretAngleWithMath(Point basket, Pose2d robot, double railDistance) {
        Point turret = robot.add(Point.polar(railDistance, toRadians(-robot.heading + 90)));

        Point projectedAttemptOnBasket = projectAttemptOnBasket(basket, turret);

        /*telemetry.addData("railDistance", railDistance);
        telemetry.addData("angle to Basket", robot.heading - (-(135) + 90));
        telemetry.addData("angle to Edge1", robot.heading - toDegrees(turret.atan2(e1)));
        telemetry.addData("angle to Edge2", robot.heading - toDegrees(turret.atan2(e2)));
        telemetry.addData("semiAngle to Edge1", toDegrees(turret.atan2(e1)) + 90);
        telemetry.addData("paraSemiAngle to Edge1", toDegrees(turret.atan2(e1)));

        if (projectedAttemptOnBasket.distanceTo(basket) < 8.5) {
            //135 is the angle from the Ox that we want the turret to be at;
            //the maths just converts it to the -45 of the roary coordinate space
            telemetry.addData("chosen", "basket");
        } else {
            if (min(turret.distanceTo(e1), turret.distanceTo(e2)) == turret.distanceTo(e1))
                telemetry.addData("chosen", "e1");
            else telemetry.addData("chosen", "e2");

        }


        if (projectedAttemptOnBasket.distanceTo(basket) < 8.5) {
            //135 is the angle from the Ox that we want the turret to be at;
            //the maths just converts it to the -45 of the roary coordinate space
            return robot.heading - (-(135) + 90);
        } else {
            if (min(turret.distanceTo(e1), turret.distanceTo(e2)) == turret.distanceTo(e1))
                return robot.heading - toDegrees(turret.atan2(e1));
            else return robot.heading - toDegrees(turret.atan2(e2));
        }
    }

 */

    public void updateElevation(double degrees) {
        /// u.M.R
        // if (degrees<35)degrees=35;
        double compensatedDegrees = degreesCompensator(degrees);
        double interpolator = (compensatedDegrees - ELEVATION_MIN_DEG) / (ELEVATION_MAX_DEG - ELEVATION_MIN_DEG);
        double elevation = inRange((1 - interpolator) * ELEVATION_MIN + interpolator * ELEVATION_MAX, ELEVATION_MIN, ELEVATION_MAX);
        Elevatie.setPosition(elevation);
    }

    public void updateTurret(double degrees) {
        degrees = inRange(degrees, TURRET_LEFT_DEG, TURRET_RIGHT_DEG) + turret_compensator;
        double interpolator = (TURRET_LEFT_DEG - degrees) / (TURRET_LEFT_DEG - TURRET_RIGHT_DEG);
        double value = (1 - interpolator) * TURRET_LEFT + interpolator * TURRET_RIGHT;
        //precalculated conversion function
        //double azimuth = -0.010204 * degrees + 0.479728;
        Turela.setPosition(value);
    }

    public void ballisticsCalculator(Point basket, Pose2d robot) {
        double turret_offset = 8;
        turret = robot.add(new Point(-turret_offset, 0).rotate(-toRadians(robot.heading)));
        //EVERY DISTANCE HERE IS IN METRES
        ///IF YOU EVER WANT TO FIGURE OUT WHAT IS HAPPENING HERE, THE ANSWER IS DESMOS
        turret.z = 0.44;
        double range = basket.distanceTo(turret) / 100;
        if (range <= 2.4) basket.z = 0.89;
        else basket.z = 0.93;

        safety = inRange(range / (range <= 2.4 ? 3 : 2.6), 0.89, 1.3);

        double c = basket.z;
        double b = Math.sqrt(abs(safety - 2) / 2);
        double a = ((turret.z - c) / range - b) / range;
        double fOfRDerivative = 2 * a * range + b;
        double alpha = atan(fOfRDerivative);
        double u = sqrt(-9.81 / (2 * a * cos(alpha) * cos(alpha)));
        double angle = 90 + toDegrees(alpha);

        this.u = u;
        this.angle = angle;
        this.range = range;

        relTurretAngle = toDegrees(turret.atan2(basket)) - robot.heading;

        updateTurret(relTurretAngle);
        updateElevation(angle);
        calcutedRPM = mpsConvertor(u);
        calculatedPower = inRange(range / 2.4, 0.58, 1);
    }

    public double mpsConvertor(double mps) {
        return inRange(mps * 3.333 * 60 * MPSCOMPENSATOR, RPM_MIN, RPM_MAX);
    }

    public void setRPM(double P_Rpm, double S_Power) {
        setRPM = P_Rpm;
        setPower = S_Power;
        startLauncher();
    }

    public void setRPM() {
        setRPM = calcutedRPM;
        setPower = calculatedPower;
        startLauncher();
    }

    public void setRPMAuto() {
        setRPM = calcutedRPM;
        setPower = calculatedPower;
        SLauncher.setPower(setPower);
    }

    public boolean isLauncherStarted() {
        return setRPM != 0 && setPower != 0;
    }

    public void startLauncherEDX() {
        velocityEnabled = true;
//        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
//
//        SLauncher.setPower(setPower);
    }

    public void stopLauncherEDX() {
        velocityEnabled = false;
//        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        setRPM = 0;
//        PLauncher.setVelocity(0);
//        SLauncher.setPower(0);
    }

    public void setRPMNoLaunch(double P_Rpm, double S_Power) {
        setRPM = P_Rpm;
        setPower = S_Power;
    }

    public double sPowerCompensator() {
        return 1;
        //return max(1, 14.0 / batteryVoltage);
    }

    public void startLauncher() {
        PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE);
        SLauncher.setPower(setPower);

        velocityEnabled = true;
    }

    public void stopLauncher() {
        setRPM = 0;
        setPower = 0;
        PLauncher.setVelocity(0);
        SLauncher.setPower(0);

        velocityEnabled = false;
    }

    public double getRPM() {
        return PLauncher.getVelocity() / TICKS_PER_ROTATION_PER_MINUTE;
    }

    public double getRPMSecondary() {
        return SLauncher.getVelocity() / TICKS_PER_ROTATION_PER_MINUTE_SECONDARY;
    }

    public boolean isAtSpeed() {
        if (!isInRange(setRPM - getRPM(), -80, 64))
            speedTimer = opmode.getRuntime() + speedStabilizer;

        return opmode.getRuntime() > speedTimer;
    }

    public boolean isAtSpeedSecondary() {
        if (!isInRange(setPower * 1340 - getRPMSecondary(), -48, 48))
            speedTimer = opmode.getRuntime() + speedStabilizer;

        return opmode.getRuntime() > speedTimer;
    }

    public double speed(double x, double vMin) {
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;
        double speed = (-4) * x * x + 4 * x;
        if (vMin > speed) {
            return vMin;
        } else return speed;
    }


    public enum TurretPos {
        STRAIGHT, LEFT, RIGHT
    }

    public enum ElevationPos {
        MIN, SHOOT_NEAR, SHOOT_FAR, MAX
    }


}
