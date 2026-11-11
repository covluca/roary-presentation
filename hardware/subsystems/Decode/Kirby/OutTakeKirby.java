package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby;


import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.Globals.Kirby.*;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isInRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isRobotInBigTriangle;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isRobotInSmallTriangle;
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
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Point;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;


public class OutTakeKirby {

    //VAL 1620
    //public static double TICKS_PER_CM_LIFT = 8.3;

    //VAL 1150
    public LinearOpMode opmode;

    public Sensors sensors;

    public Telemetry telemetry;

    public DcMotorEx PLauncher;
    public DcMotorEx SLauncher;
    public DcMotorEx Turela = null;
    public Servo Elevatie = null;
    public Servo Farul = null;


    public ElevationPos elevationPos = ElevationPos.MIN;
    public int loopNr = 0;

    public double setRPM = 0;
    public double setPower = 0;

    public double u = 0;
    public double angle = 0;
    public double range = 0;
    public double relTurretAngle = 0;
    public Point turret = new Point();
    public double calculatedRPM = 0;
    public double calculatedPower = 0;
    public double safety = 0;
    public double speedTimer = 0;
    public double deltaRPM = 0;
    public double speedStabilizer = 0.08;
    public double interpolatorRPM = 0;
    public double speedLauncher = 0;
    public boolean velocityDone = false;
    public double batteryVoltage = 13.8;
    public double powerCompensator = 1;

    public boolean velocityEnabled = false;
    public boolean turretDone = false;
    public double deltaDeg = 0;
    public double interpolatorTurret = 0;
    public double turretSpeed = 0;
    public double lastDeg = 0;
    public double turretDerivative = 0;
    public double interpolatorVelTurret = 0;
    public double turretVelSpeed = 0;
    public double flashTime = 0;
    public boolean sharpshooter = false;
    /*public void setTurretVelIterative(double degrees,Pose2d vel) {

        deltaDeg = inRange(degrees, -101.5, 101.5) - getTurretDegrees();
        turretDerivative = degrees - lastDeg;


        if (abs(deltaDeg) < (sharpshooter ? 0.35 : 0.67)) { //isInRange(deltadeltaDeg, -89,60)
            //telemetry.addData("turretDone", true);
            lastDeg = degrees;
            turretDone = true;
            Turela.setPower(0);
            return;
            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
        }
        double BREAK_DEGREES = 24.82;
        interpolatorTurret = inRange(deltaDeg / BREAK_DEGREES, -1, 1);
        turretSpeed = slowerTwoXSqrd(interpolatorTurret, sharpshooter ? 0.089 : 0.107);


        //telemetry.addData("deltaDeg", deltaDeg);
        //telemetry.addData("interpolatorTurret", interpolatorTurret);
        //telemetry.addData("turretSpeed", turretSpeed);
        if (abs(deltaDeg) < (sharpshooter ? 0.67 : 1.3) && (turretSpeed > 0 && getTurretDegrees() < 100) || (turretSpeed < 0 && getTurretDegrees() > -101)) {
            turretDone = true;
            Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turela.setPower(-turretSpeed * powerCompensator);
        } else if ((turretSpeed > 0 && getTurretDegrees() < 100) || (turretSpeed < 0 && getTurretDegrees() > -101)) {
            //telemetry.addData("turretDone", false);
            turretDone = false;
            Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turela.setPower(-turretSpeed * powerCompensator);
        } else {
            turretDone = false;
            Turela.setPower(0);

        }
        lastDeg = degrees;
    }

     */
    public double contraVel = 0;
    public double plusVel = 0;
    public double sumVel = 0;
    public double sustainPower = 0;
    public double addPower = 0;
    public double deltaVel = 0;
    public double turretPower = 0;
    public double deltaVelInterpolator = 0;
    public double velPower = 0;

    public double plusVelShoot = 0;
    public double addPowerShoot = 0;
    public double sustainPowerShoot = 0;
    public double deltaVelShoot = 0;
    public double turretPowerShoot = 0;
    public double deltaVelInterpolatorShoot = 0;
    public double velPowerShoot = 0;
    public boolean fResetFirstTime = true;
    public boolean fResetDone = false;
    public boolean isResetLeft = true;
    public double delayTimeFReset = 0;
    public double interpolatorError = 0;
    public double errorRPM = 0;
    double errorDeg = 2.1;

    public OutTakeKirby(HardwareMap hardwareMap, LinearOpMode opmode, Sensors sensors) {

        PLauncher = hardwareMap.get(DcMotorEx.class, "PLauncher");
        SLauncher = hardwareMap.get(DcMotorEx.class, "SLauncher");
        Turela = hardwareMap.get(DcMotorEx.class, "Turela");

        Elevatie = hardwareMap.get(Servo.class, "Elevatie");
        Farul = hardwareMap.get(Servo.class, "Farul");

        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        SLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        //SLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public static double slowerTwoXSqrd(double x, double vMin) {
        //functia asta e facuta pt interpolatoare cu val intre -1 si 1
        // e un P din Pid mai bun, tinde mereu spre eroare 0
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;

        //1.55 means that when x is 0.8 speed is 1, it's better than both 1 or 2
        if (x == 0) return 0;
        double speed = x * x;
        double sign = x / abs(x);
        if (vMin > Math.abs(speed)) return vMin * sign;
        else if (Math.abs(speed) > 1) return sign;
        else return speed * sign;
    }

    public void updatePowerCompensator() {
        powerCompensator = max(1, 13.8 / batteryVoltage);
    }

    public void statusCode(Pose2d robot) {

        //velocityDone;
        //turretDone;
        if (sensors.areSlotsFull() && opmode.getRuntime() > flashTime + 0.48 && !(isRobotInBigTriangle(robot, 35, 35) || isRobotInSmallTriangle(robot, 35, 35))) {
            Farul.setPosition(Codes.WHITE);
            flashTime = opmode.getRuntime() + 0.48;
        }
        if (sensors.newElement) {
            Farul.setPosition(Codes.PURPLE);
            flashTime = opmode.getRuntime() + 0.3;
        }
        if (opmode.getRuntime() < flashTime) ;
        else if (velocityDone && turretDone && (isRobotInBigTriangle(robot, 35, 35) || isRobotInSmallTriangle(robot, 35, 35)))
            Farul.setPosition(Codes.GREEN);
        else if (velocityDone || turretDone) Farul.setPosition(Codes.YELLOW);
        else Farul.setPosition(Codes.RED);
    }

    public void statusCode(Pose2d robot, int llCodes) {

        //velocityDone;
        //turretDone;
        if (llCodes == -1) {
            if (sensors.areSlotsFull() && opmode.getRuntime() > flashTime + 0.48 && !(isRobotInBigTriangle(robot, 35, 35) || isRobotInSmallTriangle(robot, 35, 35))) {
                Farul.setPosition(Codes.WHITE);
                flashTime = opmode.getRuntime() + 0.48;
            }
            if (sensors.newElement) {
                Farul.setPosition(Codes.PURPLE);
                flashTime = opmode.getRuntime() + 0.3;
            }
            if (opmode.getRuntime() < flashTime) ;
            else if (velocityDone && turretDone && (isRobotInBigTriangle(robot, 35, 35) || isRobotInSmallTriangle(robot, 35, 35)))
                Farul.setPosition(Codes.GREEN);
            else if (velocityDone || turretDone) Farul.setPosition(Codes.YELLOW);
            else Farul.setPosition(Codes.RED);
        } else if (llCodes == 1) {
            Farul.setPosition(Codes.RED);
        } else if (llCodes == 2) {
            Farul.setPosition(Codes.YELLOW);
        } else if (llCodes == 3) {
            Farul.setPosition(Codes.GREEN);
        }
    }

    public void setVelocityIterative() {
        if (!velocityEnabled) {
            SLauncher.setPower(0);
            PLauncher.setPower(0);
            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            velocityDone = false;
            return;
        }
        //PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //SLauncher.setPower(calculatedPower);


        deltaRPM = calculatedRPM - getRPM();
        double BREAK_RPM = 240;
        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, -0.64, 1);
        speedLauncher = twoXSqrd(interpolatorRPM, (sharpshooter ? 0.33 : 0.444));
        if (isInRange(speedLauncher, (sharpshooter ? 0.33 : 0.48), 0.89)) speedLauncher = 0.89;
        if (interpolatorRPM > 0.53) speedLauncher = 1;
        speedLauncher = max(speedLauncher, sharpshooter ? 0.21 : 0.19);

        //telemetry.addData("deltaRPM", deltaRPM);
        //telemetry.addData("interpolatorRPM", interpolatorRPM);
        //telemetry.addData("speedLauncher", speedLauncher);

        if (abs(deltaRPM) < (sharpshooter ? 24 : 24)) { //isInRange(deltaDeltaRPM, -89,60)
            //telemetry.addData("isAtRPM", true);
            velocityDone = true;
            PLauncher.setPower(speedLauncher);
            SLauncher.setPower(speedLauncher);

            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            SLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //PLauncher.setVelocity(calculatedRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
        } else if (calculatedRPM > 100) {
            //telemetry.addData("isAtRPM", false);
            velocityDone = false;

            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            SLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            PLauncher.setPower(speedLauncher);
            SLauncher.setPower(speedLauncher);
        } else {
            velocityDone = false;
            PLauncher.setPower(0);
            SLauncher.setPower(0);
        }
    }

    public void shooterManager() {

        double sumVelShoot = calculatedRPM;

        deltaRPM = sumVelShoot - getRPM();
        //(sharpshooter ? 32 : 64)
        if (abs(deltaRPM) < errorRPM) { //isInRange(deltadeltaDeg, -89,60)
            velocityDone = true;
        } else velocityDone = false;

        //deltaVel = sumVel - getTurretVelocity();


        telemetry.addData("sumVelShooter", calculatedRPM);
        telemetry.addData("currVelShooter", getRPM());
        telemetry.addData("velPowerShoot", velPowerShoot);
        telemetry.addData("sustainPowerShoot", sustainPowerShoot);
        telemetry.addData("addPowerShoot", addPowerShoot);
        telemetry.addData("deltaVelShoot", deltaVelShoot);


        //if (isInRange(getTurretDegrees(), -90, 90)) {

        if (abs(sumVelShoot) < 30 && abs(deltaRPM) < (sharpshooter ? 32 : 64)) {
            //telemetry.addData("T", "2");

            //turretDone = true;
            sumVelShoot = 0;

            PLauncher.setPower(0);
            SLauncher.setPower(0);

            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } else {


            sumVelShoot = inRange(sumVelShoot, 0, RPM_MAX);

            deltaVelShoot = sumVelShoot - getRPM();

            //if (deltaVel < 30) turretDone = true;

            //sustainPower = ((velToPower(getTurretVelocity()) > 0.089) ? velToPower(getTurretVelocity()) : 0);
            sustainPowerShoot = velToPowerShooter(getRPM());

            double BREAK_RPM = 120;

            double ERROR_RPM = sharpshooter ? 32 : 64;

            if (abs(deltaVelShoot) > ERROR_RPM) {
                deltaVelInterpolatorShoot = inRange(deltaVelShoot / BREAK_RPM, -1.3, 1.3);

                addPowerShoot = inRange(twoXSqrd(deltaVelInterpolatorShoot, 0.1093), -1.3, 1.7);
            } else addPowerShoot = 0;

            velPowerShoot = sustainPowerShoot + addPowerShoot;


            //telemetry.addData("turretDone", false);
            //telemetry.addData("T", "1");

            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (velocityEnabled) {
                PLauncher.setPower(velPowerShoot * powerCompensator);
                SLauncher.setPower(velPowerShoot * powerCompensator);
            } else {
                PLauncher.setPower(0.3);
                SLauncher.setPower(0.3);
            }
        }
        /*} else {
            telemetry.addData("T", "3");

            turretDone = false;
            Turela.setPower(0);
            Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }*/


    }

    public void setTurretIterative(double degrees) {

        deltaDeg = inRange(degrees, -101.5, 101.5) - getTurretDegrees();
        turretDerivative = degrees - lastDeg;

        /*if (abs(getTurretDegrees()) < 100 && abs(headingVel) > 17) {
            Turela.setVelocity(-0.64 * headingVel);
            return;
        }
         */

        if (abs(deltaDeg) < (sharpshooter ? 0.35 : 0.67)) { //isInRange(deltadeltaDeg, -89,60)
            //telemetry.addData("turretDone", true);
            lastDeg = degrees;
            turretDone = true;
            Turela.setPower(0);
            return;
            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
        }
        double BREAK_DEGREES = 24.82;
        interpolatorTurret = inRange(deltaDeg / BREAK_DEGREES, -1, 1);
        turretSpeed = slowerTwoXSqrd(interpolatorTurret, sharpshooter ? 0.089 : 0.107);


        //telemetry.addData("deltaDeg", deltaDeg);
        //telemetry.addData("interpolatorTurret", interpolatorTurret);
        //telemetry.addData("turretSpeed", turretSpeed);
        if (abs(deltaDeg) < (sharpshooter ? 0.67 : 1.3) && (turretSpeed > 0 && getTurretDegrees() < 100) || (turretSpeed < 0 && getTurretDegrees() > -101)) {
            turretDone = true;
            Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turela.setPower(turretSpeed * powerCompensator);
        } else if ((turretSpeed > 0 && getTurretDegrees() < 100) || (turretSpeed < 0 && getTurretDegrees() > -101)) {
            //telemetry.addData("turretDone", false);
            turretDone = false;
            Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turela.setPower(turretSpeed * powerCompensator);
        } else {
            turretDone = false;
            Turela.setPower(0);

        }
        lastDeg = degrees;
    }

    public void turretManager(double degrees, double hVel, boolean enable) {


        contraVel = hVel;
        deltaDeg = degrees - getTurretDegrees();

        if (abs(deltaDeg) < (sharpshooter ? errorDeg : errorDeg)) { //isInRange(deltadeltaDeg, -89,60)
            turretDone = true;
        } else turretDone = false;

        double BREAK_DEGREES = 20;
        interpolatorTurret = inRange(deltaDeg / BREAK_DEGREES, -1, 1);
        turretSpeed = twoXSqrd(interpolatorTurret, 0.089);

        plusVel = turretSpeed * TURRET_MAX_DEG_PER_SEC;

        sumVel = contraVel + plusVel;

        //deltaVel = sumVel - getTurretVelocity();


        telemetry.addData("sumVel", sumVel);
        telemetry.addData("currVel", getTurretVelocity());
        telemetry.addData("targetDegrees", degrees);
        telemetry.addData("currentDegrees", getTurretDegrees());
        telemetry.addData("velPower", velPower);
        telemetry.addData("sustainPower", sustainPower);
        telemetry.addData("addPower", addPower);
        telemetry.addData("deltaVel", deltaVel);


        //if (isInRange(getTurretDegrees(), -90, 90)) {

        if (abs(sumVel) < 30 && abs(deltaDeg) < (sharpshooter ? 0.53 : 0.8)) {
            //telemetry.addData("T", "2");

            //turretDone = true;
            sumVel = 0;
            Turela.setPower(0);
            Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } else {


            sumVel = inRange(sumVel, -TURRET_MAX_DEG_PER_SEC, TURRET_MAX_DEG_PER_SEC);

            deltaVel = sumVel - getTurretVelocity();

            //if (deltaVel < 30) turretDone = true;

            //sustainPower = ((velToPower(getTurretVelocity()) > 0.089) ? velToPower(getTurretVelocity()) : 0);
            sustainPower = velToPower(getTurretVelocity());

            double BREAK_DPS = 143;

            double ERROR_DPS = 8;

            if (abs(deltaVel) > ERROR_DPS) {
                deltaVelInterpolator = inRange(deltaVel / BREAK_DPS, -1.7, 1.7);

                addPower = inRange(twoXSqrd(deltaVelInterpolator, 0.107), -1.7, 1.7);
            } else addPower = 0;

            velPower = sustainPower + addPower;


            if ((velPower > 0 && getTurretDegrees() < 94) || (velPower < 0 && getTurretDegrees() > -94)) {
                //telemetry.addData("turretDone", false);
                //telemetry.addData("T", "1");

                Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (enable) {
                    Turela.setPower(velPower * powerCompensator);
                } else Turela.setPower(0);
            } else {
                //telemetry.addData("T", "3");
                Turela.setPower(0);
            }

        }
        /*} else {
            telemetry.addData("T", "3");

            turretDone = false;
            Turela.setPower(0);
            Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }*/


    }

    public void fReset(boolean isResetLeft) {
        if (fResetFirstTime) {
            fResetDone = false;
            this.isResetLeft = isResetLeft;
            Turela.setPower(isResetLeft ? 0.3 : -0.3);

            delayTimeFReset = opmode.getRuntime() + 0.13;

            fResetFirstTime = false;
        }

        if ((Turela.getCurrent(CurrentUnit.AMPS) > 1.3 && abs(Turela.getVelocity()) < 300) && opmode.getRuntime() > delayTimeFReset) {
            Turela.setPower(0);
            Turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fResetFirstTime = true;
            fResetDone = true;
        }

    }

    public double velToPower(double vel) {
        return vel / TURRET_MAX_DEG_PER_SEC;
    }

    public double velToPowerShooter(double vel) {
        return vel / REAL_RPM_MAX;
    }

    public double powerToVel(double pow) {
        return pow * TURRET_MAX_DEG_PER_SEC;
    }

    public void initOuttake(boolean resetEncoder) {
        //Turela.setPosition(TURRET_STRAIGHT);
        Elevatie.setPosition(ELEVATION_MIN);
        Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (Turela.getCurrentPosition() == 0 || resetEncoder) {
            Turela.setPower(0.24);

            while ((Turela.getCurrent(CurrentUnit.AMPS) < 1.3 || abs(Turela.getVelocity()) > 300) && !opmode.isStopRequested()) {
                Turela.setPower(0.24);
            }
            Turela.setPower(0);
            Turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setPowerTurret(double x) {
        Turela.setPower(x);
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

    public void updateElevation(double degrees) {
        /// u.M.R
        // if (degrees<35)degrees=35;
        double interpolator = (degrees - ELEVATION_MIN_DEG) / (ELEVATION_MAX_DEG - ELEVATION_MIN_DEG);
        double elevation = inRange((1 - interpolator) * ELEVATION_MIN + interpolator * ELEVATION_MAX, ELEVATION_MIN, ELEVATION_MAX);
        Elevatie.setPosition(elevation);
    }

    public void errorCalculator(Point robot) {
        Point A = new Point(180 - 17, 120 + 17);
        Point B = new Point(120 + 17, 180 - 17);
        Point C = new Point(180, 180);

        errorDeg = abs(toDegrees(robot.atan2(A) - robot.atan2(B))) / 2;

        interpolatorError = inRange(abs(deltaDeg) / errorDeg, 0, 1);

        errorRPM = (8 * interpolatorError + DELTA_RPM * (1 - interpolatorError)) / 2;

        telemetry.addData("errorDeg", errorDeg);
        telemetry.addData("errorRPM", errorRPM);


    }

    public double ballisticsCalculator(Point basket, Pose2d robot) {

        double turret_offset = 8;
        //turret = robot.add(new Point(-turret_offset, 0).rotate(-toRadians(robot.heading)));
        turret = robot;
        //EVERY DISTANCE HERE IS IN METRES
        ///IF YOU EVER WANT TO FIGURE OUT WHAT IS HAPPENING HERE, THE ANSWER IS DESMOS
        turret.z = 0.44;
        double range = basket.distanceTo(turret) / 100;
        if (range <= 2.4) {
            basket.z = 0.89;
        } else {
            basket.z = 1.03;
        }

        sharpshooter = range > 2.4;

        safety = inRange(range / (range <= 2.8 ? 2.1 : 2.42), 0.89, 1.52);

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

        relTurretAngle = robot.heading - toDegrees(turret.atan2(basket));

        //  updateTurret(relTurretAngle);
        updateElevation(angle);
        calculatedRPM = mpsConvertor(u);
        //calculatedRPM = 2720;
        return relTurretAngle;
        //calculatedPower = inRange(range / 2.4, 0.58, 1);
    }

    public double ballisticsCalculatorTeleop(Point basket, Pose2d robot) {

        double turret_offset = 8;
        //turret = robot.add(new Point(-turret_offset, 0).rotate(-toRadians(robot.heading)));
        turret = robot;
        //EVERY DISTANCE HERE IS IN METRES
        ///IF YOU EVER WANT TO FIGURE OUT WHAT IS HAPPENING HERE, THE ANSWER IS DESMOS
        turret.z = 0.44;
        double range = basket.distanceTo(turret) / 100;
        if (range <= 2.4) {
            basket.z = 0.953;
        } else {
            basket.z = 1.13;
        }

        sharpshooter = range > 2.4;

        safety = inRange(range / (range <= 2.8 ? 2.1 : 2.42), 0.89, 1.52);

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

        relTurretAngle = robot.heading - toDegrees(turret.atan2(basket));

        //  updateTurret(relTurretAngle);
        updateElevation(angle);
        calculatedRPM = mpsConvertor(u);
        //calculatedRPM = 2720;
        return relTurretAngle;
        //calculatedPower = inRange(range / 2.4, 0.58, 1);
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
        setRPM = calculatedRPM;
        setPower = calculatedPower;
        startLauncher();
    }

    public void setSpeed(double speed) {
        PLauncher.setPower(speed);
        SLauncher.setPower(speed);
    }

    public void setRPMAuto() {
        setRPM = calculatedRPM;
        setPower = calculatedPower;
        SLauncher.setPower(setPower);
    }

    public boolean isLauncherStarted() {
        return setRPM != 0 && setPower != 0;
    }

    public void startLauncher() {
        PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
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

    public double getTurretDegrees() {
        if (isResetLeft) return (-Turela.getCurrentPosition() / TICKS_PER_DEGREE_TURRET + 102.8);
        else return (-Turela.getCurrentPosition() / TICKS_PER_DEGREE_TURRET - 102.8);
    }

    public double getTurretVelocity() {
        return -Turela.getVelocity() / TICKS_PER_DEGREE_TURRET;
    }

    public double getRPM() {
        return PLauncher.getVelocity() / TICKS_PER_ROTATION_PER_MINUTE_PRIMARY;
    }

    public boolean isAtSpeed() {
        if (!isInRange(setRPM - getRPM(), -80, 64))
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

    public enum ElevationPos {
        MIN, SHOOT_NEAR, SHOOT_FAR, MAX
    }


}
