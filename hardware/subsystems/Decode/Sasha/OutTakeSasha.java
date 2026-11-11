package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Sasha;


import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.Globals.Sasha.*;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.FSqrt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isInRange;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Point;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;

public class OutTakeSasha {

    public Servo Elevatie;

    public DcMotorEx SLauncher;
    public DcMotorEx PLauncher;
    public DcMotorEx Turela;
    public double degress;
    OpMode opmode = null;
    Telemetry telemetry = null;
    Sensors sensors;

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

    public double setRPM = 0;
    public double setPower = 0;
    public double deltaRPM, interpolatorRPM, speedLauncher;
    public boolean velocityDone;
    public double batteryVoltage = 1;

    public boolean velocityEnabled = false;

    ElevationPos elevationPos = ElevationPos.MIN;


    public boolean turretFirstTime = true, turretDone = false, turretDisabled = false;
    double deltaTurret, deltaTurretSign, initDeltaTurret, interpolatorTurret, speedTurret;

    public double turretCorection = 0;

    public OutTakeSasha(HardwareMap hardwareMap, LinearOpMode opmode, Sensors sensors) {

        PLauncher = hardwareMap.get(DcMotorEx.class, "LauncherP");
        SLauncher = hardwareMap.get(DcMotorEx.class, "LauncherS");

        Turela = hardwareMap.get(DcMotorEx.class, "Turela");
        Elevatie = hardwareMap.get(Servo.class, "Lift");


        PLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        SLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        SLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


    public void setTurretDegreesIterative(double degrees) {
        if (turretDisabled) {
            degrees = 0;
        }

        boolean turretOutOFLimits = false;
        final double TURRET_LIMIT = 100;
        if (degrees < -TURRET_LIMIT) {
            degrees = -TURRET_LIMIT;
            turretOutOFLimits = true;
        } else if (degrees > TURRET_LIMIT) {
            degrees = TURRET_LIMIT;
            turretOutOFLimits = true;
        }

        turretFirstTime = false;
        Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        deltaTurret = degrees - getCurrDegrees();
        final double BRAKE_DISTANCE = 23;
        interpolatorTurret = inRange(deltaTurret / BRAKE_DISTANCE, -1, 1);
        speedTurret = twoXSqrd(interpolatorTurret, (0.32 * (inRange(14.2 / batteryVoltage, 1, 1.10))));


        if (abs(deltaTurret) < 0.89) {
            turretDone = true;
            Turela.setPower(0);

        } else {
            turretDone = false;
            //if (abs(degrees) > 1)
            Turela.setPower(speedTurret);
            //else
            //    Turela.setPower(deltaTurretSign * speedTurret * 0.8);

            //LiftR.setPower(deltaTurretSign * speedTurret);
        }

        if (turretOutOFLimits) {
            turretDone = false;
        }
    }

    public double speed(double x, double vMin) {
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;
        double speed = (-4) * x * x + 4 * x;
        return Math.max(vMin, speed);
    }

    public void setPowerTurret(double speed) {
        //POZITIV La dreapta cum te uiti din spatele turelei
        Turela.setPower(speed);
    }

    public double getCurrDegrees() {
        return (Turela.getCurrentPosition() / TICKS_PER_DEGREE) + TURRET_OFSET;
    }

    public double degreesCompensator(double degrees) {
        //precalculated conversion function
        /// This has huge chances of eating my days
        return degrees * 1.1 - 3.3;
    }

    public void updateElevation(double degrees) {
        if (degrees < 35) degrees = 35;
        double compensatedDegrees = degreesCompensator(degrees);
        double interpolator = (compensatedDegrees - ELEVATION_MIN_DEG) / (ELEVATION_MAX_DEG - ELEVATION_MIN_DEG);
        double elevation = inRange((1 - interpolator) * ELEVATION_MIN + interpolator * ELEVATION_MAX, ELEVATION_MIN, ELEVATION_MAX);
        Elevatie.setPosition(elevation);
    }


    public void ballisticsCalculatorFSqrt(Point basket, Pose2d robot) {
        turret = robot.add(new Point(-TURRET_OFFSET, 0).rotate(-toRadians(robot.heading)));
        //EVERY DISTANCE HERE IS IN METRES
        ///IF YOU EVER WANT TO FIGURE OUT WHAT IS HAPPENING HERE, THE ANSWER IS DESMOS
        turret.z = 0.44;
        double range = basket.distanceTo(turret) / 100;
//        if (range <= 1.4) basket.z = 0.7;
//        else if (range <= 2.4) basket.z = 0.79; // 0.84
//        else basket.z = 0.92;

        if (range <= 1.4) basket.z = 0.67;
        else if (range <= 2.4) basket.z = 0.72; // 0.84
        else basket.z = 0.93;

        //basket.z = 0.84; //0.89
        //safety = inRange(range / 2.6, 0.9, 1.3);
        //safety = inRange(range / (range <= 2.4 ? 3 : 2.6), 0.8, 1.3);
        safety = inRange(range / (range <= 2.4 ? 3 : 2.6), 0.89, 1.3);


        double c = basket.z;
        double b = FSqrt(abs(safety - 2) / 2);
        double a = ((turret.z - c) / range - b) / range;
        double fOfRDerivative = 2 * a * range + b;
        double alpha = atan(fOfRDerivative);
        double u = FSqrt(-9.81 / (2 * a * cos(alpha) * cos(alpha)));
        double angle = 90 + toDegrees(alpha);

        //opmode.telemetry.addData("angle", angle);

        this.u = u;
        this.angle = angle;
        this.range = range;

        relTurretAngle = toDegrees(turret.atan2(basket)) - robot.heading;

        // updateTurret(relTurretAngle);
        updateElevation(angle);
//
//        if (relTurretAngle < -TURRET_OFSET) relTurretAngle = -TURRET_OFSET;
//        else if (relTurretAngle > TURRET_OFSET) relTurretAngle = TURRET_OFSET;

        //opmode.telemetry.addData("relTurretAngle", relTurretAngle);
        //double turretSign = (relTurretAngle == 0 ? 0 : relTurretAngle / abs(relTurretAngle));
        relTurretAngle += turretCorection;

        setTurretDegreesIterative(relTurretAngle);

        calcutedRPM = mpsConvertor(u);
        calculatedPower = inRange(range / 3, 0.53, 1);
    }


    public void initOuttake() {
        Elevatie.setPosition(ELEVATION_MIN);
        InitTurela();
    }

    public double mpsConvertor(double mps) {
        return inRange(mps * 3.333 * 60 * MPSCOMPENSATOR, RPM_MIN, RPM_MAX);
    }

    void InitTurela() {
        Turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double waitAbit = 0;
        //double safetyTime = opmode.getRuntime() + 1.0;
        while (Sensors.TurretMag.getValue() == 0) {
            Turela.setPower(0.4);

            telemetry.addData("Turret Sensor", Sensors.TurretMag.getValue());
            telemetry.addData("Turela Ticks", Turela.getCurrentPosition());
            telemetry.addData("Turela DEg", Turela.getCurrentPosition() / TICKS_PER_DEGREE);
            telemetry.addData("Powa", Turela.getPower());

            waitAbit = opmode.getRuntime() + 0.5;
            telemetry.update();
        }
        while (waitAbit > opmode.getRuntime()) {
            Turela.setPower(0.4);
            Turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        Turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turela.setPower(0);

        telemetry.clear();

    }

    public void setRPM(double P_Rpm, double S_Power) {
        setRPM = P_Rpm;
        setPower = S_Power;
        startLauncher();
    }

    public double getRPM() {
        return PLauncher.getVelocity() / TICKS_PER_ROTATION_PER_MINUTE_PRIMARY;
    }

    public void setRPM() {
        setRPM = calcutedRPM;
        setPower = calculatedPower;
        startLauncher();
    }

    public void startLauncher() {
        velocityEnabled = true;
//        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
//
//        SLauncher.setPower(setPower);
    }

    public void stopLauncher() {
        velocityEnabled = false;
//        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        setRPM = 0;
        PLauncher.setPower(0);
        SLauncher.setPower(0);
    }

    public double speedStabilizer = 0.08;

    public boolean isAtSpeed() {
        if (!isInRange(calcutedRPM - getRPM(), -80, 64))
            speedTimer = opmode.getRuntime() + speedStabilizer;

        //opmode.telemetry.addData("setRMP - getRPM()", calcutedRPM - getRPM());
        return opmode.getRuntime() > speedTimer;
    }

    public double getRPMSecondary() {
        return SLauncher.getVelocity() / TICKS_PER_ROTATION_PER_MINUTE_SECONDARY;
    }

    public boolean isAtSpeedSecondary() {
        if (!isInRange(calculatedPower * 1340 - getRPMSecondary(), -48, 48))
            speedTimer = opmode.getRuntime() + speedStabilizer;

        return opmode.getRuntime() > speedTimer;
    }

//    public void setVelocityIterative() {
//        deltaRPM = setRPM - getRPM();
//
//        double powerKeep = (setRPM / RPM_MAX);
//        double BREAK_RPM = 100;
//        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, 0, 1);
//        speedLauncher = twoXSqrd(interpolatorRPM, powerKeep);
//
//        if (setRPM <= 0) {
//            PLauncher.setPower(0);
//            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            velocityDone = false;
//            return;
//        }
//        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//
//        if (abs(deltaRPM) < 65) {
//            //PLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //telemetry.addData("isAtRPM", true);
//            velocityDone = true;
//            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            PLauncher.setPower(0);
//            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
//        } else if (setRPM > 100) {
//            //telemetry.addData("isAtRPM", false);
//            velocityDone = false;
//            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            PLauncher.setPower(speedLauncher);
//        } else {
//            velocityDone = false;
//        }
//
//
//    }

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
        final double BREAK_RPM = 240;
//        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, -0.8, 1);
//        speedLauncher = twoXSqrd(interpolatorRPM, 0.24);
//        if (isInRange(speedLauncher, -0.08, 1.2)) speedLauncher = 1.2;
//        speedLauncher = max(speedLauncher, -0.8);

        boolean sharpshooter = range < 2.5;
        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, -0.64, 1);
        speedLauncher = twoXSqrd(interpolatorRPM, (sharpshooter ? 0.24 : 0.3));
        if (isInRange(speedLauncher, -0.08, 1)) speedLauncher = 1;
        speedLauncher = max(speedLauncher, -0.8);

//        telemetry.addData("deltaRPM", deltaRPM);
//        telemetry.addData("interpolatorRPM", interpolatorRPM);
//        telemetry.addData("speedLauncher", speedLauncher);

        if (abs(deltaRPM) < 80) { //isInRange(deltaDeltaRPM, -89,60)
            // telemetry.addData("isAtRPM", true);
            velocityDone = true;
            PLauncher.setPower(0);
            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
        } else if (calcutedRPM > 100) {
            // telemetry.addData("isAtRPM", false);
            velocityDone = false;
            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PLauncher.setPower(speedLauncher);
        } else {
            velocityDone = false;
        }

    }

//    public void setVelocityIterative() {
//        deltaRPM = setRPM - getRPM();
//
//        double powerKeep = (setRPM / RPM_MAX);
//        double BREAK_RPM = 100;
//        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, 0, 1);
//        speedLauncher = twoXSqrd(interpolatorRPM, powerKeep);
//
//        if (setRPM <= 0) {
//            PLauncher.setPower(0);
//            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            velocityDone = false;
//            return;
//        }
//        PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//
//        if (abs(deltaRPM) < 65) {
//            //PLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //telemetry.addData("isAtRPM", true);
//            velocityDone = true;
//            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            PLauncher.setPower(0);
//            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
//        } else if (setRPM > 100) {
//            //telemetry.addData("isAtRPM", false);
//            velocityDone = false;
//            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            PLauncher.setPower(speedLauncher);
//        } else {
//            velocityDone = false;
//        }
//
//
//    }


//    @Deprecated
//    public void setVelocityIterative(boolean rapidFire) {
//        deltaRPM = setRPM - getRPM();
//        PLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        double BREAK_RPM = 128;
//        interpolatorRPM = inRange(deltaRPM / BREAK_RPM, -0.794, 0.99);
//        speedLauncher = twoXSqrd(interpolatorRPM, 0.08);
//        if (isInRange(speedLauncher, 0.08, 0.64)) speedLauncher = 0.64;
//        speedLauncher = max(speedLauncher, -0.8);
//
//        if (setRPM <= 0) {
//            PLauncher.setPower(0);
//            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            telemetry.addData("DONE SHOOTING", "HAHAHHAAH");
//            return;
//        }
//
//        //Daca merge e bomba
//        if (setRPM > 100 && deltaRPM > 900) {
//            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            velocityDone = false;
//            opmode.telemetry.addData("SPIKE, prev power", PLauncher.getPower());
//            PLauncher.setPower(1);
//            return;
//        }
//
//
//        if (isInRange(abs(deltaRPM), 400, 900) && setRPM > 0 && rapidFire) {
//            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            telemetry.addData("RapidFire, Power =", speedLauncher);
//            double procentPower = abs(deltaRPM) / 900 * 0.62;
//            procentPower = inRange(procentPower, 0.05, 0.62);
//            speedLauncher *= (1 + procentPower);
//            telemetry.addData("procentPower", procentPower);
//        }
//
//        if (abs(deltaRPM) < 65) {
//            //telemetry.addData("isAtRPM", true);
//            PLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            PLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            velocityDone = true;
//            double power = (setRPM / RPM_MAX);
//            PLauncher.setPower(power);
//            //PLauncher.setVelocity(setRPM * TICKS_PER_ROTATION_PER_MINUTE_PRIMARY);
//        } else if (setRPM > 100) {
//            ///telemetry.addData("isAtRPM", false);
//            velocityDone = false;
//            PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            //PLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            PLauncher.setPower(speedLauncher);
//        } else {
//            PLauncher.setPower(0);
//            velocityDone = false;
//        }
//    }
//
//    double deltaDeltaRPM = 0;


    enum ElevationPos {
        MIN, SHOOT_NEAR, SHOOT_FAR, MAX
    }
}
