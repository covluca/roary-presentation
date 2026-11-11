package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Clippy;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;

import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class MecanumDrive {

    public static double power = 0.6;
    public static double rotPower = 0.6;
    public DcMotorEx FL, BL, BR, FR;

    ////////// DECCELERATIA ANGULARA SI DIAGONALA ROBOTULUI DIFERA DE LA ROBOT LA ROBOT
    // SE REFERA LA CATE GRADE/ SEC DECCELEREAZA ROBOTUL INTR-UN CM, PT EL TORRO ESTE 10.5
    // CA SA O AFLI FOLOSESTE ANGULAR DECCELERATION TESTER
    public double angular_decceleration = 10.5;

    // SE REFERA LA DIAGONALA ROBOTULUI DE LA ROATA MECANUM LA ROATA MECANUM
    // PT EL TORRO ESTE 42 CM
    public double robot_diagonal = 42;

    public Servo RetractN;
    LinearOpMode opMode;
    private List<DcMotorEx> motors;

    // private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public MecanumDrive(HardwareMap hardwareMap) {


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FR = hardwareMap.get(DcMotorEx.class, "FR");

        motors = Arrays.asList(FL, BL, BR, FR);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: reverse any motors using DcMotor.setDirection()

        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);

        //RetractN = hardwareMap.get(Servo.class,"RetractN");


    }

    public MecanumDrive(HardwareMap hardwareMap, String robotName) {


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FR = hardwareMap.get(DcMotorEx.class, "FR");

        motors = Arrays.asList(FL, BL, BR, FR);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: reverse any motors using DcMotor.setDirection()

        if (robotName == "Echo" || robotName == "Clippy") {
            FL.setDirection(DcMotorEx.Direction.REVERSE);
            BL.setDirection(DcMotorEx.Direction.REVERSE);
        } else if (robotName == "Macara") {
            FR.setDirection(DcMotorEx.Direction.REVERSE);
            BR.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            FL.setDirection(DcMotorEx.Direction.REVERSE);
            BL.setDirection(DcMotorEx.Direction.REVERSE);
        }//RetractN = hardwareMap.get(Servo.class,"RetractN");


    }

    public void retractN() {
        RetractN.setPosition(0);
    }

    public void deployN() {
        RetractN.setPosition(0.4);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }


    public void set(Pose2d pose, double g1RTrigger) { // mainDrive
        double x = pose.x, y = pose.y, rot = pose.heading;


        double speed = Math.hypot(x, y);

        double robotAngle = Math.atan2(y, x) - Math.PI / 4;

        rotPower = 1;

//        power = 1;

        if (g1RTrigger > 0.05) {
            if (Math.abs(speed) > 0.9 && power < 1.5) {
                double delta = Math.abs(speed) * 0.02;
                power = power + delta;
            } else if (Math.abs(speed) < 0.6) {
                power = 0.6;
            }
//            if (Math.abs(rot) > 0.9 && rotPower < 1.5) {
//                double deltaRot = Math.abs(rot) * 0.01;
//                rotPower = rotPower + deltaRot;
//            } else if (Math.abs(rot) < 0.6) {
//                rotPower = 0.6;
//            }
        } else power = 1;

        /*if(Math.abs(speed) > 0.85) power *= Math.sqrt(2);
        if(Math.abs(rot) > 0.85) rot *= Math.sqrt(2);
         */

        double FLPower = speed * Math.sin(robotAngle) * power + rot * rotPower;
        double FRPower = speed * Math.cos(robotAngle) * power - rot * rotPower;
        double BLPower = speed * Math.cos(robotAngle) * power + rot * rotPower;
        double BRPower = speed * Math.sin(robotAngle) * power - rot * rotPower;

        double[] Powers = {FLPower, FRPower, BLPower, BRPower};

        Arrays.sort(Powers);

        if (FLPower > 1 || FRPower > 1 || BLPower > 1 || BRPower > 1) {
            FLPower = FLPower / Powers[3];
            FRPower = FRPower / Powers[3];
            BLPower = BLPower / Powers[3];
            BRPower = BRPower / Powers[3];
        }

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
    }


}
