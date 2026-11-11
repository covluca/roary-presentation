package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.drive.MecanumDrive;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class OdometryLocalizer extends MecanumDrive {
    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 1.75; // 3 cm pt rotile panda
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (DcMotorEx) speed
    public static double TICKS_PER_CM = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_RADIUS * 2 * Math.PI);
    public static double LATERAL_DISTANCE = 26.5; // cm; distance between the left and right wheels
    public static double LATERAL_DISTANCE_TICKS = LATERAL_DISTANCE * TICKS_PER_CM; // cm; distance between the left and right wheels
    public static double FORWARD_OFFSET = -17;//depinde cum o iei, s-ar putea sa fie 18.5 // cm; 15 pt el torro // offset of the lateral wheel

    public static String robotName;

    public static DcMotorEx leftEncoder;
    public static DcMotorEx rightEncoder;
    public static DcMotorEx frontEncoder;

    // acest raport se masoara astfel: Distanta MASURATA / Distanta PERCEPUTA
    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    public OdometryLocalizer(HardwareMap hardwareMap) {
        super(hardwareMap);

        leftEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "BL");

        /*rightEncoder = hardwareMap.get(DcMotorEx.class, "BR");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "FL");
         */

        //leftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        //rightEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        //frontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public OdometryLocalizer(HardwareMap hardwareMap, String robotName) {
        super(hardwareMap, robotName);

        this.robotName = robotName;

        leftEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "BL");

        switch (robotName) {
            case "Echo":
                TICKS_PER_REV = 8192;
                WHEEL_RADIUS = 2.9;
                GEAR_RATIO = 1; // output (wheel) speed / input (DcMotorEx) speed
                TICKS_PER_CM = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_RADIUS * 2 * Math.PI);
                LATERAL_DISTANCE = 17; // cm; distance between the left and right wheels
                // cm; distance between the left and right wheels
                LATERAL_DISTANCE_TICKS = LATERAL_DISTANCE * TICKS_PER_CM;
                FORWARD_OFFSET = -13.5;

                break;
        }

        /*rightEncoder = hardwareMap.get(DcMotorEx.class, "BR");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "FL");
         */

        //leftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        //rightEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        //frontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public int getLeftEnc() {
        if(this.robotName == "Echo")
            return (int) (-leftEncoder.getCurrentPosition() * Y_MULTIPLIER);
        return (int) (leftEncoder.getCurrentPosition() * Y_MULTIPLIER);
    }

    public int getRightEnc() {
        return (int) (rightEncoder.getCurrentPosition() * Y_MULTIPLIER);
    }

    public int getFrontEnc() {
        if(this.robotName == "Echo")
            return (int) (-frontEncoder.getCurrentPosition() * X_MULTIPLIER);
        return (int) (frontEncoder.getCurrentPosition() * X_MULTIPLIER);
    }



}
