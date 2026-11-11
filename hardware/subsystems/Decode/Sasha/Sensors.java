package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Sasha;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.closestInt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.round2Decimals;
import static java.lang.Math.abs;
import static java.lang.Math.round;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.drive.OdometryLocalizer;


public class Sensors {

    public YawPitchRollAngles angles;
    public Orientation anglesExp;
    public AnalogInput distanceBasket;
    public DcMotorEx IndexerEncoder;
    public NormalizedColorSensor colorSensorL;
    public NormalizedColorSensor colorSensorR;
    public GoBildaPinpointDriver odo;
    public Limelight3A limelight;
    public DigitalChannel laserInput;
    Telemetry telemetry;

    public inIndexer[] slots = {inIndexer.NOTHING, inIndexer.NOTHING, inIndexer.NOTHING};
    public inIndexer whatIsInThisSlot = inIndexer.NOTHING;
    public static RevTouchSensor TurretMag;

    public Sensors(HardwareMap hardwareMap, LinearOpMode opMode, boolean resetPinpoint) {


        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-73.0, 68.0, DistanceUnit.MM); ///TODO
        odo.setEncoderResolution(OdometryLocalizer.TICKS_PER_CM / 10, DistanceUnit.MM);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED); ///TODO
        if (resetPinpoint) odo.resetPosAndIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        IndexerEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        IndexerEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        TurretMag = hardwareMap.get(RevTouchSensor.class, "TurretMag");

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigital");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");

        colorSensorL.setGain(11.1F);
        colorSensorR.setGain(15.05F);

        //telemetry = opMode.telemetry;
    }


    public Pose2D getVelocity() {
        return new Pose2D(DistanceUnit.MM, odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), AngleUnit.RADIANS, odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }


    public boolean isInAlternativeSlot() {
        return laserInput.getState();
    }

    public double normalToAlternativeConvertor(double normal) {
        if (normal == 0) return 1.6;
        else if (normal == 1) return 2.6;
        else if (normal == 2) {
            if (getIndexerSlot() < (0.6 + 3.6) / 2) return 0.6;
            else return 3.6;
        }
        return -1;
    }

    public double altToNormalConvertor(double alt) {
        if (alt > 0 && alt <= 1) return 2;
        else if (alt > 1 && alt <= 2) return 0;
        else if (alt > 2 && alt <= 3) return 1;
        else if (alt > 3 && alt <= 4) return 2;
        return -1;
    }

    boolean flipFlopIndexer = false;


    public void updateSlot(boolean isNormal) {
        if (isNormal) {
            inIndexer IndexerRight =  whatIsinIndexerRight();
            inIndexer IndexerLeft =  whatIsinIndexerLeft();


            whatIsInThisSlot = inIndexer.NOTHING;
            if (IndexerLeft == inIndexer.GREEN || IndexerRight == inIndexer.GREEN)
                whatIsInThisSlot = inIndexer.GREEN;
            else if (IndexerLeft == inIndexer.PURPLE || IndexerRight == inIndexer.PURPLE)
                whatIsInThisSlot = inIndexer.PURPLE;
            else if (IndexerLeft != inIndexer.NOTHING && IndexerRight != inIndexer.NOTHING)
                whatIsInThisSlot = inIndexer.ROBOT;


            if (isOnSlot() && nearestSlot() >= 0) {
                if (slots[nearestPhisycalSlot()] == inIndexer.NOTHING || slots[nearestPhisycalSlot()] == inIndexer.ROBOT)
                    slots[nearestPhisycalSlot()] = whatIsInThisSlot;
                if ((slots[nearestPhisycalSlot()] == inIndexer.GREEN || slots[nearestPhisycalSlot()] == inIndexer.PURPLE) && (whatIsInThisSlot == inIndexer.GREEN || whatIsInThisSlot == inIndexer.PURPLE))
                    slots[nearestPhisycalSlot()] = whatIsInThisSlot;
            }
        } else {
            whatIsInThisSlot = inIndexer.NOTHING;
            if (isInAlternativeSlot() && isOnAltSlot()) whatIsInThisSlot = inIndexer.ROBOT;
            if (isOnAltSlot() && closestInt(altToNormalConvertor(round2Decimals(getIndexerSlot()))) >= 0) {
                if (slots[closestInt(altToNormalConvertor(round2Decimals(getIndexerSlot())))] == inIndexer.NOTHING)
                    slots[closestInt(altToNormalConvertor(round2Decimals(getIndexerSlot())))] = whatIsInThisSlot;
            }
        }
    }

    public void fillSlotsRobotPurple() {
        //slots[0] = inIndexer.PURPLE;
        //slots[1] = inIndexer.PURPLE;
        //slots[2] = inIndexer.PURPLE;
//        for (inIndexer slot : slots) {
//            if (slot == inIndexer.NOTHING || slot == inIndexer.ROBOT) {
//                slot = inIndexer.PURPLE;
//            }
//        }

        for (int i = 0; i < 3; i++) {
            if (slots[i] == inIndexer.NOTHING || slots[i] == inIndexer.ROBOT) {
                slots[i] = inIndexer.PURPLE;
            }
        }
    }

    public void wipeSlots() {
        slots[0] = inIndexer.NOTHING;
        slots[1] = inIndexer.NOTHING;
        slots[2] = inIndexer.NOTHING;
    }

    public inIndexer whatIsinIndexerLeft() {
        //return inIndexer.NOTHING;

        if (distanceL() > 3.14) return inIndexer.NOTHING;
        NormalizedRGBA colors = colorSensorL.getNormalizedColors();

        if (colors.red > 0.111 || colors.green > 0.111 || colors.blue > 0.111) {
            if (colors.green > colors.blue && colors.green > colors.red) {
                return inIndexer.GREEN;
            } else if (colors.blue > colors.green && colors.blue > colors.red) {
                return inIndexer.PURPLE;
            }
        }

        return inIndexer.ROBOT;
    }

    public inIndexer whatIsinIndexerRight() {
        if (distanceR() > 2.14) return inIndexer.NOTHING;
        NormalizedRGBA colors = colorSensorR.getNormalizedColors();

        if (colors.red > 0.111 || colors.green > 0.111 || colors.blue > 0.111) {
            if (colors.green > colors.blue && colors.green > colors.red && colors.green > 0.4) {
                return inIndexer.GREEN;
            } else if (colors.blue > colors.green && colors.blue > colors.red) {
                return inIndexer.PURPLE;
            }
        }

        return inIndexer.NOTHING;
    }

    public int nearestSlot() {
        return (int) (round(getIndexerSlot()));
    }

    public int nearestPhisycalSlot() {
        return (int) inRange(nearestSlot() % 3, 0, 2);
    }

    public int nearestEmptySlot() {
        if (isCurrentSlotEmpty()) return nearestPhisycalSlot();
        else if (isSlotEmpty(nearestPhisycalSlot() + 1)) return (nearestPhisycalSlot() + 1);
        else if (isSlotEmpty(nearestPhisycalSlot() + 2)) return (nearestPhisycalSlot() + 2) % 3;
        else return -1;
    }

    public int nearestOccupiedSlot() {
        if (!isCurrentSlotEmpty()) return nearestPhisycalSlot();
        else if (!isSlotEmpty(nearestPhisycalSlot() + 1)) return (nearestPhisycalSlot() + 1) % 3;
        else if (!isSlotEmpty(nearestPhisycalSlot() + 2)) return (nearestPhisycalSlot() + 2) % 3;
        else return -1;
    }

    public int firstOccupiedSlot() {
        if (!isSlotEmpty(0)) return 0;
        else if (!isSlotEmpty(1)) return 1;
        else if (!isSlotEmpty(2)) return 2;
        else return -1;
    }

    public int lastEmptySlot() {
        if (isSlotEmpty(2)) return 2;
        else if (isSlotEmpty(1)) return 1;
        else if (isSlotEmpty(0)) return 0;
        else return -1;
    }


    public boolean isCurrentSlotEmpty() {
        return slots[nearestPhisycalSlot()] == inIndexer.NOTHING;
    }

    public boolean isSlotEmpty(int slot) {
        return slots[(int) inRange(slot % 3, 0, 2)] == inIndexer.NOTHING;
    }

    public boolean isOnSlot() {
        return abs(getIndexerSlot() - nearestSlot()) < 0.2;
    }

    public boolean isOnAltSlot() {
        if (-IndexerEncoder.getVelocity() / 8192 * 60 < 10)
            return abs(getIndexerSlot() - nearestSlot() + 0.4) < 0.2;
        else return false;
    }


    public double distanceL() {
        return ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM) / 10.0;
    }

    public double distanceR() {
        return ((DistanceSensor) colorSensorR).getDistance(DistanceUnit.CM) / 10.0;
    }


    public double getIndexerSlot() {
        return abs(-IndexerEncoder.getCurrentPosition() / 8192.0 * 3.0);
    }

    public enum inIndexer {
        NOTHING, GREEN, PURPLE, ROBOT
    }
}
