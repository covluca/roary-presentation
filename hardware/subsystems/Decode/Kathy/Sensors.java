package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kathy;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.closestInt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.closestPositiveInt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.round2Decimals;
import static java.lang.Math.abs;
import static java.lang.Math.round;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Sensors {

    public BHI260IMU imu;
    public YawPitchRollAngles angles;
    public Orientation anglesExp;
    public AnalogInput distanceBasket;
    public DcMotorEx IndexerEncoder;
    public NormalizedColorSensor colorSensorL;
    public NormalizedColorSensor colorSensorR;
    public GoBildaPinpointDriver odo;
    public Limelight3A limelight;
    public DigitalChannel laserInput;

    public inIndexer[] slots = {inIndexer.NOTHING, inIndexer.NOTHING, inIndexer.NOTHING};
    public inIndexer whatIsInThisSlot = inIndexer.NOTHING;

    public Sensors(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        angles = imu.getRobotYawPitchRollAngles();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(7.5, -11.3, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        IndexerEncoder = hardwareMap.get(DcMotorEx.class, "IndexerEncoder");

        IndexerEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigital");

        // Set the channel as an input
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");

        colorSensorL.setGain(13.8F);
        colorSensorR.setGain(13.8F);
    }

    public Sensors(HardwareMap hardwareMap, boolean resetPos) {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        angles = imu.getRobotYawPitchRollAngles();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(7.5, -11.3, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (resetPos) odo.resetPosAndIMU();

        IndexerEncoder = hardwareMap.get(DcMotorEx.class, "IndexerEncoder");

        IndexerEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigital");

        // Set the channel as an input
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");

        colorSensorL.setGain(24F);
        colorSensorR.setGain(24F);
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

    public void updateSlot(boolean isNormal) {
        if (isNormal) {
            whatIsInThisSlot = inIndexer.NOTHING;
            if (whatIsinIndexerLeft() == inIndexer.GREEN || whatIsinIndexerRight() == inIndexer.GREEN)
                whatIsInThisSlot = inIndexer.GREEN;
            else if (whatIsinIndexerLeft() == inIndexer.PURPLE || whatIsinIndexerRight() == inIndexer.PURPLE)
                whatIsInThisSlot = inIndexer.PURPLE;
            else if (whatIsinIndexerLeft() != inIndexer.NOTHING && whatIsinIndexerRight() != inIndexer.NOTHING)
                whatIsInThisSlot = inIndexer.ROBOT;
            if (isOnSlot() && nearestSlot() >= 0) {
                if (slots[nearestPhisycalSlot()] == inIndexer.NOTHING || slots[nearestPhisycalSlot()] == inIndexer.ROBOT)
                    slots[nearestPhisycalSlot()] = whatIsInThisSlot;
                if ((slots[nearestPhisycalSlot()] == inIndexer.GREEN || slots[nearestPhisycalSlot()] == inIndexer.PURPLE) && whatIsInThisSlot == inIndexer.GREEN || whatIsInThisSlot == inIndexer.PURPLE)
                    slots[nearestPhisycalSlot()] = whatIsInThisSlot;
            }
        } else {
            whatIsInThisSlot = inIndexer.NOTHING;
            if (isInAlternativeSlot() && isOnAltSlot()) whatIsInThisSlot = inIndexer.ROBOT;
            if (isOnAltSlot() && closestPositiveInt(altToNormalConvertor(round2Decimals(getIndexerSlot()))) >= 0) {
                if (slots[closestPositiveInt(altToNormalConvertor(round2Decimals(getIndexerSlot())))] == inIndexer.NOTHING)
                    slots[closestPositiveInt(altToNormalConvertor(round2Decimals(getIndexerSlot())))] = whatIsInThisSlot;
            }
        }
    }

    public void wipeSlots() {
        slots[0] = inIndexer.NOTHING;
        slots[1] = inIndexer.NOTHING;
        slots[2] = inIndexer.NOTHING;
    }

    public void fillSlotsRobot() {
        slots[0] = inIndexer.ROBOT;
        slots[1] = inIndexer.ROBOT;
        slots[2] = inIndexer.ROBOT;
    }

    public inIndexer whatIsinIndexerLeft() {
        //return inIndexer.NOTHING;
        if (((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM) > 2.4)
            return inIndexer.NOTHING;
        NormalizedRGBA colors = colorSensorL.getNormalizedColors();

        if (colors.red > 0.111 || colors.green > 0.111 || colors.blue > 0.111) {
            if (colors.green > colors.blue && colors.green > colors.red) return inIndexer.GREEN;
            else if (colors.blue > colors.green && colors.blue > colors.red)
                return inIndexer.PURPLE;
        }
        return inIndexer.NOTHING;

    }

    public inIndexer whatIsinIndexerRight() {
        if (((DistanceSensor) colorSensorR).getDistance(DistanceUnit.CM) > 3)
            return inIndexer.NOTHING;
        NormalizedRGBA colors = colorSensorR.getNormalizedColors();

        if (colors.red > 0.111 || colors.green > 0.111 || colors.blue > 0.111) {
            if (colors.green > colors.blue && colors.green > colors.red) return inIndexer.GREEN;
            else if (colors.blue > colors.green && colors.blue > colors.red)
                return inIndexer.PURPLE;
        }
        //if (((DistanceSensor) colorSensorUnder).getDistance(DistanceUnit.CM) < 1.7)
        //    return inIndexer.ROBOT;

        return inIndexer.ROBOT;
    }

    public int nearestSlot() {
        return (int) round(getIndexerSlot());
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
        return abs(getIndexerSlot() - nearestSlot()) < 0.1;
    }

    public boolean isOnAltSlot() {
        if (IndexerEncoder.getVelocity() / 8192 * 60 < 10)
            return abs(getIndexerSlot() - nearestSlot() + 0.4) < 0.1;
        else return false;
    }


    public double distanceL() {
        return ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM);
    }

    public double distanceR() {
        return ((DistanceSensor) colorSensorR).getDistance(DistanceUnit.CM);
    }


    public void resetIMU(HardwareMap hardwareMap) {

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        imu.resetYaw();
        angles = imu.getRobotYawPitchRollAngles();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getIndexerSlot() {
        return IndexerEncoder.getCurrentPosition() / 8192.0 * 3.0;
    }

    public enum inIndexer {
        NOTHING, GREEN, PURPLE, ROBOT
    }

}