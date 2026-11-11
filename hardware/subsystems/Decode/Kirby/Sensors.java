package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.closestPositiveInt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.round2Decimals;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.Sensors.inIndexer.NOTHING;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.Sensors.inIndexer.PURPLE;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.Sensors.inIndexer.ROBOT;
import static java.lang.Math.abs;
import static java.lang.Math.round;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Sensors {

    public YawPitchRollAngles angles;
    public NormalizedColorSensor colorSensorL;
    public NormalizedColorSensor colorSensorR;
    public NormalizedColorSensor colorSensorM;
    public GoBildaPinpointDriver odo;
    public Limelight3A limelight;

    public inIndexer[] slots = {NOTHING, NOTHING, NOTHING};
    public inIndexer whatIsInThisSlot = NOTHING;
    public boolean newElement = false;

    public Sensors(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-64.834, 151.271, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");
        colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "colorM");

        colorSensorL.setGain(24F);
        colorSensorR.setGain(24F);
        colorSensorM.setGain(24F);
    }

    public Sensors(HardwareMap hardwareMap, boolean resetPos) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-64.834, 151.271, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (resetPos) odo.resetPosAndIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");
        colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "colorM");

        colorSensorL.setGain(24F);
        colorSensorR.setGain(24F);
        colorSensorM.setGain(24F);
    }

    public Pose2D getVelocity() {
        return new Pose2D(DistanceUnit.MM, odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), AngleUnit.RADIANS, odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }

    public void updateSlots(int slot) {
        newElement = false;

        if (slot == 0) {
            whatIsInThisSlot = whatIsinIndexerMid();
            if (whatIsInThisSlot != NOTHING && whatIsInThisSlot != slots[0]) newElement = true;
            slots[0] = whatIsInThisSlot != NOTHING ? whatIsInThisSlot : slots[0];
        } else if (slot == 1) {
            whatIsInThisSlot = whatIsinIndexerLeft();
            if (whatIsInThisSlot != NOTHING && whatIsInThisSlot != slots[1]) newElement = true;
            slots[1] = whatIsInThisSlot != NOTHING ? whatIsInThisSlot : slots[1];
        } else if (slot == 2) {
            whatIsInThisSlot = whatIsinIndexerRight();
            if (whatIsInThisSlot != NOTHING && whatIsInThisSlot != slots[2]) newElement = true;
            slots[2] = whatIsInThisSlot != NOTHING ? whatIsinIndexerRight() : slots[2];
        } else if (slot == -1) {
            whatIsInThisSlot = whatIsinIndexerMid();
            if (whatIsInThisSlot != NOTHING && whatIsInThisSlot != slots[0]) newElement = true;
            slots[0] = whatIsInThisSlot != NOTHING ? whatIsInThisSlot : slots[0];

            whatIsInThisSlot = whatIsinIndexerLeft();
            if (whatIsInThisSlot != NOTHING && whatIsInThisSlot != slots[1]) newElement = true;
            slots[1] = whatIsInThisSlot != NOTHING ? whatIsInThisSlot : slots[1];

            whatIsInThisSlot = whatIsinIndexerRight();
            if (whatIsInThisSlot != NOTHING && whatIsInThisSlot != slots[2]) newElement = true;
            slots[2] = whatIsInThisSlot != NOTHING ? whatIsinIndexerRight() : slots[2];
        }
    }

    public boolean areSlotsEmpty() {
        return slots[0] == NOTHING && slots[1] == NOTHING && slots[2] == NOTHING;
    }

    public boolean areSlotsFull() {
        return slots[0] != NOTHING && slots[1] != NOTHING && slots[2] != NOTHING;
    }

    public int howManyBalls() {
        return ((slots[0] != NOTHING) ? 1 : 0) + ((slots[1] != NOTHING) ? 1 : 0) + ((slots[2] != NOTHING) ? 1 : 0);
    }

    public void wipeSlots() {
        slots[0] = NOTHING;
        slots[1] = NOTHING;
        slots[2] = NOTHING;
    }

    public void fillSlotsRobot() {
        for (int i = 0; i < 3; i++) {
            if (slots[i] == NOTHING || slots[i] == ROBOT) {
                slots[i] = PURPLE;
            }
        }
    }

    public inIndexer whatIsinIndexerLeft() {
        //return inIndexer.NOTHING;
        if (((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM) > 2.4) return NOTHING;
        NormalizedRGBA colors = colorSensorL.getNormalizedColors();

        if (colors.red > 0.111 || colors.green > 0.111 || colors.blue > 0.111) {
            if (colors.green > colors.blue && colors.green > colors.red) return inIndexer.GREEN;
            else if (colors.blue > colors.green && colors.blue > colors.red)
                return inIndexer.PURPLE;
        }
        return NOTHING;

    }

    public inIndexer whatIsinIndexerRight() {
        if (((DistanceSensor) colorSensorR).getDistance(DistanceUnit.CM) > 2.1) return NOTHING;
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

    public inIndexer whatIsinIndexerMid() {
        if (((DistanceSensor) colorSensorM).getDistance(DistanceUnit.CM) > 3) return NOTHING;
        NormalizedRGBA colors = colorSensorM.getNormalizedColors();

        if (colors.red > 0.111 || colors.green > 0.111 || colors.blue > 0.111) {
            if (colors.green > colors.blue && colors.green > colors.red) return inIndexer.GREEN;
            else if (colors.blue > colors.green && colors.blue > colors.red)
                return inIndexer.PURPLE;
        }

        return inIndexer.ROBOT;
    }

    public void switchForPosition() {
        limelight.pipelineSwitch(6);
    }

    public void switchForMotif() {
        limelight.pipelineSwitch(8);
    }

    public void switchForArtifacts() {
        limelight.pipelineSwitch(3);
    }

    public enum inIndexer {
        NOTHING, GREEN, PURPLE, ROBOT
    }

}