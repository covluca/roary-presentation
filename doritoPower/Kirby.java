package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.Sensors;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.IntakeKirby;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.OutTakeKirby;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.util.PositionLoggerUtil;

import java.util.Objects;

public class Kirby {

    public IntakeKirby intake = null;
    public OutTakeKirby outTake = null;
    public Sensors sensors = null;
    public Roaryx7 drive = null;
    public LinearOpMode opMode = null;

    public String side = "blank";
    public boolean isAuto = true;
    public Pose2d initPos = new Pose2d();
    public int motif = -1;


    public Kirby(HardwareMap hardwareMap, LinearOpMode opMode) {
        sensors = new Sensors(hardwareMap);
        outTake = new OutTakeKirby(hardwareMap, opMode, sensors);
        intake = new IntakeKirby(hardwareMap, opMode, sensors, outTake);
        drive = new Roaryx7(hardwareMap, opMode, "Kirby");
        this.opMode = opMode;
    }

    public Kirby(HardwareMap hardwareMap, LinearOpMode opMode, boolean resetOdo) {
        sensors = new Sensors(hardwareMap, resetOdo);
        outTake = new OutTakeKirby(hardwareMap, opMode, sensors);
        intake = new IntakeKirby(hardwareMap, opMode, sensors, outTake);
        drive = new Roaryx7(hardwareMap, opMode, "Kirby");
        this.opMode = opMode;
    }

    public double totalCurent() {
        return outTake.Turela.getCurrent(CurrentUnit.AMPS) + outTake.PLauncher.getCurrent(CurrentUnit.AMPS) + outTake.SLauncher.getCurrent(CurrentUnit.AMPS) + intake.Intake.getCurrent(CurrentUnit.AMPS) + drive.FL.getCurrent(CurrentUnit.AMPS) + drive.FR.getCurrent(CurrentUnit.AMPS) + drive.BL.getCurrent(CurrentUnit.AMPS) + drive.BR.getCurrent(CurrentUnit.AMPS);
    }

    public void setSideRed() {
        side = "red";
    }

    public void setSideBlue() {
        side = "blue";
    }

    public boolean isSideRed() {
        return Objects.equals(side, "red");
    }

    public boolean isSideBlue() {
        return Objects.equals(side, "blue");
    }

    public void setAuto(boolean isAuto) {
        this.isAuto = isAuto;
    }

    public void initKirby(boolean resetSensors) {
        while (sensors.odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) ;
        outTake.initOuttake(resetSensors);
        intake.initIntake(resetSensors, isAuto);
    }

    public void initKirby(PositionLoggerUtil.PositionData initVals) {
        side = initVals.side;
        motif = initVals.motif;
        // initPos = new Pose2d(initVals.x, initVals.y, initVals.heading);
    }


}
