package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Sasha;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.Globals.Sasha.*;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.closestPositiveInt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.round2Decimals;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSasha {
    public DcMotorEx Intake;
    Servo Tavalug;
    Servo Indexer;

    OpMode opmode;
    Telemetry telemetry;
    Sensors sensors;


    public IndexerPos indexerPos = IndexerPos.S1;
    TavalugPos tavalugPos = TavalugPos.LOAD;

    OutTakeSasha outTake;


    public IntakeState INTAKE = IntakeState.UNLOAD;

    public int rotateintPos = 0;
    public double rotateAngle = 0;

    public boolean tHFirstTime = true;

    public boolean tHDone = false;
    public boolean hExtDisabled = false;

    public double speedLift = 0;

    public double interpolatorTH = 0;

    public double dHSign = 1;

    public double initDeltaH = 0;

    public double deltaH = 0;
    public double initValBasc = 0;
    public DcMotor hExt = null;

    public double stopTime = 0;

    public double intakePower = 0;
    public boolean unloadFirstTime = false;
    public boolean unloadDone = true;
    public double currentActionUnload = 0;
    public double delayTimeUnload = 0;

    public boolean reserveIntake = false;
    public double safetyTime = 0;
    public boolean hasStartedSafety = false;
    public double safetyAction = 1;
    public boolean isIntakeNormal = true;
    public double unclogTime = 0;
    double lastIndexerPose = INDEXER_S1;
    public double delayTimeIndexer;

    public boolean indexerMoved = false;

    public boolean doneUnstuck = false;
    public boolean unstuckFirstTime = false;
    public double stuckTime = 0.0;
    public double stuckAction = 0.0;
    public IndexerPos lastIndexPos = IndexerPos.S1;
    public TavalugPos lastTavalugPos = TavalugPos.LOAD;
    public boolean inShootingZone = true;


    public IntakeSasha(HardwareMap hardwareMap, LinearOpMode opmode, Sensors sensors, OutTakeSasha outTake) {
        this.outTake = outTake;

        Indexer = hardwareMap.get(Servo.class, "Indexer");
        Tavalug = hardwareMap.get(Servo.class, "Tavalug");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        this.opmode = opmode;
        this.sensors = sensors;
    }


    public void setPowerIntake(double speed) {
        //MINUS MANANCA
        if (!reserveIntake) Intake.setPower(speed);
    }

    public double speed(double x, double vMin) {
        //ONLY FOR POSITIVE POWERS
        double speed = (-4) * x * x + 4 * x;
        return max(vMin, speed);
    }

    public void initIntake(boolean resetEncoder) {
        double sleep = opmode.getRuntime() + 0.3;
        Intake.setPower(-0.64);
        while (opmode.getRuntime() < sleep) ;

        Tavalug.setPosition(TAVALUG_HOLD);
        Indexer.setPosition(INDEXER_S1);
        sleep = opmode.getRuntime() + 1.3;
        if (resetEncoder) {
            while (opmode.getRuntime() < sleep) ;
            sensors.IndexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep = opmode.getRuntime() + 0.3;
            while (opmode.getRuntime() < sleep) ;
            sensors.IndexerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        Indexer.setPosition(INDEXER_S1);
        Intake.setPower(0);
    }

    public void updateIntake(IndexerPos indexerPos) {
        this.indexerPos = indexerPos;
        switch (indexerPos) {
            case S1: {
                Indexer.setPosition(INDEXER_S1);
                break;
            }
            case L1: {
                Indexer.setPosition(INDEXER_L1);
                break;

            }
            case S2: {
                Indexer.setPosition(INDEXER_S2);
                break;

            }
            case L2: {
                Indexer.setPosition(INDEXER_L2);
                break;

            }
            case S3: {
                Indexer.setPosition(INDEXER_S3);
                break;

            }
            case L3: {
                Indexer.setPosition(INDEXER_L3);
                break;

            }
            case S4: {
                Indexer.setPosition(INDEXER_S4);
                break;

            }
            case L4: {
                Indexer.setPosition(INDEXER_L4);
                break;

            }
            case S5: {
                Indexer.setPosition(INDEXER_S5);
                break;

            }
        }
    }

    public void updateIntake(int indexerPos) {
        switch (indexerPos) {
            case 0: {
                Indexer.setPosition(INDEXER_S1);
                this.indexerPos = IndexerPos.S1;
                break;
            }
            case 1: {
                Indexer.setPosition(INDEXER_S2);
                this.indexerPos = IndexerPos.S2;
                break;
            }
            case 2: {
                Indexer.setPosition(INDEXER_S3);
                this.indexerPos = IndexerPos.S3;
                break;
            }
            case 3: {
                Indexer.setPosition(INDEXER_S4);
                this.indexerPos = IndexerPos.S4;
                break;
            }
            case 4: {
                Indexer.setPosition(INDEXER_S5);
                this.indexerPos = IndexerPos.S5;
                break;

            }
        }
    }

    public void updateIntake(double indexerPos) {

        if (abs(indexerPos - 0) < 0.1) {
            Indexer.setPosition(INDEXER_S1);
            this.indexerPos = IndexerPos.S1;
        } else if (abs(indexerPos - 0.6) < 0.1) {
            Indexer.setPosition(INDEXER_L1);
            this.indexerPos = IndexerPos.L1;
        } else if (abs(indexerPos - 1) < 0.1) {
            Indexer.setPosition(INDEXER_S2);
            this.indexerPos = IndexerPos.S2;
        } else if (abs(indexerPos - 1.6) < 0.1) {
            Indexer.setPosition(INDEXER_L2);
            this.indexerPos = IndexerPos.L2;
        } else if (abs(indexerPos - 2) < 0.1) {
            Indexer.setPosition(INDEXER_S3);
            this.indexerPos = IndexerPos.S3;
        } else if (abs(indexerPos - 2.6) < 0.1) {
            Indexer.setPosition(INDEXER_L3);
            this.indexerPos = IndexerPos.L3;
        } else if (abs(indexerPos - 3) < 0.1) {
            Indexer.setPosition(INDEXER_S4);
            this.indexerPos = IndexerPos.S4;
        } else if (abs(indexerPos - 3.6) < 0.1) {
            Indexer.setPosition(INDEXER_L4);
            this.indexerPos = IndexerPos.L4;
        } else if (abs(indexerPos - 4) < 0.1) {
            Indexer.setPosition(INDEXER_S5);
            this.indexerPos = IndexerPos.S5;
        }
    }

    public double targetSlot() {
        switch (indexerPos) {
            case S1:
                return 0;
            case L1:
                return 0.6;
            case S2:
                return 1;
            case L2:
                return 1.6;
            case S3:
                return 2;
            case L3:
                return 2.6;
            case S4:
                return 3;
            case L4:
                return 3.6;
            case S5:
                return 4;
            default:
                return -1;
        }
    }

    public boolean unstucker = false;

    public void Unstuck() {
        if (unstuckFirstTime) {
            stuckAction = 1;
            doneUnstuck = false;
            lastIndexPos = indexerPos;
            lastTavalugPos = tavalugPos;
            unstucker = !unstucker;

            unstuckFirstTime = false;
        }


        if (stuckAction == 1) {
            updateIntake(TavalugPos.HOLD);
            setPowerIntakeAuto(-0.8);
            updateIntake(unstucker ? IndexerPos.S1 : IndexerPos.S5);
            stuckTime = opmode.getRuntime() + 0.13;
            stuckAction++;
        }
        if (stuckAction == 2 && opmode.getRuntime() > stuckTime) {
            updateIntake(unstucker ? IndexerPos.S5 : IndexerPos.S1);
            stuckTime = opmode.getRuntime() + 0.13;
            stuckAction++;
        }
        if (stuckAction == 3 && opmode.getRuntime() > stuckTime) {
            //updateIntake(lastTavalugPos);
            //setPowerIntakeAuto(0);
            stuckTime = opmode.getRuntime() + 0.13;
            updateIntake(lastIndexPos);
            //doneUnstuck = true;
            stuckAction++;
        }
        if (stuckAction == 4 && opmode.getRuntime() > stuckTime) {
            updateIntake(lastTavalugPos);
            setPowerIntakeAuto(0);
            updateIntake(lastIndexPos);
            doneUnstuck = true;
            stuckAction++;
        }
    }

    public boolean isOnTarget() {
        return abs(targetSlot() - sensors.getIndexerSlot()) < 0.13; //0.4
    }

    public void prevSlot() {
        switch (indexerPos) {
            case S1:
            case L1:
            case S2: {
                Indexer.setPosition(INDEXER_S1);
                indexerPos = IndexerPos.S1;
                break;
            }
            case L2:
            case S3: {
                Indexer.setPosition(INDEXER_S2);
                indexerPos = IndexerPos.S2;
                break;
            }
            case L3:
            case S4: {
                Indexer.setPosition(INDEXER_S3);
                indexerPos = IndexerPos.S3;
                break;
            }
            case L4:
            case S5: {
                Indexer.setPosition(INDEXER_S4);
                indexerPos = IndexerPos.S4;
                break;
            }
        }
    }

    public void nextSlot() {
        switch (indexerPos) {
            case S1:
            case L1: {
                Indexer.setPosition(INDEXER_S2);
                indexerPos = IndexerPos.S2;
                break;
            }
            case S2:
            case L2: {
                Indexer.setPosition(INDEXER_S3);
                indexerPos = IndexerPos.S3;
                break;
            }
            case S3:
            case L3: {
                Indexer.setPosition(INDEXER_S4);
                indexerPos = IndexerPos.S4;
                break;
            }
            case S4:
            case L4:
            case S5: {
                Indexer.setPosition(INDEXER_S5);
                indexerPos = IndexerPos.S5;
                break;
            }

        }
    }

    public void prevSlotAlternative() {
        switch (indexerPos) {
            case S1:
            case L1:
            case S2:
            case L2: {
                Indexer.setPosition(INDEXER_L1);
                indexerPos = IndexerPos.L1;
                break;
            }
            case S3:
            case L3: {
                Indexer.setPosition(INDEXER_L2);
                indexerPos = IndexerPos.L2;
                break;
            }
            case S4:
            case L4: {
                Indexer.setPosition(INDEXER_L3);
                indexerPos = IndexerPos.L3;
                break;
            }
            case S5: {
                Indexer.setPosition(INDEXER_L4);
                indexerPos = IndexerPos.L4;
                break;
            }
        }
    }

    public void nextSlotAlternative() {
        switch (indexerPos) {
            case S1:
            case L1: {
                Indexer.setPosition(INDEXER_L2);
                indexerPos = IndexerPos.L2;
                break;
            }
            case S2:
            case L2: {
                Indexer.setPosition(INDEXER_L3);
                indexerPos = IndexerPos.L3;
                break;
            }
            case S3:
            case L3:
            case S4:
            case L4: {
                Indexer.setPosition(INDEXER_L4);
                indexerPos = IndexerPos.L4;
                break;
            }
        }
    }

    public void launchElement() {
        switch (indexerPos) {
            case S1:
            case L1: {
                Indexer.setPosition(INDEXER_S2);
                indexerPos = IndexerPos.S2;
                break;
            }
            case S2:
            case L2: {
                Indexer.setPosition(INDEXER_S3);
                indexerPos = IndexerPos.S3;
                break;
            }
            case S3:
            case L3: {
                Indexer.setPosition(INDEXER_S4);
                indexerPos = IndexerPos.S4;
                break;
            }
            //case S5:
            case S4:
            case L4: {
                Indexer.setPosition(INDEXER_S5);
                indexerPos = IndexerPos.S5;
                break;
            }

        }
    }
//    public void launchElement() {
//        switch (indexerPos) {
//            case S1:
//            case L1: {
//                Indexer.setPosition(INDEXER_L1);
//                indexerPos = IndexerPos.L1;
//                break;
//            }
//            case S2:
//            case L2: {
//                Indexer.setPosition(INDEXER_L2);
//                indexerPos = IndexerPos.L2;
//                break;
//            }
//            case S3:
//            case L3: {
//                Indexer.setPosition(INDEXER_L3);
//                indexerPos = IndexerPos.L3;
//                break;
//            }
//            //case S5:
//            case S4:
//            case L4: {
//                Indexer.setPosition(INDEXER_L4);
//                indexerPos = IndexerPos.L4;
//                break;
//            }
//
//        }
//    }

    public void launchElementV2() {
        switch (indexerPos) {
            case S1:
            case L1: {
                Indexer.setPosition(INDEXER_S2);
                indexerPos = IndexerPos.S2;
                break;
            }
            case S2:
            case L2: {
                Indexer.setPosition(INDEXER_S3);
                indexerPos = IndexerPos.S3;
                break;
            }
            case S3:
            case L3: {
                Indexer.setPosition(INDEXER_S4);
                indexerPos = IndexerPos.S4;
                break;
            }
            case S4:
            case L4: {
                Indexer.setPosition(INDEXER_S5);
                indexerPos = IndexerPos.S5;
                break;
            }
        }
    }


    public void startUnload() {
        if (sensors.nearestOccupiedSlot() != -1) {
            unloadFirstTime = true;
            unloadDone = false;
        }
    }

    public void setPowerIntakeAuto(double speed) {
        intakePower = speed;
        //MINUS MANANCA
        Intake.setPower(speed);
    }

    public void loadManager() {
        if (lastIndexerPose != Indexer.getPosition()) {
            delayTimeIndexer = opmode.getRuntime() + 0.25;
            indexerMoved = true;
        }

        if (isIntakeNormal) {
            if (sensors.slots[sensors.nearestPhisycalSlot()] != Sensors.inIndexer.NOTHING && sensors.lastEmptySlot() != -1)
                updateIntake(sensors.lastEmptySlot());
        } else if (sensors.slots[closestPositiveInt(sensors.altToNormalConvertor(round2Decimals(sensors.getIndexerSlot())))] != Sensors.inIndexer.NOTHING && sensors.lastEmptySlot() != -1)
            updateIntake(sensors.normalToAlternativeConvertor(sensors.lastEmptySlot()));

        lastIndexerPose = Indexer.getPosition();
    }


    public void unloadManager() {
        if (unloadFirstTime) {
            unloadDone = false;
            currentActionUnload = 1;
            unloadFirstTime = false;
        }

//        if (!unloadDone) {
//            outtake.setRPMAuto();
//        }

        if (hasStartedSafety) {
            if (safetyAction == 1) {
                prevSlot();
                unclogTime = opmode.getRuntime() + 0.08;
                safetyAction++;
            }
            if (safetyAction == 2 && opmode.getRuntime() > unclogTime) {
                nextSlot();
                unclogTime = opmode.getRuntime() + 0.08;
                safetyAction++;
            }
            if (safetyAction == 3 && isOnTarget() && opmode.getRuntime() > unclogTime) {

                currentActionUnload--;
                safetyTime = opmode.getRuntime() + 1.7;
                safetyAction = 1;
                hasStartedSafety = false;
            }
        }

        boolean CONDITION = inShootingZone && outTake.velocityDone && outTake.isAtSpeedSecondary() && outTake.SLauncher.getPower() > 0 && outTake.turretDone;

        if (currentActionUnload == 1 && sensors.firstOccupiedSlot() != -1) {
            reserveIntake = true;

            setPowerIntakeAuto(-0.64);
            updateIntake(TavalugPos.HOLD);
            updateIntake(sensors.firstOccupiedSlot());
            safetyTime = opmode.getRuntime() + 0.8;
            currentActionUnload++;
        }// else if (currentActionUnload == 1) currentActionUnload = 6;
        //if (currentActionUnload == 2 && !hasStartedSafety && opmode.getRuntime() > safetyTime)
        if (currentActionUnload == 2 && !hasStartedSafety && isOnTarget()) {
            outTake.setRPM();
            updateIntake(TavalugPos.UNLOAD);

            setPowerIntakeAuto(-0.71);

            sensors.slots[sensors.nearestPhisycalSlot()] = Sensors.inIndexer.NOTHING;
            safetyTime = opmode.getRuntime() + 0.8;

            delayTimeUnload = opmode.getRuntime() + 0.32;

            currentActionUnload++;
        }
        //if (currentActionUnload == 3 && !hasStartedSafety && opmode.getRuntime() > safetyTime)
        //    hasStartedSafety = true;

        if (currentActionUnload == 3 && !hasStartedSafety && (sensors.distanceR() >= 5 || opmode.getRuntime() > delayTimeUnload)) {
            setPowerIntakeAuto(-0.64);
            delayTimeUnload = opmode.getRuntime() + 0.07;
            safetyTime = opmode.getRuntime() + 1.3;

            currentActionUnload++;
        }
        //if (currentActionUnload == 4 && !hasStartedSafety && opmode.getRuntime() > safetyTime && outtake.velocityDone)
        //    hasStartedSafety = true;
        if (currentActionUnload == 4 && !hasStartedSafety && opmode.getRuntime() > delayTimeUnload && (CONDITION)) {
            launchElement();
            safetyTime = opmode.getRuntime() + 0.8;
            updateIntake(TavalugPos.HOLD);


            currentActionUnload++;
            delayTimeUnload = opmode.getRuntime() + 0.3;
            Intake.setPower(-0.53);
        }
        //if (currentActionUnload == 5 && !hasStartedSafety && opmode.getRuntime() > safetyTime)
        //    hasStartedSafety = true;
        if (currentActionUnload == 5 && !hasStartedSafety && (isOnTarget() || opmode.getRuntime() > delayTimeUnload)) {
            //updateIntake(sensors.firstOccupiedSlot() != -1 ? sensors.firstOccupiedSlot() : sensors.lastEmptySlot());
            Intake.setPower(-0.53);
            currentActionUnload++;
            delayTimeUnload = opmode.getRuntime() + 0.3;
            unloadDone = true;
        }
        if (currentActionUnload == 6 && opmode.getRuntime() >= delayTimeUnload) {
            updateIntake(TavalugPos.HOLD);
            if (isIntakeNormal) updateIntake(sensors.lastEmptySlot());
            else updateIntake(IndexerPos.L3);

            outTake.stopLauncher();
            setPowerIntakeAuto(0);
            reserveIntake = false;

            currentActionUnload++;
        }
    }


    public void updateIntake(TavalugPos tavalugPos) {

        this.tavalugPos = tavalugPos;
        switch (tavalugPos) {
            case LOAD:
                Tavalug.setPosition(TAVALUG_LOAD);
                break;
            case UNLOAD:
                Tavalug.setPosition(TAVALUG_UNLOAD);
                break;
            case HOLD:
                Tavalug.setPosition(TAVALUG_HOLD);
                break;
        }
    }


    public enum IntakeState {
        LOAD, UNLOAD
    }


    public enum TavalugPos {
        LOAD, HOLD, UNLOAD
    }

    public enum IndexerPos {
        S1, L1, S2, L2, S3, L3, S4, L4, S5
    }
}
