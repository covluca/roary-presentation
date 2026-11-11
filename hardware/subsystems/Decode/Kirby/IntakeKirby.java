package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.Globals.Kirby.*;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.closestInt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.closestPositiveInt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.round2Decimals;
import static java.lang.Math.abs;
import static java.lang.Math.cbrt;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.round;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class IntakeKirby {

    public static double TICKS_PER_CM_HEXT = 404.0 / 37.0;

    public LinearOpMode opmode;

    public Telemetry telemetry = null;

    public Sensors sensors;
    public OutTakeKirby outtake;

    public IntakeState INTAKE = IntakeState.UNLOAD;
    public PaddleState MID = PaddleState.LOAD;
    public PaddleState RIGHT = PaddleState.LOAD;
    public PaddleState LEFT = PaddleState.LOAD;
    public Servo Mid;
    public Servo Left;
    public Servo Right;
    public DcMotorEx Intake;

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

    public boolean doneUnstuck = false;
    public boolean unstuckFirstTime = false;
    public double stuckTime = 0.0;

    public boolean launchCondition = false;

    public boolean launchDone = false;
    public boolean launchFirstTime = true;
    public double currentActionLaunch = 0;
    public double delayTimeLaunch = 0;

    public int currentPaddle = 0;
    public Servo[] Paddles = new Servo[]{Mid, Right, Left};
    public double deltaT = 0.13;
    public double startTime = 0;
    public double launchTime = 0;
    double ocTime = 0;
    double lastPow = 0;
    int shotNr = 0;

    public IntakeKirby(HardwareMap hardwareMap, LinearOpMode opmode, Sensors sensors, OutTakeKirby outtake) {

        Mid = hardwareMap.get(Servo.class, "Mid");
        Left = hardwareMap.get(Servo.class, "Left");
        Right = hardwareMap.get(Servo.class, "Right");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        this.opmode = opmode;
        this.telemetry = opmode.telemetry;
        this.sensors = sensors;
        this.outtake = outtake;

    }

    //TODO UNSTUCKER
    public void Unstuck() {
        /*if (unstuckFirstTime) {
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

         */
    }

    public void setPowerIntake(double speed) {
        //MINUS MANANCA
        if (intakePower == 0) Intake.setPower(speed);
    }

    public void setPowerIntakeAuto(double speed) {
        intakePower = speed;
        //MINUS MANANCA
        Intake.setPower(speed);
    }

    public void ocProtection() {
        if (Intake.getCurrent(CurrentUnit.AMPS) > 7.1) {
            ocTime = opmode.getRuntime() + 0.08;
        }

        if (ocTime > opmode.getRuntime()) {
            Intake.setPower(1);
        } else if (intakePower != 0) Intake.setPower(intakePower);
    }

    public double speed(double x, double vMin) {
        //ONLY FOR POSITIVE POWERS
        double speed = (-4) * x * x + 4 * x;
        return max(vMin, speed);
    }

    public void initIntake(boolean resetEncoder, boolean isAuto) {
        double sleep = opmode.getRuntime() + 0.3;
        Intake.setPower(-0.64);
        while (opmode.getRuntime() < sleep && (!opmode.isStopRequested())) ;

        Mid.setPosition(MID_DOWN);
        Left.setPosition(LEFT_DOWN);
        Right.setPosition(RIGHT_DOWN);

        Intake.setPower(0);
    }

    /*public void loadManager() {
        if (isIntakeNormal) {
            if (sensors.slots[sensors.nearestPhisycalSlot()] != Sensors.inIndexer.NOTHING && sensors.lastEmptySlot() != -1)
                updateIntake(sensors.lastEmptySlot());
        } else if (sensors.slots[closestPositiveInt(sensors.altToNormalConvertor(round2Decimals(sensors.getIndexerSlot())))] != Sensors.inIndexer.NOTHING && sensors.lastEmptySlot() != -1)
            prevSlotAlternative();

        if (sensors.lastEmptySlot() == -1) updateIntake(IndexerPos.S1);
    }*/

    public void updateMid(PaddleState state) {
        MID = state;
        switch (state) {
            case LOAD: {
                Mid.setPosition(MID_DOWN);
                break;
            }
            case UNLOAD: {
                Mid.setPosition(MID_UP);
                break;
            }
            case FORCED_UNLOAD: {
                Mid.setPosition(MID_LAUNCH);
                break;
            }
            case FORCED_LOAD: {
                Mid.setPosition(MID_FORCED_DOWN);
                break;
            }
            case RESERVED: {
                Mid.setPosition(MID_RESEVED);
                break;
            }
        }
    }

    public void updateRight(PaddleState state) {
        RIGHT = state;
        switch (state) {
            case LOAD: {
                Right.setPosition(RIGHT_DOWN);
                break;
            }
            case UNLOAD: {
                Right.setPosition(RIGHT_UP);
                break;
            }
            case FORCED_UNLOAD: {
                Right.setPosition(RIGHT_LAUNCH);
                break;
            }
            case FORCED_LOAD: {
                Right.setPosition(RIGHT_FORCED_DOWN);
                break;
            }
            case RESERVED: {
                Right.setPosition(RIGHT_RESEVED);
                break;
            }
        }
    }

    public void updateLeft(PaddleState state) {
        LEFT = state;
        switch (state) {
            case LOAD: {
                Left.setPosition(LEFT_DOWN);
                break;
            }
            case UNLOAD: {
                Left.setPosition(LEFT_UP);
                break;
            }
            case FORCED_UNLOAD: {
                Left.setPosition(LEFT_LAUNCH);
                break;
            }
            case FORCED_LOAD: {
                Left.setPosition(LEFT_FORCED_DOWN);
                break;
            }
            case RESERVED: {
                Left.setPosition(LEFT_RESEVED);
                break;
            }
        }
    }

    public void updateIntake(int currentPaddle, PaddleState state) {
        if (currentPaddle == 0) updateMid(state);
        else if (currentPaddle == 1) updateLeft(state);
        else if (currentPaddle == 2) updateRight(state);

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
                //prevSlot();
                unclogTime = opmode.getRuntime() + 0.08;
                safetyAction++;
            }
            if (safetyAction == 2 && opmode.getRuntime() > unclogTime) {
                //nextSlot();
                unclogTime = opmode.getRuntime() + 0.08;
                safetyAction++;
            }
            if (safetyAction == 3 && opmode.getRuntime() > unclogTime) {

                currentActionUnload--;
                safetyTime = opmode.getRuntime() + 1.7;
                safetyAction = 1;
                hasStartedSafety = false;
            }
        }

        if (currentActionUnload == 1 && sensors.areSlotsEmpty()) {
            reserveIntake = true;

            setPowerIntakeAuto(-0.64);
            //updateIntake(sensors.firstOccupiedSlot());
            safetyTime = opmode.getRuntime() + 0.8;
            currentActionUnload++;
        }// else if (currentActionUnload == 1) currentActionUnload = 6;
        //if (currentActionUnload == 2 && !hasStartedSafety && opmode.getRuntime() > safetyTime)
        //    hasStartedSafety = true;
        if (currentActionUnload == 2 && !hasStartedSafety) {
            outtake.setRPM();

            setPowerIntakeAuto(-0.71);

            //sensors.slots[sensors.nearestPhisycalSlot()] = Sensors.inIndexer.NOTHING;
            safetyTime = opmode.getRuntime() + 0.8;

            delayTimeUnload = opmode.getRuntime() + 0.24;

            currentActionUnload++;
        }
        //if (currentActionUnload == 3 && !hasStartedSafety && opmode.getRuntime() > safetyTime)
        //    hasStartedSafety = true;

        if (currentActionUnload == 3 && !hasStartedSafety && (opmode.getRuntime() > delayTimeUnload)) {
            setPowerIntakeAuto(-0.64);
            delayTimeUnload = opmode.getRuntime() + 0.24;
            safetyTime = opmode.getRuntime() + 1.7;

            currentActionUnload++;
        }
        //if (currentActionUnload == 4 && !hasStartedSafety && opmode.getRuntime() > safetyTime && outtake.velocityDone)
        //    hasStartedSafety = true;
        if (currentActionUnload == 4 && !hasStartedSafety && opmode.getRuntime() > delayTimeUnload && ((outtake.velocityDone) || opmode.getRuntime() > safetyTime)) {
            //launchElement();
            delayTimeUnload = opmode.getRuntime() + 0.3;
            safetyTime = opmode.getRuntime() + 0.8;

            Intake.setPower(-0.53);
            currentActionUnload++;
        }
        //if (currentActionUnload == 5 && !hasStartedSafety && opmode.getRuntime() > safetyTime)
        //    hasStartedSafety = true;
        if (currentActionUnload == 5 && !hasStartedSafety && (opmode.getRuntime() > delayTimeUnload)) {
            //updateIntake(sensors.firstOccupiedSlot() != -1 ? sensors.firstOccupiedSlot() : sensors.lastEmptySlot());
            Intake.setPower(-0.53);
            currentActionUnload++;
            delayTimeUnload = opmode.getRuntime() + 0.6;
            unloadDone = true;
        }
        if (currentActionUnload == 6 && opmode.getRuntime() >= delayTimeUnload) {
            outtake.stopLauncher();
            setPowerIntakeAuto(0);
            reserveIntake = false;

            currentActionUnload++;
        }
    }

    public void launchControl(boolean additional) {
        if (launchFirstTime) {
            launchDone = false;

            if (sensors.slots[0] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 0;
            } else if (sensors.slots[1] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 1;
            } else if (sensors.slots[2] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 2;
            } else {
                launchDone = true;
                return;
            }

            launchCondition = outtake.turretDone && outtake.velocityDone;

            currentActionLaunch = 1;
            launchFirstTime = false;
        }

        launchCondition = outtake.turretDone && outtake.velocityDone && additional;
        //launchCondition = outtake.turretDone;

        if (currentActionLaunch == 1 && launchCondition) {
            updateIntake(currentPaddle, PaddleState.FORCED_UNLOAD);
            ///TIMPUL DE URCARE A BILEI
            delayTimeLaunch = opmode.getRuntime() + 0.12;
            currentActionLaunch++;
        }

        if (currentActionLaunch == 2 && opmode.getRuntime() > delayTimeLaunch) {
            updateIntake(currentPaddle, PaddleState.UNLOAD);
            ///TIMPUL DE URCARE A BILEI
            delayTimeLaunch = opmode.getRuntime() + 0.064;
            currentActionLaunch++;
        }
        if (currentActionLaunch == 3 && opmode.getRuntime() > delayTimeLaunch) {
            updateIntake(currentPaddle, PaddleState.FORCED_LOAD);
            delayTimeLaunch = opmode.getRuntime() + 0.08;
            currentActionLaunch++;
        }
        if (currentActionLaunch == 4 && opmode.getRuntime() > delayTimeLaunch) {
            updateIntake(currentPaddle, PaddleState.LOAD);
            delayTimeLaunch = opmode.getRuntime() + 0.048;
            currentActionLaunch++;
        }
        if (currentActionLaunch == 5 && opmode.getRuntime() > delayTimeLaunch) {

            sensors.slots[currentPaddle] = Sensors.inIndexer.NOTHING;

            launchFirstTime = true;
            currentActionLaunch++;
        }
    }

    public void launchControl(boolean additional, int motif) {
        if (launchFirstTime) {
            launchDone = false;

            if (sensors.slots[0] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 0;
            } else if (sensors.slots[1] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 1;
            } else if (sensors.slots[2] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 2;
            } else {
                shotNr = 0;
                launchDone = true;
                return;
            }

            if (motif != -1 && shotNr == motif - 1) {
                for (int i = 0; i <= 2; i++) {
                    if (sensors.slots[i] == Sensors.inIndexer.GREEN) {
                        currentPaddle = i;
                    }
                }
            } else {
                for (int i = 0; i <= 2; i++) {
                    if (sensors.slots[i] == Sensors.inIndexer.PURPLE) {
                        currentPaddle = i;
                    }
                }
            }

            /*if (sensors.slots[0] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 0;
            } else if (sensors.slots[1] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 1;
            } else if (sensors.slots[2] != Sensors.inIndexer.NOTHING) {
                currentPaddle = 2;
            } else {
                shotNr = 0;
                launchDone = true;
                return;
            }*/

            deltaT = 0.089;
            //startTime = opmode.getRuntime();
            launchTime = 0.089 + opmode.getRuntime();

            launchCondition = outtake.turretDone && outtake.velocityDone;

            currentActionLaunch = 1;
            launchFirstTime = false;
        }

        if (currentActionLaunch != 2) launchTime = 0.089 + opmode.getRuntime();

        deltaT = max(0, launchTime - opmode.getRuntime());


        launchCondition = (outtake.turretDone && outtake.velocityDone && additional);
        //launchCondition = outtake.turretDone;

        if (currentActionLaunch == 1 && launchCondition) {
            updateIntake(currentPaddle, PaddleState.FORCED_UNLOAD);
            ///TIMPUL DE URCARE A BILEI
            delayTimeLaunch = opmode.getRuntime() + 0.075;
            currentActionLaunch++;
        }

        if (currentActionLaunch == 2 && opmode.getRuntime() > delayTimeLaunch) {
            updateIntake(currentPaddle, PaddleState.UNLOAD);
            ///TIMPUL DE URCARE A BILEI
            delayTimeLaunch = opmode.getRuntime() + 0.0085;
            currentActionLaunch++;
        }
        if (currentActionLaunch == 3 && opmode.getRuntime() > delayTimeLaunch) {
            updateIntake(currentPaddle, PaddleState.FORCED_LOAD);
            delayTimeLaunch = opmode.getRuntime() + 0.017;
            currentActionLaunch++;
        }
        if (currentActionLaunch == 4 && opmode.getRuntime() > delayTimeLaunch) {
            updateIntake(currentPaddle, PaddleState.LOAD);
            delayTimeLaunch = opmode.getRuntime() + 0.008;
            currentActionLaunch++;
        }
        if (currentActionLaunch == 5 && opmode.getRuntime() > delayTimeLaunch) {

            sensors.slots[currentPaddle] = Sensors.inIndexer.NOTHING;

            launchFirstTime = true;
            shotNr++;
            currentActionLaunch++;
        }
    }

    public enum IntakeState {
        LOAD, UNLOAD
    }

    public enum PaddleState {
        FORCED_LOAD, LOAD, UNLOAD, FORCED_UNLOAD, RESERVED
    }


}


