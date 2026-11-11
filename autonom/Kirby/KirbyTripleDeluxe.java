package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.autonom.Kirby;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.AngleWrap;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.H;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.doneGoTo;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.doneHeading;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.doneHeadingGoTo;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.findCircleRadius;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getH;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getPoint;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getX;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getY;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.goToK;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.splineDone;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.t;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.target;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isRobotInBigTriangle;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isRobotInTriangle;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.autonom.Kathy.kathyNear;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Kirby;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.goToValues;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Point;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kathy.IntakeKathy;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.util.PositionLoggerUtil;

import java.util.List;
import java.util.Locale;

@TeleOp
public class KirbyTripleDeluxe extends LinearOpMode {
    Kirby robot;
    ElapsedTime timer = new ElapsedTime();
    double power = 1;
    double targetDegrees = 0;
    double batteryVoltage = 0;

    double spitTime = 0;
    boolean hasSpit = false;

    goToValues[] TARGET = new goToValues[]{new goToValues(new Pose2d(), 0, 0)};

    goToValues blank = new goToValues(new Pose2d(0, 0, 0), 0, 0);
    goToValues elementManual = new goToValues(new Pose2d(0, 0, 0), 0.24, 1.7);
    Pose2d stopRobot = new Pose2d(0, 0, 240);
    Point basket = new Point(-166, 166, 0.98);

    goToValues artifact_1_slide_fh = new goToValues(new Pose2d(-60, 40, -90), 1, 6.4);
    goToValues artifact_1_slide = new goToValues(new Pose2d(-60, 35, -90), 1, 6.4);
    goToValues artifact_1 = new goToValues(new Pose2d(-135, 29, -90), 0.71, 3);
    goToValues artifact_2_slide_fh = new goToValues(new Pose2d(-60, 13, -90), 1, 6.4);
    goToValues artifact_2_slide = new goToValues(new Pose2d(-60, -13, -90), 1, 3);
    goToValues artifact_2_facing = new goToValues(new Pose2d(-80, -31, -90), 0.89, 3);
    goToValues artifact_2_qPress = new goToValues(new Pose2d(-153, -35, -45), 1, 3);

    goToValues artifact_2 = new goToValues(new Pose2d(-163, -31, -90), 0.71, 3);

    goToValues launch = new goToValues(new Pose2d(-45, 26, -90), 0.71, 6.4);
    goToValues launch_first = new goToValues(new Pose2d(-43, 28, -45), 0.71, 6.4);
    goToValues launch_slide = new goToValues(new Pose2d(-80, 17, -90), 0.89, 6.4);
    goToValues park = new goToValues(new Pose2d(-45, 130, 0), 0.3, 3);

    goToValues load_cycle_facing = new goToValues(new Pose2d(-89, -35, -35), 1, 3);
    goToValues load_cycle = new goToValues(new Pose2d(-148, -35, -51), 0.8, 3);
    goToValues load_cycle_front = new goToValues(new Pose2d(-147, -28, -74), 0.89, 3);
    goToValues load_cycle_mid = new goToValues(new Pose2d(-120, -47, -64), 0.89, 3);
    goToValues load_cycle_back = new goToValues(new Pose2d(-158, -55, -48), 0.89, 4.8);

    goToValues[] LOAD_CYCLE = new goToValues[]{load_cycle};
    goToValues[] LAUNCH = new goToValues[]{launch};

    boolean isLLEnabled = false;
    boolean DXEnabled = true;
    boolean ComputePoint = false;

    Pose2D vel = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    double lastHVel = 0;

    double safetyTime = 0;

    Pose2d direction = new Pose2d();
    boolean currentCondition = false;

    AUTO_STATE AUTO = AUTO_STATE.START;

    double currentAction = 1;
    double delayTime = 0;
    boolean hasStarted = true;
    int cycleNR = 0;

    double bnfTime = 0;
    boolean backNotForth = false;
    boolean bnfDone = false;

    Pose3D mt1;
    Pose3D mt2;
    Pose2D llpos;

    int loopNr;

    double side = 1;
    boolean goNext = true;

    boolean turretEnabled = true;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        robot = new Kirby(hardwareMap, this, true);

        robot.setAuto(true);
        robot.drive.setGoToK(true);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        //telemetry = dashboard.getTelemetry();


        robot.initKirby(true);

        robot.drive.setPosition(-(60.9 - 17.5), 140.8, 0);

        robot.sensors.limelight.start();
        dashboard.startCameraStream(robot.sensors.limelight, 30);

        robot.sensors.odo.setPosition(new Pose2D(DistanceUnit.CM, -(60.9 - 17.5), 140.8, AngleUnit.DEGREES, 0));

        while (opModeInInit()) {

            if (gamepad1.dpadLeftWasPressed()) {
                robot.sensors.odo.setPosition(new Pose2D(DistanceUnit.CM, -41, 140.8, AngleUnit.DEGREES, 0));
            }

            if (gamepad1.dpadRightWasPressed()) {
                robot.sensors.odo.setPosition(new Pose2D(DistanceUnit.CM, 41, 140.8, AngleUnit.DEGREES, 0));
            }

            if (gamepad1.a) {
                robot.sensors.odo.update();

                //ODO
                Pose2D pos = robot.sensors.odo.getPosition();
                robot.drive.setPosition(pos);
                String data = String.format(Locale.ENGLISH, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), -pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Position", data);

                robot.outTake.Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetDegrees = robot.outTake.ballisticsCalculator(basket, robot.drive.getPose());

                robot.outTake.turretManager(targetDegrees, -vel.getHeading(AngleUnit.DEGREES), !isLLEnabled || gamepad1.y);
                robot.drive.sendPower(new Pose2d(0, 0, -45 * (robot.isSideRed() ? -1 : 1)), new Point());
            } else {
                robot.outTake.Turela.setPower(0);
                robot.drive.fastSendPower();
            }
            ;


            if (gamepad1.dpad_left) robot.setSideBlue();
            if (gamepad1.dpad_right) robot.setSideRed();
            if (gamepad1.dpad_down) robot.initKirby(true);
            robot.outTake.Turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //telemetry.addData("current", robot.outTake.Turela.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("velocity", robot.outTake.Turela.getVelocity());
            //telemetry.addData("position", robot.outTake.Turela.getCurrentPosition());

            telemetry.addData("calculatedRPM", robot.outTake.calculatedRPM);
            telemetry.addData("currentRPM", robot.outTake.getRPM());

            telemetry.addData("calculatedDeg", robot.outTake.relTurretAngle);
            telemetry.addData("currentDeg", robot.outTake.getTurretDegrees());
            //telemetry.addData("derivativeDeg", robot.outTake.turretDerivative);
            //telemetry.addData("turetVelSpeed", robot.outTake.turretVelSpeed);

            telemetry.addLine("IF YOU SEE THIS, use DPAD LEFT and RIGHT to set the side (RIGHT is RED)");
            telemetry.addData("We are on: ", robot.side);
            telemetry.update();            /*if (gamepad2.right_bumper)
                robot.outTake.updateOutake(OutTakeMacara.ClawPos.OPEN);
            if (gamepad2.left_bumper)
                robot.outTake.updateOutake(OutTakeMacara.ClawPos.CLOSED);

            robot.outTake.setTurretDegreesIterative(0);

            if (robot.outTake.turretDone)
                robot.intake.updateIntake(IntakeMacara.WristPos.UNLOAD);

            telemetry.addLine("IF YOU SEE THIS, use DPAD LEFT and RIGHT to set the side (RIGHT is RED)");
            telemetry.addData("We are on: ", robot.side);
            telemetry.update();

             */
        }
        robot.outTake.Turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (robot.isSideRed()) {
            side = -1.0;
            basket.x = -basket.x;

            launch.target.x = -launch.target.x;
            launch.target.heading = -launch.target.heading;

            launch_slide.target.x = -launch_slide.target.x;
            launch_slide.target.heading = -launch_slide.target.heading;

            launch_first.target.x = -launch_first.target.x;
            launch_first.target.heading = -launch_first.target.heading;

            artifact_2_qPress.target.x = -artifact_2_qPress.target.x;
            artifact_2_qPress.target.heading = -artifact_2_qPress.target.heading;

            artifact_1.target.x = -artifact_1.target.x;
            artifact_1.target.heading = -artifact_1.target.heading;

            artifact_2.target.x = -artifact_2.target.x;
            artifact_2.target.heading = -artifact_2.target.heading;

            artifact_1_slide.target.x = -artifact_1_slide.target.x;
            artifact_1_slide.target.heading = -artifact_1_slide.target.heading;

            artifact_2_slide.target.x = -artifact_2_slide.target.x;
            artifact_2_slide.target.heading = -artifact_2_slide.target.heading;

            artifact_1_slide_fh.target.x = -artifact_1_slide_fh.target.x;
            artifact_1_slide_fh.target.heading = -artifact_1_slide_fh.target.heading;

            artifact_2_slide_fh.target.x = -artifact_2_slide_fh.target.x;
            artifact_2_slide_fh.target.heading = -artifact_2_slide_fh.target.heading;

            artifact_2_facing.target.x = -artifact_2_facing.target.x;
            artifact_2_facing.target.heading = -artifact_2_facing.target.heading;

            load_cycle.target.x = -load_cycle.target.x;
            load_cycle.target.heading = -load_cycle.target.heading;

            load_cycle_facing.target.x = -load_cycle_facing.target.x;
            load_cycle_facing.target.heading = -load_cycle_facing.target.heading;

            load_cycle_front.target.x = -load_cycle_front.target.x;
            load_cycle_front.target.heading = -load_cycle_front.target.heading;

            load_cycle_mid.target.x = -load_cycle_mid.target.x;
            load_cycle_mid.target.heading = -load_cycle_mid.target.heading;

            load_cycle_back.target.x = -load_cycle_back.target.x;
            load_cycle_back.target.heading = -load_cycle_back.target.heading;
        }

        while (opModeIsActive()) {
            //goNext = gamepad1.y;

            if (abs(robot.sensors.getVelocity().getHeading(AngleUnit.DEGREES) - lastHVel) < 200)
                vel = robot.sensors.getVelocity();

            batteryVoltage = 0;
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
                batteryVoltage = max(batteryVoltage, module.getInputVoltage(VoltageUnit.VOLTS));
            }
            robot.outTake.batteryVoltage = batteryVoltage;
            robot.drive.batteryVoltage = batteryVoltage;

            switch (AUTO) {
                case START: {
                    //robot.sensors.odo.setPosition(new Pose2D(DistanceUnit.CM,-162,82.25,AngleUnit.DEGREES,0));
                    resetRuntime();
                    robot.sensors.fillSlotsRobot();
                    AUTO = AUTO_STATE.LAUNCH;
                    hasStarted = true;
                    break;
                }
                case LAUNCH: {
                    if (hasStarted) {


                        if (robot.sensors.howManyBalls() <= 1) delayTime = getRuntime() + 0.64;
                        else if (robot.sensors.howManyBalls() == 2) delayTime = getRuntime() + 0.3;
                        else delayTime = getRuntime() + 0.13;


                        robot.intake.launchDone = false;
                        robot.outTake.velocityEnabled = true;
                        turretEnabled = true;
                        if (!robot.sensors.areSlotsFull()) robot.intake.setPowerIntakeAuto(-1);
                        else robot.intake.setPowerIntakeAuto(1);

                        robot.sensors.fillSlotsRobot();

                        //unloadState = 0;
                        //loadState = 0;

                        DXEnabled = true;
                        if (cycleNR == 0) TARGET = new goToValues[]{launch_first};
                        else if (cycleNR == 2)
                            TARGET = new goToValues[]{artifact_2_qPress, artifact_2_qPress, launch_slide, launch, launch};
                        else if (cycleNR > 2)
                            TARGET = new goToValues[]{load_cycle, load_cycle, launch_slide, launch, launch};
                        else TARGET = LAUNCH;

                        //if (cycleNR != 0) ComputePoint = true;
                        ComputePoint = false;

                        splineDone = false;
                        robot.drive.splineFirstTime = true;
                        doneGoTo = false;

                        //delayTime = getRuntime() + 0.8;

                        currentAction = 1;
                        hasStarted = false;
                    }
                    if (getRuntime() > delayTime && !splineDone) robot.intake.setPowerIntakeAuto(1);


                    ///SCAN HERE
                    if (splineDone && doneGoTo) {
                        robot.intake.setPowerIntakeAuto(1);

                        if (currentAction == 1 && goNext && robot.intake.launchDone) {
                            robot.intake.setPowerIntakeAuto(0);
                            robot.outTake.velocityEnabled = false;
                            turretEnabled = false;

                            cycleNR++;
                            hasStarted = true;
                            if (cycleNR < 3) AUTO = AUTO_STATE.LOAD_SPIKE;
                            else AUTO = AUTO_STATE.LOAD_CYCLE;
                            //else AUTO = AUTO_STATE.LOAD_CYCLE;
                        }

                        robot.drive.stopMotors();
                    }


                    break;
                }
                /*
                case LOAD_SPIKE: {
                    if (hasStarted) {
                        TARGET = new goToValues[]{blank};
                        if (cycleNR == 1) {
                            TARGET = new goToValues[]{launch, launch, artifact_1, artifact_1};
                            robot.intake.isIntakeNormal = false;
                        } else {
                            TARGET = new goToValues[]{launch, launch, artifact_2_facing, artifact_2, artifact_2};
                            robot.intake.isIntakeNormal = false;
                            //robot.intake.updateIntake(IntakeKathy.TavalugPos.LOAD);
                            //robot.intake.updateIntake(robot.sensors.lastEmptySlot());
                            //robot.intake.isIntakeNormal = true;
                        }


                        DXEnabled = true;


                        splineDone = false;
                        robot.drive.splineFirstTime = true;
                        doneGoTo = false;

                        currentAction = 1;


                        hasStarted = false;
                        if (cycleNR == 1) safetyTime = getRuntime() + 2.4;
                        else safetyTime = getRuntime() + 2.91;

                        robot.sensors.wipeSlots();
                    }

                    if (!isRobotInBigTriangle(robot.drive.getPose(), 35, 35) && robot.intake.intakePower == 0) {
                        robot.intake.setPowerIntakeAuto(-1);
                    }

                    if ((splineDone && doneGoTo) || robot.sensors.areSlotsFull() || getRuntime() > safetyTime) {

                        if (currentAction == 1 && goNext) {
                            robot.intake.setPowerIntakeAuto(-1);

                            delayTime = getRuntime() + 0.24;

                            currentAction++;
                        }
                        if (((currentAction == 2 && getRuntime() > delayTime) || getRuntime() > safetyTime || robot.sensors.areSlotsFull()) && goNext) {
                            //robot.intake.setPowerIntakeAuto(0);

                            AUTO = AUTO_STATE.LAUNCH;
                            hasStarted = true;

                        }
                        //robot.intake.setPowerIntake(-0.89);
                        if (DXEnabled) robot.drive.stopMotors();
                    }

                    ///&& (cycleNR > 0 || unloadState == 0)


                    break;
                }
                */
                case LOAD_SPIKE: {
                    if (hasStarted) {
                        TARGET = new goToValues[]{blank};
                        if (cycleNR == 2) {
                            TARGET = new goToValues[]{launch, launch, artifact_2_facing, artifact_2, artifact_2};
                        } else {
                            TARGET = new goToValues[]{launch, launch, artifact_1, artifact_1};
                        }

                        DXEnabled = true;

                        splineDone = false;
                        robot.drive.splineFirstTime = true;
                        doneGoTo = false;

                        currentAction = 1;

                        hasStarted = false;
                        if (cycleNR == 2) safetyTime = getRuntime() + 3.5;
                        else safetyTime = getRuntime() + 1.7;
                        robot.sensors.wipeSlots();
                    }

                    if (!isRobotInBigTriangle(robot.drive.getPose(), 35, 38) && robot.intake.intakePower == 0) {
                        robot.intake.setPowerIntakeAuto(-1);
                    }

                    if ((splineDone && doneGoTo) || robot.sensors.areSlotsFull() || getRuntime() > safetyTime || !DXEnabled) {

                        if (currentAction == 1 && goNext) {
                            if (cycleNR == 2) {
                                DXEnabled = false;
                            }
                            robot.intake.setPowerIntakeAuto(-1);

                            if (cycleNR == 2) delayTime = getRuntime() + 0.89;
                            else delayTime = getRuntime() + 0.24;

                            currentAction++;
                        }

                        if (cycleNR == 2 && !DXEnabled)
                            robot.drive.sendPower(new Pose2d(0, 0.135, -170 * side));
                        if (((currentAction == 2 && getRuntime() > delayTime) || ((getRuntime() > safetyTime || robot.sensors.areSlotsFull()) && cycleNR != 2)) && goNext) {
                            //robot.intake.setPowerIntakeAuto(0);

                            AUTO = AUTO_STATE.LAUNCH;
                            hasStarted = true;

                        }
                        //robot.intake.setPowerIntake(-0.89);
                        if (DXEnabled) robot.drive.stopMotors();
                    }

                    ///&& (cycleNR > 0 || unloadState == 0)


                    break;
                }

                case PARK: {
                    if (hasStarted) {
                        TARGET = new goToValues[]{blank};
                        TARGET[0] = new goToValues(launch);


                        DXEnabled = true;

                        splineDone = false;
                        robot.drive.splineFirstTime = true;
                        doneGoTo = false;

                        currentAction = 1;
                        robot.intake.currentActionUnload = 7;
                        robot.intake.unloadDone = true;


                        hasStarted = false;
                    }


                    if ((splineDone && doneGoTo) || !DXEnabled) {
                        robot.intake.setPowerIntakeAuto(0);
                        robot.outTake.velocityEnabled = false;
                        turretEnabled = false;

                        if (DXEnabled) robot.drive.stopMotors();
                    }

                    ///&& (cycleNR > 0 || unloadState == 0)


                    break;
                }
                case LOAD_CYCLE: {
                    if (hasStarted) {
                        TARGET = new goToValues[]{launch, launch, load_cycle_facing, load_cycle, load_cycle};


                        DXEnabled = true;


                        splineDone = false;
                        robot.drive.splineFirstTime = true;
                        doneGoTo = false;

                        currentAction = 1;


                        hasStarted = false;
                        safetyTime = getRuntime() + 3.5;

                        robot.sensors.wipeSlots();
                    }

                    if (!isRobotInBigTriangle(robot.drive.getPose(), 35, 35) && robot.intake.intakePower == 0) {
                        robot.intake.setPowerIntakeAuto(-1);
                    }

                    /*if ((splineDone && doneGoTo) || !DXEnabled) {


                        if (currentAction == 1 && goNext) {
                            safetyTime = getRuntime() + 2.4;
                            robot.sensors.wipeSlots();
                            delayTime = getRuntime() + 0.08;
                            currentAction++;
                            robot.intake.setPowerIntakeAuto(-1);

                        }
                        if (currentAction == 2 && getRuntime() > delayTime && goNext) {
                            delayTime = getRuntime() + 0.24;
                            direction = new Pose2d(-0.24 * side, 0, -90 * side);
                            robot.drive.sendPower(direction);
                            DXEnabled = false;

                            currentAction++;
                        }
                        if (currentAction == 3) {
                            if (cycleNR <= 1)
                                currentCondition = abs(robot.drive.getPose().x) > 138 || robot.sensors.areSlotsFull();
                            else
                                currentCondition = abs(robot.drive.getPose().x) > 138 || robot.sensors.areSlotsFull();
                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }
                        if (currentAction == 3 && currentCondition && getRuntime() > delayTime) {
                            robot.intake.setPowerIntakeAuto(-1);
                            direction = new Pose2d(0, 0, direction.heading);
                            robot.drive.sendPower(direction);
                            delayTime = getRuntime() + 0.13;
                            currentAction++;
                        }
                        if (currentAction == 4) {
                            currentCondition = getRuntime() > delayTime || robot.sensors.areSlotsFull();

                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }

                        if (((currentAction == 4 && currentCondition) || getRuntime() > safetyTime) && goNext) {
                            robot.intake.setPowerIntakeAuto(0);

                            AUTO = AUTO_STATE.LAUNCH;
                            hasStarted = true;

                        }
                        //robot.intake.setPowerIntake(-0.89);
                        if (DXEnabled) robot.drive.stopMotors();
                    }

                     */
                    if ((splineDone && doneGoTo) || robot.sensors.areSlotsFull() || (getRuntime() > safetyTime)) {


                        if (getRuntime() > bnfTime && (DXEnabled) && goNext) {
                            DXEnabled = false;
                            ///bnfDone = !bnfDone;

                            if (bnfDone) bnfTime = getRuntime() + 0.64;
                            else bnfTime = getRuntime() + 0.64;
                        }

                        if (!DXEnabled) {
                            //-24*side*(bnfDone?1:0)
                            if (bnfDone || getRuntime() < bnfTime - 0.24)
                                robot.drive.sendPower(new Pose2d((bnfDone ? -0.17 : -0.13) * side, bnfDone ? 0.24 : -0.13, load_cycle.target.heading));
                            else
                                robot.drive.sendPower(new Pose2d(0, 0, load_cycle.target.heading - 13 * side));

                        }
                        /*if (getRuntime() > bnfTime && goNext) {
                            if (TARGET[TARGET.length - 1] != load_cycle_back) {
                                TARGET = new goToValues[]{load_cycle_back};

                            } else {
                                TARGET = new goToValues[]{load_cycle_back, load_cycle_back, load_cycle_mid, load_cycle_front, load_cycle_front};

                            }
                            DXEnabled = true;
                            splineDone = false;
                            robot.drive.splineFirstTime = true;
                            doneGoTo = false;

                            bnfTime = getRuntime() + 0.35;
                        }/*


                        /*
                        if (currentAction == 1 && goNext) {
                            safetyTime = getRuntime() + 4.8;
                            robot.sensors.wipeSlots();
                            delayTime = getRuntime() + 0.089;
                            currentAction++;
                        }
                        if (currentAction == 2 && getRuntime() > delayTime && goNext) {
                            if (cycleNR <= 2) {
                                delayTime = getRuntime() + 0.24;
                                direction = new Pose2d(-0.17 * side, 0, -90 * side);
                                robot.drive.sendPower(direction);
                                DXEnabled = false;
                            } else {
                                delayTime = getRuntime() + 0.24;
                                direction = new Pose2d(0, -0.17, 48 * side);
                                robot.drive.sendPower(direction);
                                DXEnabled = false;
                            }
                            currentAction++;
                        }
                        if (currentAction == 3) {
                            if (cycleNR <= 2) currentCondition = abs(robot.drive.getPose().x) > 80;
                            else currentCondition = abs(robot.drive.getPose().y) > 135;
                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }
                        if (currentAction == 3 && currentCondition && getRuntime() > delayTime) {
                            robot.intake.setPowerIntakeAuto(-0.71);
                            direction = new Pose2d(0, 0, direction.heading);
                            robot.drive.sendPower(direction);
                            delayTime = getRuntime() + 0.24;
                            currentAction++;
                        }
                        if (currentAction == 4) {
                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }
                        if (currentAction == 4 && getRuntime() > delayTime && goNext) {
                            if (cycleNR <= 2) {
                                robot.intake.setPowerIntakeAuto(-0.98);
                                delayTime = getRuntime() + 0.24;
                                direction = new Pose2d(-0.17 * side, 0, -90 * side);
                                robot.drive.sendPower(direction);
                                DXEnabled = false;
                            } else {
                                robot.intake.setPowerIntakeAuto(-0.98);
                                delayTime = getRuntime() + 0.24;
                                direction = new Pose2d(0, -0.17, 48 * side);
                                robot.drive.sendPower(direction);
                                DXEnabled = false;
                            }
                            currentAction++;
                        }
                        if (currentAction == 5) {
                            if (cycleNR <= 2) currentCondition = abs(robot.drive.getPose().x) > 92;
                            else currentCondition = abs(robot.drive.getPose().y) > 147;

                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }
                        if (currentAction == 5 && currentCondition && getRuntime() > delayTime) {
                            direction = new Pose2d(0, 0, direction.heading);
                            robot.intake.setPowerIntakeAuto(-0.8);

                            robot.drive.sendPower(direction);

                            delayTime = getRuntime() + 0.24;
                            currentAction++;
                        }
                        if (currentAction == 6) {
                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }
                        if (currentAction == 6 && getRuntime() > delayTime && goNext) {
                            if (cycleNR <= 2) {
                                robot.intake.setPowerIntakeAuto(-0.98);

                                delayTime = getRuntime() + 0.35;
                                direction = new Pose2d(-0.17 * side, 0, -90 * side);
                                robot.drive.sendPower(direction);
                                DXEnabled = false;
                            } else {
                                robot.intake.setPowerIntakeAuto(-0.89);

                                delayTime = getRuntime() + 0.3;
                                direction = new Pose2d(0, -0.17, 53 * side);
                                robot.drive.sendPower(direction);
                                DXEnabled = false;
                            }
                            currentAction++;
                        }
                        if (currentAction == 7) {
                            if (cycleNR <= 2)
                                currentCondition = (abs(robot.drive.getPose().x) > 155);
                            else
                                currentCondition = (abs(robot.drive.getPose().y) > 155 || (new Point(vel).radius() < 3 && goToK == 1));
                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }
                        if (currentAction == 7 && currentCondition && getRuntime() > delayTime) {
                            robot.intake.setPowerIntakeAuto(-0.8);

                            direction = new Pose2d(0, 0, direction.heading);
                            robot.drive.sendPower(direction);
                            delayTime = getRuntime() + 0.48;


                            currentAction++;

                        }
                        if (currentAction == 8 || getRuntime() > safetyTime) {
                            robot.intake.setPowerIntakeAuto(-0.8);

                            if (cycleNR <= 2) currentCondition = (getRuntime() > delayTime);
                            else currentCondition = (getRuntime() > delayTime);
                            robot.drive.sendPower(direction);
                            DXEnabled = false;
                        }
                         */
                        if (currentAction == 1 && goNext) {
                            robot.intake.setPowerIntakeAuto(-1);

                            delayTime = getRuntime() + 2.4;

                            currentAction++;
                        }
                        if (((currentAction == 2 && getRuntime() > delayTime) || robot.sensors.areSlotsFull()) && goNext) {
                            //robot.intake.setPowerIntakeAuto(0);

                            AUTO = AUTO_STATE.LAUNCH;
                            hasStarted = true;

                        }
                        //robot.intake.setPowerIntake(-0.89);
                        if (DXEnabled) robot.drive.stopMotors();
                    }

                    ///&& (cycleNR > 0 || unloadState == 0)


                    break;
                }


            }

            /*if (loopNr == 3 && isLLEnabled) {
                targetDegrees = 0;
                LLResult result = robot.sensors.limelight.getLatestResult();
                if (result != null) {
                    if (result.getBotpose() != null && result.getBotposeTagCount() >= 2) {
                        mt1 = result.getBotpose();
                        String pos2D_data = String.format(Locale.ENGLISH, "{mt1: X: %.3f, Y: %.3f, H: %.3f}", -mt1.getPosition().y * 100, -mt1.getPosition().x * 100, mt1.getOrientation().getYaw());
                        telemetry.addData("tagNR:", result.getBotposeTagCount());
                        telemetry.addLine(pos2D_data);
                        gamepad1.rumbleBlips(1);
                        llpos = new Pose2D(DistanceUnit.METER, mt1.getPosition().y, -mt1.getPosition().x, AngleUnit.DEGREES, -180 + mt1.getOrientation().getYaw());

                        if (gamepad1.aWasPressed() && gamepad1.a) {
                            robot.sensors.odo.resetPosAndIMU();
                        }
                    } else if (result.getBotpose_MT2() != null && basket.distanceTo(new Point(result.getBotpose_MT2())) < 240 && new Point(result.getBotpose_MT2()).y != 0) {
                        mt2 = result.getBotpose_MT2();
                        String pos2D_data = String.format(Locale.ENGLISH, "{mt2: X: %.3f, Y: %.3f, H: %.3f}", -mt2.getPosition().y * 100, -mt2.getPosition().x * 100, mt2.getOrientation().getYaw());
                        telemetry.addLine(pos2D_data);
                        //telemetry.addData("rangeToBasket", basket.distanceTo(new Point(result.getBotpose_MT2())) > 240);
                        gamepad1.rumble(0, 0.8, 130);

                        llpos = new Pose2D(DistanceUnit.METER, mt2.getPosition().y, -mt2.getPosition().x, AngleUnit.DEGREES, -robot.drive.getPose().heading);
                    }

                    if (gamepad1.a) {
                        robot.sensors.odo.setPosition(llpos);
                    }


                    if (result.getBotpose_MT2() != null) {
                        telemetry.addData("rangeToBasket", basket.distanceTo(new Point(result.getBotpose_MT2())));
                        telemetry.addData("rawPoint", new Point(result.getBotpose_MT2()));

                    }
                }
            }
            if (isLLEnabled) {
                robot.sensors.limelight.updateRobotOrientation(toDegrees(AngleWrap(toRadians(180 - robot.drive.getPose().heading))));
            }

            if (gamepad1.bWasReleased()) {
                isLLEnabled = !isLLEnabled;
            }

             */

            mainDrive();
            robot.sensors.odo.update();

            //ODO
            Pose2D pos = robot.sensors.odo.getPosition();
            robot.drive.setPosition(pos);
            String data = String.format(Locale.ENGLISH, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            vel = robot.sensors.getVelocity();

            if (splineDone) target = TARGET[TARGET.length - 1].target;

            if (DXEnabled) robot.drive.splineToDX(TARGET, new Point(vel));

            if (!splineDone) telemetry.addData("go to:", "SPLINE");
            else if (DXEnabled) {
                telemetry.addData("go to:", !doneGoTo);
                /*
                telemetry.addData("doneTranslation:", doneTranslation);
                telemetry.addData("doneHeading:", doneHeadingGoTo);
                 */
                robot.drive.goToV2(TARGET[TARGET.length - 1].target, min(0.24, TARGET[TARGET.length - 1].speed), TARGET[TARGET.length - 1].error);
            } else {
                telemetry.addData("go to:", "manual");
                /*
                telemetry.addData("doneTranslation:", doneTranslation);
                telemetry.addData("doneHeading:", doneHeadingGoTo);
                telemetry.addData("headingPower:", robot.drive.headingPower);
                telemetry.addData("w/o macara:", twoXSqrd(robot.drive.interpolator, 0.13));
                telemetry.addData("interpolator:", robot.drive.interpolator);
                //telemetry.addData("deltaH:", robot.drive.deltaH);
                 */
                //telemetry.addData("deltaDegrees:", toDegrees(robot.drive.deltaH));

            }

            //MANAGER LAUNCH
            if (gamepad1.x && robot.intake.launchDone) {
                robot.sensors.fillSlotsRobot();
                robot.intake.launchDone = false;
            }

            //MANAGER VELOCITY
            if (gamepad1.rightBumperWasPressed()) {
                //robot.outTake.updateOutake(OutTakeKirby.ElevationPos.SHOOT_FAR);
                robot.outTake.calculatedRPM = robot.outTake.calculatedRPM == 3500 ? 0 : 3500;
                robot.outTake.velocityEnabled = !robot.outTake.velocityEnabled;
            }

            //MANAGER BALISTICA
            if (!isLLEnabled && turretEnabled)
                targetDegrees = robot.outTake.ballisticsCalculator(basket, new Pose2d(robot.drive.getPose().add(new Point(vel).multiply(robot.intake.deltaT)), robot.drive.getPose().heading));

            robot.outTake.errorCalculator(robot.drive.getPose());

            robot.outTake.shooterManager();
            //MANAGER TURELA
            robot.outTake.updatePowerCompensator();
            if (turretEnabled)
                robot.outTake.turretManager(targetDegrees, -vel.getHeading(AngleUnit.DEGREES), !isLLEnabled || gamepad1.y);
            else robot.outTake.setTurretIterative(0);
            //MANAGER LANSARE
            if (!robot.intake.launchDone)
                robot.intake.launchControl((ComputePoint ? (splineDone && doneGoTo) : true) && isRobotInBigTriangle(robot.drive.getPose(), 35, 38) && abs(vel.getHeading(AngleUnit.DEGREES)) < 48);

            //MANAGER SLOTURI
            if (loopNr % 3 == 1 || loopNr % 3 == 2 || loopNr % 3 == 0)
                robot.sensors.updateSlots(loopNr % 3);

            robot.outTake.statusCode(robot.drive.getPose());

            if (robot.sensors.areSlotsFull() && !isLLEnabled) {
                if (!hasSpit) {
                    spitTime = getRuntime() + 0.24;
                    hasSpit = true;
                }
                /*if (getRuntime() < spitTime) {
                    robot.intake.setPowerIntakeAuto(1);
                    if (!isLLEnabled) gamepad1.rumbleBlips(1);
                } else robot.intake.setPowerIntakeAuto(gamepad1.left_bumper ? -1 : 0);
                gamepad1.rumble(0.8, 0, 170);
                */
            } else {
                //robot.intake.setPowerIntakeAuto(gamepad1.left_bumper ? -1 : 0);
                hasSpit = false;
            }
            ;

            //robot.intake.setPowerIntake(gamepad1.right_stick_y);

            robot.drive.updatePowerCompensator();


            //robot.drive.fastSendPower();

            if (loopNr == 1) {
                telemetry.addData("Mid", robot.sensors.slots[0]);
                telemetry.addData("Left", robot.sensors.slots[1]);
                telemetry.addData("Right", robot.sensors.slots[2]);
            }

            telemetry.addData("SPEED:", robot.outTake.velocityDone ? "GO" : "NO GO");
            telemetry.addData("TURRET:", robot.outTake.turretDone ? "GO" : "NO GO");

            //telemetry.addData("elevation degrees:", robot.outTake.angle);
            //telemetry.addData("range from basket:", robot.outTake.range);
            //telemetry.addData("shooting safety:", robot.outTake.safety);

            //telemetry.addData("currentActionLaunch", robot.intake.currentActionLaunch);
            //telemetry.addData("launchDone", robot.intake.launchDone);
            //telemetry.addData("launchFirstTime", robot.intake.launchFirstTime);

            telemetry.addData("calculatedRPM", robot.outTake.calculatedRPM);
            telemetry.addData("currentRPM", robot.outTake.getRPM());

            telemetry.addData("calculatedDeg", robot.outTake.relTurretAngle);
            telemetry.addData("currentDeg", robot.outTake.getTurretDegrees());

            //telemetry.addData("Turret degrees:", robot.outTake.getTurretDegrees());

            //telemetry.addData("LLEnabled", isLLEnabled);

            if (loopNr == 8) {
                telemetry.addData("AUTO_STATE:", AUTO.toString());
                telemetry.addData("currentAction", currentAction);
                telemetry.addData("cycleNR", cycleNR);
                telemetry.addData("is robot in :", isRobotInBigTriangle(robot.drive.getPose(), 35, 35));

            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setRotation(PI);
            DashboardUtil.drawRobot(packet.fieldOverlay(), getY() / 2.54, -getX() / 2.54, -H);
            packet.fieldOverlay().setFill("red");
            packet.fieldOverlay().fillCircle(getPoint().y / 2.54, -getPoint().x / 2.54, 1.5);
            Point FL = new Point(-35 / 2, 35 / 2).rotate(toRadians(-robot.drive.getPose().heading)).add(robot.drive.getPose());
            Point FR = new Point(35 / 2, 35 / 2).rotate(toRadians(-robot.drive.getPose().heading)).add(robot.drive.getPose());
            Point BL = new Point(-35 / 2, -35 / 2).rotate(toRadians(-robot.drive.getPose().heading)).add(robot.drive.getPose());
            Point BR = new Point(35 / 2, -35 / 2).rotate(toRadians(-robot.drive.getPose().heading)).add(robot.drive.getPose());

            packet.fieldOverlay().fillCircle(FL.y / 2.54, -FL.x / 2.54, 1.5);
            packet.fieldOverlay().fillCircle(FR.y / 2.54, -FR.x / 2.54, 1.5);
            packet.fieldOverlay().fillCircle(BR.y / 2.54, -BR.x / 2.54, 1.5);
            packet.fieldOverlay().fillCircle(BL.y / 2.54, -BL.x / 2.54, 1.5);


            packet.fieldOverlay().setFill("blue");
            packet.fieldOverlay().fillCircle(robot.drive.futurePos.y / 2.54, -robot.drive.futurePos.x / 2.54, 1.5);
            packet.fieldOverlay().fillCircle(new Pose2d(robot.drive.getPose().add(new Point(vel).divide(3)), robot.drive.getPose().heading).y / 2.54, -new Pose2d(robot.drive.getPose().add(new Point(vel).divide(3)), robot.drive.getPose().heading).x / 2.54, 1.5);
            packet.fieldOverlay().fillCircle(basket.y / 2.54, -basket.x / 2.54, 1.5);
            packet.fieldOverlay().fillCircle(robot.outTake.turret.y / 2.54, -robot.outTake.turret.x / 2.54, 1.5);

            if (mt1 != null) {
                packet.fieldOverlay().setFill("pink");
                packet.fieldOverlay().fillCircle(-mt1.getPosition().x * 100 / 2.54, -mt1.getPosition().y * 100 / 2.54, 1.5);
            }
            if (mt2 != null) {
                packet.fieldOverlay().setFill("yellow");
                packet.fieldOverlay().fillCircle(-mt2.getPosition().x * 100 / 2.54, -mt2.getPosition().y * 100 / 2.54, 1.5);
            }


            telemetry.addData("Hz", 1.0 / timer.seconds());
            //telemetry.addData("cycleTime", timer.milliseconds());
            timer.reset();

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);

            loopNr++;
            if (loopNr > 10) loopNr = 1;
        }

        PositionLoggerUtil.savePosition(getX(), getY(), getH(), robot.side, -1);

    }

    public void mainDrive() {
        if (gamepad1.bWasPressed()) robot.drive.toggleGoToK();

        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - PI / 4;
        double TrigRotate = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.98;

        if (abs(gamepad1.left_stick_x) < 0.02 && abs(gamepad1.left_stick_y) < 0.02 && gamepad1.right_trigger < 0.02 && gamepad1.left_trigger < 0.02) {
            robot.drive.mainDrivePowers.put("FLPow", (double) 0);
            robot.drive.mainDrivePowers.put("FRPow", (double) 0);
            robot.drive.mainDrivePowers.put("BLPow", (double) 0);
            robot.drive.mainDrivePowers.put("BRPow", (double) 0);

            return;
        }


        if (Math.abs(speed) > 0.89 && power < 1.7) {
            double delta = Math.abs(speed) * 0.03;
            power = power + delta;
        } else if (Math.abs(speed) < 0.89) power = 1;

        double[] PowerVector = {

                -speed * Math.sin(robotAngle) * power + TrigRotate, //FL
                -speed * Math.cos(robotAngle) * power - TrigRotate, //FR
                -speed * Math.cos(robotAngle) * power + TrigRotate, //BL
                -speed * Math.sin(robotAngle) * power - TrigRotate, //BR
                1                                                   //Normalizer
        };

        PowerVector[4] = Math.max(Math.max(abs(PowerVector[0]), abs(PowerVector[1])), Math.max(abs(PowerVector[2]), abs(PowerVector[3])));


        if (PowerVector[4] > 1) {
            PowerVector[4] = 1 / PowerVector[4];

            PowerVector[0] *= PowerVector[4];
            PowerVector[1] *= PowerVector[4];
            PowerVector[2] *= PowerVector[4];
            PowerVector[3] *= PowerVector[4];
        }

        robot.drive.mainDrivePowers.put("FLPow", PowerVector[0]);
        robot.drive.mainDrivePowers.put("FRPow", PowerVector[1]);
        robot.drive.mainDrivePowers.put("BLPow", PowerVector[2]);
        robot.drive.mainDrivePowers.put("BRPow", PowerVector[3]);

    }

    public enum AUTO_STATE {
        START, LAUNCH, LOAD_SPIKE, LOAD_CYCLE, STOP, PARK
    }

}
