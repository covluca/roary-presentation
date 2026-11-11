package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.teleOp.Kirby;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.AngleWrap;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.H;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.doneGoTo;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getH;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getPoint;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getX;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.getY;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.i;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7.splineDone;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isRobotInBigTriangle;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.isRobotInSmallTriangle;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.Limelight.artifact;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Kirby;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.Roaryx7;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.goToValues;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.Globals;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Point;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kathy.IntakeKathy;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.IntakeKirby;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.OutTakeKirby;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.subsystems.Decode.Kirby.Sensors;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.util.PositionLoggerUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EventListener;
import java.util.List;
import java.util.Locale;


@TeleOp
public class KirbyTester extends LinearOpMode {
    Kirby robot;
    ElapsedTime timer = new ElapsedTime();
    double power = 1;
    double targetDegrees = 0;
    double batteryVoltage = 0;

    double spitTime = 0;
    boolean hasSpit = false;

    Point basket = new Point(-166, 166, 0.94);

    boolean isLLEnabled = false;
    Pose3D mt1;
    Pose3D mt2;
    Pose2D llpos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);

    boolean shootMotif = true;

    double sectorTimer = 0;
    double worstTimer = 0;
    String worstDistrict = "";
    int ssNr = 0;

    int loopNr;

    Pose2D vel = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    double lastHVel = 0;

    boolean turretEnabled = false;

    boolean resetLeft = true;

    boolean isPipelineTags = true;

    List<artifact> artifactList = new ArrayList<>();

    int cycleNr = 0;

    double cycleTime = 0;
    double lastCycleDuration = 0;

    int llCode = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = dashboard.getTelemetry();

        robot = new Kirby(hardwareMap, this, false);

        PositionLoggerUtil.PositionData positionData = PositionLoggerUtil.loadPosition();

        if (positionData.isValid) robot.initKirby(positionData);

        robot.drive.setPosition(-16.82, -140.8, 0);

        robot.sensors.limelight.start();
        dashboard.startCameraStream(robot.sensors.limelight, 30);
        robot.sensors.switchForPosition();

        while (opModeInInit()) {
            if (gamepad1.dpad_left) robot.setSideBlue();
            if (gamepad1.dpad_right) robot.setSideRed();
            if (gamepad1.dpad_down) robot.initKirby(true);
            telemetry.addData("current", robot.outTake.Turela.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("velocity", robot.outTake.Turela.getVelocity());
            telemetry.addData("position", robot.outTake.Turela.getCurrentPosition());
            /*if (gamepad2.right_bumper)
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

        robot.initKirby(false);

        if (robot.isSideRed()) basket.x = abs(basket.x);
        else basket.x = -abs(basket.x);
        while (opModeIsActive()) {
            worstTimer = 0;
            worstDistrict = "";
            sectorTimer = getRuntime();

            batteryVoltage = 0;
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
                batteryVoltage = max(batteryVoltage, module.getInputVoltage(VoltageUnit.VOLTS));
            }
            robot.outTake.batteryVoltage = batteryVoltage;
            robot.drive.batteryVoltage = batteryVoltage;

            //sectorifier("Sector 1.1");
            if (!isLLEnabled) {
                if (gamepad1.dpad_left) {
                    robot.motif = 1;
                }
                if (gamepad1.dpad_up) {
                    robot.motif = 2;
                }
                if (gamepad1.dpad_right) {
                    robot.motif = 3;
                }
                if (gamepad1.dpadDownWasPressed()) {
                    robot.outTake.fResetDone = false;
                    resetLeft = robot.outTake.getTurretDegrees() > 0;
                }
                llCode = -1;
            }
            if (isLLEnabled) {
                //robot.sensors.limelight.getLatestResult().getDetectorResults().toString();
                if (!isPipelineTags) {
                    robot.sensors.switchForPosition();
                    isPipelineTags = true;
                }
                if (gamepad1.y) {
                    llpos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
                    mt1 = null;
                    mt2 = null;
                }

                if (gamepad1.dpad_left) {
                    robot.setSideBlue();
                    if (robot.isSideRed()) basket.x = abs(basket.x);
                    else basket.x = -abs(basket.x);
                }
                if (gamepad1.dpad_right) {
                    robot.setSideRed();
                    if (robot.isSideRed()) basket.x = abs(basket.x);
                    else basket.x = -abs(basket.x);
                }
                if (gamepad1.dpad_down) robot.initKirby(true);


                targetDegrees = 0;
                LLResult result = robot.sensors.limelight.getLatestResult();
                if (result != null) {
                    if (result.getBotpose() != null && result.getFiducialResults() != null && result.getBotposeTagCount() == 1) {
                        double tagId = result.getFiducialResults().get(0).getFiducialId();
                        telemetry.addData("tagID", tagId);
                        if (tagId == 20) {
                            gamepad1.rumble(0, 0.89, 170);
                            llCode = 1;
                        } else if (tagId == 24) {
                            gamepad1.rumble(0.64, 0, 130);
                            llCode = 2;
                        }
                        if (gamepad1.aWasPressed() && gamepad1.a) {
                            robot.sensors.odo.resetPosAndIMU();
                        }
                    }
                    if (result.getBotpose() != null && result.getBotposeTagCount() >= 2) {
                        mt1 = result.getBotpose();
                        String pos2D_data = String.format(Locale.ENGLISH, "{mt1: X: %.3f, Y: %.3f, H: %.3f}", -mt1.getPosition().y * 100, -mt1.getPosition().x * 100, mt1.getOrientation().getYaw());
                        telemetry.addData("tagNR:", result.getBotposeTagCount());
                        telemetry.addLine(pos2D_data);
                        gamepad1.rumbleBlips(1);
                        llCode = 3;
                        llpos = new Pose2D(DistanceUnit.METER, mt1.getPosition().y, -mt1.getPosition().x, AngleUnit.DEGREES, -180 + mt1.getOrientation().getYaw());

                        if (gamepad1.aWasPressed() && gamepad1.a) {
                            robot.sensors.odo.resetPosAndIMU();
                        }
                    } else if (result.getBotpose_MT2() != null && abs(robot.drive.getPose().heading) > 15 && basket.distanceTo(new Point(result.getBotpose_MT2())) < 240 && new Point(result.getBotpose_MT2()).y != 0) {
                        mt2 = result.getBotpose_MT2();
                        String pos2D_data = String.format(Locale.ENGLISH, "{mt2: X: %.3f, Y: %.3f, H: %.3f}", -mt2.getPosition().y * 100, -mt2.getPosition().x * 100, mt2.getOrientation().getYaw());
                        telemetry.addLine(pos2D_data);
                        //telemetry.addData("rangeToBasket", basket.distanceTo(new Point(result.getBotpose_MT2())) > 240);
                        if (abs(robot.drive.getPose().heading) > 15) gamepad1.rumble(0, 0.8, 130);
                        //llCode = 3;

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
            //sectorifier("Sector 1.2");

            if (isLLEnabled) {
                robot.sensors.limelight.updateRobotOrientation(toDegrees(AngleWrap(toRadians(180 - robot.drive.getPose().heading))));
            }

            //sectorifier("Sector 1.3");

            if (gamepad1.aWasPressed() && !isLLEnabled) {
                //shootMotif = !shootMotif;
                if (isPipelineTags) {
                    robot.sensors.switchForArtifacts();
                    isPipelineTags = false;
                }

                robot.sensors.limelight.captureSnapshot(String.valueOf(ssNr++));
            }

            if (!isPipelineTags) {
                LLResult result = robot.sensors.limelight.getLatestResult();
                if (result != null) {
                    if (!result.getDetectorResults().isEmpty()) {
                        for (int i = 0; i < result.getDetectorResults().size(); i++) {
                            double minDist = 13;

                            artifact detectedArtifact = artifact.computeArtifact(result.getDetectorResults().get(i), robot.drive.getPose(), robot.outTake.getTurretDegrees());

                            for (int j = 0; j < artifactList.size(); j++) {
                                minDist = min(minDist, detectedArtifact.position.subtract(artifactList.get(j).position).radius());
                            }

                            if (minDist > 8 && abs(vel.getHeading(AngleUnit.DEGREES)) < 17 && new Point(vel).radius() < 30)
                                artifactList.add(detectedArtifact);
                            telemetry.addData(i + " " + "class:", result.getDetectorResults().get(i).getClassName());
                            telemetry.addData(i + " " + "xDeg:", result.getDetectorResults().get(i).getTargetXDegrees());
                            telemetry.addData(i + " " + "yDeg:", result.getDetectorResults().get(i).getTargetYDegrees());
                            telemetry.addData(i + " " + "Area:", result.getDetectorResults().get(i).getTargetArea());
                        }
                    }
                }

                artifactList.removeIf(artifact -> !artifact.isArtifactValid());

            }

            if (gamepad1.bWasReleased()) {
                isLLEnabled = !isLLEnabled;
                //robot.sensors.limelight.
            }
            //sectorifier("Sector 1.4");

            mainDrive();
            robot.sensors.odo.update();
            //sectorifier("Sector 1.5");

            //MANAGER LAUNCH
            if (gamepad1.right_bumper && !isLLEnabled && robot.intake.launchDone) {
                robot.intake.updateRight(IntakeKirby.PaddleState.LOAD);
                robot.intake.updateMid(IntakeKirby.PaddleState.LOAD);
                robot.intake.updateLeft(IntakeKirby.PaddleState.LOAD);
                robot.sensors.fillSlotsRobot();
                robot.intake.launchDone = false;
                robot.outTake.velocityEnabled = true;
                turretEnabled = true;
            }
            //sectorifier("Sector 1.6");

            //MANAGER VELOCITY
            /*if (gamepad1.rightBumperWasPressed()) {
                //robot.outTake.updateOutake(OutTakeKirby.ElevationPos.SHOOT_FAR);
                robot.outTake.calculatedRPM = robot.outTake.calculatedRPM == 3500 ? 0 : 3500;
                robot.outTake.velocityEnabled = !robot.outTake.velocityEnabled;
            }*/

            if (gamepad1.x) {
                if (robot.intake.launchDone && !robot.sensors.areSlotsEmpty() && robot.totalCurent() < 17) {
                    robot.intake.updateRight(IntakeKirby.PaddleState.LOAD);
                    robot.intake.updateMid(IntakeKirby.PaddleState.LOAD);
                    robot.intake.updateLeft(IntakeKirby.PaddleState.LOAD);
                    robot.intake.launchDone = false;
                    robot.outTake.velocityEnabled = true;
                    turretEnabled = true;

                } /*else if (robot.intake.launchDone && robot.sensors.areSlotsEmpty()) {
                    gamepad1.rumbleBlips(2);
                    robot.outTake.Farul.setPosition(Globals.Kirby.Codes.INDIGO);
                    robot.outTake.velocityEnabled = false;
                    turretEnabled=false;
                }
                */
            }

            if (robot.intake.launchDone && robot.sensors.areSlotsEmpty() && robot.outTake.velocityEnabled) {
                gamepad1.rumbleBlips(3);
                robot.outTake.Farul.setPosition(Globals.Kirby.Codes.INDIGO);
                robot.outTake.velocityEnabled = false;
                turretEnabled = false;

                cycleNr++;
                lastCycleDuration = getRuntime() - cycleTime;
                cycleTime = getRuntime();

                if (abs(robot.outTake.getTurretDegrees()) > 48 || cycleNr % 5 == 0) {
                    robot.outTake.fResetDone = false;
                    resetLeft = robot.outTake.getTurretDegrees() > 0;
                }
            }


            if (gamepad1.rightStickButtonWasPressed()) robot.sensors.wipeSlots();

            //sectorifier("Sector 1.7");

            if (abs(robot.sensors.getVelocity().getHeading(AngleUnit.DEGREES) - lastHVel) < 200)
                vel = robot.sensors.getVelocity();


            //MANAGER BALISTICA
            //if (!isLLEnabled)
            //    targetDegrees = robot.outTake.ballisticsCalculator(basket, new Pose2d(robot.drive.getPose().add(new Point(vel).divide(8)), robot.drive.getPose().heading - vel.getHeading(AngleUnit.DEGREES) / 6.4));
            if (!isLLEnabled && turretEnabled)
                targetDegrees = robot.outTake.ballisticsCalculatorTeleop(basket, new Pose2d(robot.drive.getPose().add(new Point(vel).multiply(robot.intake.deltaT)), robot.drive.getPose().heading));

            robot.outTake.errorCalculator(new Pose2d(robot.drive.getPose().add(new Point(vel).multiply(robot.intake.deltaT)), robot.drive.getPose().heading));

            robot.outTake.shooterManager();

            //MANAGER LANSARE
            if (!shootMotif) {
                if (!robot.intake.launchDone)
                    robot.intake.launchControl((isRobotInSmallTriangle(robot.drive.getPose(), 35, 38) || isRobotInBigTriangle(robot.drive.getPose(), 35, 38)) && abs(vel.getHeading(AngleUnit.DEGREES)) < 48 && robot.outTake.fResetDone && abs(new Point(vel).radius()) < 24);
            } else if (!robot.intake.launchDone)
                robot.intake.launchControl((isRobotInSmallTriangle(robot.drive.getPose(), 35, 38) || isRobotInBigTriangle(robot.drive.getPose(), 35, 38)) && abs(vel.getHeading(AngleUnit.DEGREES)) < 48 && robot.outTake.fResetDone && abs(new Point(vel).radius()) < 24, robot.motif);

            //sectorifier("Sector 2");

            //MANAGER SLOTURI
            if (loopNr == 1 || loopNr == 2 || loopNr == 3) robot.sensors.updateSlots(loopNr - 1);
            //sectorifier("Sector 3.1");

            robot.outTake.statusCode(robot.drive.getPose(), llCode);
            //sectorifier("Sector 3.2");

            if (robot.sensors.newElement && !isLLEnabled) gamepad1.rumbleBlips(1);
            //sectorifier("Sector 3.3");

            if (robot.sensors.areSlotsFull() && !isLLEnabled) {
                if (!hasSpit) {
                    spitTime = getRuntime() + 0.24;
                    hasSpit = true;
                }
                if (getRuntime() < spitTime) {
                    robot.intake.setPowerIntakeAuto(1);
                    if (!isLLEnabled) gamepad1.rumbleBlips(1);
                } else robot.intake.setPowerIntakeAuto(gamepad1.left_bumper ? -1 : 0);
                gamepad1.rumble(0.8, 0, 170);
            } else {
                robot.intake.setPowerIntakeAuto(gamepad1.left_bumper ? -1 : 0);
                hasSpit = false;
            }
            if (robot.intake.launchDone) {
                if (gamepad1.left_bumper && robot.sensors.whatIsinIndexerRight() == Sensors.inIndexer.NOTHING && robot.sensors.whatIsinIndexerLeft() == Sensors.inIndexer.NOTHING)
                    robot.intake.updateRight(IntakeKirby.PaddleState.RESERVED);
                else robot.intake.updateRight(IntakeKirby.PaddleState.LOAD);
            }

            //sectorifier("Sector 3.4");

            robot.intake.setPowerIntake(gamepad1.right_stick_y);

            //sectorifier("Sector 4.1");

            //MANAGER TURELA
            if (robot.outTake.fResetDone) {
                if (turretEnabled || isLLEnabled)
                    robot.outTake.turretManager(targetDegrees, -vel.getHeading(AngleUnit.DEGREES), !isLLEnabled || gamepad1.y);
                else robot.outTake.setTurretIterative(0);
            } else robot.outTake.fReset(resetLeft);
            //sectorifier("Sector 4.2");

            robot.outTake.updatePowerCompensator();

            robot.drive.updatePowerCompensator();
            //targetDegrees -= gamepad1.right_stick_x * 2;
            //sectorifier("Sector 4.3");

            //OC Protection
            robot.intake.ocProtection();

            //ODO
            Pose2D pos = robot.sensors.odo.getPosition();
            robot.drive.setPosition(pos);
            /*String data = String.format(Locale.ENGLISH, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), -pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.addData("calculatedRPM", robot.outTake.calculatedRPM);
            telemetry.addData("currentRPM", robot.outTake.getRPM());

            telemetry.addData("Turret degrees:", robot.outTake.getTurretDegrees());


            telemetry.addData("currentDeg", robot.outTake.getTurretDegrees());
            telemetry.addData("calculatedDeg", robot.outTake.relTurretAngle);


             */
            telemetry.addData("LLEnabled", isLLEnabled);

            telemetry.addData("cycles Done:", cycleNr);
            telemetry.addData("cycle Time:", lastCycleDuration);


            //sectorifier("Sector 4.4");

            //robot.outTake.setSpeed(gamepad1.right_bumper ? -0.13 : 0);

            //if (gamepad1.a) robot.outTake.Turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //if (gamepad1.aWasReleased())
            //    robot.outTake.Turela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.drive.fastSendPower();

            /*if (gamepad1.x) {
                Left.setPosition(LEFT_UP);
            } else Left.setPosition(LEFT_DOWN);
            if (gamepad1.b) {
                Right.setPosition(RIGHT_UP);
            } else Right.setPosition(RIGHT_DOWN);
            if (gamepad1.y) {
                Mid.setPosition(MID_UP);
            } else Mid.setPosition(MID_DOWN);

            if (gamepad1.aWasPressed()) {
                Left.setPosition(LEFT_UP);
                sleep(200);
                Left.setPosition(LEFT_DOWN);
                sleep(170);
                Mid.setPosition(MID_UP);
                sleep(200);
                Mid.setPosition(MID_DOWN);
                sleep(170);
                Right.setPosition(RIGHT_UP);
                sleep(200);
                Right.setPosition(RIGHT_DOWN);
                sleep(170);
            }
             */
/*
            telemetry.addData("Mid", robot.sensors.slots[0]);
            telemetry.addData("Left", robot.sensors.slots[1]);
            telemetry.addData("Right", robot.sensors.slots[2]);

            telemetry.addData("elevation degrees:", robot.outTake.angle);
            telemetry.addData("range from basket:", robot.outTake.range);
            telemetry.addData("shooting safety:", robot.outTake.safety);

            telemetry.addData("currentActionLaunch", robot.intake.currentActionLaunch);
            telemetry.addData("launchDone", robot.intake.launchDone);
            telemetry.addData("launchFirstTime", robot.intake.launchFirstTime);

            telemetry.addData("calculatedRPM", robot.outTake.calculatedRPM);
            telemetry.addData("currentRPM", robot.outTake.getRPM());

            telemetry.addData("Turret degrees:", robot.outTake.getTurretDegrees());

            telemetry.addData("LLEnabled", isLLEnabled);

            telemetry.addData("calculatedDeg", robot.outTake.relTurretAngle);
            telemetry.addData("currentDeg", robot.outTake.getTurretDegrees());
            telemetry.addData("derivativeDeg", robot.outTake.turretDerivative);
            telemetry.addData("turetVelSpeed", robot.outTake.turretVelSpeed);
            telemetry.addData("turret Velocity", robot.outTake.getTurretVelocity());
            telemetry.addData("heading Velocity", vel.getHeading(AngleUnit.DEGREES));
*/
            //sectorifier("Sector 5.1");

            telemetry.addData("deltaT", robot.intake.deltaT);


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setRotation(PI);
            DashboardUtil.drawRobot(packet.fieldOverlay(), getY() / 2.54, -getX() / 2.54, -H);
            packet.fieldOverlay().setFill("red");
            packet.fieldOverlay().fillCircle(getPoint().y / 2.54, -getPoint().x / 2.54, 1.5);

            packet.fieldOverlay().setFill("blue");
            packet.fieldOverlay().fillCircle(new Pose2d(robot.drive.getPose().add(new Point(vel).multiply(robot.intake.deltaT)), robot.drive.getPose().heading).y / 2.54, -new Pose2d(robot.drive.getPose().add(new Point(vel).multiply(robot.intake.deltaT)), robot.drive.getPose().heading).x / 2.54, 1.5);

            packet.fieldOverlay().fillCircle(basket.y / 2.54, -basket.x / 2.54, 1.5);
            packet.fieldOverlay().fillCircle(robot.outTake.turret.y / 2.54, -robot.outTake.turret.x / 2.54, 1.5);

            for (int i = 0; i < artifactList.size(); i++) {
                if (artifactList.get(i).color == artifact.Color.GREEN)
                    packet.fieldOverlay().setFill("green");
                else packet.fieldOverlay().setFill("purple");

                packet.fieldOverlay().fillCircle(artifactList.get(i).position.y / 2.54, -artifactList.get(i).position.x / 2.54, 1.5);
            }

            if (mt1 != null) {
                packet.fieldOverlay().setFill("pink");
                packet.fieldOverlay().fillCircle(-mt1.getPosition().x * 100 / 2.54, -mt1.getPosition().y * 100 / 2.54, 1.5);
            }
            if (mt2 != null) {
                packet.fieldOverlay().setFill("yellow");
                packet.fieldOverlay().fillCircle(-mt2.getPosition().x * 100 / 2.54, -mt2.getPosition().y * 100 / 2.54, 1.5);
            }
            telemetry.addData("goToK", Roaryx7.goToK);
            //sectorifier("Sector 5.2");

            telemetry.addData("Hz", 1.0 / timer.seconds());
            telemetry.addData("cycleTime", timer.seconds());
            //telemetry.addData("worstDistrict", worstDistrict);
            timer.reset();

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);

            lastHVel = vel.getHeading(AngleUnit.DEGREES);

            loopNr++;
            if (loopNr > 10) loopNr = 1;
        }

        PositionLoggerUtil.savePosition(getX(), getY(), getH(), robot.side, -1);

    }

    public void sectorifier(String sector) {
        telemetry.addData(sector, getRuntime() - sectorTimer);
        if (worstTimer < getRuntime() - sectorTimer) {
            worstDistrict = sector;
            worstTimer = getRuntime() - sectorTimer;
        }
        sectorTimer = getRuntime();
    }

    public void mainDrive() {
        double speed = FastMath.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
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

}
