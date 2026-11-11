package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower;

import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.FSqrt;
import static org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Meth.inRange;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.drive.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Point;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Objects;


public class Roaryx7 extends OdometryLocalizer {

    public static final double RobotDegreeTicks = 150;
    public static volatile double X = 0;
    public static volatile double Y = 0;
    public static volatile double H = 0;

    public static volatile double oldX = 0;
    public static volatile double oldY = 0, oldH = 0;
    public static volatile double oldR = 0;
    public static volatile double oldL = 0;
    public static volatile double oldN = 0;
    public static volatile double R = 0;
    public static volatile double L = 0;
    public static volatile double N = 0;
    public static volatile double totalR = 0;
    public static volatile double totalL = 0;
    public static volatile double initialH = 0;
    public static volatile double deltaR = 0;
    public static volatile double deltaL = 0;
    public static volatile double deltaN = 0;
    public static double startL = 0;
    public static double startR = 0;
    public static double startN = 0;
    public static double angleIncrement = 0;
    public static double headingMixer = 1;
    public static double goToK = 1;
    public static double rotX = 0;
    public static double rotY = 0;
    public static boolean doneHeading = false;
    public static boolean doneTranslation = false;
    public static boolean doneTranslationSpline = false;
    public static Pose2d target = new Pose2d(0, 0, 0);
    public static Point lookBehind = null;
    public static Point lookAhead = null;
    public static Point derivative = null;
    public static Point normalLeft = null;
    public static Point normalRight = null;
    public static Point derivativePower = null;
    public static Point normalPower = null;
    public static Point projectionPower = null;
    public static Point totalPower = null;
    public static Point derivativeGlobal = null;
    public static Point normalGlobalLeft = null;
    public static Point normalGlobalRight = null;

    public static double normalInterpolator = 0;
    public static double projectionInterpolator = 0;
    public static double powerMixer = 0;


    public static boolean zeroT = true;
    public static double lookAheadH;
    public static double sT = 0;
    public static double headingMS = 1;
    public static double cornerAngle = 0;
    public static double cornerSeverity = 0;
    public static boolean useInterpolator = false;
    public static boolean passedThrough = false;
    public static boolean firstTime = true;
    public static boolean doneGoTo = false;
    public static Telemetry telemetry = null;
    public static double speed;
    public static boolean splineDone = false;
    public static double length = 0;
    public static boolean isLeftNormal = true;
    public static boolean isLeftProjection = true;
    public static double localT = 0;
    public static int i = 0;
    public static double t;
    public static double deltaX;
    public static double deltaY;
    public static double xSpeed = 0;
    public static double globalSpeed = 0;
    public static double ySpeed = 0;
    public static double tVR = 0;
    public static double transInter = 1;
    public static boolean doneHeadingGoTo = false;
    public double headingPower = 0;
    public double initX;
    public double initY;
    public boolean rallyMode = false;
    public double deltaHypot;
    public double initHypot = 0;
    public double deltaH;
    public double pathH;
    public double distanceH;
    public boolean splineFirstTime = true;
    public boolean lineFirstTime = true;
    public boolean lineDone = false;
    public Point startPoint;
    public Point endPoint;
    public double lt;
    public double doneTI = 0;
    public Dictionary<String, Double> mainDrivePowers = new Hashtable<>();
    public boolean TESTINGROARY = false;
    public double angleToPath = 0;
    public double distanceToPath = 0;
    public double targetX = 0;
    public double targetY = 0;
    public double corrX = 0;
    public double corrY = 0;
    public double cVR = 0;
    public double interpolator = 0;
    public double breakingDistance = 0;
    public boolean closeTo = false;
    public double breakInterpolator = 0;
    public double breakPower = 0;
    public double distanceToEnd = 0;
    public double directionToEnd = 0;
    public double extTelemetry = 0;
    public Point lastDirection = new Point();
    public Point goToDelta = new Point();

    public boolean BRAKEPLS = true;
    public Point futurePos = new Point();
    public double batteryVoltage = 14.0;
    /*public Point catmullRomSpline(Point points[], double t) {
        int i = (int) Math.floor(t);
        Point a1 = points[i];
        Point p1 = points[i + 1];
        Point p2 = points[i + 2];
        Point a2 = points[i + 3];

        double localT = t - Math.floor(t);

        double q0 = (-1 * Math.pow(localT,3))+(2 * Math.pow(localT,2))+(-1 * localT);
        double q1 = (3 * Math.pow(localT,3))+(-5 * Math.pow(localT,2))+2;
        double q2 = (-3 * Math.pow(localT,3))+(4 * Math.pow(localT,2))+localT;
        double q3 = Math.pow(localT,3) - Math.pow(localT,2);

        Point point = new Point(0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3)),
                0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3)));

        return point;

    }
     */ Dictionary<String, Double> goTo = new Hashtable<>();
    Dictionary<String, Double> corrective = new Hashtable<>();
    Point splinePoints[] = null;
    double powerCompensator = 1;


    public Roaryx7(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap);
        telemetry = opMode.telemetry;

        oldR = getRightEnc();
        oldL = getLeftEnc();
        oldN = getFrontEnc();
    }

    public Roaryx7(HardwareMap hardwareMap, LinearOpMode opMode, String robotName) {
        super(hardwareMap, robotName);
        telemetry = opMode.telemetry;


        oldR = getRightEnc();
        oldL = getLeftEnc();
        oldN = getFrontEnc();
    }

    public static double AngleWrap(double angle) {
        //angle=toRadians(angle);
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    public static double twoXSqrd(double x, double vMin) {
        //functia asta e facuta pt interpolatoare cu val intre -1 si 1
        // e un P din Pid mai bun, tinde mereu spre eroare 0
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;

        //1.55 means that when x is 0.8 speed is 1, it's better than both 1 or 2
        if (x == 0) return 0;
        double speed = 1.55 * x * x;
        double sign = x / abs(x);
        if (vMin > Math.abs(speed)) return vMin * sign;
        else if (Math.abs(speed) > 1) return sign;
        else return speed * sign;
    }

    public static double twoXSqrd(double x, double vMin, double error) {
        //functia asta e facuta pt interpolatoare cu val intre -1 si 1
        // e un P din Pid mai bun, tinde mereu spre eroare 0
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;

        //1.55 means that when x is 0.8 speed is 1, it's better than both 1 or 2
        if (abs(x) < error) return 0;
        double speed = 1.55 * x * x;
        double sign = x / abs(x);
        if (vMin > Math.abs(speed)) return vMin * sign;
        else if (Math.abs(speed) > 1) return sign;
        else return speed * sign;
    }

    public static double getX() {
        return X / TICKS_PER_CM;
    }

    public void setX(double newX) {
        X = newX * TICKS_PER_CM;
    }

    public static double getY() {
        return Y / TICKS_PER_CM;
    }

    public void setY(double newY) {
        Y = newY * TICKS_PER_CM;
    }

    public static Point getPoint() {
        return new Point(getX(), getY());
    }

    public static Pose2d getPose2D() {
        return new Pose2d(getX(), getY(), getH());
    }

    public static double findCircleRadius(Point point1, Point point2, Point point3) {
        // Midpoints of the sides of the triangle
        Point mid1 = new Point((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);
        Point mid2 = new Point((point2.x + point3.x) / 2, (point2.y + point3.y) / 2);

        // Slopes of the sides of the triangle
        double slope1 = (point2.y - point1.y) / (point2.x - point1.x);
        double slope2 = (point3.y - point2.y) / (point3.x - point2.x);

        // Perpendicular bisectors' slopes
        double perp_slope1 = -1 / slope1;
        double perp_slope2 = -1 / slope2;

        // Center of the circle
        double center_x, center_y;
        if (!Double.isInfinite(perp_slope1) && !Double.isInfinite(perp_slope2)) {
            center_x = (perp_slope1 * perp_slope2 * (mid2.y - mid1.y) + perp_slope1 * (mid1.x + mid2.x) - perp_slope2 * (mid2.x + mid1.x)) / (2 * (perp_slope1 - perp_slope2));
            center_y = (-1 / perp_slope1) * (center_x - (mid1.x + mid2.x) / 2) + (mid1.y + mid2.y) / 2;
        } else if (Double.isInfinite(perp_slope1)) {
            center_x = (mid1.x + mid2.x) / 2;
            center_y = perp_slope2 * (center_x - (mid2.x + mid1.x) / 2) + (mid2.y + mid1.y) / 2;
        } else {
            center_x = (mid1.x + mid2.x) / 2;
            center_y = perp_slope1 * (center_x - (mid1.x + mid2.x) / 2) + (mid1.y + mid2.y) / 2;
        }

        // Radius of the circle
        Point center = new Point(center_x, center_y);
        double radius = center.distanceTo(point1);
        return radius;
    }

    /*
    this.target = target;

        deltaX = target.x * TICKS_PER_CM - X;
        deltaY = target.y * TICKS_PER_CM - Y;
        deltaHypot = Math.hypot(deltaX, deltaY);
        if (firstTime) {

            targetX = target.x;
            targetY = target.y;

            initX = getX();
            initY = getY();
            initHypot = deltaHypot;

            doneHeading = false;
            doneTranslation = false;
            passedThrough = false;
            firstTime = false;
        }
     */
    public static Point findCircleCentre(Point point1, Point point2, Point point3) {
        // Midpoints of the sides of the triangle
        Point mid1 = new Point((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);
        Point mid2 = new Point((point2.x + point3.x) / 2, (point2.y + point3.y) / 2);

        // Slopes of the sides of the triangle
        double slope1 = (point2.y - point1.y) / (point2.x - point1.x);
        double slope2 = (point3.y - point2.y) / (point3.x - point2.x);

        // Perpendicular bisectors' slopes
        double perp_slope1 = -1 / slope1;
        double perp_slope2 = -1 / slope2;

        // Center of the circle
        double center_x, center_y;
        if (!Double.isInfinite(perp_slope1) && !Double.isInfinite(perp_slope2)) {
            center_x = (perp_slope1 * perp_slope2 * (mid2.y - mid1.y) + perp_slope1 * (mid1.x + mid2.x) - perp_slope2 * (mid2.x + mid1.x)) / (2 * (perp_slope1 - perp_slope2));
            center_y = (-1 / perp_slope1) * (center_x - (mid1.x + mid2.x) / 2) + (mid1.y + mid2.y) / 2;
        } else if (Double.isInfinite(perp_slope1)) {
            center_x = (mid1.x + mid2.x) / 2;
            center_y = perp_slope2 * (center_x - (mid2.x + mid1.x) / 2) + (mid2.y + mid1.y) / 2;
        } else {
            center_x = (mid1.x + mid2.x) / 2;
            center_y = perp_slope1 * (center_x - (mid1.x + mid2.x) / 2) + (mid1.y + mid2.y) / 2;
        }

        // Radius of the circle
        Point center = new Point(center_x, center_y);
        return center;
    }

    public static double dotProduct(Point v1, Point v2) {
        return (v1.x) * (v2.x) + (v1.y) * (v2.y);
    }

    public static Point projectionOnLine(Point v1, Point v2, Point p) {
        // get dot product of e1, e2
        Point e1 = new Point(v2.x - v1.x, v2.y - v1.y);
        Point e2 = new Point(p.x - v1.x, p.y - v1.y);
        double valDp = dotProduct(e1, e2);
        // get length of vectors
        double lenLineE1 = 0; //Math.sqrt(e1.x * e1.x + e1.y * e1.y);
        double lenLineE2 = 0; //Math.sqrt(e2.x * e2.x + e2.y * e2.y);

        if (Objects.equals(robotName, "Sasha")) {
            lenLineE1 = FSqrt(e1.x * e1.x + e1.y * e1.y);
            lenLineE2 = FSqrt(e2.x * e2.x + e2.y * e2.y);
        } else {
            lenLineE1 = Math.sqrt(e1.x * e1.x + e1.y * e1.y);
            lenLineE2 = Math.sqrt(e2.x * e2.x + e2.y * e2.y);
        }


        double cos = valDp / (lenLineE1 * lenLineE2);
        // length of v1P'
        double projLenOfLine = cos * lenLineE2;
        Point projected = new Point((int) (v1.x + (projLenOfLine * e1.x) / lenLineE1), (int) (v1.y + (projLenOfLine * e1.y) / lenLineE1));
        return projected;
    }

    public static double getH() {
        return toDegrees(H);
    }

    public void setH(double newH) {
        H = Math.toRadians(newH);
    }

    public void toggleGoToK() {
        goToK = goToK == 0 ? 1 : 0;
    }

    public void setGoToK(boolean enable) {
        goToK = enable ? 1 : 0;
    }

    public double getHRadians() {
        return H;
    }

    public void initialize(double r, double l, double n) {
        oldR = getRightEnc();
        oldL = getLeftEnc();
        oldN = getFrontEnc();
        startR = r;
        startL = l;
        startN = n;
        update();
    }

    public void update() {
//angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//H = H + (getLeftEnc() - getRightEnc()) / (LATERAL_DISTANCE_TICKS / 2) - oldH;
//H = -angles.firstAngle;

        R = getRightEnc();
        L = getLeftEnc();
        N = getFrontEnc();

        deltaR = R - oldR;
        deltaL = L - oldL;
        deltaN = N - oldN;

        angleIncrement = (deltaL - deltaR) / LATERAL_DISTANCE_TICKS;

        totalR = R - startR;
        totalL = L - startL;

        double lastH = H;
        H = AngleWrap(((totalL - totalR) / LATERAL_DISTANCE_TICKS) + initialH);

        //double tracker_a_prediction = Math.toDegrees(angleIncrement)/RobotDegreeTicks;
        double tracker_a_prediction = (FORWARD_OFFSET / LATERAL_DISTANCE) * (deltaL - deltaR);

        //now subtract that from the actual

        double relativeY = (deltaL + deltaR) / 2.0;
        double relativeX = deltaN - tracker_a_prediction;

        if (Math.abs(angleIncrement) > 0) {
            //gets the radius of the turn we are in
            double radiusOfMovement = (deltaL + deltaR) / (2 * angleIncrement);
            //get the radius of our straifing circle
            double radiusOfStrafe = relativeX / angleIncrement;

            relativeY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStrafe * (1 - Math.cos(angleIncrement)));
            relativeX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + (radiusOfStrafe * Math.sin(angleIncrement));

        }

        Y += (Math.cos(lastH) * relativeY) - (Math.sin(lastH) * relativeX) - oldY;
        X += (Math.sin(lastH) * relativeY) + (Math.cos(lastH) * relativeX) - oldX;

        //save the last positions for later
        oldR = R;
        oldL = L;
        oldN = N;

        //telemetry.addData("deltaR", deltaR / TICKS_PER_CM);
//        telemetry.addData("R", R / TICKS_PER_CM);
//        telemetry.addData("OldR", oldR / TICKS_PER_CM);


        //save how far we traveled in the y dimension this update for anyone that needs it
        //currently the absolute control of the collector radius uses it to compensate for
        //robot movement
        //currentTravelYDistance = relativeY;
        /*
        23/3 working
        H = H + ((deltaR - deltaL) / 2 / RobotDegreeTicks) * kH;
        X = X + (((deltaL + deltaR) / 2 * Math.sin(Math.toRadians(H))) * kX + ((deltaN - (deltaR - deltaL) / 2) * Math.cos(Math.toRadians(H))) * kY) / TICKS_PER_CM;
        Y = Y + (((deltaL + deltaR) / 2 * Math.cos(Math.toRadians(H))) * kY - ((deltaN - (deltaR - deltaL) / 2) * Math.sin(Math.toRadians(H))) * kX) / TICKS_PER_CM;
         */
/*
        oldX = X;
        oldY = Y;
        oldH = H;
 */

/*
        H = H + (getLeftEnc() - getRightEnc()) / (LATERAL_DISTANCE_TICKS/2) - oldH;
        oldH = (getLeftEnc() - getRightEnc()) / (LATERAL_DISTANCE_TICKS/2); // daca nu da bine incearca sa pui oldH = H
*/
/*
        Y = Y + Math.sin(H) * (getLeftEnc() + getRightEnc()) / 2 + Math.sin(H + Math.PI / 2) * getFrontEnc() - oldY;
        oldY = Math.sin(H) * (getLeftEnc() + getRightEnc()) / 2 + Math.sin(H + Math.PI / 2) * getFrontEnc();

*/


    }

    public double getSpeed(Pose2d oldPose, Pose2d currentPose, double oldTime, double currTime) {
        double speed = 0;

        double deltaDist = currentPose.distanceToPose(oldPose); // centi metri
        double deltaTime = (currTime - oldTime) / 1000; // secunde

        speed = deltaDist / deltaTime;

        return speed;
    }

    public void setPosition(double x, double y, double angle) {
        X = x * TICKS_PER_CM;
        Y = y * TICKS_PER_CM;
        H = Math.toRadians(angle);

        //oldX = x * TICKS_PER_CM;
        //oldY = y * TICKS_PER_CM;

        //remember where we were at the time of the reset
        startL = L;
        startR = R;
        initialH = Math.toRadians(angle);
    }

    public void setPosition(Pose2D pose) {
        X = pose.getX(DistanceUnit.CM) * TICKS_PER_CM;
        Y = pose.getY(DistanceUnit.CM) * TICKS_PER_CM;
        H = Math.toRadians(-pose.getHeading(AngleUnit.DEGREES));

        //oldX = pose.x*TICKS_PER_CM;
        //oldY = pose.y*TICKS_PER_CM;

        //remember where we were at the time of the reset
        startL = L;
        startR = R;
        initialH = Math.toRadians(pose.getHeading(AngleUnit.DEGREES));
    }

    public void setPosition(Pose2d pose) {
        X = pose.x * TICKS_PER_CM;
        Y = pose.y * TICKS_PER_CM;
        H = Math.toRadians(pose.heading);

        //oldX = pose.x*TICKS_PER_CM;
        //oldY = pose.y*TICKS_PER_CM;

        //remember where we were at the time of the reset
        startL = L;
        startR = R;
        initialH = Math.toRadians(pose.heading);
    }

    public double sqrdSine(double x, double vMin) {
        double speed = sin(PI * x) * sin(PI * x);

        if (Math.abs(vMin) > Math.abs(speed)) {
            if (speed == 0) return Math.abs(vMin);
            else return Math.abs(vMin * speed / Math.abs(speed));
        } else return Math.abs(speed);

    }

    public double negCos(double x, double vMin) {
        double speed = -1.5 * (Math.cos(Math.PI * x) * Math.cos(Math.PI * x) - 1);

        if (Math.abs(vMin) > Math.abs(speed)) {
            if (speed == 0) return Math.abs(vMin);
            else return Math.abs(vMin * speed / Math.abs(speed));
        } else return Math.abs(speed);

    }

    public double unitSW(double x, double vMin) {
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;
        double speed = Math.sin(PI * 2 * x - PI / 2) + 1;
        if (Math.abs(vMin) > Math.abs(speed)) {
            if (speed == 0) return Math.abs(vMin);
            else return Math.abs(vMin * speed / Math.abs(speed));
        } else return Math.abs(speed);
    }

    public double unitPolynomial(double x, double vMin) {
        //double speed = (-4 * vMax) * x * x + 4 * vMax * x;
        double speed = (-4) * x * x + 4 * x;
        if (Math.abs(vMin) > Math.abs(speed)) {
            if (speed == 0) return Math.abs(vMin);
            else return Math.abs(vMin * speed / Math.abs(speed));
        } else return Math.abs(speed);
    }

    public void goTo(Pose2d target, double speed, double error) {
        deltaX = target.x * TICKS_PER_CM - X;
        deltaY = target.y * TICKS_PER_CM - Y;
        deltaHypot = Math.hypot(deltaX, deltaY);

        if (firstTime) {
            initX = getX();
            initY = getY();

            targetX = target.x;
            targetY = target.y;

            initHypot = deltaHypot;

            doneHeading = false;
            doneTranslation = false;
            passedThrough = false;
            firstTime = false;
        }

        deltaH = AngleWrap(Math.toRadians(target.heading) - H);
        pathH = atan2(deltaX, deltaY) - H;
        distanceH = Math.toDegrees(Math.abs(deltaH)) * RobotDegreeTicks;

        if (deltaHypot < distanceH) {
            //telemetry.addData("incepe si scade", deltaHypot / distanceH);
            headingMixer = deltaHypot / distanceH;
        } else {
            headingMixer = 1;
        }

        if (Math.abs(deltaHypot) / TICKS_PER_CM < error) {
            headingMixer = 0;
        }


        pathH = atan2(deltaX, deltaY) - H;

        //VITEZA ROTATIONALA(roteste robotul la cat ai tu nevoie)
        //double headingSpeedAdjustor = 0;
        if (useInterpolator) {
            interpolator = AngleWrap(pathH * headingMixer + deltaH * (1 - headingMixer)) / (PI / 1.7);
            //sign=(pathH * headingMixer + deltaH * (1 - headingMixer))/Math.abs(pathH * headingMixer + deltaH * (1 - headingMixer));
        } else {
            interpolator = deltaH / (PI / 1.7);
        }

        if (Math.abs(Math.toDegrees(deltaH)) > 0.75 * error) {
            doneHeading = false;
            headingPower = twoXSqrd(interpolator, 0.38) * (robotName == "Macara" ? -1 : 1);
        } else {
            headingPower = 0;
            //doneHeading = true;
        }

        /////////// VITEZA TRANSLATIONALA(te duce spre target)


        double SAFE_ZONE_RADIUS = 25;
        double interpolator = min(deltaHypot / (TICKS_PER_CM * SAFE_ZONE_RADIUS), 1);
        transInter = twoXSqrd(interpolator, 0.2);


        //double tVR = -pathH - 2 * getRotationalVelocity() + Math.PI / 4;
        tVR = -pathH + Math.PI / 4;
        /////////////DESCOMPUNE VITEZA SI O TRIMITE LA MOTOARE
        //error * (2 * Math.abs(this.interpolator) + 1)
        if (Math.abs(deltaHypot) / TICKS_PER_CM > error) {
            //TODO This IF statement kicks in if the robot hasn't arrived at destination

            if (robotName == "Clippy") {
                xSpeed = speed * transInter;
                ySpeed = speed * transInter;
            } else if (robotName == "Echo" || robotName == "Clippy" || robotName == "Kathy" || robotName == "Kirby") {
                xSpeed = -speed * transInter;
                ySpeed = -speed * transInter;
            } else {
                xSpeed = speed * transInter;
                ySpeed = speed * transInter;
            }

            if (Objects.equals(robotName, "Sasha") || robotName == "Kathy") {
                if (Math.abs(deltaX) / TICKS_PER_CM <= error * FSqrt(0.5)) xSpeed = 0;
                if (Math.abs(deltaY) / TICKS_PER_CM <= error * FSqrt(0.5)) ySpeed = 0;
            } else {
                if (Math.abs(deltaX) / TICKS_PER_CM <= error / sqrt(2)) xSpeed = 0;
                if (Math.abs(deltaY) / TICKS_PER_CM <= error / sqrt(2)) ySpeed = 0;
            }

            rotX = -xSpeed * Math.cos(tVR) + ySpeed * Math.sin(tVR);
            rotY = -xSpeed * Math.sin(tVR) - ySpeed * Math.cos(tVR);

            double ultimateMinX = 0.24;
            double ultimateMinY = 0.24;

            if (abs(rotX) < ultimateMinX && Math.abs(deltaHypot) / TICKS_PER_CM > error)
                rotX = ultimateMinX * rotX / abs(rotX);
            if (abs(rotY) < ultimateMinY && Math.abs(deltaHypot) / TICKS_PER_CM > error)
                rotY = ultimateMinY * rotY / abs(rotY);


            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
            goTo.put("FLPow", (rotY + rotX + headingPower) / denominator);
            goTo.put("FRPow", (rotY - rotX - headingPower) / denominator);
            goTo.put("BLPow", (rotY - rotX + headingPower) / denominator);
            goTo.put("BRPow", (rotY + rotX - headingPower) / denominator);

            doneTranslation = false;


        } else {
            double denominator = Math.max(Math.abs(headingPower), 1);
            goTo.put("FLPow", (headingPower) / denominator);
            goTo.put("FRPow", (headingPower) / denominator);
            goTo.put("BLPow", (headingPower) / denominator);
            goTo.put("BRPow", (headingPower) / denominator);

            doneTranslation = true;
        }

        //////////// VITEZA CORECTIVA

        angleToPath = AngleWrap(atan2(getY() - targetY, getX() - targetX) - atan2(initY - targetY, initX - targetX));
        distanceToPath = Math.abs(deltaHypot) / TICKS_PER_CM * sin(angleToPath);
        double correctivePower = 0;
        double denominator = 1;
        corrX = 0;
        corrY = 0;/*
        if (Math.abs(distanceToPath) >= error && deltaHypot / TICKS_PER_CM >= error * 4) {
            // 60 is there arbitrarily, reasonable value
            double correctiveInter = distanceToPath / 30;

            correctivePower = -twoXSqrd(correctiveInter, 0.25) * speed;
            double corrSign = correctivePower / abs(correctivePower);
            if (abs(correctivePower) > 0.5)
                correctivePower = 0.5 * corrSign;
            if (abs(correctivePower) < 0.25)
                correctivePower = 0.25 * corrSign;

            double xSpeed = X_MULTIPLIER * speed * correctivePower;
            cVR = atan2(targetY - initY, targetX - initX) + H - PI / 2;
            corrX = xSpeed * Math.cos(cVR);
            corrY = xSpeed * Math.sin(cVR);

            denominator = Math.max(Math.abs(corrX + corrY), 1);
        }
        */
        corrective.put("FLPow", (corrY + corrX) / denominator);
        corrective.put("FRPow", (corrY - corrX) / denominator);
        corrective.put("BLPow", (corrY - corrX) / denominator);
        corrective.put("BRPow", (corrY + corrX) / denominator);

        if (Math.abs(deltaHypot) / TICKS_PER_CM < error * 4) {
            passedThrough = true;
        }

        double FLPower = goTo.get("FLPow") * goToK + mainDrivePowers.get("FLPow") + corrective.get("FLPow") * goToK;
        double FRPower = goTo.get("FRPow") * goToK + mainDrivePowers.get("FRPow") + corrective.get("FRPow") * goToK;
        double BLPower = goTo.get("BLPow") * goToK + mainDrivePowers.get("BLPow") + corrective.get("BLPow") * goToK;
        double BRPower = goTo.get("BRPow") * goToK + mainDrivePowers.get("BRPow") + corrective.get("BRPow") * goToK;

        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }


        if (!TESTINGROARY) {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }


        /*telemetry.addData("FL", FLPower);
        telemetry.addData("goToPow", goTo.get("FLPow"));
        telemetry.addData("mainPow", mainDrivePowers.get("FLPow"));

         */

        if (Math.abs(Math.toDegrees(AngleWrap(deltaH))) < 1 * error) {
            //telemetry.addLine("am ajuns la rotatie");
            doneHeading = true;
        }

        if (doneTranslation && doneHeading) {
            doneGoTo = true;
        } else doneGoTo = false;


    }

    public void goTo(goToValues values) {


        deltaX = values.target.x * TICKS_PER_CM - X;
        deltaY = values.target.y * TICKS_PER_CM - Y;
        deltaHypot = Math.hypot(deltaX, deltaY);
        if (firstTime) {

            if (values.init == null) {
                initX = getX();
                initY = getY();
            } else {
                initX = values.init.x;
                initY = values.init.y;
            }
            targetX = values.target.x;
            targetY = values.target.y;

            initHypot = deltaHypot;

            doneHeading = false;
            doneTranslation = false;
            passedThrough = false;
            firstTime = false;
        }

        deltaH = AngleWrap(Math.toRadians(values.target.heading) - H);
        pathH = atan2(deltaX, deltaY) - H;
        distanceH = Math.toDegrees(Math.abs(deltaH)) * RobotDegreeTicks;

        if (deltaHypot < distanceH) {
            //telemetry.addData("incepe si scade", deltaHypot / distanceH);
            headingMixer = deltaHypot / distanceH;
        } else {
            headingMixer = 1;
        }

        if (Math.abs(deltaHypot) / TICKS_PER_CM < 2) {
            headingMixer = 0;
        }


        pathH = atan2(deltaX, deltaY) - H;

        //VITEZA ROTATIONALA(roteste robotul la cat ai tu nevoie)
        //double headingSpeedAdjustor = 0;
        if (useInterpolator) {
            interpolator = AngleWrap(pathH * headingMixer + deltaH * (1 - headingMixer)) / (PI / 1.7);
            //sign=(pathH * headingMixer + deltaH * (1 - headingMixer))/Math.abs(pathH * headingMixer + deltaH * (1 - headingMixer));
        } else {
            interpolator = deltaH / (PI / 1.7);
        }
        if (Math.abs(Math.toDegrees(deltaH)) > 0.75 * values.error) {
            doneHeading = false;
            headingPower = twoXSqrd(interpolator, 0.3) * (robotName == "Macara" ? -1 : 1);
        } else {
            headingPower = 0;
            //doneHeading = true;
        }

        /////////// VITEZA TRANSLATIONALA(te duce spre target)


        double SAFE_ZONE_RADIUS = 25;
        double interpolator = deltaHypot / (TICKS_PER_CM * SAFE_ZONE_RADIUS);
        transInter = twoXSqrd(interpolator, 0.2);
        /*if (deltaHypot < TICKS_PER_CM * SAFE_ZONE_RADIUS) {
            double xSqrd = (deltaHypot / (TICKS_PER_CM * SAFE_ZONE_RADIUS)) * (deltaHypot / (TICKS_PER_CM * SAFE_ZONE_RADIUS));
            if (xSqrd < 0.3)
                transInter = 0.3;
            else
                transInter = xSqrd;
        } else transInter = 1;


         */
        //double tVR = -pathH - 2 * getRotationalVelocity() + Math.PI / 4;
        double tVR = -pathH + Math.PI / 4;
        /////////////DESCOMPUNE VITEZA SI O TRIMITE LA MOTOARE
        if (Math.abs(deltaHypot) / TICKS_PER_CM > values.error * (2 * Math.abs(this.interpolator) + 1)) {
            //TODO This IF statement kicks in if the robot hasn't arrived at destination

            double xSpeed = speed * transInter;
            double ySpeed = speed * transInter;
            rotX = -xSpeed * Math.cos(tVR) + ySpeed * Math.sin(tVR);
            rotY = -xSpeed * Math.sin(tVR) - ySpeed * Math.cos(tVR);

            double ultimateMinX = 0.3;
            double ultimateMinY = 0.24;

            if (abs(rotX) < ultimateMinX && Math.abs(deltaX) / TICKS_PER_CM > values.error)
                rotX = ultimateMinX * rotX / abs(rotX);
            if (abs(rotY) < ultimateMinY && Math.abs(deltaY) / TICKS_PER_CM > values.error)
                rotY = ultimateMinY * rotY / abs(rotY);


            //telemetry.addData("rotX",rotX);
            //telemetry.addData("rotY",rotY);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
            goTo.put("FLPow", (rotY + rotX + headingPower) / denominator);
            goTo.put("FRPow", (rotY - rotX - headingPower) / denominator);
            goTo.put("BLPow", (rotY - rotX + headingPower) / denominator);
            goTo.put("BRPow", (rotY + rotX - headingPower) / denominator);

            doneTranslation = false;


        } else {
            double denominator = Math.max(Math.abs(headingPower), 1);
            goTo.put("FLPow", (headingPower) / denominator);
            goTo.put("FRPow", (headingPower) / denominator);
            goTo.put("BLPow", (headingPower) / denominator);
            goTo.put("BRPow", (headingPower) / denominator);

            doneTranslation = true;
        }

        //////////// VITEZA CORECTIVA

        angleToPath = AngleWrap(atan2(getY() - targetY, getX() - targetX) - atan2(initY - targetY, initX - targetX));
        distanceToPath = Math.abs(deltaHypot) / TICKS_PER_CM * sin(angleToPath);
        double correctivePower = 0;
        double denominator = 1;
        corrX = 0;
        corrY = 0;
        /*if (Math.abs(distanceToPath) >= values.error && deltaHypot / TICKS_PER_CM >= values.error * 3) {
            // 60 is there arbitrarily, reasonable value
            double correctiveInter = distanceToPath / 30;

            correctivePower = -twoXSqrd(correctiveInter, 0.25) * values.speed;
            double corrSign = correctivePower / abs(correctivePower);
            if (abs(correctivePower) > 0.5)
                correctivePower = 0.5 * corrSign;
            if (abs(correctivePower) < 0.25)
                correctivePower = 0.25 * corrSign;

            double xSpeed = X_MULTIPLIER * values.speed * correctivePower;
            cVR = atan2(targetY - initY, targetX - initX) + H - PI / 2;
            corrX = xSpeed * Math.cos(cVR);
            corrY = xSpeed * Math.sin(cVR);

            denominator = Math.max(Math.abs(corrX + corrY), 1);
        }

         */
        corrective.put("FLPow", (corrY + corrX) / denominator);
        corrective.put("FRPow", (corrY - corrX) / denominator);
        corrective.put("BLPow", (corrY - corrX) / denominator);
        corrective.put("BRPow", (corrY + corrX) / denominator);

        if (Math.abs(deltaHypot) / TICKS_PER_CM < values.error * 2) {
            passedThrough = true;
        }

        double FLPower = goTo.get("FLPow") * goToK + mainDrivePowers.get("FLPow") + corrective.get("FLPow") * goToK;
        double FRPower = goTo.get("FRPow") * goToK + mainDrivePowers.get("FRPow") + corrective.get("FRPow") * goToK;
        double BLPower = goTo.get("BLPow") * goToK + mainDrivePowers.get("BLPow") + corrective.get("BLPow") * goToK;
        double BRPower = goTo.get("BRPow") * goToK + mainDrivePowers.get("BRPow") + corrective.get("BRPow") * goToK;

        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }


        if (!TESTINGROARY) {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }


//        telemetry.addData("FL", FLPower);
//        telemetry.addData("goToPow", goTo.get("FLPow"));
//        telemetry.addData("mainPow", mainDrivePowers.get("FLPow"));

        if (Math.abs(Math.toDegrees(AngleWrap(deltaH))) < 0.75 * values.error) {
            //telemetry.addLine("am ajuns la rotatie");
            doneHeading = true;
        }

        if (doneTranslation && doneHeading) {
            doneGoTo = true;
        } else doneGoTo = false;

            /*rightFront.setPower(speed*deltaH/Math.abs(deltaH));
            leftFront.setPower(speed*-deltaH/Math.abs(deltaH));
            rightRear.setPower(speed*deltaH/Math.abs(deltaH));
            leftRear.setPower(speed*-deltaH/Math.abs(deltaH));
*/


        /*rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);

         */


    }

    public double getYVelocity() {
        return (FL.getVelocity(AngleUnit.RADIANS) + FR.getVelocity(AngleUnit.RADIANS) + BL.getVelocity(AngleUnit.RADIANS) + BR.getVelocity(AngleUnit.RADIANS)) * 4.52 / 4;
    }

    public double getXVelocity() {
        return (FL.getVelocity(AngleUnit.RADIANS) - FR.getVelocity(AngleUnit.RADIANS) - BL.getVelocity(AngleUnit.RADIANS) + BR.getVelocity(AngleUnit.RADIANS)) * 4.52 / 4;
    }

    public double getMaxVelocity() {

        ///returneaza valori mereu pozitive
        return max(max(Math.abs(FL.getVelocity(AngleUnit.RADIANS)), Math.abs(FR.getVelocity(AngleUnit.RADIANS))), max(Math.abs(BL.getVelocity(AngleUnit.RADIANS)), Math.abs(BR.getVelocity(AngleUnit.RADIANS)))) * 4.52;
    }

    public double getMediumPower() {

        ///returneaza valori mereu pozitive
        return (abs(FL.getPower()) + abs(FR.getPower()) + abs(BL.getPower()) + abs(BR.getPower())) / 4;
    }

    public double getRotationalVelocity() {

        // inmultirea cu 9.4 e necesara deoarece get velocity lucreaza in grade
        // deci si diagonala robotului trebuie convertita in grade pt a respecta unitatea de masura

        return (FL.getVelocity(AngleUnit.DEGREES) - FR.getVelocity(AngleUnit.DEGREES) + BL.getVelocity(AngleUnit.DEGREES) - BR.getVelocity(AngleUnit.DEGREES)) / (4 * robot_diagonal * 9.4);
    }

    public double getBreakLenght() {

        ////////// DECCELERATIA ANGULARA DIFERA DE LA ROBOT LA ROBOT, SE REFERA LA CATE GRADE/ SEC DECCELEREAZA ROBOTUL INTR-UN CM, PT ELTORRO ESTE 10.5
        /// CA SA O AFLI FOLOSESTE ANGULAR DECCELERATION TESTER

        return getMaxVelocity() / angular_decceleration;

    }

    public double getDeltaHypot() {
        return deltaHypot / TICKS_PER_CM;
    }

    public double getDeltaHeading() {
        return Math.toDegrees(AngleWrap(deltaH));
    }

    public void stopMotors() {
        if (goToK == 1) {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }

    public void startMotors(double speed) {
        FL.setPower(speed);
        FR.setPower(speed);
        BL.setPower(speed);
        BR.setPower(speed);
    }

    public void runWithoutEncoder() {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetAll() {
        initX = X;
        initY = Y;
        initHypot = deltaHypot;

        doneHeading = false;
        doneTranslation = false;
        passedThrough = false;
        firstTime = false;

        t = 0;
        lt = 0;
        splineDone = false;
        lineDone = false;
        splineFirstTime = true;
        lineFirstTime = true;

        //RM.splineTo(points, 90, 0.3, 0.1);
        //doneGoTo=false;
        //setPosition(0, 0, 0);
    }

    public goToValues lineTo(goToValues goTo, double leeway) {
        if (lineFirstTime) {
            lt = 0;
            firstTime = true;
            doneGoTo = false;
            lineDone = false;
            lineFirstTime = false;
            useInterpolator = false;
            startPoint = new Point(getX(), getY());
            endPoint = new Point(goTo.target.x, goTo.target.y);

        }

        Point splinePoints[] = {startPoint, startPoint, endPoint, endPoint};
        lookAhead = catmullRomSpline(splinePoints, lt + 0.2);
        lookAheadH = atan2(lookAhead.x - catmullRomSpline(splinePoints, lt).x, lookAhead.y - catmullRomSpline(splinePoints, lt).y);
        cornerAngle = Math.abs(Math.toDegrees(AngleWrap(PI - (atan2(lookAhead.y - catmullRomSpline(splinePoints, lt).y, lookAhead.x - catmullRomSpline(splinePoints, lt).x) - atan2(getY() - catmullRomSpline(splinePoints, lt).y, getX() - catmullRomSpline(splinePoints, lt).x)))));

        cornerSeverity = -Math.abs(leeway * cornerAngle / 45) + Math.abs(leeway * (1 - cornerAngle / 45));
        cornerSeverity = 0.01;
        ///22/6 INCERC SA FAC AUTO HYBRID CU ROARY SUNT F DISPERAT
        /*telemetry.addData("cornerAngle", cornerAngle);
        telemetry.addData("brake:", Math.abs(leeway * cornerAngle / 60));
        telemetry.addData("accel:", Math.abs(leeway * (1 - cornerAngle / 45)));
        telemetry.addData("cornerSeverity", cornerSeverity);
        telemetry.addData("speed", goTo.speed * sqrdSine(lt, goTo.speed - leeway) + cornerSeverity);


         */

        //lookAheadH = atan2(catmullRomSpline(splinePoints, localT + 0.2).x - getX(), catmullRomSpline(splinePoints, localT + 0.1).y - getY());
        sT = lt / (splinePoints.length - 3.1);
        double deltaST = 1 - sT;
        double tresh = 0.4;
        if (deltaST < tresh) headingMS = 0.1 * deltaST / tresh;
        else headingMS = 1;

        if (headingMS < 0) headingMS = 0;

        //pathH = atan2(deltaX, deltaY) - H;

        double interpolator = (lookAheadH) * headingMS + Math.toRadians(goTo.target.heading) * (1 - headingMS);

        if (rallyMode)
            target = new Pose2d(catmullRomSpline(splinePoints, lt), Math.toDegrees(interpolator));
        else target = new Pose2d(catmullRomSpline(splinePoints, lt), goTo.target.heading);

        if (cornerSeverity < -leeway) cornerSeverity = -leeway;
        else if (cornerSeverity > leeway) cornerSeverity = leeway;

        //goTo(target, speed + cornerSeverity, 2 + points.length / (t + 0.1));

        //telemetry.addData("doneTranslation", doneTranslation);
        //telemetry.addData("target X: ", target.x);
        //telemetry.addData("target Y: ", target.y);
        //telemetry.addData("target H: ", target.heading);

        //telemetry.addData("lt", lt);
        //telemetry.addData("passed", passedThrough);
//        telemetry.addData("sT", sT);
//        telemetry.addData("hMS", headingMS);
//        telemetry.addData("heading", goTo.target.heading);
//        telemetry.addData("pathH", Math.toDegrees(lookAheadH));
//        telemetry.addData("inter", Math.toDegrees(interpolator));


        //telemetry.update();

        if (passedThrough && lt <= splinePoints.length - 3) {
            firstTime = true;
            doneGoTo = false;
            lt += 0.05;
        } else if (doneGoTo) {
            firstTime = true;
            doneGoTo = false;
            lt += 0.05;
        }


        speed = (goTo.speed + cornerSeverity) * sqrdSine(lt, leeway);


        if (speed < leeway) speed = leeway;
        if (speed > 0.9) speed = 0.9;

        if (lt >= splinePoints.length - 3) {
            //zeroT = true;
            lineDone = true;

            return new goToValues(target, speed, goTo.error, startPoint);
        }
        /*if (lt >= splinePoints.length - 3.1 && doneHeading) {
            //zeroT = true;
            lineDone = true;

            return new goToValues(target, speed, goTo.error);
        } else if (lt >= splinePoints.length - 3) {
            //zeroT = true;
            //goTo(target, speed / 2, 1);
            lt -= 0.05;
            return new goToValues(target, speed, goTo.error);
        }
        */

        return new goToValues(target, speed, 3 + splinePoints.length / (lt + 0.1), startPoint);


    }

    public goToValues splineTo(goToValues goToValues[], double leeway) {
        if (splineFirstTime) {
            t = 0;
            firstTime = true;
            doneGoTo = false;
            splineDone = false;
            splineFirstTime = false;
            useInterpolator = false;

            if (goToValues.length == 1) {

                length = 4;
                startPoint = new Point(getX(), getY());
                endPoint = new Point(goToValues[0].target.x, goToValues[0].target.y);

                target = goToValues[0].target;
                speed = goToValues[0].speed;

            } else length = goToValues.length;
        }

        double localT = t - Math.floor(t);
        int i = (int) Math.floor(t);
        Point splinePoints[] = null;
        if (goToValues.length > 1) {
            ///TODO DACA E UN SPLINE( PT UN LINE TRB SA DAI UN SINGUR PCT, TARGETUL)
            splinePoints = new Point[]{new Point(goToValues[i].target), new Point(goToValues[i + 1].target), new Point(goToValues[i + 2].target), new Point(goToValues[i + 3].target)};
            Point headingPoints[] = {new Point(goToValues[i].target.heading, 0), new Point(goToValues[i + 1].target.heading, 0), new Point(goToValues[i + 2].target.heading, 0), new Point(goToValues[i + 3].target.heading, 0)};
            Point speedPoints[] = {new Point(goToValues[i].speed, 0), new Point(goToValues[i + 1].speed, 0), new Point(goToValues[i + 2].speed, 0), new Point(goToValues[i + 3].speed, 0)};

            //lookAheadH = atan2(catmullRomSpline(splinePoints, localT + 0.2).x - getX(), catmullRomSpline(splinePoints, localT + 0.1).y - getY());


            //pathH = atan2(deltaX, deltaY) - H;

            //double interpolator = (lookAheadH) * headingMS + Math.toRadians(heading) * (1 - headingMS);
            Point interH = catmullRomSpline(headingPoints, localT);
            Point interS = catmullRomSpline(speedPoints, localT);
            speed = interS.x;

            target = new Pose2d(catmullRomSpline(splinePoints, localT), interH.x);
        } else {
            splinePoints = new Point[]{startPoint, startPoint, endPoint, endPoint};
            target = new Pose2d(catmullRomSpline(splinePoints, localT), goToValues[0].target.heading);
            speed = goToValues[0].speed;
        }
        lookAhead = catmullRomSpline(splinePoints, localT + 0.2);
        cornerAngle = Math.abs(Math.toDegrees(AngleWrap(PI - (atan2(lookAhead.y - catmullRomSpline(splinePoints, localT).y, lookAhead.x - catmullRomSpline(splinePoints, localT).x) - atan2(getY() - catmullRomSpline(splinePoints, localT).y, getX() - catmullRomSpline(splinePoints, localT).x)))));

        cornerSeverity = -Math.abs(leeway * cornerAngle / 45) + Math.abs(leeway * (1 - cornerAngle / 45));

        //telemetry.addData("cornerAngle", cornerAngle);
        //telemetry.addData("brake:", Math.abs(leeway * cornerAngle / 60));
        //telemetry.addData("accel:", Math.abs(leeway * (1 - cornerAngle / 45)));
        //telemetry.addData("cornerSeverity", cornerSeverity);
        //telemetry.addData("speedInSpline",speed);
        if (cornerSeverity < -leeway) cornerSeverity = -leeway;
        else if (cornerSeverity > leeway) cornerSeverity = leeway;

        //goTo(target, speed + cornerSeverity, 2 + points.length / (t + 0.1));
/*
        telemetry.addData("doneTranslation", doneTranslation);
        telemetry.addData("target X: ", target.x);
        telemetry.addData("target Y: ", target.y);
        telemetry.addData("target H: ", target.heading);

        telemetry.addData("t", t);
        telemetry.addData("sT", sT);
        telemetry.addData("hMS", headingMS);
        //telemetry.addData("heading", heading);
        telemetry.addData("pathH", Math.toDegrees(lookAheadH));


        telemetry.update();

 */

        if (passedThrough && lt <= splinePoints.length - 3.2) {
            firstTime = true;
            doneGoTo = false;
            t += 0.075;
        } else if (doneGoTo) {
            firstTime = true;
            doneGoTo = false;
            t += 0.075;
        }
        if (t >= length - 3.2) {
            //zeroT = true;
            closeTo = true;
        } else closeTo = false;
        // la conditia asta mai era un doneHeading
        if (t >= length - 3) {
            //zeroT = true;
            splineDone = true;
            return new goToValues(target, speed / 1.5, 3);
        } /*else if (t >= length - 3) {
            //zeroT = true;
            //goTo(target, speed / 2, 1);

            return new goToValues(target, speed / 1.5, 3);
        }
        */

        return new goToValues(target, speed + cornerSeverity, 3 + length / (t + 0.1));


    }

    public double closestT(Point splinePoints[]) {
        double closest = 100;
        double closestT = 0;
        for (double t = 0; t <= 1.05; t += 0.05) {
            if (getPoint().distanceTo(catmullRomSpline(splinePoints, t)) <= closest) {
                closest = getPoint().distanceTo(catmullRomSpline(splinePoints, t));
                closestT = t;
            }
            if (closest <= 3) break;
        }

        return closestT;
    }

    public boolean hasPassed(Point point) {
        double closest = 100;
        double closestT = 0;

        double closestToPoint = 100;
        double closestTOfPoint = 0;
        for (double t = 0; t <= 1.05; t += 0.05) {
            if (getPoint().distanceTo(catmullRomSpline(splinePoints, t)) <= closest) {
                closest = getPoint().distanceTo(catmullRomSpline(splinePoints, t));
                closestT = t;
            }
            if (point.distanceTo(catmullRomSpline(splinePoints, t)) <= closestToPoint) {
                closestToPoint = getPoint().distanceTo(catmullRomSpline(splinePoints, t));
                closestTOfPoint = t;
            }
        }

        return closestTOfPoint < closestT;
    }

    public boolean hasPassed(Point point, double pT) {
        //Functia asta iti deruleaza cu pT fata de punctul dat de tine
        double closest = 100;
        double closestT = 0;

        double closestToPoint = 100;
        double closestTOfPoint = 0;
        for (double t = 0; t <= 1.05; t += 0.05) {
            if (getPoint().distanceTo(catmullRomSpline(splinePoints, t)) <= closest) {
                closest = getPoint().distanceTo(catmullRomSpline(splinePoints, t));
                closestT = t;
            }
            if (point.distanceTo(catmullRomSpline(splinePoints, t)) <= closestToPoint) {
                closestToPoint = getPoint().distanceTo(catmullRomSpline(splinePoints, t));
                closestTOfPoint = t;
            }
        }

        return closestTOfPoint + pT < closestT;
    }

    public void splineToDX(goToValues goToValues[]) {
        if (splineFirstTime) {
            t = 0;
            i = 0;
            localT = 0;
            firstTime = false;
            doneGoTo = true;
            splineDone = false;
            splineFirstTime = false;
            useInterpolator = false;
            doneTranslationSpline = false;


            if (goToValues.length == 1) {

                length = 4;
                startPoint = new Point(getX(), getY());
                endPoint = new Point(goToValues[0].target.x, goToValues[0].target.y);

                target = goToValues[0].target;
                speed = goToValues[0].speed;

            } else length = goToValues.length;
        }

        t = i + localT;
        if (localT >= 1 && t <= length - 3.3) i++;

        if (goToValues.length > 1) {
            ///TODO DACA E UN SPLINE( PT UN LINE TRB SA DAI UN SINGUR PCT, TARGETUL)
            splinePoints = new Point[]{new Point(goToValues[i].target), new Point(goToValues[i + 1].target), new Point(goToValues[i + 2].target), new Point(goToValues[i + 3].target)};
            Point[] headingPoints = {new Point(goToValues[i].target.heading, 0), new Point(goToValues[i + 1].target.heading, 0), new Point(goToValues[i + 2].target.heading, 0), new Point(goToValues[i + 3].target.heading, 0)};
            Point[] speedPoints = {new Point(goToValues[i].speed, 0), new Point(goToValues[i + 1].speed, 0), new Point(goToValues[i + 2].speed, 0), new Point(goToValues[i + 3].speed, 0)};
            localT = closestT(splinePoints);

            //lookAheadH = atan2(catmullRomSpline(splinePoints, localT + 0.2).x - getX(), catmullRomSpline(splinePoints, localT + 0.1).y - getY());


            //pathH = atan2(deltaX, deltaY) - H;

            //double interpolator = (lookAheadH) * headingMS + Math.toRadians(heading) * (1 - headingMS);
            Point interH = catmullRomSpline(headingPoints, localT);
            Point interS = catmullRomSpline(speedPoints, localT);
            speed = interS.x;

            target = new Pose2d(catmullRomSpline(splinePoints, localT), interH.x);
        } else {
            splinePoints = new Point[]{startPoint, startPoint, endPoint, endPoint};
            localT = closestT(splinePoints);
            target = new Pose2d(catmullRomSpline(splinePoints, localT), goToValues[0].target.heading);
            speed = goToValues[0].speed;
        }

        distanceToEnd = goToValues[goToValues.length - 1].target.distanceToPose(target);

        ///31/1/2024 HERE WAS AN && doneHeading
        if (doneTranslationSpline) {
            splineDone = true;
            return;
            //telemetry.addData(".","FUNCTIONEZ");
        } else splineDone = false;

        lookBehind = catmullRomSpline(splinePoints, localT - 0.2 * (length - 3));

        lookAhead = catmullRomSpline(splinePoints, localT + 0.2 * (length - 3));

        derivative = catmullRomDerivative(splinePoints, localT);
        normalLeft = catmullRomNormalLeft(splinePoints, localT);
        normalRight = catmullRomNormalRight(splinePoints, localT);

        derivativeGlobal = catmullRomSpline(splinePoints, localT).add(derivative);
        normalGlobalLeft = catmullRomSpline(splinePoints, localT).add(normalLeft);
        normalGlobalRight = catmullRomSpline(splinePoints, localT).add(normalRight);

        //derivativePower = Point.polar(-0.5, derivative.atan());

        //this is an interpolator, it's error divided by an arbitrary value
        // here the value is 60, because at 60 cm of target i want the robot to send full power towards correction
        //normalInterpolator = getPoint().distanceTo(target) / 60;


        if (getPoint().distanceTo(target) <= 5) normalInterpolator = 0;
        else normalInterpolator = getPoint().distanceTo(target) / 48;

        ///PUTEREA NORMALA
        if (getPoint().distanceTo(normalGlobalLeft) > getPoint().distanceTo(normalGlobalRight)) {
            normalPower = Point.polar(-min(twoXSqrd(normalInterpolator, 0.17, 0.05), speed), normalLeft.atan());
            isLeftNormal = true;
        } else {
            normalPower = Point.polar(-min(twoXSqrd(normalInterpolator, 0.17, 0.05), speed), normalRight.atan());
            isLeftNormal = false;
        }
        ///PUTEREA CENTRIPETA
        Point projectedP = projectionOnLine(normalGlobalLeft, normalGlobalRight, lookAhead);

        if (projectedP.distanceTo(target) <= 5) projectionInterpolator = 0;
        else projectionInterpolator = projectedP.distanceTo(target) / 33;

        if (projectedP.distanceTo(normalGlobalLeft) > projectedP.distanceTo(normalGlobalRight)) {
            projectionPower = Point.polar(-min(twoXSqrd(projectionInterpolator, 0.2, 0.05), speed - normalPower.radius()), normalRight.atan());
            isLeftProjection = false;
        } else {
            projectionPower = Point.polar(-min(twoXSqrd(projectionInterpolator, 0.2, 0.05), speed - normalPower.radius()), normalLeft.atan());
            isLeftProjection = true;
        }
//+min(twoXSqrd(abs(projectionInterpolator), 0.1),0.3)
        breakInterpolator = 1;
        breakPower = 1;
        distanceToEnd = goToValues[goToValues.length - 1].target.distanceToPose(target);
        directionToEnd = getPoint().distanceTo(lookAhead) - getPoint().distanceTo(lookBehind);
        if (t >= length - 4) {
            //zeroT = true;
            breakInterpolator = min(distanceToEnd / (robotName == "Sasha" ? 80 : 64), 1);
            breakPower = twoXSqrd(breakInterpolator, 0.64);
        }
        double powerMixer = 0;
        if (distanceToEnd <= 4.8) {
            doneTranslationSpline = true;
            doneGoTo = true;
        } else if (!doneTranslationSpline) {
            if (Objects.equals(robotName, "Sasha")) {
                powerMixer = max(FSqrt(pow(speed, 2) - pow(normalPower.radius() - 0.0000001, 2)) * breakPower, 0.24);
            } else {
                powerMixer = max(sqrt(pow(speed, 2) - pow(normalPower.radius() - 0.0000001, 2)) * breakPower, 0.24);
            }
        }

        double curveBreak = min(twoXSqrd(abs(projectionInterpolator), 0.08, 0.02), 0.24);
        derivativePower = Point.polar(-powerMixer + min(curveBreak, powerMixer), derivative.atan());

        if (!doneTranslationSpline)
            totalPower = normalPower.add(derivativePower).add(projectionPower);
        else totalPower = new Point();

        /*if (t >= length - 3.1) {
            //zeroT = true;
            splineDone = true;
        }

         */
        //return new goToValues(target, speed + cornerSeverity, 3 + length / (t + 0.1));
        if (!splineDone) sendPower(target, totalPower);


    }

    public void splineToDX(goToValues goToValues[], Point vel) {
        if (splineFirstTime) {
            t = 0;
            i = 0;
            localT = 0;
            firstTime = false;
            doneGoTo = true;
            splineDone = false;
            splineFirstTime = false;
            useInterpolator = false;
            doneTranslationSpline = false;


            if (goToValues.length == 1) {

                length = 4;
                startPoint = new Point(getX(), getY());
                endPoint = new Point(goToValues[0].target.x, goToValues[0].target.y);

                target = goToValues[0].target;
                speed = goToValues[0].speed;

            } else length = goToValues.length;
        }

        t = i + localT;
        if (localT >= 1 && t <= length - 3.3) i++;

        if (goToValues.length > 1) {
            ///TODO DACA E UN SPLINE( PT UN LINE TRB SA DAI UN SINGUR PCT, TARGETUL)
            splinePoints = new Point[]{new Point(goToValues[i].target), new Point(goToValues[i + 1].target), new Point(goToValues[i + 2].target), new Point(goToValues[i + 3].target)};
            Point[] headingPoints = {new Point(goToValues[i].target.heading, 0), new Point(goToValues[i + 1].target.heading, 0), new Point(goToValues[i + 2].target.heading, 0), new Point(goToValues[i + 3].target.heading, 0)};
            Point[] speedPoints = {new Point(goToValues[i].speed, 0), new Point(goToValues[i + 1].speed, 0), new Point(goToValues[i + 2].speed, 0), new Point(goToValues[i + 3].speed, 0)};
            localT = closestT(splinePoints);

            //lookAheadH = atan2(catmullRomSpline(splinePoints, localT + 0.2).x - getX(), catmullRomSpline(splinePoints, localT + 0.1).y - getY());


            //pathH = atan2(deltaX, deltaY) - H;

            //double interpolator = (lookAheadH) * headingMS + Math.toRadians(heading) * (1 - headingMS);
            Point interH = catmullRomSpline(headingPoints, localT);
            Point interS = catmullRomSpline(speedPoints, localT);
            speed = interS.x;

            target = new Pose2d(catmullRomSpline(splinePoints, localT), interH.x);
        } else {
            splinePoints = new Point[]{startPoint, startPoint, endPoint, endPoint};
            localT = closestT(splinePoints);
            target = new Pose2d(catmullRomSpline(splinePoints, localT), goToValues[0].target.heading);
            speed = goToValues[0].speed;
        }

        distanceToEnd = goToValues[goToValues.length - 1].target.distanceToPose(target);

        ///31/1/2024 HERE WAS AN && doneHeading
        if (doneTranslationSpline) {
            splineDone = true;
            return;
            //telemetry.addData(".","FUNCTIONEZ");
        } else splineDone = false;

        lookBehind = catmullRomSpline(splinePoints, localT - 0.2 * (length - 3));

        lookAhead = catmullRomSpline(splinePoints, localT + 0.2 * (length - 3));

        derivative = catmullRomDerivative(splinePoints, localT);
        normalLeft = catmullRomNormalLeft(splinePoints, localT);
        normalRight = catmullRomNormalRight(splinePoints, localT);

        derivativeGlobal = catmullRomSpline(splinePoints, localT).add(derivative);
        normalGlobalLeft = catmullRomSpline(splinePoints, localT).add(normalLeft);
        normalGlobalRight = catmullRomSpline(splinePoints, localT).add(normalRight);

        //derivativePower = Point.polar(-0.5, derivative.atan());

        //this is an interpolator, it's error divided by an arbitrary value
        // here the value is 60, because at 60 cm of target i want the robot to send full power towards correction
        //normalInterpolator = getPoint().distanceTo(target) / 60;


        if (getPoint().distanceTo(target) <= 4.25) normalInterpolator = 0;
        else normalInterpolator = getPoint().distanceTo(target) / 48;

        ///PUTEREA NORMALA
        if (getPoint().distanceTo(normalGlobalLeft) > getPoint().distanceTo(normalGlobalRight)) {
            normalPower = Point.polar(-min(twoXSqrd(normalInterpolator, 0.17, 0.05), speed), normalLeft.atan());
            isLeftNormal = true;
        } else {
            normalPower = Point.polar(-min(twoXSqrd(normalInterpolator, 0.17, 0.05), speed), normalRight.atan());
            isLeftNormal = false;
        }
        ///PUTEREA CENTRIPETA
        Point projectedP = projectionOnLine(normalGlobalLeft, normalGlobalRight, lookAhead);

        if (projectedP.distanceTo(target) <= 5) projectionInterpolator = 0;
        else projectionInterpolator = inRange(projectedP.distanceTo(target) / 33, -0.64, 0.64);

        if (projectedP.distanceTo(normalGlobalLeft) > projectedP.distanceTo(normalGlobalRight)) {
            projectionPower = Point.polar(-min(twoXSqrd(projectionInterpolator, 0.13, 0.05), speed - normalPower.radius()), normalRight.atan());
            isLeftProjection = false;
        } else {
            projectionPower = Point.polar(-min(twoXSqrd(projectionInterpolator, 0.13, 0.05), speed - normalPower.radius()), normalLeft.atan());
            isLeftProjection = true;
        }
//+min(twoXSqrd(abs(projectionInterpolator), 0.1),0.3)
        breakInterpolator = 1;
        breakPower = 1;
        distanceToEnd = goToValues[goToValues.length - 1].target.distanceToPose(target);
        directionToEnd = getPoint().distanceTo(lookAhead) - getPoint().distanceTo(lookBehind);

        //zeroT = true;
        breakInterpolator = min(distanceToEnd / 71, 1);
        breakPower = twoXSqrd(breakInterpolator, 0.8);

        powerMixer = 0;
        if (distanceToEnd <= 6.4 || (goToValues[goToValues.length - 1].target.distanceTo(futurePos) < 33 && vel.radius() < 33)) {
            doneTranslationSpline = true;
            //doneGoTo = true;
        } else if (!doneTranslationSpline) {
            breakPower = twoXSqrd(breakInterpolator, 0.8);

            futurePos = getPoint().add(vel.divide(8));

            if (goToValues[goToValues.length - 1].target.distanceTo(futurePos) < 8) {
                BRAKEPLS = true;
                breakPower = 0;
            }
            if (goToValues[goToValues.length - 1].target.distanceTo(futurePos) < 35) {
                //BRAKEPLS = false;
                breakPower = min(breakPower, 0.35);
            } else BRAKEPLS = false;

            powerMixer = max(sqrt(pow(speed, 2) - pow(normalPower.radius() - 0.0000001, 2)) * breakPower, BRAKEPLS ? 0 : 0.24);
        }
        double curveBreak = min(twoXSqrd(abs(projectionInterpolator), 0.08, 0.02), 0.17);
        derivativePower = Point.polar(-powerMixer + min(curveBreak, powerMixer), derivative.atan());

        if (!doneTranslationSpline)
            totalPower = normalPower.add(derivativePower).add(projectionPower);
        else totalPower = new Point();

        /*if (t >= length - 3.1) {
            //zeroT = true;
            splineDone = true;
        }

         */
        //return new goToValues(target, speed + cornerSeverity, 3 + length / (t + 0.1));
        if (!splineDone) sendPower(target, totalPower);


    }

    public void sendPower(Pose2d target, Point direction) {
        //derivative space
        lastDirection = direction;

        deltaH = AngleWrap(Math.toRadians(target.heading) - H);

        //VITEZA ROTATIONALA(roteste robotul la cat ai tu nevoie)

        interpolator = deltaH / (PI / 2.4);

        if (Math.abs(Math.toDegrees(deltaH)) > 2.4 && abs(target.heading) <= 180) {
            doneHeading = false;
            headingPower = twoXSqrd(interpolator, ((robotName == "Clippy" || robotName == "Kathy" || robotName == "Kirby" || robotName == "Sasha") ? 0.17 : 0.24) * (robotName == "Macara" ? -1 : 1));
        } else {
            headingPower = 0;
            doneHeading = true;
        }

        double tVR = -(-direction.atan() - PI / 2 - H) + Math.PI / 4;
        /////////////DESCOMPUNE VITEZA SI O TRIMITE LA MOTOARE

        if (robotName == "Echo" || robotName == "Clippy" || robotName == "Kathy" || robotName == "Kirby") {
            rotX = direction.radius() * Math.cos(tVR) - direction.radius() * Math.sin(tVR);
            rotY = direction.radius() * Math.sin(tVR) + direction.radius() * Math.cos(tVR);
        } else {
            rotX = -direction.radius() * Math.cos(tVR) + direction.radius() * Math.sin(tVR);
            rotY = -direction.radius() * Math.sin(tVR) - direction.radius() * Math.cos(tVR);
        }


        //telemetry.addData("rotX",rotX);
        //telemetry.addData("rotY",rotY);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
        goTo.put("FLPow", (rotY + rotX + headingPower) * powerCompensator / denominator);
        goTo.put("FRPow", (rotY - rotX - headingPower) * powerCompensator / denominator);
        goTo.put("BLPow", (rotY - rotX + headingPower) * powerCompensator / denominator);
        goTo.put("BRPow", (rotY + rotX - headingPower) * powerCompensator / denominator);

        if (mainDrivePowers.get("FLPow") == null) {
            mainDrivePowers.put("FLPow", (double) 0);
            mainDrivePowers.put("FRPow", (double) 0);
            mainDrivePowers.put("BLPow", (double) 0);
            mainDrivePowers.put("BRPow", (double) 0);
        }

        double FLPower = goTo.get("FLPow") * goToK + mainDrivePowers.get("FLPow");
        double FRPower = goTo.get("FRPow") * goToK + mainDrivePowers.get("FRPow");
        double BLPower = goTo.get("BLPow") * goToK + mainDrivePowers.get("BLPow");
        double BRPower = goTo.get("BRPow") * goToK + mainDrivePowers.get("BRPow");


        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }


        if (!TESTINGROARY) {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }


    }

    public void sendPower(Pose2d roaryTarget) {
        //roary space
        Point target = new Point(roaryTarget.x, roaryTarget.y).rotate(toRadians(180));
        lastDirection = target;

        deltaH = toDegrees(AngleWrap(Math.toRadians(roaryTarget.heading) - H));

        //VITEZA ROTATIONALA(roteste robotul la cat ai tu nevoie)

        interpolator = inRange(deltaH / 89, -0.89, 0.89);

        if (Math.abs(deltaH) > 1.3 && abs(roaryTarget.heading) <= 180) {
            doneHeading = false;
            headingPower = twoXSqrd(interpolator, 0.17) * (robotName == "Macara" ? -1 : 1);
        } else {
            headingPower = 0;
            doneHeading = true;
        }

        double tVR = -(-target.atan() - PI / 2 - H) + Math.PI / 4;
        /////////////DESCOMPUNE VITEZA SI O TRIMITE LA MOTOARE

        if (robotName == "Echo" || robotName == "Clippy" || robotName == "Kathy" || robotName == "Kirby") {
            rotX = target.radius() * Math.cos(tVR) - target.radius() * Math.sin(tVR);
            rotY = target.radius() * Math.sin(tVR) + target.radius() * Math.cos(tVR);
        } else {
            rotX = -target.radius() * Math.cos(tVR) + target.radius() * Math.sin(tVR);
            rotY = -target.radius() * Math.sin(tVR) - target.radius() * Math.cos(tVR);
        }


        //telemetry.addData("rotX",rotX);
        //telemetry.addData("rotY",rotY);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
        goTo.put("FLPow", (rotY + rotX + headingPower) * powerCompensator / denominator);
        goTo.put("FRPow", (rotY - rotX - headingPower) * powerCompensator / denominator);
        goTo.put("BLPow", (rotY - rotX + headingPower) * powerCompensator / denominator);
        goTo.put("BRPow", (rotY + rotX - headingPower) * powerCompensator / denominator);

        if (mainDrivePowers.get("FLPow") == null) {
            mainDrivePowers.put("FLPow", (double) 0);
            mainDrivePowers.put("FRPow", (double) 0);
            mainDrivePowers.put("BLPow", (double) 0);
            mainDrivePowers.put("BRPow", (double) 0);
        }


        double FLPower = goTo.get("FLPow") * goToK + mainDrivePowers.get("FLPow");
        double FRPower = goTo.get("FRPow") * goToK + mainDrivePowers.get("FRPow");
        double BLPower = goTo.get("BLPow") * goToK + mainDrivePowers.get("BLPow");
        double BRPower = goTo.get("BRPow") * goToK + mainDrivePowers.get("BRPow");


        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }


        if (!TESTINGROARY) {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }


    }

    public void fastSendPower() {
        double FLPower = 0;
        double FRPower = 0;
        double BLPower = 0;
        double BRPower = 0;

        if (mainDrivePowers.get("FLPow") != null) {
            FLPower = mainDrivePowers.get("FLPow");
            FRPower = mainDrivePowers.get("FRPow");
            BLPower = mainDrivePowers.get("BLPow");
            BRPower = mainDrivePowers.get("BRPow");
        }

        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
    }

    public void fastSendPowerAssumeAlreadyInterpolated() {
        double FLPower = 0;
        double FRPower = 0;
        double BLPower = 0;
        double BRPower = 0;

        if (mainDrivePowers.get("FLPow") != null) {
            FLPower = mainDrivePowers.get("FLPow");
            FRPower = mainDrivePowers.get("FRPow");
            BLPower = mainDrivePowers.get("BLPow");
            BRPower = mainDrivePowers.get("BRPow");
        }

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
    }

    public void updatePowerCompensator() {
        powerCompensator = max(1, 14.0 / batteryVoltage);
    }

    public void goToV2(Pose2d target, double speed, double error) {


        goToDelta = target.subtract(getPoint());

        deltaH = AngleWrap(Math.toRadians(target.heading) - H);
        pathH = goToDelta.atan() - H;

        //VITEZA ROTATIONALA(roteste robotul la cat ai tu nevoie)

        ///Interpolam pe 120 grade
        interpolator = deltaH / (PI * 2.0 / 3.0);

        if (Math.abs(Math.toDegrees(deltaH)) > 0.8 * error) {
            doneHeadingGoTo = false;
            double ultimateMinH = 0.134;
            headingPower = twoXSqrd(interpolator, 0.134) * (robotName == "Macara" ? -1 : 1);
            if (abs(headingPower) < ultimateMinH)
                headingPower = ultimateMinH * headingPower / abs(headingPower);
        } else {
            headingPower = 0;
            doneHeadingGoTo = true;
        }

        /////////// VITEZA TRANSLATIONALA(te duce spre target)

        double SAFE_ZONE_RADIUS = 24;
        double interpolator = min(goToDelta.radius() / SAFE_ZONE_RADIUS, 1);
        transInter = twoXSqrd(interpolator, 0.22);

        tVR = -pathH + Math.PI / 4;
        /////////////DESCOMPUNE VITEZA SI O TRIMITE LA MOTOARE

        if (robotName == "Echo" || robotName == "Clippy" || robotName == "Kathy" || robotName == "Kirby") {
            globalSpeed = -speed * transInter;
        } else {
            globalSpeed = speed * transInter;
        }

        double errX = -goToDelta.radius() * Math.cos(tVR) + goToDelta.radius() * Math.sin(tVR);
        double errY = -goToDelta.radius() * Math.sin(tVR) - goToDelta.radius() * Math.cos(tVR);

        rotX = -globalSpeed * Math.cos(tVR) + globalSpeed * Math.sin(tVR);
        rotY = -globalSpeed * Math.sin(tVR) - globalSpeed * Math.cos(tVR);

        double ultimateMinX = 0.22;
        double ultimateMinY = 0.17;

        if (abs(rotX) < ultimateMinX) rotX = ultimateMinX * rotX / abs(rotX);
        if (abs(rotY) < ultimateMinY) rotY = ultimateMinY * rotY / abs(rotY);

        if (goToDelta.radius() < error) {
            rotX = 0;
            rotY = 0;
            doneTranslation = true;
        } else doneTranslation = false;

        if (doneTranslation && doneHeadingGoTo && doneGoTo) {
            fastSendPower();
            return;
        }

        if (abs(errX) < error) rotX = 0;
        if (abs(errY) < error) rotY = 0;


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
        goTo.put("FLPow", (rotY + rotX + headingPower) * powerCompensator / denominator);
        goTo.put("FRPow", (rotY - rotX - headingPower) * powerCompensator / denominator);
        goTo.put("BLPow", (rotY - rotX + headingPower) * powerCompensator / denominator);
        goTo.put("BRPow", (rotY + rotX - headingPower) * powerCompensator / denominator);

        //doneTranslation = false;

        if (mainDrivePowers.get("FLPow") == null) {
            mainDrivePowers.put("FLPow", (double) 0);
            mainDrivePowers.put("FRPow", (double) 0);
            mainDrivePowers.put("BLPow", (double) 0);
            mainDrivePowers.put("BRPow", (double) 0);
        }

        double FLPower = goTo.get("FLPow") * goToK + mainDrivePowers.get("FLPow");
        double FRPower = goTo.get("FRPow") * goToK + mainDrivePowers.get("FRPow");
        double BLPower = goTo.get("BLPow") * goToK + mainDrivePowers.get("BLPow");
        double BRPower = goTo.get("BRPow") * goToK + mainDrivePowers.get("BRPow");


        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }

        if (!TESTINGROARY) {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }

        doneGoTo = doneTranslation && doneHeadingGoTo;


    }

    public void goToV2Clippy(Pose2d target, double speed, double error) {

        goToDelta = target.subtract(getPoint());

        deltaH = AngleWrap(Math.toRadians(target.heading) - H);
        pathH = goToDelta.atan() - H;

        //VITEZA ROTATIONALA(roteste robotul la cat ai tu nevoie)

        interpolator = deltaH / (PI / 2);

        if (Math.abs(Math.toDegrees(deltaH)) > 0.8 * error) {
            doneHeadingGoTo = false;
            double ultimateMinH = 0.16;
            headingPower = twoXSqrd(interpolator, 0.15) * (robotName == "Macara" ? -1 : 1);
            if (abs(headingPower) < ultimateMinH)
                headingPower = ultimateMinH * headingPower / abs(headingPower);
        } else {
            headingPower = 0;
            doneHeadingGoTo = true;
        }

        /////////// VITEZA TRANSLATIONALA(te duce spre target)

        double SAFE_ZONE_RADIUS = 33;
        double interpolator = min(goToDelta.radius() / SAFE_ZONE_RADIUS, 1);
        transInter = twoXSqrd(interpolator, 0.174);

        tVR = -pathH + Math.PI / 4;
        /////////////DESCOMPUNE VITEZA SI O TRIMITE LA MOTOARE

        if (robotName == "Echo" || robotName == "Clippy" || robotName == "Kathy" || robotName == "Kirby") {
            globalSpeed = -speed * transInter;
        } else {
            globalSpeed = speed * transInter;
        }

        double errX = -goToDelta.radius() * Math.cos(tVR) + goToDelta.radius() * Math.sin(tVR);
        double errY = -goToDelta.radius() * Math.sin(tVR) - goToDelta.radius() * Math.cos(tVR);

        rotX = -globalSpeed * Math.cos(tVR) + globalSpeed * Math.sin(tVR);
        rotY = -globalSpeed * Math.sin(tVR) - globalSpeed * Math.cos(tVR);

        double ultimateMinX = 0.24; //0.21
        double ultimateMinY = 0.2; //0.17

        if (abs(rotX) < ultimateMinX) rotX = ultimateMinX * rotX / abs(rotX);
        if (abs(rotY) < ultimateMinY) rotY = ultimateMinY * rotY / abs(rotY);

        if (goToDelta.radius() < error) {
            rotX = 0;
            rotY = 0;
            doneTranslation = true;
        } else doneTranslation = false;

        if (doneTranslation && doneHeadingGoTo && doneGoTo) {
            fastSendPower();
            return;
        }

        if (abs(errX) < error) rotX = 0;
        if (abs(errY) < error) rotY = 0;


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
        goTo.put("FLPow", (rotY + rotX + headingPower) / denominator);
        goTo.put("FRPow", (rotY - rotX - headingPower) / denominator);
        goTo.put("BLPow", (rotY - rotX + headingPower) / denominator);
        goTo.put("BRPow", (rotY + rotX - headingPower) / denominator);

        //doneTranslation = false;

        if (mainDrivePowers.get("FLPow") == null) {
            mainDrivePowers.put("FLPow", (double) 0);
            mainDrivePowers.put("FRPow", (double) 0);
            mainDrivePowers.put("BLPow", (double) 0);
            mainDrivePowers.put("BRPow", (double) 0);
        }

        double FLPower = goTo.get("FLPow") * goToK + mainDrivePowers.get("FLPow");
        double FRPower = goTo.get("FRPow") * goToK + mainDrivePowers.get("FRPow");
        double BLPower = goTo.get("BLPow") * goToK + mainDrivePowers.get("BLPow");
        double BRPower = goTo.get("BRPow") * goToK + mainDrivePowers.get("BRPow");


        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }

        if (!TESTINGROARY) {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }

        doneGoTo = doneTranslation && doneHeadingGoTo;


    }


    public void manualGoTo(Pose2d target, double speed, double error) {

        goToDelta = target;

        deltaH = AngleWrap(Math.toRadians(target.heading) - H);
        pathH = goToDelta.atan() - H;

        //VITEZA ROTATIONALA(roteste robotul la cat ai tu nevoie)

        interpolator = deltaH / (PI / 1.7);

        if (Math.abs(Math.toDegrees(deltaH)) > 0.75 * error) {
            doneHeadingGoTo = false;
            headingPower = twoXSqrd(interpolator, 0.15) * (robotName == "Macara" ? -1 : 1);
        } else {
            headingPower = 0;
            doneHeadingGoTo = true;
        }

        /////////// VITEZA TRANSLATIONALA(te duce spre target)

        double SAFE_ZONE_RADIUS = 44;
        double interpolator = min(goToDelta.radius() / SAFE_ZONE_RADIUS, 1);
        transInter = twoXSqrd(interpolator, 0.13);

        tVR = -pathH + Math.PI / 4;
        /////////////DESCOMPUNE VITEZA SI O TRIMITE LA MOTOARE

        if (robotName == "Echo" || robotName == "Clippy" || robotName == "Kathy" || robotName == "Kirby") {
            globalSpeed = -speed * transInter;
        } else {
            globalSpeed = speed * transInter;
        }
        double errX = -goToDelta.radius() * Math.cos(tVR) + goToDelta.radius() * Math.sin(tVR);
        double errY = -goToDelta.radius() * Math.sin(tVR) - goToDelta.radius() * Math.cos(tVR);

        rotX = -globalSpeed * Math.cos(tVR) + globalSpeed * Math.sin(tVR);
        rotY = -globalSpeed * Math.sin(tVR) - globalSpeed * Math.cos(tVR);

        double ultimateMinX = 0.21;
        double ultimateMinY = 0.13;

        if (abs(rotX) < ultimateMinX) rotX = ultimateMinX * rotX / abs(rotX);
        if (abs(rotY) < ultimateMinY) rotY = ultimateMinY * rotY / abs(rotY);

        if (abs(errX) < error) rotX = 0;
        if (abs(errY) < error) rotY = 0;

        if (goToDelta.radius() < error) {
            rotX = 0;
            rotY = 0;
            doneTranslation = true;
        } else doneTranslation = false;


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
        goTo.put("FLPow", (rotY + rotX + headingPower) / denominator);
        goTo.put("FRPow", (rotY - rotX - headingPower) / denominator);
        goTo.put("BLPow", (rotY - rotX + headingPower) / denominator);
        goTo.put("BRPow", (rotY + rotX - headingPower) / denominator);

        //doneTranslation = false;

        if (mainDrivePowers.get("FLPow") == null) {
            mainDrivePowers.put("FLPow", (double) 0);
            mainDrivePowers.put("FRPow", (double) 0);
            mainDrivePowers.put("BLPow", (double) 0);
            mainDrivePowers.put("BRPow", (double) 0);
        }

        double FLPower = goTo.get("FLPow") * goToK + mainDrivePowers.get("FLPow");
        double FRPower = goTo.get("FRPow") * goToK + mainDrivePowers.get("FRPow");
        double BLPower = goTo.get("BLPow") * goToK + mainDrivePowers.get("BLPow");
        double BRPower = goTo.get("BRPow") * goToK + mainDrivePowers.get("BRPow");


        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }

        if (!TESTINGROARY) {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }

        doneGoTo = doneTranslation && doneHeadingGoTo;


    }

    public Point catmullRomDerivative(Point points[], double t) {
        //int i = (int) Math.floor(t);
        Point a1 = points[0];
        Point p1 = points[1];
        Point p2 = points[2];
        Point a2 = points[3];

        double q0 = (-3 * Math.pow(t, 2)) + (4 * t) - 1;
        double q1 = (9 * Math.pow(t, 2)) + (-10 * t);
        double q2 = (-9 * Math.pow(t, 2)) + (8 * t) + 1;
        double q3 = 3 * Math.pow(t, 2) - (2 * t);

        Point point = new Point(0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3)), 0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3)));

        return point;

    }

    public Point catmullRomNormalLeft(Point points[], double t) {
        //int i = (int) Math.floor(t);
        Point a1 = points[0];
        Point p1 = points[1];
        Point p2 = points[2];
        Point a2 = points[3];

        double q0 = (-3 * Math.pow(t, 2)) + (4 * t) - 1;
        double q1 = (9 * Math.pow(t, 2)) + (-10 * t);
        double q2 = (-9 * Math.pow(t, 2)) + (8 * t) + 1;
        double q3 = 3 * Math.pow(t, 2) - (2 * t);

        Point point = new Point(-0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3)), 0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3)));

        return point;

    }

    public Point catmullRomNormalRight(Point points[], double t) {
        //int i = (int) Math.floor(t);
        Point a1 = points[0];
        Point p1 = points[1];
        Point p2 = points[2];
        Point a2 = points[3];

        double q0 = (-3 * Math.pow(t, 2)) + (4 * t) - 1;
        double q1 = (9 * Math.pow(t, 2)) + (-10 * t);
        double q2 = (-9 * Math.pow(t, 2)) + (8 * t) + 1;
        double q3 = 3 * Math.pow(t, 2) - (2 * t);

        Point point = new Point(0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3)), -0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3)));

        return point;

    }

    public Point catmullRomSpline(Point points[], double t) {
        //int i = (int) Math.floor(t);
        Point a1 = points[0];
        Point p1 = points[1];
        Point p2 = points[2];
        Point a2 = points[3];

        double q0 = (-1 * Math.pow(t, 3)) + (2 * Math.pow(t, 2)) + (-1 * t);
        double q1 = (3 * Math.pow(t, 3)) + (-5 * Math.pow(t, 2)) + 2;
        double q2 = (-3 * Math.pow(t, 3)) + (4 * Math.pow(t, 2)) + t;
        double q3 = Math.pow(t, 3) - Math.pow(t, 2);

        Point point = new Point(0.5 * ((a1.x * q0) + (p1.x * q1) + (p2.x * q2) + (a2.x * q3)), 0.5 * ((a1.y * q0) + (p1.y * q1) + (p2.y * q2) + (a2.y * q3)));

        return point;

    }

    public Pose2d getPose() {
        return new Pose2d(getX(), getY(), getH());
    }

    public void set(Pose2d newPose) {
        X = newPose.x * TICKS_PER_CM;
        Y = newPose.y * TICKS_PER_CM;
        H = Math.toRadians(newPose.heading);

    }

    public void mainDriveForTestsWITHOUTRoary(Gamepad gamepad1) {

        double speed = -Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        //finds the angle the robot is moving at

        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;


        double FLPower = (speed * Math.sin(robotAngle)) + gamepad1.right_stick_x;
        double FRPower = (speed * Math.cos(robotAngle)) - gamepad1.right_stick_x;
        double BLPower = (speed * Math.cos(robotAngle)) + gamepad1.right_stick_x;
        double BRPower = (speed * Math.sin(robotAngle)) - gamepad1.right_stick_x;



        double Normalizer = Math.max(Math.max(abs(FLPower), abs(FRPower)), Math.max(abs(BLPower), abs(BRPower)));

        if (FLPower > 1 && FRPower > 1 && BLPower > 1 && BRPower > 1) {
            FLPower = FLPower / Normalizer;
            FRPower = FRPower / Normalizer;
            BLPower = BLPower / Normalizer;
            BRPower = BRPower / Normalizer;
        }


        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
    }

    public goToValues elementToRobotPos(goToValues values, Point element) {
        goToValues goToVals = new goToValues(values);
        Point elemClone = new Point(element);
        //this point represents where the element is relative to the robot
        Point relRobotPos = new Point(-elemClone.x, elemClone.y - 1.3);

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-getH() - 90));

        goToVals.target.x = relRobotPos.rotate(radHeading).x + getX();
        goToVals.target.y = relRobotPos.rotate(radHeading).y + getY();

        return goToVals;
    }

    public Point elementPosFinder(Point element, double ext) {
        //goToValues goToVals = new goToValues(values);
        Point elemClone = new Point(element);
        //this point represents where the element is relative to the robot
        Point relRobotPos = new Point(-elemClone.x, elemClone.y + 30 + ext);

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-getH() - 90));

        elemClone.x = relRobotPos.rotate(radHeading).x + getX();
        elemClone.y = relRobotPos.rotate(radHeading).y + getY();

        return elemClone;
    }

    public Point polarFinder(Point robot, Point element) {
        Point robotClone = new Point(robot);
        Point elemClone = new Point(element);

        //this point represents where the element is relative to the robot
        //Point relRobotPos = new Point(-elemClone.x, elemClone.y + 30 + ext);
        Point relExtension = Point.polar(abs(robotClone.distanceTo(elemClone) - 31.9), AngleWrap(-robotClone.atan2(elemClone) - toRadians(90)));

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        //double radHeading = AngleWrap(Math.toRadians(-getH() - 90));

        //elemClone.x = relRobotPos.rotate(radHeading).x + getX();
        //elemClone.y = relRobotPos.rotate(radHeading).y + getY();

        return relExtension;
    }


    public Point elementLocalPosFinder(Point element) {
        //goToValues goToVals = new goToValues(values);
        Point elemClone = new Point(element);
        //this point represents where the element is relative to the robot
        Point relRobotPos = new Point(-elemClone.x, elemClone.y);

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-getH() - 90));

        elemClone.x = relRobotPos.rotate(radHeading).x;
        elemClone.y = relRobotPos.rotate(radHeading).y;

        return elemClone;
    }

    public goToValues clawToRobotPos(goToValues claw, double ext) {
        goToValues goToVals = new goToValues(new Pose2d(claw.target.x, claw.target.y, claw.target.heading), claw.speed, claw.error);

        //this point represents where the claw is relative to the robot
        Point relRobotPos = new Point(0, -(30 + ext));

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-goToVals.target.heading - 90));

        goToVals.target.x += relRobotPos.rotate(radHeading).x;
        goToVals.target.y += relRobotPos.rotate(radHeading).y;

        return goToVals;
    }

    public Pose2d robotToClawPos(Pose2d robot, double ext) {
        Pose2d claw = new Pose2d(robot.x, robot.y, robot.heading);

        //this point represents where the robot is relative to the Claw
        //used to be 30+ext for Octav
        Point relRobotPos = new Point(0, (30 + ext));

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-claw.heading - 90));
        claw.x += relRobotPos.rotate(radHeading).x;
        claw.y += relRobotPos.rotate(radHeading).y;

        return claw;
    }


    public goToValues specimenToRobotPos(goToValues specimen) {
        goToValues goToVals = new goToValues(new Pose2d(specimen.target.x, specimen.target.y, specimen.target.heading), specimen.speed, specimen.error);

        //this point represents where the specimen is relative to the center of the robot
        Point relSpecPos = new Point(14.5, -21.5);

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-goToVals.target.heading - 90));

        goToVals.target.x += relSpecPos.rotate(radHeading).x;
        goToVals.target.y += relSpecPos.rotate(radHeading).y;

        return goToVals;
    }


    public Pose2d robotToSpecimenPos(Pose2d robot) {
        Pose2d specimen = new Pose2d(robot.x, robot.y, robot.heading);

        //this point represents where the robot is relative to the specimen
        Point relSpecPos = new Point(-14.5, 21.5);

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-specimen.heading - 90));
        specimen.x += relSpecPos.rotate(radHeading).x;
        specimen.y += relSpecPos.rotate(radHeading).y;

        return specimen;
    }

    public goToValues exOutTakePos(goToValues outTake, double ext) {
        goToValues goToVals = new goToValues(new Pose2d(outTake.target.x, outTake.target.y, outTake.target.heading), outTake.speed, outTake.error);
        //formula este distantaFataDeCentru + (Extensie+Extensie initiala)*cos(ungiul glisierelor)
        ext = (ext + 17) * cos(toRadians(70.1));
        extTelemetry = ext;
        double radHeading = AngleWrap(Math.toRadians(goToVals.target.heading));
        goToVals.target.x -= ext * sin(radHeading);
        goToVals.target.y -= ext * cos(radHeading);
        return goToVals;
    }

    public goToValues outTakeToRobotPos(goToValues specimen, double ext) {
        goToValues goToVals = new goToValues(new Pose2d(specimen.target.x, specimen.target.y, specimen.target.heading), specimen.speed, specimen.error);

        //this point represents where the outTake is relative to the center of the robot
        Point relSpecPos = new Point(0, -(ext + 17) * cos(toRadians(70.1)));

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-goToVals.target.heading + 90));

        goToVals.target.x += relSpecPos.rotate(radHeading).x;
        goToVals.target.y += relSpecPos.rotate(radHeading).y;

        return goToVals;
    }

    public Pose2d robotToOutTakePos(Pose2d robot, double ext) {
        Pose2d outTake = new Pose2d(robot.x, robot.y, robot.heading);

        //this point represents where the robot is relative to the outTake
        Point relSpecPos = new Point(0, (ext + 17));

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-outTake.heading + 90));

        outTake.x += relSpecPos.rotate(radHeading).x;
        outTake.y += relSpecPos.rotate(radHeading).y;

        return outTake;
    }

    public Point macaraToOutTakePos(Pose2d robot, double distance) {
        Pose2d outTake = new Pose2d(robot.x, robot.y, robot.heading);

        //this point represents where the robot is relative to the outTake
        Point relSpecPos = new Point(0, distance);

        // -AngleWrap ca sa contracarezi sensul trignometric
        // +90 ca sa aduci punctul initial unde e in realitate
        double radHeading = AngleWrap(Math.toRadians(-outTake.heading + 90));

        outTake.x += relSpecPos.rotate(radHeading).x;
        outTake.y += relSpecPos.rotate(radHeading).y;

        return outTake;
    }
}