package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math;

import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.round;
import static java.lang.Math.toRadians;
import static java.util.Collections.swap;

public class Meth {
    public static double inRange(double x, double min, double max) {
        if (min > max) {
            double tmp = min;
            min = max;
            max = tmp;
        }

        return max(min(x, max), min);
    }

    public static boolean isInRange(double x, double min, double max) {
        if (min > max) {
            double tmp = min;
            min = max;
            max = tmp;
        }

        return x >= min && x <= max;
    }

    public static int closestInt(double x) {
        if (x > 0) return (int) (x + 0.5);
        else return (int) (x - 0.5);
    }

    public static int closestPositiveInt(double x) {
        if (x > 0) return (int) (x + 0.5);
        else return 0;
    }

    public static double round2Decimals(double x) {
        return round(x * 100.0) / 100.0;
    }

    /// Ceva nebunie ce se bazeaza pe algoritmul lui newton si e mult mai rapid ca sqrt
    public static double FSqrt(double a) {
        if (a == 0) return 0;

        double x = a / 2;
        final double epsilon = 1e-9; // Tolerance level for convergence

        for (int i = 0; i < 8; ++i) { // Customizable iteration count
            double newX = (x + a / x) / 2;
            if (Math.abs(newX - x) < epsilon) // Check for convergence
                break;
            x = newX;
        }

        return x;
    }

    public static double area(Point first, Point second, Point third) {
        return Math.abs((first.x * (second.y - third.y) + second.x * (third.y - first.y) + third.x * (first.y - second.y)) / 2.0);
    }

    /* A function to check whether point P(x, y) lies
       inside the triangle formed by A(first),
       B(second) and C(third) */
    public static boolean isInside(Point first, Point second, Point third, Point x) {
        /* Calculate area of triangle ABC */
        double A = area(first, second, third);

        /* Calculate area of triangle PBC */
        double A1 = area(x, second, third);

        /* Calculate area of triangle PAC */
        double A2 = area(first, x, third);

        /* Calculate area of triangle PAB */
        double A3 = area(first, second, x);

        /* Check if sum of A1, A2 and A3 is same as A */
        return (A == A1 + A2 + A3);
    }

    public static boolean isRobotInTriangle(Point f, Point s, Point t, Pose2d robot, double witdh, double length) {

        Point FL = new Point(-witdh / 2, length / 2).rotate(toRadians(-robot.heading)).add(robot);
        Point FR = new Point(witdh / 2, length / 2).rotate(toRadians(-robot.heading)).add(robot);
        Point BL = new Point(-witdh / 2, -length / 2).rotate(toRadians(-robot.heading)).add(robot);
        Point BR = new Point(witdh / 2, -length / 2).rotate(toRadians(-robot.heading)).add(robot);

        return isInside(f, s, t, FL) || isInside(f, s, t, FR) || isInside(f, s, t, BL) || isInside(f, s, t, BR);


    }

    public static boolean isRobotInBigTriangle(Pose2d robot, double witdh, double length) {

        Point f = new Point(-180, 180);
        Point s = new Point(180, 180);
        Point t = new Point();

        return isRobotInTriangle(f, s, t, robot, witdh, length);
    }

    public static boolean isRobotInSmallTriangle(Pose2d robot, double witdh, double length) {

        Point f = new Point(-60, -180);
        Point s = new Point(60, -180);
        Point t = new Point(0, -120);

        return isRobotInTriangle(f, s, t, robot, witdh, length);
    }

}


