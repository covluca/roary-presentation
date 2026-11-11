package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math;

import androidx.annotation.NonNull;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Point {
    public double x, y, z;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Point() {
        this(0, 0, 0);
    }

    public Point(Pose2d point) {
        this.x = point.x;
        this.y = point.y;
        this.z = point.z;
    }

    public Point(Point point) {
        if (point == null) {
            this.x = 0;
            this.y = 0;
        } else {
            this.x = point.x;
            this.y = point.y;
        }

    }

    public Point(Pose2D odo) {
        this.x = odo.getX(DistanceUnit.CM);
        this.y = odo.getY(DistanceUnit.CM);
    }

    public Point(Pose3D botposeMt2) {
        this.x = botposeMt2.getPosition().y * 100;
        this.y = -botposeMt2.getPosition().x * 100;
    }

    public static Point polar(double r, double angle) {
        return new Point(Math.cos(angle) * r, Math.sin(angle) * r);
    }

    public Point subtract(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public Point add(Point other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point divide(double div) {
        return new Point(x / div, y / div);
    }
    public Point multiply(double mul) {
        return new Point(x * mul, y * mul);
    }

    public double distanceTo(Point other) {
        return other.subtract(this).radius();
    }

    public double atan() {
        return Math.atan2(x, y);
    }

    public double atan2(Point other) {
        return Math.atan2(other.x - x, other.y - y);
    }

    public double radius() {
        return FastMath.hypot(x, y);
    }

    public Point rotate(double amount) {
        return Point.polar(radius(), atan() + amount);
    }

    @Override
    public String toString() {
        return "x=" + x + ", y=" + y;
    }
}