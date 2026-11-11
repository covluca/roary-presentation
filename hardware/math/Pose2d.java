package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower.goToValues;

import java.util.Locale;

public class Pose2d extends Point {

    public double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(Pose2d pose2d) {
        this.x = pose2d.x;
        this.y = pose2d.y;
        this.heading = pose2d.heading;
    }

    public Pose2d(Pose2D pos2D) {
        this.x = pos2D.getX(DistanceUnit.CM);
        this.y = pos2D.getY(DistanceUnit.CM);
        this.heading = -pos2D.getHeading(AngleUnit.DEGREES);
    }

    public Pose2d(Point p, double heading) {
        if (p != null) {
            this.x = p.x;
            this.y = p.y;
        } else {
            this.x = 0;
            this.y = 0;
        }
        this.heading = heading;
    }

    public Pose2d() {
        this(0, 0, 0);
    }

    public Pose2d(goToValues values) {
        this.x = values.target.x;
        this.y = values.target.y;
        this.heading = values.target.heading;
    }

    public Pose2d subtract(Pose2d other) {
        return new Pose2d(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    public double distanceToPose(Pose2d other) {
        return other.subtract(this).radius();
    }

    public double radius() {
        return FastMath.hypot(x, y);
    }

    public Pose2d divide(Pose2d other) {
        return new Pose2d(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }

    public Pose2d addVel(Pose2D vel, double div) {
        return new Pose2d(this.x - vel.getX(DistanceUnit.CM) / div, this.y - vel.getY(DistanceUnit.CM) / div, this.heading - vel.getHeading(AngleUnit.DEGREES) / div);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.2f", x, y, heading);
    }

    public static Pose2d Pose2Dto2d(Pose2D pos2D) {
        return new Pose2d(pos2D.getX(DistanceUnit.CM), pos2D.getY(DistanceUnit.CM), -pos2D.getHeading(AngleUnit.DEGREES));
    }


    public double getHeading() {
        return heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

}
