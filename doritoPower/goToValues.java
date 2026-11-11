package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.doritoPower;

import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Point;
import org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware.math.Pose2d;

public class goToValues {
    public Pose2d target;
    public double speed;
    public double error;
    public Point init;
    public goToValues() {
        this.target = new Pose2d();
        this.speed = 0;
        this.error = 0;
    }
    public goToValues(Pose2d target, double speed, double error) {
        this.target = target;
        this.speed = speed;
        this.error = error;
    }
    public goToValues(goToValues vals) {
        this.target = new Pose2d(vals);
        this.speed = vals.speed;
        this.error = vals.error;
    }
    public goToValues(Pose2d target, double speed, double error,Point init) {
        this.target = target;
        this.speed = speed;
        this.error = error;
        this.init = init;
    }
}
