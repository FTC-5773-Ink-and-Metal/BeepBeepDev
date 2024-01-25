package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class TurnTraj extends Trajectory {

    private int turnDirection;

    public TurnTraj(Pose2d startPose, Pose2d endPose) {
        super(startPose, endPose, "linear");
        endPose = new Pose2d(startPose.getX(), startPose.getY(), endPose.getHeading());

        double c_dist = 0, cc_dist = 0;

        if (startPose.getHeading() < endPose.getHeading()) {
            cc_dist = endPose.getHeading() - startPose.getHeading(); // 190
            c_dist = startPose.getHeading() + Math.toRadians(360) - endPose.getHeading();
        } else if (startPose.getHeading() > endPose.getHeading()) {
            c_dist = endPose.getHeading() + Math.toRadians(360) - startPose.getHeading();
            cc_dist = startPose.getHeading() - endPose.getHeading();
        }

        if (c_dist > cc_dist) {
            this.turnDirection = 1;
        } else {
            this.turnDirection = -1;
        }
    }

    public double angular_displacement() {
        return correctAngle(endPose.getHeading() * turnDirection);
    }

    private double correctAngle(double radians) {
        while (radians < 0) {
            radians += 2*Math.PI;
        }

        while (radians >= 2*Math.PI) {
            radians -= 2*Math.PI;
        }

        return radians;
    }
}
