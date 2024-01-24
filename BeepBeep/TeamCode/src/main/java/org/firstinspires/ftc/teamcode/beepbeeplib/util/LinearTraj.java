package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class LinearTraj extends Trajectory {

    public LinearTraj(Pose2d startPose, Pose2d endPose) {
        super(startPose, endPose);
    }

    public double path_distance() {
        return Math.sqrt(Math.pow(endPose.getX()-startPose.getX(), 2) + Math.pow(endPose.getY()-startPose.getY(), 2));
    }

    public double path_angle() {
        return angleWrap(Math.atan2(endPose.getY()-startPose.getY(), endPose.getX()-startPose.getX()));
    }

    // +1 = clockwise, -1 = counter-clockwise
    public double turn_direction() {
        double c_dist = 0, cc_dist = 0;

        if (startPose.getHeading() < endPose.getHeading()) {
            cc_dist = endPose.getHeading() - startPose.getHeading(); // 190
            c_dist = startPose.getHeading() + Math.toRadians(360) - endPose.getHeading();
        } else if (startPose.getHeading() > endPose.getHeading()) {
            c_dist = endPose.getHeading() + Math.toRadians(360) - startPose.getHeading();
            cc_dist = startPose.getHeading() - endPose.getHeading();
        }

        if (c_dist > cc_dist) {
            return 1;
        }

        return -1;
    }

    private double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}
