package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_y;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;

public class LinearTraj extends Trajectory {

    public LinearTraj(Pose2d startPose, Pose2d endPose) {
        super(startPose, endPose, "linear");
    }

    @Override
    public double curveLength() {
        return Math.sqrt(Math.pow(endPose.getX()-startPose.getX(), 2) + Math.pow(endPose.getY()-startPose.getY(), 2));
    }

    @Override
    public Vector2d calculatePow(MotionProfile motionProfile, double fullPathDistTravelled, double currTime, PIDController pid_x, PIDController pid_y, Pose2d currPose) {
        double path_angle = path_angle();
        double direction = turn_direction();

        double instantTargetPosition = motionProfile.getPosition(currTime) - fullPathDistTravelled;
        double xTargetPos = instantTargetPosition * Math.cos(path_angle) + currPose.getX();
        double yTargetPos = instantTargetPosition * Math.sin(path_angle) + currPose.getY();

        double vel = motionProfile.getVelocity(currTime);
        double accel = motionProfile.getAcceleration(currTime);

//            double xTargetPos = instantTargetPosition * Math.cos(path_angle) + startPose.getX();
        double xTargetVel = vel * Math.cos(path_angle);
        double xTargetAccel = accel * Math.cos(path_angle);

//            double yTargetPos = instantTargetPosition * Math.sin(path_angle) + startPose.getY();
        double yTargetVel = vel * Math.sin(path_angle);
        double yTargetAccel = accel * Math.sin(path_angle);

        double powX = (xTargetVel * kV_x + xTargetAccel * kA_x);
        double powY = (yTargetVel * kV_y + yTargetAccel * kA_y);

        powX = powX + kS_x * powX/Math.abs(powX);
        powY = powY + kS_y * powY/Math.abs(powY);

        if (Double.isNaN(powX)) {
            powX = 0;
            xTargetPos = endPose.getX();
        }

        if (Double.isNaN(powY)) {
            powY = 0;
            yTargetPos = endPose.getY();
        }

//            if (pathBreakerLinear < Math.sqrt(Math.pow((xTargetPos - poseEstimate.getX()), 2) + (Math.pow((yTargetPos - poseEstimate.getY()), 2))) && !pathBroken) {
//                pathBroken = true;
//                powX = 0;
//                xTargetPos = poseEstimate.getX();
//                powY = 0;
//                yTargetPos = poseEstimate.getY();
//                instantTargetPositionHeading = poseEstimate.getHeading();
//                pauseTime = timer.time();
//            }

        double control_signal_x = pid_x.calculate(xTargetPos, currPose.getX()) + powX;
        double control_signal_y = pid_y.calculate(yTargetPos, currPose.getY()) + powY;

        return new Vector2d(control_signal_x, control_signal_y);
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
