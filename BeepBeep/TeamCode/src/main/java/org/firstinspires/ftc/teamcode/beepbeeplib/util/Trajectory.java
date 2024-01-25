package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;

import java.util.Vector;

public class Trajectory {
    Pose2d startPose;
    Pose2d endPose;
    String pathType;

    public Trajectory(Pose2d startPose, Pose2d endPose, String pathType) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.pathType = "trajectory";
    }

    public double curveLength() {
        return 0;
    }

    public Vector2d calculatePow(MotionProfile motionProfile, double fullPathDistTravelled, double currTime, PIDController pid_x, PIDController pid_y, Pose2d currPose) {
        return new Vector2d(0, 0);
    }
}
