package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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
}
