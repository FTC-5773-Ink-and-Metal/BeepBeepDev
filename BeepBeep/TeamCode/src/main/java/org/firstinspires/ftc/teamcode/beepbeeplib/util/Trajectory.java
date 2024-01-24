package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Trajectory {
    Pose2d startPose;
    Pose2d endPose;

    public Trajectory(Pose2d startPose, Pose2d endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
    }
}
