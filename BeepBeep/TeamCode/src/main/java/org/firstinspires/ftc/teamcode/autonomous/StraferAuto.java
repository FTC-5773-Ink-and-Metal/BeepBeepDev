package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class StraferAuto extends LinearOpMode {

    public static double Traj_1_X = 31;
    public static double Traj_1_Y = 12;
    public static double Traj_1_Angle = 90;
    public static double Traj_2_X = 43;
    public static double Traj_2_Y = 24;
    public static double Traj_2_Angle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive strafer = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        strafer.setPoseEstimate(startPose);

        TrajectorySequence traj1 = strafer.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(Traj_1_X, Traj_1_Y), Math.toRadians(Traj_1_Angle))
                .splineToConstantHeading(new Vector2d(Traj_2_X, Traj_2_Y), Math.toRadians(Traj_2_Angle)).build();


        waitForStart();
        if (isStopRequested()) return;

        strafer.followTrajectorySequence(traj1);
    }
}

