package org.firstinspires.ftc.teamcode.beepbeep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;

@Config
@TeleOp(group = "dev")
public class BackAndForth extends LinearOpMode {

    // Target positions and heading
    public static double distance = 60;
    public static String tuningMode = "STRAIGHT";

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        Drive drive = new Drive(hardwareMap, telemetry);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        int direction = -1;
        double xPos = distance;
        Pose2d targetPos = new Pose2d(xPos, 0, 0);
        Pose2d targetNeg = new Pose2d(-1*xPos, 0, 0);

        if (tuningMode.equals("STRAFE")) {
            targetPos = new Pose2d(0, xPos, 0);
            targetNeg = new Pose2d(0, -1*xPos, 0);
        }

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            drive.followTrajectory(targetPos.getX(), targetPos.getY(), 0);
            drive.setPoseEstimate(new Pose2d(0, 0));
            drive.followTrajectory(targetNeg.getX(), targetNeg.getY(), 0);
            drive.setPoseEstimate(new Pose2d(0, 0));
        }
    }
    public double angleWrap(double radians) {

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