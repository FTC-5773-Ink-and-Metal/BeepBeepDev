package org.firstinspires.ftc.teamcode.custom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "DriveTest", group = "Custom")
public class DriveTest extends LinearOpMode {

    // Constants
    public static double decayFactor = 0.95;
    public static double decelerationRate = 0.5;
    public static double updateInterval = 0.01;
    public static double turnMultiplier = 1.0;
    public static double moveMultiplier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        double decay;
        double previousTime = timer.time();
        double currentTime, elapsedTime;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            currentTime = timer.time();
            elapsedTime = currentTime - previousTime;

            // Joystick is active
            if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * moveMultiplier,
                                -gamepad1.left_stick_x * moveMultiplier,
                                -gamepad1.right_stick_x * turnMultiplier
                        )
                );
            }

            // Joystick released, start decelerating
            else {
                decay = Math.pow(decayFactor, (elapsedTime / updateInterval));
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * moveMultiplier * decay,
                                -gamepad1.left_stick_x * moveMultiplier * decay,
                                -gamepad1.right_stick_x * turnMultiplier * decay
                        )
                );
            }

            drive.update();
            previousTime = currentTime;

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
