package org.firstinspires.ftc.teamcode.pathtests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.BezierTraj;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.LinearTraj;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.Trajectory;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp(group = "dev")
public class SequenceTest extends LinearOpMode {

    // Target positions and heading
    public static double p1X = 0;
    public static double p2X = 40;
    public static double p3X = 0;
    public static double p4X = 40;

    // 0, 24, 1, 30
    public static double p1Y = 0;
    public static double p2Y = 0;
    public static double p3Y = 30;
    public static double p4Y = 30;

    public static double desired_x = 60;
    public static double desired_y = 30;

    public static double desired_x2 = 70;
    public static double desired_y2 = 30;

    public static double deg_desired_heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        Drive drive = new Drive(hardwareMap, telemetry);
//        telemetry.addData("Sign", 0);
//        telemetry.addData("currHeading", 0);
//        telemetry.addData("In if condition", 0);
        telemetry.addData("Error to Inst. Pos", 0);
        telemetry.addData("Error to Inst. Vel", 0);
        telemetry.addData("pathBroken", 0);
        telemetry.addData("Battery Voltage", 0);
        telemetry.addData("Is in loop 1", 0);
        telemetry.addData("Is in loop 2", 0);
        telemetry.addData("Is in loop 3", 0);
        telemetry.addData("currPath", 0);
        telemetry.addData("fullPathDistTravelled", 0);
        telemetry.addData("motionProfile.getPosition(currTime) - fullPathDistTravelled", 0);
        telemetry.addData("totalPathFinished", 0);
        telemetry.addData("powX", 0);
        telemetry.update();

        waitForStart();

        BezierTraj bezier = new BezierTraj(drive.getPoseEstimate(), new Pose2d(p4X, p4Y, Math.toRadians(deg_desired_heading)), new Vector2d(p2X, p2Y), new Vector2d(p3X, p3Y));
        LinearTraj linear = new LinearTraj(new Pose2d(p4X, p4Y, Math.toRadians(deg_desired_heading)), new Pose2d(desired_x, desired_y, Math.toRadians(deg_desired_heading)));
        LinearTraj linear2 = new LinearTraj(new Pose2d(desired_x, desired_y, Math.toRadians(deg_desired_heading)), new Pose2d(desired_x2, desired_y2, Math.toRadians(deg_desired_heading)));

        drive.followTrajectorySequence(new ArrayList<>(Arrays.asList(bezier, linear, linear2)));
    }

    /*
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Variables for PID control
        PIDController pid_x = new PIDController(Kp_x, Ki_x, Kd_x);
        PIDController pid_y = new PIDController(Kp_y, Ki_y, Kd_y);
        PIDController pid_heading = new PIDController(Kp_heading, Ki_heading, Kd_heading);

        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;

        double path_distance = Math.sqrt(Math.pow(desired_x-0, 2) + Math.pow(desired_y-0, 2));
        double path_angle = Math.atan2(desired_y, desired_x);
        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (opModeIsActive() && !isStopRequested()) {
            double motionMultiplier = 1;

            double instantTargetPosition = motionProfile.getPosition(timer.time());

            if (motionProfile.isFinished(timer.time())) {
                instantTargetPosition = path_distance;
            }
            double vel = motionProfile.getVelocity(timer.time());
            double accel = motionProfile.getAcceleration(timer.time());

            double xTargetPos = instantTargetPosition * Math.cos(path_angle);
            double xTargetVel = vel * Math.cos(path_angle);
            double xTargetAccel = accel * Math.cos(path_angle);

            double yTargetPos = instantTargetPosition * Math.sin(path_angle);
            double yTargetVel = vel * Math.sin(path_angle);
            double yTargetAccel = accel * Math.sin(path_angle);

            double powX = motionMultiplier * (xTargetVel * kV_x + xTargetAccel * kA);
            double powY = motionMultiplier * (yTargetVel * kV_y + yTargetAccel * kA);

            powX = powX + kS * powX/Math.abs(powX);
            powY = powY + kS * powY/Math.abs(powY);

            if (Double.isNaN(powX)) {
                powX = 0;
                xTargetPos = desired_x;
            }

            if (Double.isNaN(powY)) {
                powY = 0;
                yTargetPos = desired_y;
            }

            Pose2d poseEstimate = drive.getPoseEstimate();

            control_signal_x = pid_x.calculate(xTargetPos, poseEstimate.getX()) + powX;
            control_signal_y = pid_y.calculate(yTargetPos, poseEstimate.getY()) + powY;
            control_signal_heading = pid_heading.calculate(angleWrap(Math.toRadians(desired_heading)), angleWrap(poseEstimate.getHeading()));

            Vector2d input = new Vector2d(
                    control_signal_x,
                    control_signal_y
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            control_signal_heading
                    )
            );

            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("x error", xTargetPos - poseEstimate.getX());
            telemetry.addData("y error", yTargetPos - poseEstimate.getY());
            telemetry.addData("heading error", desired_heading - poseEstimate.getHeading());
            telemetry.addData("path position", instantTargetPosition);
            telemetry.update();
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
     */

}