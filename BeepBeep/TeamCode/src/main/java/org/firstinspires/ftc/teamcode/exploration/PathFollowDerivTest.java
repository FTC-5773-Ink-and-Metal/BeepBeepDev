package org.firstinspires.ftc.teamcode.exploration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;

@Config
@TeleOp(group = "Pathing")
public class PathFollowDerivTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    public static double maxMotorPower = 0.5;  // Set the maximum motor power here

    public static double x1 = 0.85;
    public static double y1 = 0.5;
    public static double x2 = 1.0;
    public static double y2 = 17.65;
    public static double x3 = 16.9;
    public static double y3 = 1;
    public static double x4 = 16.3;
    public static double y4 = 16.24;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.update();

        // Initialize the motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Reverse the necessary motors if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Create BezierCurves for x and y coordinates
        BezierCurve xBezier = new BezierCurve(x1, x2, x3, x4);
        BezierCurve yBezier = new BezierCurve(y1, y2, y3, y4);

        // Set up the path points
        double start = 0;
        double end = 1;
        double increment = 0.01;

        // Wait for the start button to be pressed
        waitForStart();
        runtime.reset();

        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("X Velocity", 0);
        telemetry.addData("Y Velocity", 0);
        telemetry.addData("X Velocity (Odo)", 0);
        telemetry.addData("Y Velocity (Odo)", 0);
        telemetry.update();

        sleep(5000);

        // Iterate through the path points and set motor powers accordingly
        for (double t = start; t <= end; t += increment) {
            double xVelocity = xBezier.bezier_deriv(t);
            double yVelocity = yBezier.bezier_deriv(t);

            // Calculate motor powers using mecanum drive equations
            double leftFrontPower = yVelocity + xVelocity;
            double rightFrontPower = yVelocity - xVelocity;
            double backLeftPower = yVelocity - xVelocity;
            double backRightPower = yVelocity + xVelocity;

            // Scale the motor powers to the maximum power if necessary
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (maxPower > maxMotorPower) {
                leftFrontPower *= maxMotorPower / maxPower;
                rightFrontPower *= maxMotorPower / maxPower;
                backLeftPower *= maxMotorPower / maxPower;
                backRightPower *= maxMotorPower / maxPower;
            }

            // Set motor powers
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(backLeftPower);
            rightRear.setPower(backRightPower);

            drive.update();
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("X Velocity (Odo)", drive.getPoseVelocity().getX());
            telemetry.addData("Y Velocity (Odo)", drive.getPoseVelocity().getY());
            telemetry.update();
            
            sleep(15);
        }

        // Stop the motors after the path is completed
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
