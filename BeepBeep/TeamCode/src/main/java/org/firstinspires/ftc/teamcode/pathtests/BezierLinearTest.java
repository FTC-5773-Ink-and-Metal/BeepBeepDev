package org.firstinspires.ftc.teamcode.pathtests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;

@Config
@TeleOp
public class BezierLinearTest extends LinearOpMode {

    // 0, 1, 30, 30
    public static double p1X = 30;
    public static double p2X = 70;
    public static double p3X = 30;
    public static double p4X = 70;

    // 0, 24, 1, 30
    public static double p1Y = 0;
    public static double p2Y = 0;
    public static double p3Y = 30;
    public static double p4Y = 30;

    public static double linearX = 30;
    public static double linearY = 0;

    public static double desired_heading = 0;

    public static double time_factor = 2.7;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        Drive drive = new Drive(hardwareMap, telemetry);
        BezierCurve bezier_x_forward = new BezierCurve(p1X, p2X, p3X, p4X);
        BezierCurve bezier_y_forward = new BezierCurve(p1Y, p2Y, p3Y, p4Y);
        BezierCurve bezier_x_back = new BezierCurve(p4X, p3X, p2X, p1X);
        BezierCurve bezier_y_back = new BezierCurve(p4Y, p3Y, p2Y, p1Y);

        waitForStart();

        drive.followTrajectory(linearX, linearY, desired_heading);
        drive.followTrajectory(bezier_x_forward, bezier_y_forward, desired_heading, time_factor);
        drive.followTrajectory(bezier_x_back, bezier_y_back, desired_heading, time_factor);
        drive.followTrajectory(0, 0, desired_heading);
    }
}
