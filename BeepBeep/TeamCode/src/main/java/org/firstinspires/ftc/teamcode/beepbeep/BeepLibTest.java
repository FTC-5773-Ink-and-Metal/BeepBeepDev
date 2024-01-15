package org.firstinspires.ftc.teamcode.beepbeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;

@Config
@TeleOp
public class BeepLibTest extends LinearOpMode {

    // 0, 1, 30, 30
    public static double p1X = 0;
    public static double p2X = 0;
    public static double p3X = 30;
    public static double p4X = 30.0;

    // 0, 24, 1, 30
    public static double p1Y = 0;
    public static double p2Y = 40;
    public static double p3Y = 0;
    public static double p4Y = 40;

    public static double linearX = 0;
    public static double linearY = 0;

    public static double desired_heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        Drive drive = new Drive(hardwareMap, telemetry);
        BezierCurveCalc bezier_calc = new BezierCurveCalc();
        BezierCurve bezier_y = new BezierCurve(p1X, p2X, p3X, p4X);
        BezierCurve bezier_x = new BezierCurve(p1Y, p2Y, p3Y, p4Y);

        waitForStart();

        telemetry.addData("Path Status", "Bezier Started");
        telemetry.update();
        drive.followTrajectory(bezier_x, bezier_y, desired_heading);
        telemetry.addData("Path Status", "Bezier Finished, Linear Started");
        telemetry.update();
        drive.followTrajectory(linearX, linearY, desired_heading);
        telemetry.addData("Path Status", "Linear Finished");
        telemetry.update();
    }
}
