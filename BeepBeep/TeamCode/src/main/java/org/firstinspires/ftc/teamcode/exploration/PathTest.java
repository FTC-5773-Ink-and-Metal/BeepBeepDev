package org.firstinspires.ftc.teamcode.exploration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;

public class PathTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create BezierCurves for x and y coordinates
        BezierCurve xBezier = new BezierCurve(0.85, 1, 16.9, 16.3);
        BezierCurve yBezier = new BezierCurve(0.5, 17.65, 1, 16.24);

        // Set up the path points
        double start = 0;
        double end = 1;
        double increment = 0.1;

        // Wait for the start button to be pressed
        waitForStart();

        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("X Velocity", 0);
        telemetry.addData("Y Velocity", 0);
        telemetry.update();

        sleep(3000);

        // Iterate through the path points and log velocities in telemetry
        for (double t = start; t <= end; t += increment) {
            double xVelocity = xBezier.bezier_deriv(t);
            double yVelocity = yBezier.bezier_deriv(t);
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.update();
            sleep(1000);  // Delay for one second
        }
    }
}
