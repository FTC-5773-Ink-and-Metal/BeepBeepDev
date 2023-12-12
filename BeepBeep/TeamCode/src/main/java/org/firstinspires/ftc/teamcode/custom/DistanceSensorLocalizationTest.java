package org.firstinspires.ftc.teamcode.custom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class DistanceSensorLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double[] sensorReadings;
        Pose2d pos;

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            sensorReadings = drive.getWallDistances();
            pos = drive.localizeDistanceSensors(sensorReadings[0], sensorReadings[1], sensorReadings[2]);

            telemetry.addData("L1", sensorReadings[0]);
            telemetry.addData("L2", sensorReadings[1]);
            telemetry.addData("L3", sensorReadings[2]);
            telemetry.addData("Robot X", pos.getX());
            telemetry.addData("Robot Y", pos.getY());
            telemetry.addData("Robot Heading", pos.getHeading());
            telemetry.update();
        }
    }
}
