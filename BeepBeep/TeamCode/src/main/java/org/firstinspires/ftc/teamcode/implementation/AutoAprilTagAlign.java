package org.firstinspires.ftc.teamcode.implementation;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.*;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;
import org.firstinspires.ftc.teamcode.beepbeep.PIDController;
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp
public class AutoAprilTagAlign extends LinearOpMode {

    public static int tagID = 5;
    public static int targetDist = 15;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, telemetry);
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

        double path_distance = 0;
        double path_angle = 0;
        MotionProfile motionProfile = null;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "backWebcam"))
                .setCameraResolution(new Size(960,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255); // change value

        double desired_x = 0, desired_y = 0, desired_heading = 0;
        boolean goToTag = false;

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                for (AprilTagDetection tag : tagProcessor.getDetections()) {
                    if (tag.id == 4) {
                        desired_y = tag.ftcPose.x + 6;
                        desired_x = -1 * tag.ftcPose.y + targetDist;
                        desired_heading = (tag.ftcPose.yaw);

                        telemetry.addData("x", tag.ftcPose.x);
                        telemetry.addData("y", tag.ftcPose.y);
//                        telemetry.addData("z", tag.ftcPose.z);
//                        telemetry.addData("roll", tag.ftcPose.roll);
//                        telemetry.addData("pitch", tag.ftcPose.pitch);
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.update();
                    } else if (tag.id == 5) {
                        desired_y = tag.ftcPose.x;
                        desired_x = -1 * tag.ftcPose.y + targetDist;
                        desired_heading = (tag.ftcPose.yaw);

                        telemetry.addData("x", tag.ftcPose.x);
                        telemetry.addData("y", tag.ftcPose.y);
//                        telemetry.addData("z", tag.ftcPose.z);
//                        telemetry.addData("roll", tag.ftcPose.roll);
//                        telemetry.addData("pitch", tag.ftcPose.pitch);
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.update();
                    } else if (tag.id == 6) {
                        desired_y = tag.ftcPose.x - 6;
                        desired_x = -1 * tag.ftcPose.y + targetDist;
                        desired_heading = (tag.ftcPose.yaw);

                        telemetry.addData("x", tag.ftcPose.x);
                        telemetry.addData("y", tag.ftcPose.y);
//                        telemetry.addData("z", tag.ftcPose.z);
//                        telemetry.addData("roll", tag.ftcPose.roll);
//                        telemetry.addData("pitch", tag.ftcPose.pitch);
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.update();
                    }
                }
            }

            if (gamepad1.a && !goToTag) {
                while (gamepad1.a) {}

                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

                goToTag = true;
                timer.reset();
                timer.time();
            }

            if (goToTag) {
                drive.followTrajectory(desired_x, desired_y, correctAngle(desired_heading));
            }

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

    public double correctAngle(double radians) {
        while (radians < 0) {
            radians += 2*Math.PI;
        }

        while (radians >= 2*Math.PI) {
            radians -= 2*Math.PI;
        }

        return radians;
    }
}
