package org.firstinspires.ftc.teamcode.implementation;

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
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp
public class AutoAprilTagAlign extends LinearOpMode {

    public static int targetDist = 15;
    public static int targetID = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

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
        boolean tagVisible = false;

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            tagVisible = false;

            if (gamepad1.right_bumper) {
                targetID++;
                if (targetID > 6) targetID = 4;
            }

            if (tagProcessor.getDetections().size() > 0) {
                for (AprilTagDetection tag : tagProcessor.getDetections()) {
                    if (tag.id == targetID) {
                        tagVisible = true;
                        desired_y = tag.ftcPose.x;
                        desired_x = -1 * tag.ftcPose.y + targetDist;
                        desired_heading = (tag.ftcPose.yaw);

                        telemetry.addData("tag id", tag.id);
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

            if (gamepad1.a && !goToTag && tagVisible) {
                while (gamepad1.a) {}
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                goToTag = true;
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();
            }

            if (goToTag) {
                drive.followTrajectory(desired_x, desired_y, correctAngle(desired_heading));
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );

                drive.update();

                goToTag = false;
            }

            telemetry.addData("targetID", targetID);
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
