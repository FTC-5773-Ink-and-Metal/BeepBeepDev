package org.firstinspires.ftc.teamcode.exploration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class DriveVelocityTest extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private double avgXVel = 0, avgYVel = 0;
    private int counter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        // Initialize the motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Reverse the necessary motors if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        telemetry.addData("MODE", "FORWARD");
        telemetry.update();
        forward();
        sleep(50);

        while (!isStopRequested() && counter <= 10) {
            sleep(30);
            double xVel = drive.getVel().getX();
            double yVel = drive.getVel().getY();
            avgXVel += xVel;
            avgYVel += yVel;

            telemetry.addData("vel x", drive.getVel().getX());
            telemetry.addData("vel y", drive.getVel().getY());
            telemetry.addData("vel heading", drive.getVel().getHeading());
            telemetry.update();

            counter++;
        }

        halt();
        telemetry.addData("MODE", "RESET");
        telemetry.addData("Avg Vel X FORWARD", avgXVel / counter);
        telemetry.addData("Avg Vel Y FORWARD", avgYVel / counter);
        telemetry.update();
        sleep(15000);
        counter = 0;
        avgXVel = 0;
        avgYVel = 0;
        strafeLeft();
        sleep(50);

        while (!isStopRequested() && counter <= 10) {
            sleep(30);
            double xVel = drive.getVel().getX();
            double yVel = drive.getVel().getY();
            avgXVel += xVel;
            avgYVel += yVel;

            telemetry.addData("vel x", drive.getVel().getX());
            telemetry.addData("vel y", drive.getVel().getY());
            telemetry.addData("vel heading", drive.getVel().getHeading());
            telemetry.update();

            counter++;
        }

        halt();
        telemetry.addData("MODE", "RESET");
        telemetry.addData("Avg Vel X STRAFE", avgXVel / counter);
        telemetry.addData("Avg Vel Y STRAFE", avgYVel / counter);
        telemetry.update();
        sleep(15000);
    }

    public void forward() {
        leftFront.setPower(1.0);
        leftRear.setPower(1.0);
        rightRear.setPower(1.0);
        rightFront.setPower(1.0);
    }

    public void strafeLeft() {
        leftFront.setPower(-1.0);
        leftRear.setPower(1.0);
        rightRear.setPower(-1.0);
        rightFront.setPower(1.0);
    }

    public void halt() {
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
        rightFront.setPower(0.0);
    }
}
