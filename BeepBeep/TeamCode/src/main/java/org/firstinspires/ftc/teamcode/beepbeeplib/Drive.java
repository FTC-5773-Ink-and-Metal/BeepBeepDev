package org.firstinspires.ftc.teamcode.beepbeeplib;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_y;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.BezierTraj;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.PIDController;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.TrajFollower;

@Config
public class Drive extends SampleMecanumDrive {
    PIDController pid_x = new PIDController(Kp_x, Ki_x, Kd_x);
    PIDController pid_y = new PIDController(Kp_y, Ki_y, Kd_y);
    PIDController pid_heading = new PIDController(Kp_heading, Ki_heading, Kd_heading);

    Telemetry telemetry;
    TrajFollower follower;

    public Drive(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap);
        this.telemetry = telemetry;
        this.follower = new TrajFollower(this, telemetry, pid_x, pid_y, pid_heading);
    }

    public void followTrajectory(BezierCurve bezier_x, BezierCurve bezier_y, double desired_heading, double time_factor) {
        follower.followBezier(bezier_x, bezier_y, pid_x, pid_y, pid_heading, desired_heading, time_factor);
    }

    public void followTrajectory(BezierTraj bezier) {
        follower.followBezier(bezier);
    }

    public void followTrajectory(double x, double y, double desired_heading) {
        follower.followLinear(x, y, desired_heading, pid_x, pid_y, pid_heading);
    }

    public void followTurn(double desired_heading, double start_heading, int direction) {
        follower.turn(desired_heading, start_heading, pid_x, pid_y, pid_heading, direction);
    }
}
