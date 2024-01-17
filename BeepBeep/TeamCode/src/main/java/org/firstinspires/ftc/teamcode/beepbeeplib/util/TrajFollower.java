package org.firstinspires.ftc.teamcode.beepbeeplib.util;

// Bezier path1 = new Bezier();
// drive.followPath(path1)
// trajFollower.followBezier(Bezier)

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAccel;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAngAccel;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAngVel;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxVel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurveCalc;
import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;

import java.util.Objects;

public class TrajFollower {
    Drive dt;
    Telemetry telemetry;


    public TrajFollower(Drive dt, Telemetry telemetry) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        this.dt = dt;
    }

    public void turn(double error, double desired_heading, PIDController px, PIDController py, PIDController pheading) {
        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;

        Pose2d poseEstimate = dt.getPoseEstimate();
        Pose2d desiredPose = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), desired_heading);
        MotionProfile motionProfileHeading = new MotionProfile(maxAngAccel, maxAngVel, desired_heading);

        Pose2d startPose = poseEstimate;

        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (!calcError(error, poseEstimate, desiredPose)) {
            poseEstimate = dt.getPoseEstimate();
            int motionMultiplier = 1;

            double instantTargetPositionHeading = motionProfileHeading.getPosition(timer.time());
            double velHeading = motionProfileHeading.getVelocity(timer.time());
            double accelHeading = motionProfileHeading.getAcceleration(timer.time());

            double powAng = motionMultiplier * (velHeading * kV_ang + accelHeading * kA_ang);
            powAng = powAng + kS_ang * powAng/Math.abs(powAng);

            if (motionProfileHeading.isFinished(timer.time())) {
                instantTargetPositionHeading = desired_heading;
                powAng = 0;
            }

            control_signal_x = px.calculate(0, poseEstimate.getX());
            control_signal_y = py.calculate(0, poseEstimate.getY());
            control_signal_heading = pheading.calculate((instantTargetPositionHeading), (poseEstimate.getHeading())) + powAng;

            Vector2d input = new Vector2d(
                    control_signal_x,
                    control_signal_y
            ).rotated(-poseEstimate.getHeading());

            dt.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            control_signal_heading
                    )
            );

            dt.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("x error", 0 - poseEstimate.getX());
            telemetry.addData("y error", 0 - poseEstimate.getY());
            telemetry.addData("heading error", desired_heading - poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void followLinear(double error, double desired_x, double desired_y, double desired_heading, PIDController px, PIDController py, PIDController pheading) {
        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;

        Pose2d poseEstimate = dt.getPoseEstimate();
        Pose2d desiredPose = new Pose2d(desired_x, desired_y, desired_heading);

        Pose2d startPose = poseEstimate;

        // desired_x = 10, desired_y = 10
        // poseX = 30, poseY = 40
        // desX - posX = -20; desY - posY = -30

        double path_distance = Math.sqrt(Math.pow(desired_x-poseEstimate.getX(), 2) + Math.pow(desired_y-poseEstimate.getY(), 2));
        double path_angle = Math.atan2(desired_y - poseEstimate.getY(), desired_x - poseEstimate.getX());
        path_angle = angleWrap(path_angle);
        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

        telemetry.addData("Path Distance", path_distance);
        telemetry.addData("Path Angle", path_angle);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.time();

        // calcError --> true when finished
        // motion prof --> true when finished
        int direction = 1;
        // !calcError(error, poseEstimate, desiredPose) &&
        //  && (motionProfile.getTotalTime() >= timer.time())

        while (!calcError(error, poseEstimate, desiredPose)) {
            double motionMultiplier = 1;
            direction *= -1;

            // REMOVE when not tuning
            px = new PIDController(Kp_x, Ki_x, Kd_x);
            py = new PIDController(Kp_y, Ki_y, Kd_y);
            pheading = new PIDController(Kp_heading, Ki_heading, Kd_heading);

            double instantTargetPosition = motionProfile.getPosition(timer.time());

            if (motionProfile.isFinished(timer.time())) {
                instantTargetPosition = path_distance;
            }
            double vel = motionProfile.getVelocity(timer.time());
            double accel = motionProfile.getAcceleration(timer.time());

            double xTargetPos = instantTargetPosition * Math.cos(path_angle) + startPose.getX();
            double xTargetVel = vel * Math.cos(path_angle);
            double xTargetAccel = accel * Math.cos(path_angle);

            double yTargetPos = instantTargetPosition * Math.sin(path_angle) + startPose.getY();
            double yTargetVel = vel * Math.sin(path_angle);
            double yTargetAccel = accel * Math.sin(path_angle);

            double powX = motionMultiplier * (xTargetVel * kV_x + xTargetAccel * kA_x);
            double powY = motionMultiplier * (yTargetVel * kV_y + yTargetAccel * kA_y);

            powX = powX + kS_x * powX/Math.abs(powX);
            powY = powY + kS_y * powY/Math.abs(powY);

            if (Double.isNaN(powX)) {
                powX = 0;
                xTargetPos = desired_x;
            }

            if (Double.isNaN(powY)) {
                powY = 0;
                yTargetPos = desired_y;
            }

            poseEstimate = dt.getPoseEstimate();

            control_signal_x = px.calculate(xTargetPos, poseEstimate.getX()) + powX;
            control_signal_y = py.calculate(yTargetPos, poseEstimate.getY()) + powY;
//            control_signal_x = px.calculate(desired_x, poseEstimate.getX());
//            control_signal_y = py.calculate(desired_y, poseEstimate.getY());
//            control_signal_x = powX;
//            control_signal_y = powY;
            control_signal_heading = pheading.calculate(angleWrap(desired_heading), angleWrap(poseEstimate.getHeading()));

            Vector2d input = new Vector2d(
                    control_signal_x,
                    control_signal_y
            ).rotated(-poseEstimate.getHeading());

            dt.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            control_signal_heading
                    )
            );

            dt.update();

            // Print pose to telemetry
//            Pose2d poseVelo = Objects.requireNonNull(dt.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
//            double currentVeloX = poseVelo.getX();
//            double currentVeloY = poseVelo.getY();

            // update telemetry
            telemetry.addData("y error", desired_y - poseEstimate.getY());
            telemetry.addData("x error", desired_x - poseEstimate.getX());
//            telemetry.addData("target x", 0);
//            telemetry.addData("target y", 0);
//            telemetry.addData("start x", 0);
//            telemetry.addData("start y", 0);
//            telemetry.addData("x aboslute error",  desired_x- poseEstimate.getX());
//            telemetry.addData("y aboslute error",  desired_y- poseEstimate.getY());
//            telemetry.addData("xPredicted",  xTargetPos);
//            telemetry.addData("targetVelocityX", xTargetVel);
//            telemetry.addData("targetVelocityY", yTargetVel);
//            telemetry.addData("measuredVelocityX", currentVeloX);
//            telemetry.addData("measuredVelocityY", currentVeloY);
//            telemetry.addData("xPredicted",  xTargetPos);
//            telemetry.addData("xCurrent", poseEstimate.getX());
//            telemetry.addData("yError", yTargetPos - poseEstimate.getY());
//            telemetry.addData("HeadingError", desired_heading - poseEstimate.getHeading());
//            telemetry.addData("error", direction*motionProfile.getVelocity(timer.time()) - currentVelo);
            telemetry.update();
        }
    }

    public void followBezier(double error, BezierCurve bezier_x, BezierCurve bezier_y, PIDController px, PIDController py, PIDController pheading, double desired_heading) {
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        double control_signal_x = 0, control_signal_y = 0, control_signal_heading = 0;
        double instantTargetPosition, u;
        double instantTargetVelocity, duds, dxdu, dydu, x_vel, y_vel;
        double instantTargetAcceleration, du2ds, dx2du, dy2du, x_accel, y_accel;
        double totalY, totalX;

        Pose2d poseEstimate = dt.getPoseEstimate();
        Pose2d desiredPose = new Pose2d(bezier_x.getD(), bezier_y.getD(), desired_heading);

        Pose2d startPose = poseEstimate;

        BezierCurveCalc bezier_calc = new BezierCurveCalc();
//        BezierCurve bezier_y_temp = new BezierCurve(bezier_x.getA()-startPose.getY(), bezier_x.getB()-startPose.getY(), bezier_x.getC()-startPose.getY(), bezier_x.getD()-startPose.getY());
//        BezierCurve bezier_x_temp = new BezierCurve(bezier_y.getA()-startPose.getX(), bezier_y.getB()-startPose.getX(), bezier_y.getC()-startPose.getX(), bezier_y.getD()-startPose.getX());
        double curve_length = bezier_calc.bezier_length(bezier_x, bezier_y);

        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, curve_length);

//        while(timer.time() >= motionProfile.getTotalTime() && )

        while(!calcError(error, poseEstimate, desiredPose) && motionProfile.getTotalTime() >= timer.time()) {
            poseEstimate = dt.getPoseEstimate();

            // Position
            instantTargetPosition = motionProfile.getPosition(timer.time());
            u = bezier_calc.bezier_param_of_disp(instantTargetPosition, bezier_calc.get_sums(), bezier_calc.get_upsilon());
            control_signal_x = px.calculate(bezier_x.bezier_get(u), poseEstimate.getX());
            control_signal_y = py.calculate(bezier_y.bezier_get(u), poseEstimate.getY());
            control_signal_heading = pheading.calculate(angleWrap(desired_heading), angleWrap(poseEstimate.getHeading()));

//            telemetry.addData("target x", bezier_x.bezier_get(u));
//            telemetry.addData("target y", bezier_y.bezier_get(u))   ;
//            telemetry.addData("start x", startPose.getX());
//            telemetry.addData("start y", startPose.getY());
//            telemetry.update();

            // Velocity
            // s′(t)u′(s(t))x′(u(s(t)))
            instantTargetVelocity = motionProfile.getVelocity(timer.time());
            duds = bezier_calc.bezier_param_of_disp_deriv(bezier_x, bezier_y, u);
            dxdu = bezier_x.bezier_deriv(u);
            dydu = bezier_y.bezier_deriv(u);
            x_vel = dxdu * duds * instantTargetVelocity;
            y_vel = dydu * duds * instantTargetVelocity;

            // Acceleration
            // (s'(t))^2 * (u'(s(t)))^2 * x''(u(s(t))) + ((s'(t))^2 * u''(s(t)) + s''(t) * u'(s(t))) * x'(u(s(t)))
            instantTargetAcceleration = motionProfile.getAcceleration(timer.time());
            du2ds = bezier_calc.bezier_param_of_disp_deriv2(bezier_x, bezier_y, u);
            dx2du = bezier_x.bezier_deriv2(u);
            dy2du = bezier_y.bezier_deriv2(u);
            x_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dx2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dxdu;
            y_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dy2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dydu;

            // Combined
            totalX = control_signal_x + x_vel * kV_x + x_accel * kS_x;
            totalY = control_signal_y + y_vel * kV_y + y_accel * kS_y;

            Vector2d input = new Vector2d(
                    totalX,
                    totalY
            ).rotated(-poseEstimate.getHeading());

            dt.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            control_signal_heading
                    )
            );

            dt.update();

            telemetry.addData("y error", bezier_y.getD() - poseEstimate.getY());
            telemetry.addData("x error", bezier_x.getD() - poseEstimate.getX());
            telemetry.update();
        }
    }

    private boolean calcError(double err, Pose2d curPose, Pose2d desiredPose) {
        double x = Math.pow(desiredPose.getX() - curPose.getX(), 2);
        double y = Math.pow(desiredPose.getY() - curPose.getY(), 2);
        double dist = Math.sqrt(x+y);

        double headingError = Math.abs(desiredPose.getHeading() - curPose.getHeading());

        if (dist <= err && headingError < Math.toRadians(3)) return true;

        return false;
    }

    private double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}
