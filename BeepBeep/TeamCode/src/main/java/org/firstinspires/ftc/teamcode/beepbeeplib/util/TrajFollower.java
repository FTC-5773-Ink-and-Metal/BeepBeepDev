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
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.angular_error;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_y;
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
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.pathBreakerLinear;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.pathSaverLinear;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.translational_error;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.velo_error;

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

import java.util.ArrayList;
import java.util.Objects;

public class TrajFollower {
    Drive dt;
    Telemetry telemetry;
    PIDController pX, pY, pHeading;


    public TrajFollower(Drive dt, Telemetry telemetry, PIDController px, PIDController py, PIDController pheading) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        this.dt = dt;

        this.pX = px; this.pY = py; this.pHeading = pheading;
    }

    /**
     * @param desired_heading
     * @param px
     * @param py
     * @param pheading
     * @param direction - negative = clockwise, positive = counter-clockwise
     */
    public void turn(double desired_heading, double start_heading, PIDController px, PIDController py, PIDController pheading, int direction) {
        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;

        // remove after testing done
//        dt.setPoseEstimate(new Pose2d(dt.getPoseEstimate().getX(), dt.getPoseEstimate().getY(), start_heading));

        Pose2d poseEstimate = dt.getPoseEstimate();
        Pose2d desiredPose = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), desired_heading);
        double startHeading = start_heading;

        desired_heading = desired_heading - startHeading;
        double angularDisplacement = correctAngle(desired_heading*direction);

//        telemetry.addData("angularDisplacement", angularDisplacement);
//        telemetry.update();

        MotionProfile motionProfileHeading = new MotionProfile(maxAngAccel, maxAngVel, angularDisplacement); // 270 becomes 90

        double target = angleWrap(desired_heading);

        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (true) {
            poseEstimate = dt.getPoseEstimate();

            if (isMotionless() && atTarget(poseEstimate, desiredPose)) {
                break;
            }

            double instantTargetPositionHeading = direction * motionProfileHeading.getPosition(timer.time());
            double velHeading = motionProfileHeading.getVelocity(timer.time());
            double accelHeading = motionProfileHeading.getAcceleration(timer.time());

            double powAng = (velHeading * kV_ang + accelHeading * kA_ang);
            powAng = direction * (powAng + kS_ang * powAng);

            control_signal_x = px.calculate(desiredPose.getX(), poseEstimate.getX());
            control_signal_y = py.calculate(desiredPose.getY(), poseEstimate.getY());

            double currHeading = angleWrap(poseEstimate.getHeading() - start_heading);

//            if (motionProfileHeading.isFinished(timer.time())) {
//                instantTargetPositionHeading = target;
//            }

            if (motionProfileHeading.isFinished(timer.time())) {
//                telemetry.addData("In if condition", 1);
                if (Math.abs(angularDisplacement) == Math.toRadians(180)) {
                    instantTargetPositionHeading = Math.toRadians(180) * currHeading/Math.abs(currHeading);
                } else {
                    instantTargetPositionHeading = target;
                }
            }

            control_signal_heading = pheading.calculate(instantTargetPositionHeading, currHeading) + powAng;

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
            telemetry.addData("instant heading pos", (instantTargetPositionHeading));
            telemetry.addData("currHeading", currHeading);
            telemetry.addData("PID output", control_signal_heading-powAng*direction);
            telemetry.addData("FF output", powAng*direction);
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("heading error", desired_heading - poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void followLinear(LinearTraj linear) {
        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;

        boolean pathBroken = false;
        double pauseTime = 0;

        Pose2d poseEstimate = dt.getPoseEstimate();
        Pose2d desiredPose = linear.endPose;

        Pose2d startPose = poseEstimate;

        double startHeading = poseEstimate.getHeading();
        double desired_heading = desiredPose.getHeading() - startHeading;

        double direction = linear.turn_direction();
        double angularDisplacement = correctAngle(desired_heading*direction);
        MotionProfile motionProfileHeading = new MotionProfile(maxAngAccel, maxAngVel, angularDisplacement); // 270 becomes 90
        double target = angleWrap(desired_heading);

        // desired_x = 10, desired_y = 10
        // poseX = 30, poseY = 40
        // desX - posX = -20; desY - posY = -30

        double path_distance = linear.curveLength();
        double path_angle = linear.path_angle();
        path_angle = angleWrap(path_angle);
        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

        telemetry.addData("Path Distance", path_distance);
        telemetry.addData("Path Angle", path_angle);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (true) {
            if (isMotionless() && atTarget(poseEstimate, desiredPose)) {
                break;
            }
            double motionMultiplier = 1;
            double currTime = timer.time();

            double instantTargetPosition = motionProfile.getPosition(currTime);
            double xTargetPos = instantTargetPosition * Math.cos(path_angle) + startPose.getX();
            double yTargetPos = instantTargetPosition * Math.sin(path_angle) + startPose.getY();

            double instantTargetPositionHeading = direction * motionProfileHeading.getPosition(currTime);
            double velHeading = motionProfileHeading.getVelocity(currTime);
            double accelHeading = motionProfileHeading.getAcceleration(currTime);

            double powAng = (velHeading * kV_ang + accelHeading * kA_ang);
            powAng = direction * (powAng + kS_ang * powAng);
            double currHeading = angleWrap(poseEstimate.getHeading() - startHeading);

            if (motionProfile.isFinished(currTime)) {
                instantTargetPosition = path_distance;
            }

            if (motionProfileHeading.isFinished(timer.time())) {
                if (Math.abs(angularDisplacement) == Math.toRadians(180)) {
                    instantTargetPositionHeading = Math.toRadians(180) * currHeading/Math.abs(currHeading);
                } else {
                    instantTargetPositionHeading = target;
                }
            }

            double vel = motionProfile.getVelocity(currTime);
            double accel = motionProfile.getAcceleration(currTime);

            double xTargetVel = vel * Math.cos(path_angle);
            double xTargetAccel = accel * Math.cos(path_angle);

            double yTargetVel = vel * Math.sin(path_angle);
            double yTargetAccel = accel * Math.sin(path_angle);

            double powX = motionMultiplier * (xTargetVel * kV_x + xTargetAccel * kA_x);
            double powY = motionMultiplier * (yTargetVel * kV_y + yTargetAccel * kA_y);

            powX = powX + kS_x * powX/Math.abs(powX);
            powY = powY + kS_y * powY/Math.abs(powY);

            if (Double.isNaN(powX)) {
                powX = 0;
                xTargetPos = desiredPose.getX();
            }

            if (Double.isNaN(powY)) {
                powY = 0;
                yTargetPos = desiredPose.getY();
            }

            poseEstimate = dt.getPoseEstimate();

            control_signal_x = pX.calculate(xTargetPos, poseEstimate.getX()) + powX;
            control_signal_y = pY.calculate(yTargetPos, poseEstimate.getY()) + powY;
            control_signal_heading = pHeading.calculate(instantTargetPositionHeading, currHeading) + powAng;

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
            telemetry.update();
        }
    }

    public void followBezier(BezierTraj bz) {
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        double control_signal_x = 0, control_signal_y = 0, control_signal_heading = 0;
        double instantTargetPosition, u;
        double instantTargetVelocity, duds, dxdu, dydu, x_vel, y_vel;
        double instantTargetAcceleration, du2ds, dx2du, dy2du, x_accel, y_accel;
        double totalY, totalX;

        Pose2d poseEstimate = dt.getPoseEstimate();
        Pose2d desiredPose = new Pose2d(bz.x.getD(), bz.y.getD(), bz.desiredHeading);

        double startHeading = poseEstimate.getHeading();
        double desired_heading = bz.desiredHeading - startHeading;
        double direction = bz.turn_direction();
        double angularDisplacement = correctAngle(desired_heading*direction);
        MotionProfile motionProfileHeading = new MotionProfile(maxAngAccel, maxAngVel, angularDisplacement); // 270 becomes 90
        double target = angleWrap(desired_heading);

        double curve_length = bz.bezier_length();

        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, curve_length);

        while (true) {
//            if (isMotionless() && atTarget(poseEstimate, desiredPose)) {
//                break;
//            }
            poseEstimate = dt.getPoseEstimate();

            if(!Objects.isNull(dt.getPoseVelocity())) {
                Pose2d poseVelo = dt.getPoseVelocity();

                double currentVeloX = poseVelo.getX();
                double currentVeloY = poseVelo.getY();

                double totalVelo = Math.sqrt(Math.pow(currentVeloX, 2) + Math.pow(currentVeloY, 2));
                telemetry.addData("calc vel", totalVelo);
            } else telemetry.addData("calc vel", 0);

            double instantTargetPositionHeading = direction * motionProfileHeading.getPosition(timer.time());
            double velHeading = motionProfileHeading.getVelocity(timer.time());
            double accelHeading = motionProfileHeading.getAcceleration(timer.time());

            double powAng = (velHeading * kV_ang + accelHeading * kA_ang);
            powAng = direction * (powAng + kS_ang * powAng);
            double currHeading = angleWrap(poseEstimate.getHeading() - startHeading);

            if (motionProfileHeading.isFinished(timer.time())) {
                if (Math.abs(angularDisplacement) == Math.toRadians(180)) instantTargetPositionHeading = Math.toRadians(180) * currHeading/Math.abs(currHeading);
                else instantTargetPositionHeading = target;
            }

            instantTargetPosition = motionProfile.getPosition(timer.time());
            u = bz.bezier_param_of_disp(instantTargetPosition, bz.get_sums(), bz.get_upsilon());
            control_signal_x = pX.calculate(bz.x.bezier_get(u), poseEstimate.getX());
            control_signal_y = pY.calculate(bz.y.bezier_get(u), poseEstimate.getY());
            control_signal_heading = pHeading.calculate(instantTargetPositionHeading, currHeading) + powAng;

            instantTargetVelocity = motionProfile.getVelocity(timer.time());
            duds = bz.bezier_param_of_disp_deriv(u);
            dxdu = bz.x.bezier_deriv(u);
            dydu = bz.y.bezier_deriv(u);
            x_vel = dxdu * duds * instantTargetVelocity;
            y_vel = dydu * duds * instantTargetVelocity;

            // Acceleration
            // (s'(t))^2 * (u'(s(t)))^2 * x''(u(s(t))) + ((s'(t))^2 * u''(s(t)) + s''(t) * u'(s(t))) * x'(u(s(t)))
            instantTargetAcceleration = motionProfile.getAcceleration(timer.time());
            du2ds = bz.bezier_param_of_disp_deriv2(u);
            dx2du = bz.x.bezier_deriv2(u);
            dy2du = bz.y.bezier_deriv2(u);
            x_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dx2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dxdu;
            y_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dy2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dydu;

            // Combined
            totalX = control_signal_x + x_vel * kV_x + x_accel * kS_x; // add control signals
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
        }
    }

    public void followTrajSequence(ArrayList<Trajectory> trajs) {
        double totalLength = 0;
        boolean totalPathFinished = false;
        for (Trajectory t : trajs) totalLength += t.curveLength();

        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, totalLength);
        MotionProfile motionProfileAng = new MotionProfile(maxAngAccel, maxAngVel, Math.toRadians(trajs.get(trajs.size() - 1).endPose.getHeading()));

        Pose2d endPose = trajs.get(trajs.size()-1).endPose;
        Pose2d poseEstimate = dt.getPoseEstimate();
        double fullPathDistTravelled = 0;
        int currPath = 0;

        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (true) {
            if (isMotionless() && atTarget(poseEstimate, endPose)) {
                telemetry.addData("Is in loop 3", 1);
                telemetry.update();
                break;
            }
            poseEstimate = dt.getPoseEstimate();

            double instantTargetPosition = motionProfile.getPosition(timer.time());
            if (totalPathFinished) instantTargetPosition = totalLength;

            Trajectory currTraj = trajs.get(currPath);
            telemetry.addData("currPath", currPath);

            if (!totalPathFinished && instantTargetPosition >= currTraj.curveLength()+fullPathDistTravelled) {
                telemetry.addData("Is in loop 1", 1);
                currPath++;
                telemetry.addData("currPath", currPath);
                if (currPath >= trajs.size()) {
                    telemetry.addData("Is in loop 2", 1);
                    telemetry.update();
                    totalPathFinished = true;
                }
                fullPathDistTravelled = 0;
                for (int i = 0; i < currPath; i++) fullPathDistTravelled += trajs.get(i).curveLength();
                currTraj = trajs.get(currPath);
            }

            telemetry.addData("fullPathDistTravelled", fullPathDistTravelled);
            telemetry.addData("motionProfile.getPosition(currTime) - fullPathDistTravelled", motionProfile.getPosition(timer.time()) - fullPathDistTravelled);
            telemetry.addData("totalPathFinished", totalPathFinished);


            Vector2d pow = currTraj.calculatePow(motionProfile, fullPathDistTravelled, timer.time(), pX, pY, poseEstimate);
            double totalX = pow.getX();
            double totalY = pow.getY();

            if (totalPathFinished) {
                totalX = pX.calculate(endPose.getX(), poseEstimate.getX());
                totalY = pY.calculate(endPose.getY(), poseEstimate.getY());
            }

            Vector2d input = new Vector2d(
                    totalX,
                    totalY
            ).rotated(-poseEstimate.getHeading());

            dt.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            0
                    )
            );

            dt.update();

            telemetry.addData("powX", totalX);
            telemetry.update();
        }
    }

    private boolean atTarget(Pose2d curPose, Pose2d desiredPose) {
        double x = Math.pow(desiredPose.getX() - curPose.getX(), 2);
        double y = Math.pow(desiredPose.getY() - curPose.getY(), 2);
        double dist = Math.sqrt(x+y);

        double headingError = Math.abs(desiredPose.getHeading() - curPose.getHeading());

        return dist <= translational_error && headingError < angular_error;
    }

    private boolean isMotionless() {
        if (Objects.isNull(dt.getPoseVelocity())) {
            return false;
        }

        Pose2d poseVelo = dt.getPoseVelocity();

        double currentVeloX = poseVelo.getX();
        double currentVeloY = poseVelo.getY();

        double totalVelo = Math.sqrt(Math.pow(currentVeloX, 2) + Math.pow(currentVeloY, 2));

        return totalVelo < velo_error;
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

    private double correctAngle(double radians) {
        while (radians < 0) {
            radians += 2*Math.PI;
        }

        while (radians >= 2*Math.PI) {
            radians -= 2*Math.PI;
        }

        return radians;
    }

    private double getTurnDirection(double curr, double target) {
        double c_dist = 0, cc_dist = 0;

        if (curr < target) {
            cc_dist = target - curr; // 190
            c_dist = curr + Math.toRadians(360) - target;
        } else if (curr > target) {
            c_dist = target + Math.toRadians(360) - curr;
            cc_dist = curr - target;
        }

        if (c_dist > cc_dist) {
            return 1;
        }

        return -1;
    }
}
