package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_y;
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurveCalc;
import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;

import java.util.Objects;

public class OldTrajFollower {
//    public void followBezier(BezierCurve bezier_x, BezierCurve bezier_y, PIDController px, PIDController py, PIDController pheading, double desired_heading, double time_factor) {
//        ElapsedTime timer = new ElapsedTime();
//        timer.time();
//
//        double control_signal_x = 0, control_signal_y = 0, control_signal_heading = 0;
//        double instantTargetPosition, u;
//        double instantTargetVelocity, duds, dxdu, dydu, x_vel, y_vel;
//        double instantTargetAcceleration, du2ds, dx2du, dy2du, x_accel, y_accel;
//        double totalY, totalX;
//
//        Pose2d poseEstimate = dt.getPoseEstimate();
//        Pose2d desiredPose = new Pose2d(bezier_x.getD(), bezier_y.getD(), desired_heading);
//
////        Pose2d startPose = poseEstimate;
//        double startHeading = poseEstimate.getHeading();
//        desired_heading = desired_heading - startHeading;
//        double direction = getTurnDirection(startHeading, desired_heading);
//        double angularDisplacement = correctAngle(desired_heading*direction);
//        MotionProfile motionProfileHeading = new MotionProfile(maxAngAccel, maxAngVel, angularDisplacement); // 270 becomes 90
//        double target = angleWrap(desired_heading);
//
//        BezierCurveCalc bezier_calc = new BezierCurveCalc();
////        BezierCurve bezier_y_temp = new BezierCurve(bezier_x.getA()-startPose.getY(), bezier_x.getB()-startPose.getY(), bezier_x.getC()-startPose.getY(), bezier_x.getD()-startPose.getY());
////        BezierCurve bezier_x_temp = new BezierCurve(bezier_y.getA()-startPose.getX(), bezier_y.getB()-startPose.getX(), bezier_y.getC()-startPose.getX(), bezier_y.getD()-startPose.getX());
//        double curve_length = bezier_calc.bezier_length(bezier_x, bezier_y);
//
//        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, curve_length);
//
////        while(timer.time() >= motionProfile.getTotalTime() && )
//        //  && motionProfile.getTotalTime() >= 1.5 * timer.time()
//
//        while (true) {
//            if (isMotionless() && atTarget(poseEstimate, desiredPose)) {
//                break;
//            }
//
//            poseEstimate = dt.getPoseEstimate();
//
//            if (Objects.isNull(dt.getPoseVelocity())) {
//                telemetry.addData("calc vel", 0);
//            } else {
//                Pose2d poseVelo = dt.getPoseVelocity();
//
//                double currentVeloX = poseVelo.getX();
//                double currentVeloY = poseVelo.getY();
//
//                double totalVelo = Math.sqrt(Math.pow(currentVeloX, 2) + Math.pow(currentVeloY, 2));
//                telemetry.addData("calc vel", totalVelo);
//            }
//
//            //Heading
//            double instantTargetPositionHeading = direction * motionProfileHeading.getPosition(timer.time());
//            double velHeading = motionProfileHeading.getVelocity(timer.time());
//            double accelHeading = motionProfileHeading.getAcceleration(timer.time());
//
//            double powAng = (velHeading * kV_ang + accelHeading * kA_ang);
//            powAng = direction * (powAng + kS_ang * powAng);
//            double currHeading = angleWrap(poseEstimate.getHeading() - startHeading);
//
//            if (motionProfileHeading.isFinished(timer.time())) {
////                telemetry.addData("In if condition", 1);
//                if (Math.abs(angularDisplacement) == Math.toRadians(180)) {
//                    instantTargetPositionHeading = Math.toRadians(180) * currHeading/Math.abs(currHeading);
//                } else {
//                    instantTargetPositionHeading = target;
//                }
//            }
//
//            // Position
//            instantTargetPosition = motionProfile.getPosition(time_factor * timer.time());
//            u = bezier_calc.bezier_param_of_disp(instantTargetPosition, bezier_calc.get_sums(), bezier_calc.get_upsilon());
//            control_signal_x = px.calculate(bezier_x.bezier_get(u), poseEstimate.getX());
//            control_signal_y = py.calculate(bezier_y.bezier_get(u), poseEstimate.getY());
//            control_signal_heading = pheading.calculate(instantTargetPositionHeading, currHeading) + powAng;
//
////            telemetry.addData("target x", bezier_x.bezier_get(u));
////            telemetry.addData("target y", bezier_y.bezier_get(u))   ;
////            telemetry.addData("start x", startPose.getX());
////            telemetry.addData("start y", startPose.getY());
////            telemetry.update();
//
//            // Velocity
//            // s′(t)u′(s(t))x′(u(s(t)))
//            instantTargetVelocity = motionProfile.getVelocity(time_factor * timer.time());
//            duds = bezier_calc.bezier_param_of_disp_deriv(bezier_x, bezier_y, u);
//            dxdu = bezier_x.bezier_deriv(u);
//            dydu = bezier_y.bezier_deriv(u);
//            x_vel = dxdu * duds * instantTargetVelocity;
//            y_vel = dydu * duds * instantTargetVelocity;
//
//            // Acceleration
//            // (s'(t))^2 * (u'(s(t)))^2 * x''(u(s(t))) + ((s'(t))^2 * u''(s(t)) + s''(t) * u'(s(t))) * x'(u(s(t)))
//            instantTargetAcceleration = motionProfile.getAcceleration(time_factor * timer.time());
//            du2ds = bezier_calc.bezier_param_of_disp_deriv2(bezier_x, bezier_y, u);
//            dx2du = bezier_x.bezier_deriv2(u);
//            dy2du = bezier_y.bezier_deriv2(u);
//            x_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dx2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dxdu;
//            y_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dy2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dydu;
//
//            // Combined
//            totalX = control_signal_x + x_vel * kV_x + x_accel * kS_x; // add control signals
//            totalY = control_signal_y + y_vel * kV_y + y_accel * kS_y;
//
//
//            Vector2d input = new Vector2d(
//                    totalX,
//                    totalY
//            ).rotated(-poseEstimate.getHeading());
//
//            dt.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX(),
//                            input.getY(),
//                            control_signal_heading
//                    )
//            );
//
//            dt.update();
//
//            telemetry.addData("y error", bezier_y.getD() - poseEstimate.getY());
//            telemetry.addData("x error", bezier_x.getD() - poseEstimate.getX());
//            telemetry.addData("heading error", desired_heading-poseEstimate.getHeading());
////            telemetry.addData("Error to Inst. Pos", Math.sqrt(Math.pow((bezier_x.bezier_get(u) - poseEstimate.getX()), 2) + (Math.pow((bezier_y.bezier_get(u) - poseEstimate.getY()), 2))));
////            if (Objects.isNull(dt.getPoseVelocity())) {
////                telemetry.addData("Error to Inst. Vel", 0);
////            } else {
////                Pose2d poseVelo = dt.getPoseVelocity();
////
////                double currentVeloX = poseVelo.getX();
////                double currentVeloY = poseVelo.getY();
////
//////                double totalVelo = Math.sqrt(Math.pow(currentVeloX, 2) + Math.pow(currentVeloY, 2));
////                telemetry.addData("Error to Inst. Vel", Math.sqrt(Math.pow((x_vel - currentVeloX), 2) + (Math.pow((x_vel - currentVeloY), 2))));
////            }
////            telemetry.addData("Battery Voltage", dt.getVoltage());
//            telemetry.update();
//            telemetry.update();
//        }
//    }
//public void followLinear(double desired_x, double desired_y, double desired_heading, PIDController px, PIDController py, PIDController pheading) {
//    double control_signal_x = 0;
//    double control_signal_y = 0;
//    double control_signal_heading = 0;
//
//    boolean pathBroken = false;
//    double pauseTime = 0;
//
//    Pose2d poseEstimate = dt.getPoseEstimate();
//    Pose2d desiredPose = new Pose2d(desired_x, desired_y, desired_heading);
//
//    Pose2d startPose = poseEstimate;
//
//    double startHeading = poseEstimate.getHeading();
//    desired_heading = desired_heading - startHeading;
//    double direction = getTurnDirection(startHeading, desired_heading);
//    double angularDisplacement = correctAngle(desired_heading*direction);
//    MotionProfile motionProfileHeading = new MotionProfile(maxAngAccel, maxAngVel, angularDisplacement); // 270 becomes 90
//    double target = angleWrap(desired_heading);
//
//    // desired_x = 10, desired_y = 10
//    // poseX = 30, poseY = 40
//    // desX - posX = -20; desY - posY = -30
//
//    double path_distance = Math.sqrt(Math.pow(desired_x-poseEstimate.getX(), 2) + Math.pow(desired_y-poseEstimate.getY(), 2));
//    double path_angle = Math.atan2(desired_y - poseEstimate.getY(), desired_x - poseEstimate.getX());
//    path_angle = angleWrap(path_angle);
//    MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);
//
//    telemetry.addData("Path Distance", path_distance);
//    telemetry.addData("Path Angle", path_angle);
//    telemetry.update();
//
//    ElapsedTime timer = new ElapsedTime();
//    timer.time();
//
//    // calcError --> true when finished
//    // motion prof --> true when finished
////        int direction = 1;
//    // !calcError(error, poseEstimate, desiredPose) &&
//    //  && (motionProfile.getTotalTime() >= timer.time())
//    while (true) {
//        if (isMotionless() && atTarget(poseEstimate, desiredPose)) {
//            break;
//        }
//        double motionMultiplier = 1;
////            direction *= -1;
//
//        // REMOVE when not tuning
//        px = new PIDController(Kp_x, Ki_x, Kd_x);
//        py = new PIDController(Kp_y, Ki_y, Kd_y);
//        pheading = new PIDController(Kp_heading, Ki_heading, Kd_heading);
//
//        double currTime = timer.time();
//
////            if (pathBroken) {
//////                timer = new ElapsedTime((long) pauseTime);
////                currTime = pauseTime;
////            }
//
//        double instantTargetPosition = motionProfile.getPosition(currTime);
//        double xTargetPos = instantTargetPosition * Math.cos(path_angle) + startPose.getX();
//        double yTargetPos = instantTargetPosition * Math.sin(path_angle) + startPose.getY();
//
////            if (pathSaverLinear > Math.sqrt(Math.pow((xTargetPos - poseEstimate.getX()), 2) + (Math.pow((yTargetPos - poseEstimate.getY()), 2))) && pathBroken) {
////                pathBroken = false;
////                timer = new ElapsedTime((long) pauseTime);
////                currTime = timer.time();
////            }
//
////            instantTargetPosition = motionProfile.getPosition(currTime);
////            xTargetPos = instantTargetPosition * Math.cos(path_angle) + startPose.getX();
////            yTargetPos = instantTargetPosition * Math.sin(path_angle) + startPose.getY();
//
////            double instantTargetPosition = motionProfile.getPosition(currTime);
//
//        double instantTargetPositionHeading = direction * motionProfileHeading.getPosition(currTime);
//        double velHeading = motionProfileHeading.getVelocity(currTime);
//        double accelHeading = motionProfileHeading.getAcceleration(currTime);
//
//        double powAng = (velHeading * kV_ang + accelHeading * kA_ang);
//        powAng = direction * (powAng + kS_ang * powAng);
//        double currHeading = angleWrap(poseEstimate.getHeading() - startHeading);
//
//        if (motionProfile.isFinished(currTime)) {
//            instantTargetPosition = path_distance;
//        }
//
////            telemetry.addData("Sign", currHeading/Math.abs(currHeading));
////            telemetry.addData("currHeading", currHeading);
////            telemetry.addData("In if condition", 0);
//
//        if (motionProfileHeading.isFinished(timer.time())) {
////                telemetry.addData("In if condition", 1);
//            if (Math.abs(angularDisplacement) == Math.toRadians(180)) {
//                instantTargetPositionHeading = Math.toRadians(180) * currHeading/Math.abs(currHeading);
//            } else {
//                instantTargetPositionHeading = target;
//            }
//        }
//
//        double vel = motionProfile.getVelocity(currTime);
//        double accel = motionProfile.getAcceleration(currTime);
//
////            double xTargetPos = instantTargetPosition * Math.cos(path_angle) + startPose.getX();
//        double xTargetVel = vel * Math.cos(path_angle);
//        double xTargetAccel = accel * Math.cos(path_angle);
//
////            double yTargetPos = instantTargetPosition * Math.sin(path_angle) + startPose.getY();
//        double yTargetVel = vel * Math.sin(path_angle);
//        double yTargetAccel = accel * Math.sin(path_angle);
//
//        double powX = motionMultiplier * (xTargetVel * kV_x + xTargetAccel * kA_x);
//        double powY = motionMultiplier * (yTargetVel * kV_y + yTargetAccel * kA_y);
//
//        powX = powX + kS_x * powX/Math.abs(powX);
//        powY = powY + kS_y * powY/Math.abs(powY);
//
//        if (Double.isNaN(powX)) {
//            powX = 0;
//            xTargetPos = desired_x;
//        }
//
//        if (Double.isNaN(powY)) {
//            powY = 0;
//            yTargetPos = desired_y;
//        }
//
//        poseEstimate = dt.getPoseEstimate();
//
//        if (pathBreakerLinear < Math.sqrt(Math.pow((xTargetPos - poseEstimate.getX()), 2) + (Math.pow((yTargetPos - poseEstimate.getY()), 2))) && !pathBroken) {
//            pathBroken = true;
//            powX = 0;
//            xTargetPos = poseEstimate.getX();
//            powY = 0;
//            yTargetPos = poseEstimate.getY();
//            instantTargetPositionHeading = poseEstimate.getHeading();
//            pauseTime = timer.time();
//        }
//
//        control_signal_x = px.calculate(xTargetPos, poseEstimate.getX()) + powX;
//        control_signal_y = py.calculate(yTargetPos, poseEstimate.getY()) + powY;
////            control_signal_x = px.calculate(desired_x, poseEstimate.getX());
////            control_signal_y = py.calculate(desired_y, poseEstimate.getY());
////            control_signal_x = powX;
////            control_signal_y = powY;
////            control_signal_heading = pheading.calculate(angleWrap(desired_heading), angleWrap(poseEstimate.getHeading()));
//        control_signal_heading = pheading.calculate(instantTargetPositionHeading, currHeading) + powAng;
//
//        Vector2d input = new Vector2d(
//                control_signal_x,
//                control_signal_y
//        ).rotated(-poseEstimate.getHeading());
//
//        dt.setWeightedDrivePower(
//                new Pose2d(
//                        input.getX(),
//                        input.getY(),
//                        control_signal_heading
//                )
//        );
//
//        dt.update();
//
//        // Print pose to telemetry
////            Pose2d poseVelo = Objects.requireNonNull(dt.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
////            double currentVeloX = poseVelo.getX();
////            double currentVeloY = poseVelo.getY();
//
//        // update telemetry
////            telemetry.addData("y error", desired_y - poseEstimate.getY());
////            telemetry.addData("x error", desired_x - poseEstimate.getX());
////            telemetry.addData("heading error", desired_heading - poseEstimate.getHeading());
////            telemetry.addData("target x", 0);
////            telemetry.addData("target y", 0);
////            telemetry.addData("start x", 0);
////            telemetry.addData("start y", 0);
////            telemetry.addData("x aboslute error",  desired_x- poseEstimate.getX());
////            telemetry.addData("y aboslute error",  desired_y- poseEstimate.getY());
////            telemetry.addData("xPredicted",  xTargetPos);
////            telemetry.addData("targetVelocityX", xTargetVel);
////            telemetry.addData("targetVelocityY", yTargetVel);
////            telemetry.addData("measuredVelocityX", currentVeloX);
////            telemetry.addData("measuredVelocityY", currentVeloY);
////            telemetry.addData("xPredicted",  xTargetPos);
////            telemetry.addData("xCurrent", poseEstimate.getX());
////            telemetry.addData("yError", yTargetPos - poseEstimate.getY());
////            telemetry.addData("HeadingError", desired_heading - poseEstimate.getHeading());
////            telemetry.addData("error", direction*motionProfile.getVelocity(timer.time()) - currentVelo);
////            telemetry.addData("Error to Inst. Pos", Math.sqrt(Math.pow((xTargetPos - poseEstimate.getX()), 2) + (Math.pow((yTargetPos - poseEstimate.getY()), 2))));
////            if (Objects.isNull(dt.getPoseVelocity())) {
////                telemetry.addData("Error to Inst. Vel", 0);
////            } else {
////                Pose2d poseVelo = dt.getPoseVelocity();
////
////                double currentVeloX = poseVelo.getX();
////                double currentVeloY = poseVelo.getY();
////
//////                double totalVelo = Math.sqrt(Math.pow(currentVeloX, 2) + Math.pow(currentVeloY, 2));
////                telemetry.addData("Error to Inst. Vel", Math.sqrt(Math.pow((xTargetVel - currentVeloX), 2) + (Math.pow((yTargetVel - currentVeloY), 2))));
////            }
////            telemetry.addData("pathBroken", (pathBroken) ? 1 : 0);
////            telemetry.addData("Battery Voltage", dt.getVoltage());
////            telemetry.addData("pathBroken", pathBroken);
//        telemetry.update();
//    }
//}

}