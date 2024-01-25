package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_y;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;
import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;

public class BezierTraj extends Trajectory {
    private double[] sums, upsilon;
    
    public Vector2d i1, i2;
    public BezierCurve x, y;
    public double desiredHeading;
    
    public BezierTraj(Pose2d startPose, Pose2d endPose,Vector2d i1, Vector2d i2) {
        super(startPose, endPose, "bezier");
        this.i1 = i1;
        this.i2 = i2;

        this.desiredHeading = endPose.getHeading();
        
        this.x = new BezierCurve(startPose.getX(), i1.getX(), i2.getX(), endPose.getX());
        this.y = new BezierCurve(startPose.getY(), i1.getY(), i2.getY(), endPose.getY());
    }

    public double bezier_get_x(double u) {
        return Math.pow((1 - u), 3) * startPose.getX() + 3 * u * Math.pow((1 - u), 2) * i1.getX() + 3 * Math.pow(u, 2) * (1 - u) * i2.getX() + Math.pow(u, 3) * endPose.getX();
    }

    public double bezier_deriv_x(double u) {
        return -3 * startPose.getX() * Math.pow((1 - u), 2) + 3 * i1.getX() * (3 * Math.pow(u, 2) - 4 * u + 1) + 3 * i2.getX() * (2 * u - 3 * Math.pow(u, 2)) + 3 * endPose.getX() * Math.pow(u, 2);
    }

    public double bezier_deriv2_x(double u) {
        return -6*startPose.getX()*u + 6*startPose.getX() + 18*i1.getX()*u - 12*i1.getX() - 18*i2.getX()*u + 6*i2.getX() + 6*endPose.getX()*u;
    }

    public double bezier_get_y(double u) {
        return Math.pow((1 - u), 3) * startPose.getX() + 3 * u * Math.pow((1 - u), 2) * i1.getX() + 3 * Math.pow(u, 2) * (1 - u) * i2.getX() + Math.pow(u, 3) * endPose.getX();
    }

    public double bezier_deriv_y(double u) {
        return -3 * startPose.getX() * Math.pow((1 - u), 2) + 3 * i1.getX() * (3 * Math.pow(u, 2) - 4 * u + 1) + 3 * i2.getX() * (2 * u - 3 * Math.pow(u, 2)) + 3 * endPose.getX() * Math.pow(u, 2);
    }

    public double bezier_deriv2_y(double u) {
        return -6*startPose.getX()*u + 6*startPose.getX() + 18*i1.getX()*u - 12*i1.getX() - 18*i2.getX()*u + 6*i2.getX() + 6*endPose.getX()*u;
    }

    public double[] get_sums() {
        return this.sums;
    }

    public double[] get_upsilon() {
        return this.upsilon;
    }

    public double bezier_param_of_disp(double s, double[] sums, double[] upsilon) {
        for (int i = 0; i < sums.length; i++) {
            if (s <= sums[i]) {
                return i / 100.0;
            }
        }

        return 0.0;
    }

    public double bezier_param_of_disp_deriv(double u) {
        double sum = Math.pow(x.bezier_deriv(u), 2) + Math.pow(y.bezier_deriv(u), 2);
        return 1.0 / (Math.sqrt(sum));
    }

    public double bezier_param_of_disp_deriv2(double u) {
        double dxdu = x.bezier_deriv(u);
        double dydu = y.bezier_deriv(u);
        double dx2du = x.bezier_deriv2(u);
        double dy2du = y.bezier_deriv2(u);

        return -1 * (dxdu*dx2du + dydu*dy2du) / (Math.pow(Math.pow(dxdu, 2) + Math.pow(dydu, 2), 2));
    }

    public double bezier_length() {
        double[] upsilon = new double[100];
        double[] integrand = new double[100];
        double[] sums = new double[100];
        for (int i = 0; i < 100; i += 1) {
            upsilon[i] = i / 100;
        }
        double dupsilon = 0.01;
        double last_sum = 0;

        for (int i = 0; i < 100; i++) {
            integrand[i] = Math.sqrt(Math.pow(x.bezier_deriv(i/100), 2) + Math.pow(y.bezier_deriv(i/100), 2));
            sums[i] = last_sum + integrand[i] * dupsilon;
            last_sum = sums[i];
        }
        this.sums = sums;
        this.upsilon = upsilon;

        return sums[99];
    }

    @Override
    public double curveLength() {
        double[] upsilon = new double[100];
        double[] integrand = new double[100];
        double[] sums = new double[100];
        for (int i = 0; i < 100; i += 1) {
            upsilon[i] = i / 100;
        }
        double dupsilon = 0.01;
        double last_sum = 0;

        for (int i = 0; i < 100; i++) {
            integrand[i] = Math.sqrt(Math.pow(x.bezier_deriv(i/100), 2) + Math.pow(y.bezier_deriv(i/100), 2));
            sums[i] = last_sum + integrand[i] * dupsilon;
            last_sum = sums[i];
        }
        this.sums = sums;
        this.upsilon = upsilon;

        return sums[99];
    }

    @Override
    public Vector2d calculatePow(MotionProfile motionProfile, double fullPathDistTravelled, double currTime, PIDController pid_x, PIDController pid_y, Pose2d currPose) {
        double instantTargetPosition = motionProfile.getPosition(currTime);
        double instantTargetVelocity = motionProfile.getVelocity(currTime);
        double instantTargetAcceleration = motionProfile.getAcceleration(currTime);
        double path_distance = motionProfile.getDistance();

        double u = bezier_param_of_disp(instantTargetPosition-fullPathDistTravelled, this.get_sums(), this.get_upsilon());
        double control_signal_x = pid_x.calculate(x.bezier_get(u), currPose.getX());
        double control_signal_y = pid_y.calculate(y.bezier_get(u), currPose.getY());

        // Velocity
        // s′(t)u′(s(t))x′(u(s(t)))
        double duds = bezier_param_of_disp_deriv(u);
        double dxdu = x.bezier_deriv(u);
        double dydu = y.bezier_deriv(u);
        double x_vel = dxdu * duds * instantTargetVelocity;
        double y_vel = dydu * duds * instantTargetVelocity;

        // Acceleration
        // (s'(t))^2 * (u'(s(t)))^2 * x''(u(s(t))) + ((s'(t))^2 * u''(s(t)) + s''(t) * u'(s(t))) * x'(u(s(t)))
        double du2ds = bezier_param_of_disp_deriv2(u);
        double dx2du = x.bezier_deriv2(u);
        double dy2du = y.bezier_deriv2(u);
        double x_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dx2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dxdu;
        double y_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dy2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dydu;

        // Combined
        double totalX = control_signal_x + x_vel * kV_x + x_accel * kS_x;
        double totalY = control_signal_y + y_vel * kV_y + y_accel * kS_y;

        return new Vector2d(totalX, totalY);
    }

    public double turn_direction() {
        double c_dist = 0, cc_dist = 0;

        if (startPose.getHeading() < endPose.getHeading()) {
            cc_dist = endPose.getHeading() - startPose.getHeading(); // 190
            c_dist = startPose.getHeading() + Math.toRadians(360) - endPose.getHeading();
        } else if (startPose.getHeading() > endPose.getHeading()) {
            c_dist = endPose.getHeading() + Math.toRadians(360) - startPose.getHeading();
            cc_dist = startPose.getHeading() - endPose.getHeading();
        }

        if (c_dist > cc_dist) {
            return 1;
        }

        return -1;
    }
}
