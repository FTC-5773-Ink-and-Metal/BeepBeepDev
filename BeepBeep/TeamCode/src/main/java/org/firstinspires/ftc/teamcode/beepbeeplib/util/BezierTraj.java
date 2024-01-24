package org.firstinspires.ftc.teamcode.beepbeeplib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;

public class BezierTraj extends Trajectory {
    private double[] sums, upsilon;
    
    Vector2d i1, i2;
    BezierCurve x, y;
    
    public BezierTraj(Pose2d startPose, Pose2d endPose,Vector2d i1, Vector2d i2) {
        super(startPose, endPose);
        this.i1 = i1;
        this.i2 = i2;
        
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
}
