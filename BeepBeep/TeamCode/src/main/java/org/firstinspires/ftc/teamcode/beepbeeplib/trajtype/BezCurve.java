package org.firstinspires.ftc.teamcode.beepbeeplib.trajtype;

import org.firstinspires.ftc.teamcode.beepbeep.BezierCurve;

public class BezCurve {
    private double a, b, c, d;
    private double[] sums, upsilon;

    /**
     * @param x1 - start
     * @param x2 - interp1
     * @param x3 - interp2
     * @param x4 - end
     */
    public BezCurve(double x1, double x2, double x3, double x4) {
        this.a = x1;
        this.b = x2;
        this.c = x3;
        this.d = x4;
    }

    public double bezier_get(double u) {
        return Math.pow((1 - u), 3) * a + 3 * u * Math.pow((1 - u), 2) * b + 3 * Math.pow(u, 2) * (1 - u) * c + Math.pow(u, 3) * d;
    }

    public double bezier_deriv(double u) {
        return -3 * a * Math.pow((1 - u), 2) + 3 * b * (3 * Math.pow(u, 2) - 4 * u + 1) + 3 * c * (2 * u - 3 * Math.pow(u, 2)) + 3 * d * Math.pow(u, 2);
    }

    public double bezier_deriv2(double u) {
        return -6 * a * u + 6 * a + 18 * b * u - 12 * b - 18 * c * u + 6 * c + 6 * d * u;
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

    public double bezier_param_of_disp_deriv(BezierCurve x_bezier, BezierCurve y_bezier, double u) {
        double sum = Math.pow(x_bezier.bezier_deriv(u), 2) + Math.pow(y_bezier.bezier_deriv(u), 2);
        return 1.0 / (Math.sqrt(sum));
    }

    public double bezier_param_of_disp_deriv2(BezierCurve x_bezier, BezierCurve y_bezier, double u) {
        double dxdu = x_bezier.bezier_deriv(u);
        double dydu = y_bezier.bezier_deriv(u);
        double dx2du = x_bezier.bezier_deriv2(u);
        double dy2du = y_bezier.bezier_deriv2(u);

        return -1 * (dxdu*dx2du + dydu*dy2du) / (Math.pow(Math.pow(dxdu, 2) + Math.pow(dydu, 2), 2));
    }

    public double bezier_length(BezierCurve x_bezier, BezierCurve y_bezier) {
        double[] upsilon = new double[100];
        double[] integrand = new double[100];
        double[] sums = new double[100];
        for (int i = 0; i < 100; i += 1) {
            upsilon[i] = i / 100;
        }
        double dupsilon = 0.01;
        double last_sum = 0;

        for (int i = 0; i < 100; i++) {
            integrand[i] = Math.sqrt(Math.pow(x_bezier.bezier_deriv(i/100), 2) + Math.pow(y_bezier.bezier_deriv(i/100), 2));
            sums[i] = last_sum + integrand[i] * dupsilon;
            last_sum = sums[i];
        }
        this.sums = sums;
        this.upsilon = upsilon;

        return sums[99];
    }
}
