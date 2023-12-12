package org.firstinspires.ftc.teamcode.exploration;

import java.util.Arrays;

public class PointFollow {
    public void PathFollow() {

    }

    public void followPath() {
    }

    public double[] normalizePow(double[] mPow, double maxPow) {
        double maxMPow = Arrays.stream(mPow).reduce(Double.NEGATIVE_INFINITY, (max, number) -> Double.max(max, Math.abs(number)));
        double[] normalizedPow = Arrays.stream(mPow).map(num -> num / Math.abs(maxMPow)).toArray();

        return normalizedPow;
    }
}
