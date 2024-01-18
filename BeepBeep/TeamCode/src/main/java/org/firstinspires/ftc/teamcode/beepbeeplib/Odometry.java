package org.firstinspires.ftc.teamcode.beepbeeplib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.beepbeeplib.util.PIDController;

public class Odometry {
    // All units are in CM!!
    HardwareMap hwMp;

    DcMotorEx leftEnc, rightEnc, normalEnc;

    public static double lateralDistance, normalOffset, wheelRadius, ticksPerRev;
    public double cmTick = (2.0 * Math.PI * wheelRadius) / ticksPerRev;

    public int curLeft = 0, curRight = 0, curNorm = 0;
    public int prevLeft = 0, prevRight = 0, prevNorm = 0;

    public double initX, initY, initTheta;
    public double curX, curY, curTheta;
    public double dx, dy, dtheta;

    public PIDController PIDx;
    public PIDController PIDy;
    public PIDController PIDtheta;

    public Odometry(double startX, double startY, double startHeading){
        this.initX = startX; this.curX = startX;
        this.initY = startY; this.curY = startY;
        this.initTheta = startHeading; this.curTheta = startHeading;
    }

    public Odometry(){
        this.initX = 0; this.curX = 0;
        this.initY = 0; this.curY = 0;
        this.initTheta = 0; this.curTheta = 0;
    }

    public void update() {
        prevLeft = curLeft; curLeft = leftEnc.getCurrentPosition();
        prevRight = curRight; curRight = rightEnc.getCurrentPosition();
        prevNorm = curNorm; curNorm = normalEnc.getCurrentPosition();

        int de1 = prevLeft - curLeft;
        int de2 = prevRight - curRight;
        int de3 = prevNorm - curNorm;

        dtheta = cmTick * ((de2-de1) / lateralDistance);
        dx = cmTick * (de1+de2)/2.0;
        dy = cmTick * (de3 - (de2-de1) * normalOffset / lateralDistance);

        curX += dx;
        curY += dy;
        curTheta = dtheta;
    }
}
