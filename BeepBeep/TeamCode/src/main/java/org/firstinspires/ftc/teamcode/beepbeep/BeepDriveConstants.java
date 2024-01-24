package org.firstinspires.ftc.teamcode.beepbeep;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BeepDriveConstants {
    public static double Kp_x = 0.07;
    public static double Ki_x = 0.0;
    public static double Kd_x = 0.0;

    public static double Kp_y = 0.1;
    public static double Ki_y = 0.0;
    public static double Kd_y = 0.0;

    public static double Kp_heading = 1.5;
    public static double Ki_heading = 0.0;
    public static double Kd_heading = 0.0;

    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;

    public static double kV_x = 0.012; // 0.0151
    public static double kA_x = 0.004;
    public static double kS_x = 0.0005;
//    public static double kV_x = 0.0157;
//    public static double kA_x = 0.003554;
//    public static double kS_x = 0.00015;

    public static double kV_y = 0.01; // 0.0245
    public static double kA_y = 0.01;
    public static double kS_y = 0.0;

    public static double kV_ang = 0.265;
    public static double kA_ang = 0.0023;
    public static double kS_ang = 0.02;

    public static double maxAccel = 80;
    public static double maxVel = 80;

    public static double maxAngAccel = Math.toRadians(360);
    public static double maxAngVel = Math.toRadians(360);

    public static double translational_error = 0.3;
    public static double angular_error = Math.toRadians(1.5);
    public static double velo_error = 1;

    public static double pathBreakerLinear = 6;
    public static double pathSaverLinear = 1.5;
}
