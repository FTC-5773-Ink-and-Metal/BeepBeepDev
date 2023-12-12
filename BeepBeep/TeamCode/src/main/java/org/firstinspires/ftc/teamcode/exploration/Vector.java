package org.firstinspires.ftc.teamcode.exploration;

public class Vector {
    private double magnitude; // Magnitude (length) of the vector
    private double angle; // Angle of the vector (in degrees)

    public Vector(double magnitude, double angle) {
        this.magnitude = magnitude;
        this.angle = angle;
    }

    public void rotate2D(double rotAngle) {
        this.angle = this.angle - rotAngle;
        while (this.angle < 0) {
            this.angle += 2*Math.PI;
        }
        while (this.angle > 2*Math.PI) {
            this.angle -= 2*Math.PI;
        }
    }

    // Optional: toString() method to display the vector's information
    @Override
    public String toString() {
        return String.format("Vector (magnitude: %.2f, angle: %.2f degrees)", magnitude, angle);
    }
}