package org.firstinspires.ftc.teamcode;

public class PIDController {

    private double kP, kI, kD; // PID gains
    private double integral, previousError;
    private double targetAmount; // Desired value

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
        this.targetAmount = 0;
    }

    public void setTargetAmount(double targetAmount) {
        this.targetAmount = targetAmount;
    }

    public double update(double currentValue, double deltaTime) {
        double error = targetAmount - currentValue;
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        previousError = error;
        return output;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
