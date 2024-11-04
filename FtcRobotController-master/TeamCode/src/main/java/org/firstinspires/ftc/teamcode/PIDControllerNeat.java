package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDControllerNeat
{
    public double kP, kI, kD, kF, tolerance; // PID gains
    public double integral, previousError, derivative, error;
    private double targetAmount; // Desired value
    private double maxOutput = 0.5; // Maximum allowable motor power
    private double output;

    public PIDControllerNeat(double kP, double kI, double kD, double kF, double givenTolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integral = 0;
        this.previousError = 0;
        this.targetAmount = 0;
        this.tolerance = givenTolerance;
    }
    public PIDControllerNeat(String type) {
        if (type.equals("Straight"))
        {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.integral = 0;
            this.previousError = 0;
            this.targetAmount = 0;
            this.tolerance = 200;
        }

    }

    public void setTargetAmount(double targetAmount) {
        this.targetAmount = targetAmount;
    }

    public double update(double currentValue, double deltaTime) {
        error = (targetAmount - currentValue);
        integral += error * deltaTime;
        derivative = (error - previousError) / deltaTime;
        double accelTicks = targetAmount/4;

        double baseOutput = (kP * error) + (kI * integral) + (kD * derivative);
        if (currentValue < accelTicks){
            output = kF + baseOutput * (currentValue/accelTicks);
            previousError = error;
            return clampOutput(output);
        }
        output = kF+ baseOutput;
        previousError = error;

        return clampOutput(output);
    }
    public double clampOutput(double output) {
        return Math.max(-maxOutput, Math.min(maxOutput, output));
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        previousOutput = 0; // Reset the previous output as well
    }

    private double previousOutput = 0; // Field to store the previous output
}
