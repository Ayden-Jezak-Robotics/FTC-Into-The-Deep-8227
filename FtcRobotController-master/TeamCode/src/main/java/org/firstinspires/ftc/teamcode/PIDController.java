package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDController {

    public double kP, kI, kD; // PID gains
    public double integral, previousError, derivative, error;
    private double targetAmount; // Desired value
    private double maxOutput = 1; // Maximum allowable motor power
    private double rampRate = 0.1; // Rate at which the power ramps up

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

    // Sai
    public void makeStraight(DcMotor leftOne, DcMotor rightOne)
    {

    }
    //

    public double update(double currentValue, double deltaTime) {
        error = (targetAmount - currentValue);
        integral += error * deltaTime;
        derivative = (error - previousError) / deltaTime;

        double baseOutput = (kP * error) + (kI * integral) + (kD * derivative);

        double output = baseOutput;

        previousError = error;
        return clampOutput(output);
    }

    /*
    private double rampUpOutput(double baseOutput, double deltaTime) {
        double targetOutput = baseOutput;
        double currentOutput = previousOutput;

        // Calculate the maximum change in output based on the ramp rate
        double maxChange = rampRate * deltaTime;

        // Calculate the new output by ramping up
        if (targetOutput > currentOutput) {
            currentOutput += maxChange;
        } else {
            currentOutput = targetOutput; // Directly set to target if ramping down
        }

        previousOutput = currentOutput; // Update the previous output
        return currentOutput;
    }
    */


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