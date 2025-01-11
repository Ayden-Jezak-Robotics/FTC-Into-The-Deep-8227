package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDTurn {
    private static final double kP = 0.00001;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 0.15;

    private double targetHeading;

    private double priorError;
    private double integralSum;

    private Telemetry telemetry;

    public PIDTurn(Telemetry telemetry) {

        this.priorError = 0;
        this.integralSum = 0;

        this.telemetry = telemetry;

    }

    public void setTargetHeading(double newTarget) {

        this.targetHeading = newTarget;

    }

    /// currentPosition in Inches; currentHeading in Radians; time in Seconds
    public double calculatePower(double currentHeading, double time)
    {
        double error = (targetHeading - currentHeading);

        /// Prevent zero or very small time steps
        double deltaTime = Math.max(time, Constants.MINIMUM_TIME_IN_SECONDS);

        double kProportionalValue = Range.clip(kP * error, -Constants.MAX_KP, Constants.MAX_KP);

        // Decay the Integral Sum over time
        integralSum = (integralSum * 0.98) + (error * deltaTime);

        double kIntegralValueX = kI * Range.clip(integralSum, -Constants.MAX_INTEGRAL_TURN, Constants.MAX_INTEGRAL_TURN);

        double kDerivativeValue = kD * ((error - priorError) / deltaTime);

        double kFeedForwardValue = kF * Math.signum(error);

        priorError = error;

        double baseOutput = kProportionalValue + kIntegralValueX + kDerivativeValue + kFeedForwardValue;

/*

        if (baseOutput.x < Constants.MINIMUM_POWER_OUTPUT_DRIVE) {
            baseOutput.x = Math.signum(baseOutput.x) * Constants.MINIMUM_POWER_OUTPUT_DRIVE;
        }

        if (baseOutput.y < Constants.MINIMUM_POWER_OUTPUT_DRIVE) {
            baseOutput.y = Math.signum(baseOutput.y) * Constants.MINIMUM_POWER_OUTPUT_DRIVE;
        }

*/

        return baseOutput;
    }
}
