package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDArm {
    private static final double kP = 0.015; //Fix
    private static final double kI = 0.015;
    private static final double kD = 0;
    private static final double kF = 0; //FIND the minimum value needed that equal the force of weight

    private double targetHeight;

    private double error;
    private double priorError;
    private double integralSum;

    public PIDArm(RobotState currentState, RobotState targetState) {

        this.error = targetState.armHeight - currentState.armHeight;
        this.targetHeight = targetState.armHeight;
        this.priorError = 0;
        this.integralSum = 0;

    }

    public boolean arrivedAtHeight() {
        return Math.abs(error) < Constants.HEIGHT_TOLERANCE;
    }

    /// currentPosition in Inches; currentHeight in Radians; time in Seconds
    public double calculatePower(double currentHeight, double time) {
        error = (targetHeight - currentHeight);

        // Prevent zero or very small time steps
        double deltaTime = Math.max(time, Constants.MINIMUM_TIME_IN_SECONDS);

        double kProportionalValue = Range.clip(kP * error, -Constants.MAX_KP, Constants.MAX_KP);

        // Decay the Integral Sum over time
        integralSum = (integralSum * 0.98) + (error * deltaTime);

        double kIntegralValue = kI * Range.clip(integralSum, -Constants.MAX_INTEGRAL_TURN, Constants.MAX_INTEGRAL_TURN);

        double kDerivativeValue = kD * ((error - priorError) / deltaTime);

        double kFeedForwardValue = kF * Math.signum(error);

        priorError = error;

        double baseOutput = kProportionalValue + kIntegralValue + kDerivativeValue + kFeedForwardValue;

        return baseOutput;
    }
}