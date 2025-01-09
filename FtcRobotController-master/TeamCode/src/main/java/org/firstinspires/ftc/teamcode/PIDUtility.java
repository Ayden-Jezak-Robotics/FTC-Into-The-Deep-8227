package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDUtility {
    private PIDType type;

    private double kP, kI, kD, kF;

    private double initialPosition; // in inches
    private double targetPosition; // in inches
    private double originalError; //in ticks

    //private double aMaxPoint; // in ticks
    private double integralSum;
    private double priorError;
    // Adjust based on your system's needs

    private Telemetry telemetry;

    public PIDUtility(double kP, double kI, double kD, double kF, Telemetry telemetry) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0;
        this.priorError = 0;
        this.telemetry = telemetry;
    }

    public PIDUtility(PIDType type, Telemetry telemetry) {
        this.type = type;
        this.telemetry = telemetry;

        /// DID SOME EXCEL CALCULATIONS AND SUGGEST WE TRY THE COMMENTED VALUES

        switch (type) {
            case STRAIGHT:
                this.kP = 0.000066;
                this.kI = 0.0000568;
                this.kD = 0.000002;
                this.kF = 0; //0.2
                break;
            case STRAFE:
                this.kP = 0.000066;
                this.kI = 0.0000568;
                this.kD = 0.000002;
                this.kF = 0; //0.22
                break;
            case TURN:
                this.kP = 0.015;
                this.kI = 0.015;
                this.kD = 0.00025;
                this.kF = 0;
                break;
            default:
        }

        this.integralSum = 0;
        this.priorError = 0;
    }

    public void setOriginalError(double initialPosition, double targetPosition) {
        this.initialPosition = initialPosition;
        this.targetPosition = targetPosition;

        this.originalError = (targetPosition - initialPosition);

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            this.originalError = this.originalError * Constants.DEAD_WHEEL_TICKS_PER_INCH;
        }
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    

    public double calculatePower(double currentPosition, double time) // time is in Seconds
    {
        double error;

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            error = (targetPosition - currentPosition) * Constants.DEAD_WHEEL_TICKS_PER_INCH;

            if (Math.abs(error) < Constants.MINIMUM_DISTANCE) {
                return 0;
            }
        }
        else { // For Turn based calculations
            //NEW need to normalize hypothetically what is target = -45 and you are at 180?
            error = normalizeAngle(targetPosition - currentPosition);

            if (Math.abs(error) < Constants.TURN_TOLERANCE) {
                return 0;
            }
        }

        double deltaTime;

        if (time < Constants.MINIMUM_TIME_IN_SECONDS) {
            deltaTime = Constants.MINIMUM_TIME_IN_SECONDS; // Prevent zero or very small time steps
        } else {
            deltaTime = time;
        }

        double kProportionalValue = Range.clip(kP * error, -Constants.MAX_KP, Constants.MAX_KP);

        // Decay the Integral Sum over time
        integralSum = (integralSum * 0.98) + (error * deltaTime);

        double kIntegralValue;

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            kIntegralValue = kI * Range.clip(integralSum, -Constants.MAX_INTEGRAL_XY, Constants.MAX_INTEGRAL_XY);
        } else {
            kIntegralValue = kI * Range.clip(integralSum, -Constants.MAX_INTEGRAL_TURN, Constants.MAX_INTEGRAL_TURN);
        }

        double kDerivativeValue = kD * ((error - priorError) / deltaTime);

        double kFeedForwardValue = kF * Math.signum(error);

        priorError = error;

        double baseOutput = kProportionalValue + kIntegralValue + kDerivativeValue;

        telemetry.addData("error", error);
        telemetry.addData("priorError", priorError);
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("integralSum", integralSum);
        telemetry.addData("kIntegralValue", kIntegralValue);
        telemetry.addData("kDerivativeValue", kDerivativeValue);
        telemetry.addData("kProportionalValue", kProportionalValue);
        telemetry.update();

        /*if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            if (Math.abs(baseOutput) < Constants.MINIMUM_POWER_OUTPUT_DRIVE){
                // Enforce minimum power while maintaining the sign
                baseOutput = Math.signum(baseOutput) * Constants.MINIMUM_POWER_OUTPUT_DRIVE;
            }
        }
        else {
            if (Math.abs(baseOutput) < Constants.MINIMUM_POWER_OUTPUT_TURN) {
                // Enforce minimum power while maintaining the sign
                baseOutput = Math.signum(baseOutput) * Constants.MINIMUM_POWER_OUTPUT_TURN;
            }
        }*/



        return baseOutput;
    }
}
