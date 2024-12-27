package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Robot {

    private final LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private final Position currentPosition;
    private double currentHeading;

    private final MotorUtility motors;
    private final DeadWheelUtility deadWheels;
    private final GyroUtility gyros;
    private final VisionUtility myAprilTagProcessor;

    public Robot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, Position initialPosition, double initialHeading) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.currentPosition = initialPosition;
        this.currentHeading = initialHeading;

        this.motors = new MotorUtility(this.hardwareMap, this.telemetry);
        this.deadWheels = new DeadWheelUtility(this.hardwareMap, this.telemetry);
        this.gyros = new GyroUtility(this.hardwareMap, this.telemetry);
        this.myAprilTagProcessor = new VisionUtility(this.hardwareMap);
    }

    public void moveToPositionAndHeading(Position targetPosition, double targetHeading) {
        PIDUtility xPID = new PIDUtility(PIDType.STRAIGHT);
        PIDUtility yPID = new PIDUtility(PIDType.STRAFE);
        PIDUtility turnPID = new PIDUtility(PIDType.TURN);

        deadWheels.resetEncoders();

        xPID.setOriginalError(currentPosition.x, targetPosition.x);
        yPID.setOriginalError(currentPosition.y, targetPosition.y);
        turnPID.setOriginalError(currentHeading, targetHeading);

       ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive()) {
            updatePosition();
            updateFromAprilTags();

            double remainingX = targetPosition.x - currentPosition.x;
            double remainingY = targetPosition.y - currentPosition.y;
            double remainingTheta = targetHeading-currentHeading;

            // Break condition
            if (Math.abs(remainingX) < Constants.MINIMUM_DISTANCE && Math.abs(remainingY) < Constants.MINIMUM_DISTANCE && Math.abs(remainingTheta) < Constants.TURN_TOLERANCE) {
                break;
            }

            // Calculate power outputs using PID
            double xPower = xPID.calculatePower(currentPosition.x, timer.milliseconds());
            double yPower = yPID.calculatePower(currentPosition.y, timer.milliseconds());
            double turnPower = turnPID.calculatePower(currentHeading, timer.milliseconds());
            timer.reset();

            // Convert x, y, and turn power to motor powers
            double[] motorPowers = calculateMotorPowers(xPower, yPower, turnPower);

            // Apply motor powers
            motors.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);

        }

        motors.stopMotors();
    }

    private double[] calculateMotorPowers(double xPower, double yPower, double turnPower) {
        double frontLeftPower = yPower + xPower + turnPower;
        double frontRightPower = yPower - xPower - turnPower;
        double backLeftPower = yPower - xPower + turnPower;
        double backRightPower = yPower + xPower - turnPower;

        // Normalize powers if they exceed 1
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }

    public void updatePosition() {

        double imuWeight = 0.5; // Initial weight for IMU
        double encoderWeight = 0.5; // Initial weight for encoder

        double leftEncoder = deadWheels.getPosition(DeadWheel.LEFT); //in ticks
        double rightEncoder = deadWheels.getPosition(DeadWheel.RIGHT);
        double centerEncoder = deadWheels.getPosition(DeadWheel.CENTER);

        // Get changes in encoder values
        double deltaLeft = leftEncoder - deadWheels.getPreviousLeft();
        double deltaRight = rightEncoder - deadWheels.getPreviousRight();
        double deltaTheta = Math.toDegrees((deltaRight - deltaLeft) / Constants.WHEEL_BASE_WIDTH);

        // Update previous encoder values
        deadWheels.setPreviousLeft(leftEncoder); //in ticks
        deadWheels.setPreviousRight(rightEncoder);
        deadWheels.setPreviousCenter(centerEncoder);

        // Normalize IMU and encoder headings
        double imuHeading = gyros.getHeading();
        double encoderHeadingNormalized = gyros.normalizeHeading(currentHeading + deltaTheta);

        // Compare IMU and encoder headings to adjust weights
        double headingDifference = Math.abs(imuHeading - encoderHeadingNormalized);
        if (headingDifference > 10) { // Threshold for significant discrepancy
            imuWeight = 0.3; // Reduce trust in IMU
            encoderWeight = 0.7; // Increase trust in encoders
        } else {
            imuWeight = 0.5; // Restore balanced trust
            encoderWeight = 0.5;
        }

        // Blend IMU and encoder headings
        currentHeading = (imuWeight * imuHeading) + (encoderWeight * encoderHeadingNormalized);

        // Local displacements
        double deltaXLocal = (deltaLeft + deltaRight) / 2.0;
        double deltaYLocal = centerEncoder - deadWheels.getPreviousCenter();

        // Transform local displacements to global coordinates
        double deltaXGlobal = deltaYLocal * Math.cos(Math.toRadians(currentHeading)) + deltaXLocal * Math.sin(Math.toRadians(currentHeading));
        double deltaYGlobal = deltaXLocal * Math.cos(Math.toRadians(currentHeading)) - deltaYLocal * Math.sin(Math.toRadians(currentHeading));

        // Update global position
        currentPosition.x += deltaXGlobal/Constants.DEAD_WHEEL_TICKS_PER_INCH;
        currentPosition.y += deltaYGlobal/Constants.DEAD_WHEEL_TICKS_PER_INCH;
    }

    public void updateFromAprilTags() {

        final double aprilTagWeight = Constants.APRIL_TAG_WEIGHT;

        final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();

        if (currentAprilTagPosition != null) {
            currentPosition.x = (1 - aprilTagWeight) * currentPosition.x + aprilTagWeight * currentAprilTagPosition.getPosition().x;
            currentPosition.y = (1 - aprilTagWeight) * currentPosition.y + aprilTagWeight * currentAprilTagPosition.getPosition().y;
            currentHeading = (1 - aprilTagWeight) * currentHeading + aprilTagWeight * currentAprilTagPosition.getOrientation().getYaw();
        }
    }
}
