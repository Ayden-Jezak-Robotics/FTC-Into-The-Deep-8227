package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Robot {

    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final Position currentPosition;
    private double currentHeading;

    private final MotorUtility motors;
    private final DeadWheelUtility deadWheels;
    private IMUUtility imu;
    private final VisionUtility myAprilTagProcessor;

    public Robot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, Position initialPosition, double initialHeading) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.currentPosition = initialPosition;
        this.currentHeading = initialHeading;

        this.motors = new MotorUtility(this.hardwareMap);
        this.deadWheels = new DeadWheelUtility(this.hardwareMap);
        // this.imu = new IMUUtility(this.hardwareMap, this.telemetry);
        this.myAprilTagProcessor = new VisionUtility(this.hardwareMap);
    }

    public void moveToPositionAndHeading(Position targetPosition, double targetHeading) {

        this.imu = new IMUUtility(this.hardwareMap, this.telemetry);

        PIDUtility yPID = new PIDUtility(PIDType.STRAIGHT);
        PIDUtility xPID = new PIDUtility(PIDType.STRAFE);
        PIDUtility turnPID = new PIDUtility(PIDType.TURN);
        // telemetry.addLine("Starting move");
        // telemetry.update();

        deadWheels.resetEncoders();

        //NEW Don't need this
//        xPID.setOriginalError(currentPosition.x, targetPosition.x);
//        yPID.setOriginalError(currentPosition.y, targetPosition.y);
//        turnPID.setOriginalError(currentHeading, targetHeading);

       ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive()) {

            double remainingX = (targetPosition.x - currentPosition.x) * Constants.DEAD_WHEEL_TICKS_PER_INCH;
            double remainingY = (targetPosition.y - currentPosition.y) * Constants.DEAD_WHEEL_TICKS_PER_INCH;
            double remainingTheta = targetHeading - currentHeading;

            // Break condition
            if (Math.abs(remainingX) < Constants.MINIMUM_DISTANCE && Math.abs(remainingY) < Constants.MINIMUM_DISTANCE && Math.abs(remainingTheta) < Constants.TURN_TOLERANCE) {
                telemetry.addLine("Breaks due to tolerance");
                telemetry.update();
                break;
            }
            double time = timer.seconds();

            // Calculate power outputs using PID
            double xPower = xPID.calculatePower(currentPosition.x, timer.seconds());
            double yPower = yPID.calculatePower(currentPosition.y, timer.seconds());
            double turnPower = turnPID.calculatePower(currentHeading, timer.seconds());

            telemetry.addData("CurrentX", currentPosition.x);
            telemetry.addData("CurrentY", currentPosition.y);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("xPower", xPower);
            telemetry.addData("yPower", yPower);
            telemetry.addData("turnPower", turnPower);
            telemetry.update();
            timer.reset();

            // Apply motor powers
            motors.setMotorPowers(xPower, yPower, turnPower);

            updatePosition();

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

        int leftEncoder = deadWheels.getCurrentValue(DeadWheel.LEFT); //in ticks
        int rightEncoder = deadWheels.getCurrentValue(DeadWheel.RIGHT);
        int centerEncoder = deadWheels.getCurrentValue(DeadWheel.CENTER);

        float imuHeadingInDegrees = imu.getOrientation().firstAngle;

        // Get changes in encoder values
        int deltaLeft = leftEncoder - deadWheels.getPreviousValue(DeadWheel.LEFT);
        int deltaRight = rightEncoder - deadWheels.getPreviousValue(DeadWheel.RIGHT);
        int deltaCenter = centerEncoder - deadWheels.getPreviousValue(DeadWheel.CENTER);

        double deltaTheta = Math.toDegrees( (deltaRight - deltaLeft) * (Constants.DEAD_WHEEL_MM_PER_TICK / Constants.WHEEL_BASE_WIDTH));
        double deltaIMU = imu.normalizeHeading(imuHeadingInDegrees - imu.getPreviousHeading());

        // Update previous encoder values
        deadWheels.setPreviousLeft(leftEncoder); //in ticks
        deadWheels.setPreviousRight(rightEncoder);
        deadWheels.setPreviousCenter(centerEncoder);

        imu.setPreviousHeading(imuHeadingInDegrees);

        //double encoderHeadingNormalized = gyros.normalizeHeading(currentHeading + deltaTheta);

         //NEW telemetry and normalizing currentHeading
        currentHeading = imu.normalizeHeading(currentHeading + (deltaIMU));
//        telemetry.addData("deltaTheta", deltaTheta);
//        telemetry.addData("yPower", deltaIMU);
//        telemetry.addData("currentHeading", currentHeading);
//        telemetry.update();

        // Local displacements
        double deltaYLocal = (deltaLeft + deltaRight) / 2.0;
        double deltaXLocal = deltaCenter;

        // Transform local displacements to global coordinates
        // NEW If heading is not calculated properly, the position will be inaccurate
        double deltaYGlobal = deltaYLocal * Math.cos(Math.toRadians(currentHeading)) + deltaXLocal * Math.sin(Math.toRadians(currentHeading));
        double deltaXGlobal = deltaXLocal * Math.cos(Math.toRadians(currentHeading)) - deltaYLocal * Math.sin(Math.toRadians(currentHeading));

        // Update global position
        currentPosition.x += deltaXGlobal/Constants.DEAD_WHEEL_TICKS_PER_INCH;
        currentPosition.y += deltaYGlobal/Constants.DEAD_WHEEL_TICKS_PER_INCH;
    }

    /* public void updateFromAprilTags() {

        final double aprilTagWeight = Constants.APRIL_TAG_WEIGHT;

        final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();

        if (currentAprilTagPosition != null) {
            currentPosition.x = (1 - aprilTagWeight) * currentPosition.x + aprilTagWeight * currentAprilTagPosition.getPosition().x;
            currentPosition.y = (1 - aprilTagWeight) * currentPosition.y + aprilTagWeight * currentAprilTagPosition.getPosition().y;
            //currentHeading = (1 - aprilTagWeight) * currentHeading + aprilTagWeight * currentAprilTagPosition.getOrientation().getYaw();
        }

        telemetry.addData("currentPosition.x", currentPosition.x);
        telemetry.addData("currentAprilPos.x", currentAprilTagPosition.getPosition().x);
        telemetry.addData("currentPosition.y", currentPosition.y);
        telemetry.addData("currentAprilPos.y", currentAprilTagPosition.getPosition().y);
        telemetry.update();
    }*/

    public void checkSensorReadings() {

        this.imu = new IMUUtility(this.hardwareMap, this.telemetry);
        deadWheels.resetEncoders();

        // double totalDeltaTheta = 0;

        while (opMode.opModeIsActive()) {

            //
            int leftEncoder = deadWheels.getCurrentValue(DeadWheel.LEFT); //in ticks
            int rightEncoder = deadWheels.getCurrentValue(DeadWheel.RIGHT);
            int centerEncoder = deadWheels.getCurrentValue(DeadWheel.CENTER);

            float imuHeadingInDegrees = imu.getOrientation().firstAngle;

            // Calculate changes in encoder values
            int deltaLeft = leftEncoder - deadWheels.getPreviousValue(DeadWheel.LEFT);
            int deltaRight = rightEncoder - deadWheels.getPreviousValue(DeadWheel.RIGHT);
            int deltaCenter = centerEncoder - deadWheels.getPreviousValue(DeadWheel.CENTER);

            double deltaTheta = Math.toDegrees( (deltaRight - deltaLeft) * (Constants.DEAD_WHEEL_MM_PER_TICK / Constants.WHEEL_BASE_WIDTH));
            double deltaIMU = imu.normalizeHeading(imuHeadingInDegrees - imu.getPreviousHeading());

            // Update previous encoder values
            deadWheels.setPreviousLeft(leftEncoder); //in ticks
            deadWheels.setPreviousRight(rightEncoder);
            deadWheels.setPreviousCenter(centerEncoder);

            imu.setPreviousHeading(imuHeadingInDegrees);

            double rawTheta = Math.toDegrees((rightEncoder - leftEncoder)*Constants.DEAD_WHEEL_MM_PER_TICK / Constants.WHEEL_BASE_WIDTH);

            final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();


            // Print Out Various Values

            telemetry.addData("Current X", currentPosition.x);
            //telemetry.addData("April X", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().x : "none");

            telemetry.addData("Current Y", currentPosition.y);
            //telemetry.addData("April Y", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().y : "none");

            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Encoder Raw", rawTheta);
            telemetry.addData("IMU Raw", imuHeadingInDegrees);

            telemetry.addData("deltaTheta", deltaTheta);
            telemetry.addData("deltaIMU", deltaIMU);

            telemetry.update();

            updatePosition();
        }
    }
}
