package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Robot2 {

    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final VisionType cameraPosition;

    private final MotorUtility motors;
    private final DeadWheelUtility deadWheels;
    private final VisionUtility myAprilTagProcessor;

    private IMUUtility imu;

    private final Position currentPosition;
    private double currentHeading;

    // private final double initialHeading = 0;

    public Robot2(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, VisionType cameraPosition, Position initialPosition, double initialHeading) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.cameraPosition = cameraPosition;

        this.currentPosition = initialPosition;
        // this.initialHeading = Math.toRadians(initialHeading);
        this.currentHeading = Math.toRadians(initialHeading);

        this.motors = new MotorUtility(this.hardwareMap);
        this.deadWheels = new DeadWheelUtility(this.hardwareMap);
        this.imu = new IMUUtility(this.hardwareMap);
        this.myAprilTagProcessor = new VisionUtility(this.hardwareMap, this.cameraPosition);
    }

    public void moveToPositionAndHeading(Position targetPosition, double targetHeading) {

        /// Does this wait to initialize?
        this.imu = new IMUUtility(this.hardwareMap);

        PIDDrive xyPID = new PIDDrive(telemetry);
        PIDTurn turnPID = new PIDTurn(telemetry);

        deadWheels.resetEncoders();

        xyPID.setTargetPosition(targetPosition);
        turnPID.setTargetHeading(targetHeading);

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
            XYValue motorPower = xyPID.calculatePower(currentPosition, currentHeading, timer.seconds());
            double turnPower = turnPID.calculatePower(currentHeading, timer.seconds());

            telemetry.addData("CurrentX", currentPosition.x);
            telemetry.addData("CurrentY", currentPosition.y);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Remaining Theta", remainingTheta);
            telemetry.addData("xPower", motorPower.x);
            telemetry.addData("yPower", motorPower.y);
            telemetry.addData("turnPower", turnPower);
            telemetry.update();
            timer.reset();

            // Apply motor powers
            motors.setMotorPowers(motorPower.x, motorPower.y, turnPower);

            updatePosition();
        }
        motors.stopMotors();
    }

    public void updatePosition() {

        int leftEncoder = deadWheels.getCurrentValue(DeadWheel.LEFT); //in ticks
        int rightEncoder = deadWheels.getCurrentValue(DeadWheel.RIGHT);
        int centerEncoder = deadWheels.getCurrentValue(DeadWheel.CENTER);

        /// IMU Heading in Radians
        double imuHeadingInRadians = imu.getCurrentHeading();

        /// Get changes in encoder values
        int deltaLeft = leftEncoder - deadWheels.getPreviousValue(DeadWheel.LEFT);
        int deltaRight = rightEncoder - deadWheels.getPreviousValue(DeadWheel.RIGHT);
        int deltaCenter = centerEncoder - deadWheels.getPreviousValue(DeadWheel.CENTER);

        /// deltaTheta from IMU in Radians
        double deltaThetaIMU = imuHeadingInRadians - imu.getPreviousHeading();
        /// deltaTheta from Encoders in Radians
        double deltaThetaEncoders = (deltaRight - deltaLeft) * (Constants.DEAD_WHEEL_MM_PER_TICK / Constants.WHEEL_BASE_WIDTH);

        /// Encoder Heading in Radians
        // double rawTheta = (rightEncoder - leftEncoder) * (Constants.DEAD_WHEEL_MM_PER_TICK / Constants.WHEEL_BASE_WIDTH);

        currentHeading = currentHeading + ((deltaThetaIMU + deltaThetaEncoders) / 2);

        /// Update previous encoder values
        deadWheels.setPreviousLeft(leftEncoder); //in ticks
        deadWheels.setPreviousRight(rightEncoder);
        deadWheels.setPreviousCenter(centerEncoder);

        imu.setPreviousHeading(imuHeadingInRadians);

        //double encoderHeadingNormalized = gyros.normalizeHeading(currentHeading + deltaTheta);

        //NEW telemetry and normalizing currentHeading
        //currentHeading = imu.normalizeHeading(currentHeading + (deltaIMU));
//        telemetry.addData("deltaTheta", deltaTheta);
//        telemetry.addData("yPower", deltaIMU);
//        telemetry.addData("currentHeading", currentHeading);
//        telemetry.update();

        // Local displacements
        double deltaYLocal = (deltaLeft + deltaRight) / 2.0;
        double deltaXLocal = (deltaCenter - (Constants.WHEEL_BASE_LENGTH * deltaThetaEncoders));

        // Transform local displacements to global coordinates
        // changed the signs for globals
        double deltaYGlobal = deltaXLocal * Math.sin(Math.toRadians(currentHeading)) + deltaYLocal * Math.cos(Math.toRadians(currentHeading));
        double deltaXGlobal = deltaXLocal * Math.cos(Math.toRadians(currentHeading)) - deltaYLocal * Math.sin(Math.toRadians(currentHeading));

        final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();

        XYValue aprilPosition = new XYValue(0, 0);

        if (currentAprilTagPosition != null) {
            aprilPosition.x = currentAprilTagPosition.getPosition().x;
            aprilPosition.y = currentAprilTagPosition.getPosition().y;
        }

        // Update global position
        currentPosition.x += deltaXGlobal / Constants.DEAD_WHEEL_TICKS_PER_INCH;
        currentPosition.y += deltaYGlobal / Constants.DEAD_WHEEL_TICKS_PER_INCH;


    }

    public void checkSensorReadings() {

        this.imu = new IMUUtility(this.hardwareMap);
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

            double deltaTheta = Math.toDegrees((deltaRight - deltaLeft) * (Constants.DEAD_WHEEL_MM_PER_TICK / Constants.WHEEL_BASE_WIDTH));
            double deltaIMU = (imuHeadingInDegrees - imu.getPreviousHeading());

            // Update previous encoder values
            deadWheels.setPreviousLeft(leftEncoder); //in ticks
            deadWheels.setPreviousRight(rightEncoder);
            deadWheels.setPreviousCenter(centerEncoder);

            imu.setPreviousHeading(imuHeadingInDegrees);

            double rawTheta = Math.toDegrees((rightEncoder - leftEncoder) * Constants.DEAD_WHEEL_MM_PER_TICK / Constants.WHEEL_BASE_WIDTH);

            final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();


            // Print Out Various Values

            telemetry.addData("Current X", currentPosition.x);
            telemetry.addData("April X", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().x : "none");

            telemetry.addData("Current Y", currentPosition.y);
            telemetry.addData("April Y", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().y : "none");

            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Encoder Raw", rawTheta);
            telemetry.addData("IMU Raw", imuHeadingInDegrees);

            //telemetry.addData("deltaTheta", deltaTheta);
            //telemetry.addData("deltaIMU", deltaIMU);

            telemetry.update();

            updatePosition();
        }
    }
}