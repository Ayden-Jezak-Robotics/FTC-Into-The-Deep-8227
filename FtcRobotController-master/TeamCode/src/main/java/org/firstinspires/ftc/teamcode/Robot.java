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

    private final CameraPosition cameraPosition;

    private final MotorUtility motors;
    private final DeadWheelUtility deadWheels;
    private final VisionUtility myAprilTagProcessor;

    private IMUUtility imu;

    private final Position currentPosition;
    private double currentHeading;

    private int armHeight = 0;
    private boolean armIsExtended = false;
    private boolean wristIsExtended = false;

    // private final double initialHeading = 0;

    public Robot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, RobotState initialState, CameraPosition side) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.cameraPosition = side;

        this.currentPosition = initialState.position;
        this.currentHeading = Math.toRadians(initialState.heading);

        this.motors = new MotorUtility(this.hardwareMap);
        this.deadWheels = new DeadWheelUtility(this.hardwareMap);
        this.imu = new IMUUtility(this.hardwareMap);
        this.myAprilTagProcessor = new VisionUtility(this.hardwareMap, this.cameraPosition);
    }

    public void moveToPositionAndHeading(RobotState targetState) {

        Position targetPosition = targetState.position;
        double targetHeading = targetState.heading;

        PIDDrive xyPID = new PIDDrive(telemetry);
        PIDTurn turnPID = new PIDTurn(telemetry);

        /// Reset everything to 0
        imu.resetIMU();
        deadWheels.resetEncoders();

        /// Does the PID Controller need to know the target heading, or just the current heading?
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

        int EncoderDrive = deadWheels.getCurrentValue(DeadWheel.DRIVE); //in ticks
        int EncoderStrafe = deadWheels.getCurrentValue(DeadWheel.STRAFE);

        /// IMU Heading in Radians
        double imuHeadingInRadians = imu.getCurrentHeading();

        /// Get changes in encoder values
        int deltaDrive = EncoderDrive - deadWheels.getPreviousValue(DeadWheel.DRIVE);
        int deltaStrafe = EncoderStrafe - deadWheels.getPreviousValue(DeadWheel.STRAFE);

        /// deltaTheta from IMU in Radians
        double deltaThetaIMU = imuHeadingInRadians - imu.getPreviousHeading();

        /// Encoder Heading in Radians

        currentHeading = currentHeading + deltaThetaIMU;

        /// Update previous encoder values
        deadWheels.setPreviousDrive(EncoderDrive); //in ticks
        deadWheels.setPreviousStrafe(EncoderStrafe);

        imu.setPreviousHeading(imuHeadingInRadians);

        //double encoderHeadingNormalized = gyros.normalizeHeading(currentHeading + deltaTheta);

        //NEW telemetry and normalizing currentHeading
        //currentHeading = imu.normalizeHeading(currentHeading + (deltaIMU));
//        telemetry.addData("deltaTheta", deltaTheta);
//        telemetry.addData("yPower", deltaIMU);
//        telemetry.addData("currentHeading", currentHeading);
//        telemetry.update();

        // Local displacements
        double deltaYLocal = (deltaDrive - (Constants.WHEEL_BASE_LENGTH * deltaThetaIMU));
        double deltaXLocal = (deltaStrafe - (Constants.WHEEL_BASE_LENGTH * deltaThetaIMU));

        // Transform local displacements to global coordinates
        // changed the signs for globals
        double deltaYGlobal = deltaXLocal * Math.sin(currentHeading) + deltaYLocal * Math.cos(currentHeading);
        double deltaXGlobal = deltaXLocal * Math.cos(currentHeading) - deltaYLocal * Math.sin(currentHeading);

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
            int EncoderDrive = deadWheels.getCurrentValue(DeadWheel.DRIVE); //in ticks
            int EncoderStrafe = deadWheels.getCurrentValue(DeadWheel.STRAFE);

            double imuHeadingInRadians = imu.getCurrentHeading();

            // Calculate changes in encoder values
            int deltaDrive = EncoderDrive - deadWheels.getPreviousValue(DeadWheel.DRIVE);
            int deltaStrafe = EncoderStrafe - deadWheels.getPreviousValue(DeadWheel.STRAFE);

            double deltaIMU = (imuHeadingInRadians - imu.getPreviousHeading());

            // Update previous encoder values
            deadWheels.setPreviousDrive(EncoderDrive); //in ticks
            deadWheels.setPreviousStrafe(EncoderStrafe);

            imu.setPreviousHeading(imuHeadingInRadians);

            final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();

            // Print Out Various Values

            telemetry.addData("Current X", currentPosition.x);
            telemetry.addData("April X", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().x : "none");

            telemetry.addData("Current Y", currentPosition.y);
            telemetry.addData("April Y", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().y : "none");

            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("IMU Raw", Math.toDegrees(imuHeadingInRadians));

            telemetry.update();

            updatePosition();
        }
    }
}