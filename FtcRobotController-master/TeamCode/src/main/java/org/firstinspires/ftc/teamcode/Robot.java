package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Robot {

    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    //private final CameraPosition cameraPosition;

    private final MotorUtility motors;
    private final ArmUtility arms;
    private final DeadWheelUtility deadWheels;
    //private final VisionUtility myAprilTagProcessor;

    private IMUUtility imu;

    private final Position currentPosition;
    private double currentHeading, currentHeight, currentArmAngle, currentExtend;

    private int armHeight = 0;
    private boolean armIsExtended = false;
    private boolean wristIsExtended = false;

    private double armTime;

    // private final double initialHeading = 0;

    public Robot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, RobotState initialState, CameraPosition side) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //this.cameraPosition = side;
        this.currentPosition = initialState.position;
        this.currentHeight = initialState.height;
        this.currentHeading = LMMHS.setAngle(initialState.heading);
        this.currentArmAngle = initialState.armAngle;
        this.currentExtend = initialState.extend;

        this.motors = new MotorUtility(this.hardwareMap);
        this.arms = new ArmUtility(this.hardwareMap);
        this.deadWheels = new DeadWheelUtility(this.hardwareMap);
        this.imu = new IMUUtility(this.hardwareMap);
        //this.myAprilTagProcessor = new VisionUtility(this.hardwareMap, this.cameraPosition);
    }

    public void turnOn(double powerLevel) {
        /// Reset everything to 0
        //imu.resetIMU();
        deadWheels.resetEncoders();
        while (opMode.opModeIsActive()) {
            motors.setMotorPower(powerLevel);
            int EncoderDrive = deadWheels.getCurrentValue(DeadWheel.DRIVE); //in ticks
            int EncoderStrafe = deadWheels.getCurrentValue(DeadWheel.STRAFE);
            telemetry.addData("EncoderDrive", EncoderDrive);
            telemetry.addData("EncoderStrafe", EncoderStrafe);
            telemetry.addData("IMU Heading", imu.getCurrentHeading());
            telemetry.update();
        }
    }

    public void armUp(double target)
    {
        ElapsedTime timer = new ElapsedTime();
        PIDArm armPID = new PIDArm(telemetry);
        armPID.setTargetHeight(target);
        while (opMode.opModeIsActive()) {
            double time = timer.seconds();
            double armPower = armPID.calculatePower(currentHeight, timer.seconds());
            arms.setArmPowers(armPower);
            updatePosition();
            double tolerance = target - currentHeight;
            if (tolerance < 200) {
                arms.setHoldingPower();
            }
        }
        motors.stopMotors();
    }

    public void justArm(double targetAngle, double targetExtend, double targetAngleTime, double targetExtendTime, boolean open)    {
        ElapsedTime armTimer = new ElapsedTime();
        double initialAngle = arms.getCurrentAngledPosition();
        double initialExtend = arms.getCurrentExtend();
        double presentAngle = 0;
        double presentExtend = 0;

        double boundTime = Math.max(targetAngleTime,targetExtendTime);
        telemetry.addData("bound time", boundTime);
        telemetry.update();
        double armTime = armTimer.seconds();

        while (opMode.opModeIsActive()) {
            while (armTime <= boundTime)
            {
                if (arms.getCurrentAngledPosition() != targetAngle)
                {
                    double ratioArm = Range.clip((armTime / targetAngleTime),0,1);
                    presentAngle = initialAngle + (ratioArm*(targetAngle-initialAngle));
                    arms.angleArmTo(presentAngle);
                }
                if (arms.getCurrentExtend() != targetExtend) {

                    double ratioExtend = Range.clip((armTime / targetExtendTime),0,1);
                    presentExtend = initialExtend + (ratioExtend * (targetExtend- initialExtend));
                    arms.extendElbow(presentExtend);
                }
                telemetry.addData("currentAngle",arms.getCurrentAngledPosition());
                telemetry.addData("currentExtend",arms.getCurrentExtend());
                telemetry.addData("presentAngle",presentAngle);
                telemetry.addData("presentExtend",presentExtend);
                telemetry.update();

                armTime = armTimer.seconds();
            }
            telemetry.addLine("Breaking out");
            break;
        }
        if (open == true)
        {
            arms.openGrabber();
        }
        else{
            arms.closeGrabber();
        }
    }

    public void wristUp()
    {
        arms.wristUp();
    }

    public void wristDown()
    {
        arms.wristDown();
    }
    public void handleArmWithTime(double initialAngle, double initialExtend, double targetAngle, double targetExtend, double targetAngleTime, double targetExtendTime)
    {
        double presentAngle = 0;
        double presentExtend = 0;

        double boundTime = Math.max(targetAngleTime,targetExtendTime);

        if (armTime <= boundTime)
            {
                if (arms.getCurrentAngledPosition() != targetAngle)
                {
                    double ratioArm = Range.clip((armTime / targetAngleTime),0,1);
                    presentAngle = initialAngle + (ratioArm*(targetAngle-initialAngle));
                    arms.angleArmTo(presentAngle);
                }
                if (arms.getCurrentExtend() != targetExtend) {

                    double ratioExtend = Range.clip((armTime / targetExtendTime),0,1);
                    presentExtend = initialExtend + (ratioExtend * (targetExtend- initialExtend));
                    arms.extendElbow(presentExtend);
                }
                telemetry.addData("currentAngle",arms.getCurrentAngledPosition());
                telemetry.addData("currentExtend",arms.getCurrentExtend());
                telemetry.update();
            }
    }

    /*public void testPick()
    {
        arms.openGrabber();
        arms.angleArmToBase();
        arms.closeGrabber();
        arms.angleArmTo(0);
        arms.setWristPosition(1.0);
    }

     */

    public void moveToPositionAndHeading(RobotState targetState) {

        Position targetPosition = targetState.position;
        double targetHeading = LMMHS.setAngle(targetState.heading);
        double targetHeight = targetState.height;
        double targetArmAngle = targetState.armAngle;
        double targetExtend = targetState.extend;
        double targetArmAngleTime = targetState.armAngleTime;
        double targetExtendTime = targetState.extendTime;

        double initialArmAngle = currentArmAngle;
        double initialExtend = currentExtend;

        PIDDrive xyPID = new PIDDrive(telemetry);
        PIDTurn turnPID = new PIDTurn(telemetry);
        PIDArm armPID = new PIDArm(telemetry);

        /// Reset everything to 0
        //imu.resetIMU(); //PROBLEM
        deadWheels.resetEncoders();

        /// Does the PID Controller need to know the target heading, or just the current heading?
        xyPID.setTargetPosition(targetPosition);
        turnPID.setTargetHeading(targetHeading);
        armPID.setTargetHeight(targetHeight);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime armTimer = new ElapsedTime();
        armTime = timer.seconds();

        while (opMode.opModeIsActive()) {

            double remainingX = (targetPosition.x - currentPosition.x) * Constants.DEAD_WHEEL_TICKS_PER_INCH;
            double remainingY = (targetPosition.y - currentPosition.y) * Constants.DEAD_WHEEL_TICKS_PER_INCH;
            double remainingTheta = targetHeading - currentHeading;

            // Break condition
            if (Math.abs(remainingX) < Constants.MINIMUM_DISTANCE && Math.abs(remainingY) < Constants.MINIMUM_DISTANCE && Math.abs(remainingTheta) < LMMHS.turnTolerance()) {
                telemetry.addLine("Breaks due to tolerance");
                telemetry.update();
                break;
            }
            double time = timer.seconds();
            armTime = armTimer.seconds();

            // Calculate power outputs using PID
            XYValue motorPower = xyPID.calculatePower(currentPosition, currentHeading, timer.seconds());
            double turnPower = turnPID.calculatePower(currentHeading, timer.seconds());
            double armPower = armPID.calculatePower(currentHeight, timer.seconds());

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

            double remainingArm = (targetHeight - currentHeight) * Constants.DEAD_WHEEL_TICKS_PER_INCH;
            
            if (remainingArm < Constants.MINIMUM_DISTANCE)
            {
                arms.setHoldingPower();
            }
            else
            {
                arms.setArmPowers(armPower); //need to add something that will keep thhe arm up there when it reaches the tolerance
            }

            //handleArmWithTime(initialArmAngle,initialExtend,targetArmAngle,targetExtend,targetArmAngleTime,targetExtendTime);

            updatePosition();
        }
        motors.stopMotors();
    }

    public void updatePosition() {

        int EncoderDrive = deadWheels.getCurrentValue(DeadWheel.DRIVE); //in ticks
        int EncoderStrafe = deadWheels.getCurrentValue(DeadWheel.STRAFE);
        int EncoderArm = arms.getAverageCurrentPosition();

        /*telemetry.addData("EncoderDrive", EncoderDrive / Constants.DEAD_WHEEL_TICKS_PER_INCH);
        telemetry.addData("EncoderStrafe", EncoderStrafe / Constants.DEAD_WHEEL_TICKS_PER_INCH);
        telemetry.addData("EncoderArm", EncoderArm / Constants.DEAD_WHEEL_TICKS_PER_INCH);
        telemetry.update();*/


        /// IMU Heading in Degrees
        double imuHeading = imu.getCurrentHeading();

        /// Get changes in encoder values
        int deltaDrive = EncoderDrive - deadWheels.getPreviousValue(DeadWheel.DRIVE);
        int deltaStrafe = EncoderStrafe - deadWheels.getPreviousValue(DeadWheel.STRAFE);
        int deltaArm = EncoderArm - arms.getPreviousArm();

        /// deltaTheta from IMU
        double deltaThetaIMU = imuHeading - imu.getPreviousHeading();

        /// Encoder Height, Heading
        currentHeight = currentHeight + deltaArm;
        currentHeading = currentHeading + deltaThetaIMU;
        currentArmAngle = arms.getCurrentAngledPosition();
        currentExtend = arms.getCurrentExtend();

        /// Update previous encoder values
        deadWheels.setPreviousDrive(EncoderDrive); //in ticks
        deadWheels.setPreviousStrafe(EncoderStrafe);
        arms.setPreviousArm(EncoderArm);

        imu.setPreviousHeading(imuHeading);

        //double encoderHeadingNormalized = gyros.normalizeHeading(currentHeading + deltaTheta);

        //NEW telemetry and normalizing currentHeading
        //currentHeading = imu.normalizeHeading(currentHeading + (deltaIMU));
//        telemetry.addData("deltaTheta", deltaTheta);
//        telemetry.addData("yPower", deltaIMU);
//        telemetry.addData("currentHeading", currentHeading);
//        telemetry.update();

        // Local displacements
        double deltaYLocal = (deltaDrive - LMMHS.arcLength(Constants.DRIVE_RADIUS, deltaThetaIMU));
        double deltaXLocal = (deltaStrafe - LMMHS.arcLength(Constants.STRAFE_RADIUS, deltaThetaIMU));

        // Transform local displacements to global coordinates
        // changed the signs for globals
        double deltaYGlobal = deltaXLocal * LMMHS.sin(currentHeading) + deltaYLocal * LMMHS.cos(currentHeading);
        double deltaXGlobal = deltaXLocal * LMMHS.cos(currentHeading) - deltaYLocal * LMMHS.sin(currentHeading);

        //final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();

        //XYValue aprilPosition = new XYValue(0, 0);

        /*if (currentAprilTagPosition != null) {
            aprilPosition.x = currentAprilTagPosition.getPosition().x;
            aprilPosition.y = currentAprilTagPosition.getPosition().y;
        }*/

        // Update global position
        currentPosition.x += (deltaXGlobal / Constants.DEAD_WHEEL_TICKS_PER_INCH);
        currentPosition.y += (deltaYGlobal / Constants.DEAD_WHEEL_TICKS_PER_INCH);

    }

    /*public void pickUpObject(){
        arms.openGrabber();
        arms.closeGrabber();
    }*/

    /*public void dropObject(){
        arms.setWristPosition(1.0);
        arms.openGrabber();
        arms.setWristPosition(0.0);
        //arms.
    }

     */

    /*public void hangObject(){
        arms.setWristPosition(0.0);
        arms.angleArmTo(0.3); //CHANGE 0.3
        arms.openGrabber();
    }

     */

    public void checkSensorReadings() {

        this.imu = new IMUUtility(this.hardwareMap);

        imu.resetIMU();
        deadWheels.resetEncoders();

        // double totalDeltaTheta = 0;

        while (opMode.opModeIsActive()) {

            //
            int EncoderDrive = deadWheels.getCurrentValue(DeadWheel.DRIVE); //in ticks
            int EncoderStrafe = deadWheels.getCurrentValue(DeadWheel.STRAFE);

            double imuHeading = imu.getCurrentHeading();

            // Calculate changes in encoder values
            int deltaDrive = EncoderDrive - deadWheels.getPreviousValue(DeadWheel.DRIVE);
            int deltaStrafe = EncoderStrafe - deadWheels.getPreviousValue(DeadWheel.STRAFE);

            double deltaIMU = (imuHeading - imu.getPreviousHeading());

            // Update previous encoder values
            deadWheels.setPreviousDrive(EncoderDrive); //in ticks
            deadWheels.setPreviousStrafe(EncoderStrafe);

            imu.setPreviousHeading(imuHeading);

            //final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();

            // Print Out Various Values
            /*telemetry.addData("Encoder Drive", EncoderDrive);
            telemetry.addData("Encoder Strafe", EncoderStrafe);*/
            telemetry.addData("Current X", currentPosition.x);
            //telemetry.addData("April X", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().x : "none");

            telemetry.addData("Current Y", currentPosition.y);
            //telemetry.addData("April Y", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().y : "none");

            telemetry.addData("Current Heading", LMMHS.reportAngle(currentHeading));
            telemetry.addData("IMU Raw", LMMHS.reportAngle(imuHeading));

            telemetry.update();

            updatePosition();
        }
    }
}
