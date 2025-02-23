package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot {

    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

//    private final CameraPosition cameraPosition;
//    private final VisionUtility myAprilTagProcessor;

    private final MotorUtility motors;
    private final ArmUtility arm;
    private final DeadWheelUtility deadWheels;
    private IMUUtility imu;

    private final RobotState currentState;

    private double armTime;

    //------------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------------

    public Robot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, RobotState initialState, CameraPosition side) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.motors = new MotorUtility(this.hardwareMap);
        this.arm = new ArmUtility(this.hardwareMap);
        this.deadWheels = new DeadWheelUtility(this.hardwareMap);
        this.imu = new IMUUtility(this.hardwareMap);

        this.currentState = new RobotState(initialState);
        this.currentState.heading = LMMHS.setAngle(initialState.heading);

//        this.cameraPosition = side;
//        this.myAprilTagProcessor = new VisionUtility(this.hardwareMap, this.cameraPosition);
    }

    //------------------------------------------------------------------------------------------------
    // Primary Drive Function
    //------------------------------------------------------------------------------------------------

    public void moveToNewRobotState(RobotState targetState) {

        PIDDrive xyPID = new PIDDrive(currentState, targetState);
        PIDTurn turnPID = new PIDTurn(currentState, targetState);
        PIDArm armPID = new PIDArm(currentState, targetState);

        boolean driveLoop = true;
        boolean armLoop = true;

        deadWheels.resetEncoders();

        while (opMode.opModeIsActive() && driveLoop && armLoop) {

            ElapsedTime timer = new ElapsedTime();

            XYValue motorPower;
            double turnPower;

            while (driveLoop) {

                // Calculate power outputs using PID
                motorPower = xyPID.calculatePower(currentState.position, currentState.heading, timer.seconds());
                turnPower = turnPID.calculatePower(currentState.heading, timer.seconds());
                timer.reset();

                setDriveMotors(motorPower, turnPower);

                updateRobotPosition();

                telemetry.addData("Target X", targetState.position.x);
                telemetry.addData("CurrentX", currentState.position.x);
                telemetry.addData("Target Y", targetState.position.y);
                telemetry.addData("CurrentY", currentState.position.y);
                telemetry.addData("Target Heading ", targetState.heading);
                telemetry.addData("Current Heading", currentState.heading);
                telemetry.addData("xPower", motorPower.x);
                telemetry.addData("yPower", motorPower.y);
                telemetry.addData("turnPower", turnPower);
                telemetry.update();

                if (xyPID.arrivedAtX() && xyPID.arrivedAtY() && turnPID.arrivedAtTheta()) {
                    driveLoop = false;
                }
            }

            // Now do all the arm movements

            double initialShoulderAngle = arm.getShoulderPosition();
            double elapsedTime = 1.5;
            timer.reset();

            while (timer.seconds() < elapsedTime) {

                if (arm.getShoulderPosition() != targetState.armAngle) {
                    double ratioArm = Range.clip((timer.seconds() / elapsedTime), 0, 1);
                    arm.rotateArmToAngle(initialShoulderAngle + (ratioArm * (targetState.armAngle - initialShoulderAngle)));
                }
            }
            currentState.armAngle = arm.getShoulderPosition();

            while (armLoop) {

                double armPower = armPID.calculatePower(currentState.armHeight, timer.seconds());
                double remainingArm = (targetState.armHeight - currentState.armHeight) * Constants.DEAD_WHEEL_TICKS_PER_INCH;

                if (Math.abs(remainingArm) < Constants.MINIMUM_DISTANCE_IN_TICKS) {
                    arm.setHoldingPower();
                } else {
                    arm.setArmPowers(armPower); //need to add something that will keep thhe arm up there when it reaches the tolerance
                }

                updateArmPosition();

                if (armPID.arrivedAtHeight()) {
                    armLoop = false;
                }

            }

            if (currentState.wristIsUp != targetState.wristIsUp) {
                if (targetState.wristIsUp) {
                    turnWristUp();
                } else {
                    turnWristDown();
                }
                arm.sleep(500);
            }

            if (currentState.grabberIsOpen != targetState.grabberIsOpen) {
                if (targetState.grabberIsOpen) {
                    openGrabber();
                } else {
                    closeGrabber();
                }
                arm.sleep(500);
            }

            //handleArmWithTime(initialArmAngle,initialExtend,targetArmAngle,targetExtend,targetArmAngleTime,targetExtendTime);

            motors.stopMotors();
            arm.stopMotors();
            break;
        }
    }

    //------------------------------------------------------------------------------------------------
    // Arm Functions
    //------------------------------------------------------------------------------------------------

    public void extendElbow() {
        ElapsedTime armTimer = new ElapsedTime();
        double initialShoulderAngle = arm.getShoulderPosition();
        double initialElbowAngle = 0;
        double elapsedTime = 1.5;
        double targetAngle = 0.5;

        while (armTimer.seconds() < elapsedTime) {

            if (arm.getShoulderPosition() != targetAngle) {
                double ratioArm = Range.clip((armTimer.seconds() / elapsedTime), 0, 1);
                arm.rotateArmToAngle(initialShoulderAngle + (ratioArm * (targetAngle - initialShoulderAngle)));
            }
            if (arm.getElbowPosition() < Constants.ELBOW_EXTEND_SETTING) {
                double ratioElbow = Range.clip((armTimer.seconds() / elapsedTime), 0, 1);
                arm.extendElbow(initialElbowAngle + (ratioElbow * (Constants.ELBOW_EXTEND_SETTING - initialElbowAngle)));
            }
        }
        currentState.armAngle = arm.getShoulderPosition();
    }

    public void justArm(double targetAngle, double targetExtend, double targetAngleTime, double targetExtendTime, boolean open) {
        ElapsedTime armTimer = new ElapsedTime();
        double initialAngle = arm.getShoulderPosition();
        double initialExtend = arm.getElbowPosition();
        double presentAngle = 0;
        double presentExtend = 0;

        double boundTime = Math.max(targetAngleTime, targetExtendTime);
        telemetry.addData("bound time", boundTime);
        telemetry.update();
        double armTime = armTimer.seconds();

        while (opMode.opModeIsActive()) {
            while (armTime <= boundTime) {
                if (arm.getShoulderPosition() != targetAngle) {
                    double ratioArm = Range.clip((armTime / targetAngleTime), 0, 1);
                    presentAngle = initialAngle + (ratioArm * (targetAngle - initialAngle));
                    arm.rotateArmToAngle(presentAngle);
                }
                if (arm.getElbowPosition() != targetExtend) {

                    double ratioExtend = Range.clip((armTime / targetExtendTime), 0, 1);
                    presentExtend = initialExtend + (ratioExtend * (targetExtend - initialExtend));
                    arm.extendElbow(presentExtend);
                }
                telemetry.addData("currentAngle", arm.getShoulderPosition());
                telemetry.addData("currentExtend", arm.getElbowPosition());
                telemetry.addData("presentAngle", presentAngle);
                telemetry.addData("presentExtend", presentExtend);
                telemetry.update();

                armTime = armTimer.seconds();
            }
            telemetry.addLine("Breaking out");
            break;
        }

        if (open) {
            arm.openGrabber();
        } else {
            arm.closeGrabber();
        }
    }

    public void handleArmWithTime(double initialAngle, double initialExtend, double targetAngle, double targetExtend, double targetAngleTime, double targetExtendTime) {
        double presentAngle = 0;
        double presentExtend = 0;

        double boundTime = Math.max(targetAngleTime, targetExtendTime);

        if (armTime <= boundTime) {
            if (arm.getShoulderPosition() != targetAngle) {
                double ratioArm = Range.clip((armTime / targetAngleTime), 0, 1);
                presentAngle = initialAngle + (ratioArm * (targetAngle - initialAngle));
                arm.rotateArmToAngle(presentAngle);
            }
            if (arm.getElbowPosition() != targetExtend) {

                double ratioExtend = Range.clip((armTime / targetExtendTime), 0, 1);
                presentExtend = initialExtend + (ratioExtend * (targetExtend - initialExtend));
                arm.extendElbow(presentExtend);
            }
            telemetry.addData("currentAngle", arm.getShoulderPosition());
            telemetry.addData("currentExtend", arm.getElbowPosition());
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


    public void updateArmPosition() {
        int encoderArm = arm.getAverageCurrentPosition();
        int deltaArm = encoderArm - arm.getPreviousArm();

        currentState.armHeight = currentState.armHeight + deltaArm;
        currentState.armAngle = arm.getShoulderPosition();

        arm.setPreviousArm(encoderArm);
    }

    public void updateRobotPosition() {

        int encoderDrive = deadWheels.getCurrentValue(DeadWheel.DRIVE); //in ticks
        int encoderStrafe = deadWheels.getCurrentValue(DeadWheel.STRAFE);

        // IMU Heading in Degrees
        double imuHeading = imu.getCurrentHeading();

        // Get changes in encoder values
        int deltaDrive = encoderDrive - deadWheels.getPreviousValue(DeadWheel.DRIVE);
        int deltaStrafe = encoderStrafe - deadWheels.getPreviousValue(DeadWheel.STRAFE);

        // deltaTheta from IMU
        double deltaThetaIMU = imuHeading - imu.getPreviousHeading();

        // Encoder Height, Heading
        currentState.heading = currentState.heading + deltaThetaIMU;

        // Update previous encoder values
        deadWheels.setPreviousDrive(encoderDrive); //in ticks
        deadWheels.setPreviousStrafe(encoderStrafe);

        imu.setPreviousHeading(imuHeading);

        //double encoderHeadingNormalized = gyros.normalizeHeading(currentHeading + deltaTheta);

        //NEW telemetry and normalizing currentHeading
        //currentHeading = imu.normalizeHeading(currentHeading + (deltaIMU));
//        telemetry.addData("deltaTheta", deltaTheta);
//        telemetry.addData("yPower", deltaIMU);
//        telemetry.addData("currentHeading", currentHeading);
//        telemetry.update();

        // Local displacements
        double deltaXLocal = (deltaStrafe - LMMHS.arcLength(Constants.STRAFE_RADIUS, deltaThetaIMU));
        double deltaYLocal = (deltaDrive - LMMHS.arcLength(Constants.DRIVE_RADIUS, deltaThetaIMU));

        // Transform local displacements to global coordinates
        // changed the signs for globals
        double deltaXGlobal = deltaXLocal * LMMHS.cos(currentState.heading) - deltaYLocal * LMMHS.sin(currentState.heading);
        double deltaYGlobal = deltaXLocal * LMMHS.sin(currentState.heading) + deltaYLocal * LMMHS.cos(currentState.heading);

        //final Pose3D currentAprilTagPosition = myAprilTagProcessor.getPose();

        //XYValue aprilPosition = new XYValue(0, 0);

        /*if (currentAprilTagPosition != null) {
            aprilPosition.x = currentAprilTagPosition.getPosition().x;
            aprilPosition.y = currentAprilTagPosition.getPosition().y;
        }*/

        // Update global position
        currentState.position.x += (deltaXGlobal / Constants.DEAD_WHEEL_TICKS_PER_INCH);
        currentState.position.y += (deltaYGlobal / Constants.DEAD_WHEEL_TICKS_PER_INCH);

    }

    public void setArmHeight(RobotState targetState) {
        ElapsedTime timer = new ElapsedTime();
        PIDArm armPID = new PIDArm(currentState, targetState);

        while (opMode.opModeIsActive()) {
            double time = timer.seconds();
            double armPower = armPID.calculatePower(currentState.armHeight, timer.seconds());
            arm.setArmPowers(armPower);
            updateRobotPosition();
            double tolerance = targetState.armHeight - currentState.armHeight;
            if (tolerance < 200) {
                arm.setHoldingPower();
            }
        }
        motors.stopMotors();
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

        while (opMode.opModeIsActive()) {

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
            updateRobotPosition();

            // Print Out Various Values
//            telemetry.addData("Encoder Drive", EncoderDrive);
//            telemetry.addData("Encoder Strafe", EncoderStrafe);
            telemetry.addData("Current X", currentState.position.x);
            //telemetry.addData("April X", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().x : "none");

            telemetry.addData("Current Y", currentState.position.y);
            //telemetry.addData("April Y", (currentAprilTagPosition != null) ? currentAprilTagPosition.getPosition().y : "none");

            telemetry.addData("Current Heading", LMMHS.reportAngle(currentState.heading));
            telemetry.addData("IMU Raw", LMMHS.reportAngle(imuHeading));

            telemetry.update();
        }
    }

    public void spin(double powerLevel) {

        deadWheels.resetEncoders();

        while (opMode.opModeIsActive()) {
            motors.spinInPlace(powerLevel);
            int EncoderDrive = deadWheels.getCurrentValue(DeadWheel.DRIVE); //in ticks
            int EncoderStrafe = deadWheels.getCurrentValue(DeadWheel.STRAFE);
            telemetry.addData("EncoderDrive", EncoderDrive);
            telemetry.addData("EncoderStrafe", EncoderStrafe);
            telemetry.addData("IMU Heading", imu.getCurrentHeading());
            telemetry.update();
        }
    }

    public boolean getGrabberState() {
        return currentState.grabberIsOpen;
    }

    public void openGrabber() {
        arm.openGrabber();
        currentState.grabberIsOpen = true;
    }

    public void closeGrabber() {
        arm.closeGrabber();
        currentState.grabberIsOpen = false;
    }

    public boolean getWristPosition() {
        return currentState.wristIsUp;
    }

    public void turnWristUp() {
        arm.wristUp();
        currentState.wristIsUp = true;
    }

    public void turnWristDown() {
        arm.wristDown();
        currentState.wristIsUp = false;
    }

    public void setDriveMotors(XYValue motorPower, double turnPower) {
        motors.setMotorPowers(motorPower.x, motorPower.y, turnPower);
    }

    public void setDriveMotorsDirectly(double fl, double fr, double bl, double br) {
        motors.setMotorsDirectly(fl, fr, bl, br);
    }

    public void stop() {
        motors.stopMotors();
    }
}
