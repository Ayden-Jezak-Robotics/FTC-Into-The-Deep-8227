package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "DeadStraightNew", group = "Concept")
public class DeadStraightNEW extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor rightDeadWheel;
    private DcMotor leftDeadWheel;
    private DcMotor centerDeadWheel;
    private double motorTicks;
    private double currentValue;
    private double leftCurrentValue, rightCurrentValue, centerCurrentValue;
    private double deltaTime;
    private PIDStrafer StraferPIDController;
    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private ElapsedTime timer;
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60);
    double inchesPerTick = (wheelCircumference / ticksPerRotation) / 2.54;
    double ticksPerInch = (ticksPerRotation / wheelCircumference) * 25.4;
    double leftOutput, rightOutput, centerOutput;
    double leftError, rightError, centerError;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rightDeadWheel = hardwareMap.get(DcMotor.class, "rightDeadWheel");
        leftDeadWheel = hardwareMap.get(DcMotor.class, "leftDeadWheel");
        centerDeadWheel = hardwareMap.get(DcMotor.class, "centerDeadWheel");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        rightDeadWheel.setDirection(DcMotor.Direction.REVERSE);
        leftDeadWheel.setDirection(DcMotor.Direction.FORWARD);
        centerDeadWheel.setDirection(DcMotor.Direction.FORWARD);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //0.00002, 0, 0.0001, 0.1
        leftPIDController = new PIDController(0.00002, 0.0001, 0.001, 0.1);
        rightPIDController = new PIDController(0.00002, 0.0001, 0.001, 0.1);
        StraferPIDController = new PIDStrafer(0.00002, 0, 0, 0.3);

        //kP= 0.00003, kI= 0.000003, kD= 0.000005

        //part which makes it actually move
        timer = new ElapsedTime();

        waitForStart();
        telemetry.addData("LOOK",getRuntime());
        setPosition(12);
        driveStraight();
        doAllTelemetryData();
        reset();
    }


    private void setIndividualMotorPower(double leftMotorPower, double rightMotorPower){
        backLeft.setPower(leftMotorPower);
        backRight.setPower(rightMotorPower);
        frontLeft.setPower(leftMotorPower);
        frontRight.setPower(rightMotorPower);

    }

    private void driveStraight(){
        timer.reset();
        leftOutput = leftPIDController.update(leftCurrentValue, deltaTime);
        rightOutput = rightPIDController.update(rightCurrentValue, deltaTime);
        leftError = leftPIDController.error;
        rightError = rightPIDController.error;
        while (Math.abs(leftError) >= leftPIDController.tolerance && Math.abs(rightError) >= rightPIDController.tolerance) {
            leftCurrentValue = leftDeadWheel.getCurrentPosition();
            rightCurrentValue = rightDeadWheel.getCurrentPosition();
            deltaTime = timer.seconds();
            timer.reset();
            leftOutput = leftPIDController.update(leftCurrentValue, deltaTime);
            rightOutput = rightPIDController.update(rightCurrentValue, deltaTime);
            leftError = leftPIDController.error;
            rightError = rightPIDController.error;
            setIndividualMotorPower(leftOutput, rightOutput);
            doAllTelemetryData();
        }
    }
    private void strafe (){
        timer.reset();
        StraferPIDController.update(centerDeadWheel.getCurrentPosition(), deltaTime);
        while (Math.abs(StraferPIDController.error) >= StraferPIDController.tolerance) {
            centerCurrentValue = centerDeadWheel.getCurrentPosition();
            deltaTime = timer.seconds();
            timer.reset();
            centerOutput = StraferPIDController.update(centerCurrentValue, deltaTime);
            setStrafingMotorPower(centerOutput);
            doStrafingData();
        }
    }

    private void doAllTelemetryData(){
        telemetry.addData("Left Wheel", leftDeadWheel.getCurrentPosition());
        telemetry.addData("Right Wheel", rightDeadWheel.getCurrentPosition());
        telemetry.addData("Center Wheel", centerDeadWheel.getCurrentPosition());
        telemetry.addData("Target Ticks", motorTicks);
        telemetry.addData("ElapsedTime", deltaTime);
        telemetry.addData("Left Error", leftPIDController.error);
        telemetry.addData("Right Error", rightPIDController.error);
        telemetry.addData("Center Error", StraferPIDController.error);
        telemetry.addData("Left Current Value", leftCurrentValue);
        telemetry.addData("Right Current Value", rightCurrentValue);
        telemetry.addData("Center Current Value", centerCurrentValue);
        telemetry.addData("Left Tolerance", leftPIDController.tolerance);
        telemetry.addData("Right Tolerance", rightPIDController.tolerance);
        telemetry.addData("Center Tolerance", StraferPIDController.tolerance);
        telemetry.addData("Motor Power Left", leftOutput);
        telemetry.addData("Motor Power Right", rightOutput);
        telemetry.addData("left Derivative", leftPIDController.derivative);
        telemetry.addData("right Derivative", rightPIDController.derivative);
        telemetry.update();
    }
    private void doStrafingData(){
        telemetry.addData("Center Wheel", centerDeadWheel.getCurrentPosition());
        telemetry.addData("Center Error", StraferPIDController.error);
        telemetry.addData("Center Current Value", centerCurrentValue);
        telemetry.addData("Center Tolerance", StraferPIDController.tolerance);
        telemetry.update();
    }

    private void setAllMotorPower(double motorPower){
        backLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        frontRight.setPower(motorPower);

    }

    private void setStrafingMotorPower(double motorPower){
        backLeft.setPower(motorPower);
        backRight.setPower(-motorPower);
        frontLeft.setPower(motorPower);
        frontRight.setPower(-motorPower);
    }

    private void reset(){
        setAllMotorPower(0);
        setPosition(0);
        setStrafePosition(0);
        leftPIDController.reset();
        rightPIDController.reset();

        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setStrafePosition(double inches)
    {
        motorTicks = inches * ticksPerInch;
        StraferPIDController.setTargetAmount(motorTicks);
    }
    private void setPosition(double inches) {
        // I hate you - Cruz Swinson
        motorTicks = inches * ticksPerInch;
        leftPIDController.setTargetAmount(motorTicks);
        rightPIDController.setTargetAmount(motorTicks);
    }
}
