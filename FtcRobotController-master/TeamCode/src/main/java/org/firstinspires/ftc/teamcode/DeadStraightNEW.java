package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Manual PID NEW ", group = "Concept")
public class DeadStraightNEW extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor rightDeadWheel;
    private DcMotor leftDeadWheel;
    private double motorTicks;
    private double currentValue;
    private double leftCurrentValue;
    private double rightCurrentValue;
    private double deltaTime;
    private PIDController PIDController;
    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private ElapsedTime timer;
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60);
    double inchesPerTick = (wheelCircumference / ticksPerRotation) / 2.54;
    double ticksPerInch = (ticksPerRotation / wheelCircumference) * 25.4;
    double leftOutput, rightOutput;
    double leftError, rightError;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontRight");
        rightDeadWheel = hardwareMap.get(DcMotor.class, "rightDeadWheel");
        leftDeadWheel = hardwareMap.get(DcMotor.class, "leftDeadWheel");


        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDeadWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // PIDController = new PIDController(0.00002, 0, 0, 0.1);
        leftPIDController = new PIDController(0.00002, 0, 0.0001, 0.1);
        rightPIDController = new PIDController(0.00002, 0, 0.0001, 0.1);

        //kP= 0.00003, kI= 0.000003, kD= 0.000005

        timer = new ElapsedTime();
        waitForStart();
        setPosition(48);
        timer.reset();
        leftPIDController.update(0, deltaTime);
        rightPIDController.update(0, deltaTime);
        telemetry.addData("Before Loop", "");
        telemetry.update();
        leftError = leftPIDController.error;
        rightError = rightPIDController.error;
        while (Math.abs(leftError) >= leftPIDController.tolerance && Math.abs(rightError) >= rightPIDController.tolerance) {
            leftCurrentValue = leftDeadWheel.getCurrentPosition();
            rightCurrentValue = rightDeadWheel.getCurrentPosition();
            deltaTime = timer.seconds();
            timer.reset();
            leftOutput = leftPIDController.update(leftCurrentValue, deltaTime);
            rightOutput = rightPIDController.update(rightCurrentValue, deltaTime);
            setIndividualMotorPower(leftOutput, rightOutput);
            doTelemetryData();
        }
        setMotorPower(0);
        setPosition(0);
        reset();
    }


    private void setIndividualMotorPower(double leftMotorPower, double rightMotorPower){
        backLeft.setPower(leftMotorPower);
        backRight.setPower(rightMotorPower);
        frontLeft.setPower(leftMotorPower);
        frontRight.setPower(rightMotorPower);

    }

    private void doTelemetryData(){
        telemetry.addData("Left Wheel", leftDeadWheel.getCurrentPosition());
        telemetry.addData("Right Wheel", rightDeadWheel.getCurrentPosition());
        telemetry.addData("Target Ticks", motorTicks);
        telemetry.addData("ElapsedTime", deltaTime);
        telemetry.addData("Left Error", leftPIDController.error);
        telemetry.addData("Right Error", rightPIDController.error);
        telemetry.addData("Left Current Value", leftCurrentValue);
        telemetry.addData("Right Current Value", rightCurrentValue);
        telemetry.addData("Tolerance", leftPIDController.tolerance);
        telemetry.addData("Tolerance", rightPIDController.tolerance);
        telemetry.addData("Motor Power Left", leftOutput);
        telemetry.addData("Motor Power Right", rightOutput);
        telemetry.addData("left Derivative", leftPIDController.derivative);
        telemetry.addData("right Derivative", rightPIDController.derivative);
        telemetry.update();
    }

    private void setMotorPower(double motorPower){
        backLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        frontRight.setPower(motorPower);

    }
    private void reset(){
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void setPosition(double inches) {
        // I hate you - Cruz Swinson
        motorTicks = inches * ticksPerInch;
        leftPIDController.setTargetAmount(motorTicks);
        rightPIDController.setTargetAmount(motorTicks);
    }
}
