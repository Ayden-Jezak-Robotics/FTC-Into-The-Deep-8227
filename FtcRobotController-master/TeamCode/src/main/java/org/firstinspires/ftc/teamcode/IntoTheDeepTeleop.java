package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "Into The Deep Teleop", group = "Main")
public class IntoTheDeepTeleop extends LinearOpMode
{

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor armLeft, armRight;
    private  Servo wristServo, grabberServo, leftArmServo, rightArmServo, elbowServo;
    private double r;
    private double robotAngle;
    private double rightX;
    private double v1, v2, v3, v4;
    private double speedMulti = 4;
    private boolean grabberClosed = false;
    private int armUpFlag = 0;
    private double armCurrentHeight, armCurrentHeightFlag;
    private String wristPosition;
    int xPressed = 0;


    @Override
    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        wristServo.setPosition(0.1);
        wristPosition = "DOWN";
        leftArmServo.setPosition(0.8);
        rightArmServo.setPosition(0.2);
        armUpFlag = 0;
        while (opModeIsActive()){
            elbowServo.setPosition(0);
            driveWheels();
            speedMulti();
            openGrabber();
            spinArmServos();
            spinWrist();
            armMovement();
            //armBrake();


            updateTelemetry();
        }
    }
    private void speedMulti() {
        if (gamepad1.circle == true) {
            speedMulti += 2;
            if (speedMulti > 4) {
                speedMulti = 1;
            }
            while (gamepad1.circle == true) {
            }
        }
    }
    private void driveWheels(){
        // Trig that I DO understand
        r = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180 - 45;
        rightX = gamepad1.right_stick_x * 0.75;
        v1 = r * Math.cos(robotAngle / 180 * Math.PI) + rightX;
        v2 = r * Math.sin(robotAngle / 180 * Math.PI) - rightX;
        v3 = r * Math.sin(robotAngle / 180 * Math.PI) + rightX;
        v4 = r * Math.cos(robotAngle / 180 * Math.PI) - rightX;
        frontLeft.setPower(v1/speedMulti);
        frontRight.setPower(v2/speedMulti);
        backLeft.setPower(v3/speedMulti);
        backRight.setPower(v4/speedMulti);
    }
    private void armBrake(){
       if (armLeft.getCurrentPosition() < armCurrentHeight || armLeft.getCurrentPosition() > armCurrentHeight && gamepad2.right_stick_y == 0){
           armLeft.setPower(0.001 * (armCurrentHeight - armLeft.getCurrentPosition()));
       }
    }
    private void spinWrist(){
        if (gamepad2.circle){
            if (wristPosition == "UP"){
                wristServo.setPosition(0.1);
                wristPosition = "DOWN";
            }
            else{
                wristServo.setPosition(0.8);
                wristPosition = "UP";
            }
            while (gamepad2.circle == true) {
            }
        }
    }

    private double getArmSpeed(){
        if (armLeft.getCurrentPosition() > 3500 && armRight.getCurrentPosition() > 3500){
            return 0.75 * map(3500, 4300, 1, 0.1, armLeft.getCurrentPosition());
        }
        else{
            return 0.75;
        }

    }
    private void armMovement(){
        if (armLeft.getCurrentPosition() >= 0 && armRight.getCurrentPosition() >= 0) {
           armLeft.setPower(-gamepad2.right_stick_y * getArmSpeed());
           armRight.setPower(-gamepad2.right_stick_y * getArmSpeed());
           armCurrentHeight = armLeft.getCurrentPosition();
        }
        if( gamepad2.right_bumper) {
            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(gamepad2.right_bumper){

            }
            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (armLeft.getCurrentPosition() < 0 && gamepad2.right_stick_y == 0){
            armLeft.setPower(0.1);
        }
        if (armRight.getCurrentPosition() < 0 && gamepad2.right_stick_y == 0){
            armRight.setPower(0.1);
        }
    }
    private void openGrabber(){
        if (gamepad2.square){
            if (grabberClosed == false){
                grabberServo.setPosition(0);
                grabberClosed = true;
            }
            else{
                grabberServo.setPosition(0.6);
                grabberClosed = false;
            }
            while (gamepad2.square == true) {
            }
        }
    }

    private void spinArmServos(){
        if (gamepad2.triangle){
            if (armUpFlag == 0){
                rightArmServo.setPosition(0.6);
                leftArmServo.setPosition(0.4);
                armUpFlag = 1;
            } else {
                rightArmServo.setPosition(0.4);
                leftArmServo.setPosition(0.6);
                sleep(2000);
                rightArmServo.setPosition(0.3);
                leftArmServo.setPosition(0.7);
                sleep(2000);
                rightArmServo.setPosition(0.2);
                leftArmServo.setPosition(0.8);
                armUpFlag = 0;
            }
            while (gamepad2.triangle == true) {
            }
        }
    }
    private void updateTelemetry(){
        telemetry.addData("Speed", speedMulti);
        telemetry.addData("Arm Position", rightArmServo.getPosition());
        telemetry.addData("Arm Height", armLeft.getCurrentPosition());
        telemetry.addData("Ideal Arm Height", armCurrentHeight);
        telemetry.addData("Grabber Position", grabberServo.getPosition());
        telemetry.addData("armSpeed", getArmSpeed());
        telemetry.addData("leftArmServo Position", leftArmServo.getPosition());
        telemetry.addData("rightArmServo Position", rightArmServo.getPosition());
        telemetry.update();
    }
    private double map(double in1, double in2, double out1, double out2, double value){
        return out1 + ((value - in1)*(out2 - out1))/(in2 - in1);
    }
}
