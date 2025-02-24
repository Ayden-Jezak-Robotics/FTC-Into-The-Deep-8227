package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.concurrent.TimeUnit;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "IntoTheDeepTeleop", group = "Main")
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
    private int direction;
    private boolean isMovingUp, isMovingDown = false;
    private double leftStartPosition, rightStartPosition, progress;
    private ElapsedTime moveTimer = new ElapsedTime();
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
        leftArmServo.setPosition(1);
        rightArmServo.setPosition(0);
        elbowServo.setPosition(0.7);
        armUpFlag = 0;
        /*
        rightArmServo.setPosition(.25);
        leftArmServo.setPosition(.75);
        sleep(750);
        elbowServo.setPosition(0.7);
        sleep(750);
        rightArmServo.setPosition(1);
        leftArmServo.setPosition(0);
        sleep(500);
        wristServo.setPosition(0.8);
        wristPosition = "UP";*/
        while (opModeIsActive()){
            driveWheels();
            speedMulti();
            openGrabber();
            spinArmServosPrecise();
            spinArmServosNonprecise();
            spinWrist();
            //armMovement();
            //armBrake();


            updateTelemetry();
        }
    }
    private void speedMulti() {
        if (gamepad1.circle == true) {
            speedMulti += 1;
            if (speedMulti > 3) {
                speedMulti = 1.5;
            }
            while (gamepad1.circle == true) {
            }
        }
        if (gamepad1.cross == true) {
            if (direction == 1){
                direction = -1;
            }
            else{
                direction = 1;
            }
            while (gamepad1.cross == true) {
            }
        }
    }
    private void driveWheels(){
        if (direction == 1) {
            // Trig that I DO understand
            r = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
            robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180 - 45;
            rightX = gamepad1.right_stick_x * 0.75;
            v1 = r * Math.cos(robotAngle / 180 * Math.PI) + rightX;
            v2 = r * Math.sin(robotAngle / 180 * Math.PI) - rightX;
            v3 = r * Math.sin(robotAngle / 180 * Math.PI) + rightX;
            v4 = r * Math.cos(robotAngle / 180 * Math.PI) - rightX;
            frontLeft.setPower(v1 / speedMulti);
            frontRight.setPower(v2 / speedMulti);
            backLeft.setPower(v3 / speedMulti);
            backRight.setPower(v4 / speedMulti);
        }
        if (direction == -1) {
            // Trig that I DO understand
            r = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
            robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180 - 45;
            rightX = -gamepad1.right_stick_x * 0.75;
            v1 = r * Math.cos(robotAngle / 180 * Math.PI) + rightX;
            v2 = r * Math.sin(robotAngle / 180 * Math.PI) - rightX;
            v3 = r * Math.sin(robotAngle / 180 * Math.PI) + rightX;
            v4 = r * Math.cos(robotAngle / 180 * Math.PI) - rightX;
            frontLeft.setPower(-v1 / speedMulti);
            frontRight.setPower(-v2 / speedMulti);
            backLeft.setPower(-v3 / speedMulti);
            backRight.setPower(-v4 / speedMulti);
        }
    }
    private void armBrake(){
        if (armLeft.getCurrentPosition() < armCurrentHeight || armLeft.getCurrentPosition() > armCurrentHeight && gamepad2.right_stick_y == 0){
            armLeft.setPower(0.001 * (armCurrentHeight - armLeft.getCurrentPosition()));
        }
    }
    private void spinWrist(){
        if (gamepad2.left_bumper){
            if (wristPosition == "UP"){
                wristServo.setPosition(0.1);
                wristPosition = "DOWN";
            }
            else{
                wristServo.setPosition(0.8);
                wristPosition = "UP";
            }
            while (gamepad2.left_bumper == true) {
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
    /*
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

     */
    private void openGrabber(){
        if (gamepad2.right_bumper){
            if (grabberClosed == false){
                grabberServo.setPosition(0);
                grabberClosed = true;
            }
            else{
                grabberServo.setPosition(0.6);
                grabberClosed = false;
            }
            while (gamepad2.right_bumper == true) {
            }
        }
    }

    private void spinArmServosNonprecise(){
        if (gamepad2.dpad_down && !isMovingUp && !isMovingDown) {
            isMovingDown = true;
            moveTimer.reset();
            leftStartPosition = leftArmServo.getPosition();
            rightStartPosition = rightArmServo.getPosition();
        }
        if (isMovingDown) {
            long elapsedTime = moveTimer.time(TimeUnit.MILLISECONDS);
            progress = (double) elapsedTime / 500;
            if (progress >= 1.0) {
                progress = 1.0;
                isMovingDown = false;
            }
            //Max left position = 0.8
            double newLeftPosition = leftStartPosition + progress * (1 - leftStartPosition);
            double newRightPosition = rightStartPosition - progress * (rightStartPosition);
            newLeftPosition = Range.clip(newLeftPosition,0,0.8);
            newRightPosition = Range.clip(newRightPosition,0.2,1);
            leftArmServo.setPosition(newLeftPosition);
            rightArmServo.setPosition(newRightPosition);
        }

        if (gamepad2.dpad_up && !isMovingDown && !isMovingUp) {
            isMovingUp = true;
            moveTimer.reset();
            leftStartPosition = leftArmServo.getPosition();
            rightStartPosition = rightArmServo.getPosition();
        }
        if (isMovingUp) {
            long elapsedTime = moveTimer.time(TimeUnit.MILLISECONDS);
            progress = (double) elapsedTime / 500;
            if (progress >= 1.0) {
                progress = 1.0;
                isMovingUp = false;
            }
            //Max right is 0.8
            double newLeftPosition = leftStartPosition - progress * (0 + leftStartPosition);
            double newRightPosition = rightStartPosition + progress * (1 - rightStartPosition);
            newLeftPosition = Range.clip(newLeftPosition,0,0.8);
            newRightPosition = Range.clip(newRightPosition,0.2,1);
            leftArmServo.setPosition(newLeftPosition);
            rightArmServo.setPosition(newRightPosition);
        }

    }

    private void spinArmServosPrecise(){
        if (Math.abs(gamepad2.left_stick_y) > 0.05) { // Deadzone
            isMovingUp = false;
            isMovingDown = false;
            leftArmServo.setPosition(leftArmServo.getPosition() + gamepad2.left_stick_y * 0.01);
            rightArmServo.setPosition(rightArmServo.getPosition() + -gamepad2.left_stick_y * 0.01);
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
        telemetry.addData("Arm Progress", progress);
        telemetry.addData("Moving down?", isMovingDown);
        telemetry.addData("Moving up?", isMovingUp);
        telemetry.addData("Direction", direction);
        telemetry.update();
    }
    private double map(double in1, double in2, double out1, double out2, double value){
        return out1 + ((value - in1)*(out2 - out1))/(in2 - in1);
    }
}