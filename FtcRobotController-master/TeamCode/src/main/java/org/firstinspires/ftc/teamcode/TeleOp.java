package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.concurrent.TimeUnit;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "Teleop", group = "Main")
public class IntoTheDeepTeleop extends LinearOpMode
{

    // TODO: What will the initial state of the robot be when TeleOp starts?
    RobotState initialState = new RobotState(0, 0, 0, 0, 0, 0, 0, 0);

    // private double r;
    // private double robotAngle;
    // private double rightX;
    // private double v1, v2, v3, v4;
    private double speedMultiplier = 4;
    private boolean grabberClosed = false;
    private boolean wristPositionIsUp = true;
    private int armUpFlag = 0;
    private double armCurrentHeight, armCurrentHeightFlag;
    // private String wristPosition;
    private boolean movingForward = true;
    private boolean isMovingUp, isMovingDown = false;
    private double leftStartPosition, rightStartPosition, progress;
    private ElapsedTime moveTimer = new ElapsedTime();
    int xPressed = 0;


    @Override
    public void runOpMode(){

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        // TODO: What position should servos initialize to at start of TeleOp?

        wristServo.setPosition(0.1);
        wristPosition = "DOWN";
        leftArmServo.setPosition(1);
        rightArmServo.setPosition(0);
        armUpFlag = 0;
        rightArmServo.setPosition(.25);
        leftArmServo.setPosition(.75);
        sleep(750);
        elbowServo.setPosition(1);
        sleep(750);
        rightArmServo.setPosition(1);
        leftArmServo.setPosition(0);
        sleep(500);
        wristServo.setPosition(0.8);
        wristPosition = "UP";


        while (opModeIsActive()){

            driveWheels();
            decreaseSpeed();
            switchDirection();
            openGrabber();
            spinArmServosPrecise();
            spinArmServosNonprecise();
            spinWrist();
            armMovement();
            //armBrake();

            updateTelemetry();
        }
    }
    private void decreaseSpeed() {
        if (gamepad1.triangle) {
            speedMultiplier += 2;
            if (speedMultiplier > 4) {
                speedMultiplier = 1;
            }
            while (gamepad1.triangle) {
            }
        }
    }

    private void switchDirection() {
        if (gamepad1.cross) {
            if (movingForward){
                movingForward = false;
            }
            else{
                movingForward = true;
            }
            while (gamepad1.cross) {
            }
        }
    }

    private void driveWheels(){

        double r = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180 - 45;
        double rightX = gamepad1.right_stick_x * 0.75;
        double fl = (r * Math.cos(robotAngle / 180 * Math.PI) + rightX) / speedMultiplier;
        double fr = (r * Math.sin(robotAngle / 180 * Math.PI) - rightX) / speedMultiplier;
        double bl = (r * Math.sin(robotAngle / 180 * Math.PI) + rightX) / speedMultiplier;
        double br = (r * Math.cos(robotAngle / 180 * Math.PI) - rightX) / speedMultiplier;

        if (movingForward) {
            robot.motors.setPowerDirectly(fl, fr, bl, br);
        } else {
            robot.motors.setPowerDirectly(-fl, -fr, -bl, -br);
        }
    }

    private void armBrake(){
        if (armLeft.getCurrentPosition() < armCurrentHeight || armLeft.getCurrentPosition() > armCurrentHeight && gamepad2.right_stick_y == 0){
            armLeft.setPower(0.001 * (armCurrentHeight - armLeft.getCurrentPosition()));
        }
    }

    private void spinWrist(){
        if (gamepad2.circle) {
            if (wristPositionIsUp){
                robot.arms.wristDown();
                wristPositionIsUp = false;
            }
            else{
                robot.arms.wristUp();
                wristPositionIsUp = true;
            }
            while (gamepad2.circle) {
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

    private void spinArmServosNonprecise(){
        if (gamepad2.dpad_down && !isMovingUp && !isMovingDown) {
            isMovingDown = true;
            moveTimer.reset();
            leftStartPosition = leftArmServo.getPosition();
            rightStartPosition = rightArmServo.getPosition();
        }
        if (isMovingDown) {
            long elapsedTime = moveTimer.time(TimeUnit.MILLISECONDS);
            progress = (double) elapsedTime / 1500;
            if (progress >= 1.0) {
                progress = 1.0;
                isMovingDown = false;
            }
            //Max left position = 0.8
            double newLeftPosition = leftStartPosition + progress * (0.8 - leftStartPosition);
            double newRightPosition = rightStartPosition - progress * (0 + rightStartPosition);
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
            double newRightPosition = rightStartPosition + progress * (0.8 - rightStartPosition);
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
