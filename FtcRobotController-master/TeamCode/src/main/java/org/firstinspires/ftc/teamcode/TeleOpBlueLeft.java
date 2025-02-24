package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "TeleOpBlueLeft", group = "Main")
public class TeleOpBlueLeft extends LinearOpMode {
    RobotState initialState = new RobotState(-53, -53, -45, 0, 0, false, false);

    private boolean movingForward = true;
    private boolean isMovingUp, isMovingDown = false;
    private double leftStartPosition, rightStartPosition, progress, triggerMultiplier;
    private ElapsedTime moveTimer = new ElapsedTime();
    private boolean grabberToggle = false;
    private boolean wristToggle = false;
    private boolean directionToggle = false;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        while (opModeIsActive()) {

            driveWheels(robot);
            switchDirection();
            openGrabber(robot);
            spinArmServosPrecise();
            spinArmServosNonprecise();
            spinWrist(robot);
            armMovement();
            armBrake(robot);

            updateTelemetry(robot);
        }
    }

    //------------------------------------------------------------------------------------------------
    // Game Pad 1
    //------------------------------------------------------------------------------------------------

    private void driveWheels(Robot robot) {

        triggerMultiplier = (1.0 / 3.0) + ((2.0 / 3.0) * (1.0 - gamepad1.left_trigger));

        double r = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180 - 45;
        double rightX = gamepad1.right_stick_x * 0.75;
        double fl = (r * Math.cos(robotAngle / 180 * Math.PI) + rightX) * triggerMultiplier;
        double fr = (r * Math.sin(robotAngle / 180 * Math.PI) - rightX) * triggerMultiplier;
        double bl = (r * Math.sin(robotAngle / 180 * Math.PI) + rightX) * triggerMultiplier;
        double br = (r * Math.cos(robotAngle / 180 * Math.PI) - rightX) * triggerMultiplier;

        if (movingForward) {
            robot.setDriveMotorsDirectly(fl, fr, bl, br);
        } else {
            robot.setDriveMotorsDirectly(-fl, -fr, -bl, -br);
        }
    }

    private void switchDirection() {
        if (gamepad1.cross && !directionToggle) {
            directionToggle = true;

            if (movingForward) {
                movingForward = false;
            } else {
                movingForward = true;
            }
        }
        if (!gamepad1.cross) { // Reset toggle when button is released
            directionToggle = false;
        }
    }

    //------------------------------------------------------------------------------------------------
    // Game Pad 2
    //------------------------------------------------------------------------------------------------

    private void openGrabber(Robot robot) {
        if (gamepad2.square && !grabberToggle) { // Button press detected
            grabberToggle = true; // Mark the button as "pressed"

            if (robot.getGrabberState()) {
                robot.closeGrabber();
            } else {
                robot.openGrabber();
            }
        }

        if (!gamepad2.square) { // Reset toggle when button is released
            grabberToggle = false;
        }
    }

    private void spinWrist(Robot robot) {
        if (gamepad2.circle && !wristToggle) {
            wristToggle = true;

            if (robot.getWristPosition()) {
                robot.turnWristDown();
            } else {
                robot.turnWristUp();
            }
        }
        if (!gamepad2.circle) { // Reset toggle when button is released
            grabberToggle = false;
        }
    }

    private void spinArmServosPrecise() {
        if (Math.abs(gamepad2.left_stick_y) > 0.05) { // Deadzone
            isMovingUp = false;
            isMovingDown = false;
//            leftArmServo.setPosition(leftArmServo.getPosition() + gamepad2.left_stick_y * 0.01);
//            rightArmServo.setPosition(rightArmServo.getPosition() + -gamepad2.left_stick_y * 0.01);
        }
    }

    private void spinArmServosNonprecise() {
        if (gamepad2.dpad_down && !isMovingUp && !isMovingDown) {
            isMovingDown = true;
            moveTimer.reset();
//            leftStartPosition = leftArmServo.getPosition();
//            rightStartPosition = rightArmServo.getPosition();
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
//            leftArmServo.setPosition(newLeftPosition);
//            rightArmServo.setPosition(newRightPosition);
        }

        if (gamepad2.dpad_up && !isMovingDown && !isMovingUp) {
            isMovingUp = true;
            moveTimer.reset();
//            leftStartPosition = leftArmServo.getPosition();
//            rightStartPosition = rightArmServo.getPosition();
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
//            leftArmServo.setPosition(newLeftPosition);
//            rightArmServo.setPosition(newRightPosition);
        }

    }

    //------------------------------------------------------------------------------------------------
    // Utilities
    //------------------------------------------------------------------------------------------------

    private void armBrake(Robot robot) {
//        if (armLeft.getCurrentPosition() < armCurrentHeight || armLeft.getCurrentPosition() > armCurrentHeight && gamepad2.right_stick_y == 0){
//            armLeft.setPower(0.001 * (armCurrentHeight - armLeft.getCurrentPosition()));
//        }
    }

    private void armMovement() {
//        if (armLeft.getCurrentPosition() >= 0 && armRight.getCurrentPosition() >= 0) {
//            armLeft.setPower(-gamepad2.right_stick_y * getArmSpeed());
//            armRight.setPower(-gamepad2.right_stick_y * getArmSpeed());
//            armCurrentHeight = armLeft.getCurrentPosition();
//        }
        if (gamepad2.right_bumper) {
//            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while (gamepad2.right_bumper) {

            }
//            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
//        if (armLeft.getCurrentPosition() < 0 && gamepad2.right_stick_y == 0){
//            armLeft.setPower(0.1);
//        }
//        if (armRight.getCurrentPosition() < 0 && gamepad2.right_stick_y == 0){
//            armRight.setPower(0.1);
//        }
    }

    private void updateTelemetry(Robot robot) {
//        telemetry.addData("Arm Position", rightArmServo.getPosition());
//        telemetry.addData("Arm Height", armLeft.getCurrentPosition());
//        telemetry.addData("Ideal Arm Height", robot.getArmHeight());
//        telemetry.addData("Grabber Position", grabberServo.getPosition());
//        telemetry.addData("armSpeed", getArmSpeed());
//        telemetry.addData("leftArmServo Position", leftArmServo.getPosition());
//        telemetry.addData("rightArmServo Position", rightArmServo.getPosition());
        telemetry.addData("Speed", triggerMultiplier);
        telemetry.addData("Arm Progress", progress);
        telemetry.addData("Moving down?", isMovingDown);
        telemetry.addData("Moving up?", isMovingUp);
        telemetry.addData("Direction", movingForward ? "Forward" : "Reverse");
        telemetry.update();
    }

    private double map(double in1, double in2, double out1, double out2, double value) {
        return out1 + ((value - in1) * (out2 - out1)) / (in2 - in1);
    }
}
