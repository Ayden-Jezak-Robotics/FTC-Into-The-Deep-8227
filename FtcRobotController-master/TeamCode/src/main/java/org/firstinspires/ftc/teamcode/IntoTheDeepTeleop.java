package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "Into The Deep Teleop", group = "Main")
public class IntoTheDeepTeleop extends LinearOpMode
{

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    //private DcMotor armLeft, armRight;
    private double r;
    private double robotAngle;
    private double rightX;
    private double v1, v2, v3, v4;
    private double speedMulti = 4;
    int xPressed = 0;


    @Override
    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        //armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        //armRight = hardwareMap.get(DcMotor.class, "armRight");

        waitForStart();
        while (opModeIsActive()){
            speedMulti();
            //armMovement();
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
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

            updateTelemetry();
        }
    }
    private void speedMulti() {
        if (gamepad1.circle == true) {
            speedMulti += 1;
            if (speedMulti > 4) {
                speedMulti = 1;
            }
            while (!!(gamepad1.circle == true)) {
            }
        }
    }
    //private void armMovement(){
    //    if (armLeft.getCurrentPosition() < 8000 && armLeft.getCurrentPosition() > 50) {
    //        armLeft.setPower(-gamepad2.right_stick_y);
    //        armRight.setPower(-gamepad2.right_stick_y);
    //    }
    //}
    private void updateTelemetry(){
        telemetry.addData("Speed", speedMulti);
        telemetry.update();
    }
}
