package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@TeleOp(name = "Dead Test", group = "Concept")
public class DeadStraight_Sai extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotorEx leftDeadWheel, rightDeadWheel; // assuming your dead wheels are connected to REV Throughbore encoders

    double wheelCircumference = 60 * Math.PI; // Dead wheel diameter = 60 mm
    double encoderCountsPerRevolution = 8192; // REV Throughbore encoder counts
    double distancePerCount = wheelCircumference / encoderCountsPerRevolution;

    // PID variables
    double kp = 0.1;  // Tune these values
    double ki = 0.0;
    double kd = 0.0;
    double integral = 0;
    double lastError = 0;
    double targetDistance = 1000; // Example target distance in mm

    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        leftDeadWheel = hardwareMap.get(DcMotorEx.class, "leftDeadWheel");
        rightDeadWheel = hardwareMap.get(DcMotorEx.class, "rightDeadWheel");

        // Reset encoders
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run without encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Get encoder readings
            double leftDistance = leftDeadWheel.getCurrentPosition() * distancePerCount;
            double rightDistance = rightDeadWheel.getCurrentPosition() * distancePerCount;

            double averageDistance = (leftDistance + rightDistance) / 2;

            // Stop when target distance is reached
            if (averageDistance >= targetDistance) {
                stopRobot();
                break;
            }

            // PID for straight line correction
            double error = leftDistance - rightDistance;
            integral += error;
            double derivative = error - lastError;
            lastError = error;

            double correction = (kp * error) + (ki * integral) + (kd * derivative);

            // Adjust motor powers for straight movement
            frontLeft.setPower(0.5 + correction);
            frontRight.setPower(0.5 - correction);
            backLeft.setPower(0.5 + correction);
            backRight.setPower(0.5 - correction);
        }
    }

    public void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

