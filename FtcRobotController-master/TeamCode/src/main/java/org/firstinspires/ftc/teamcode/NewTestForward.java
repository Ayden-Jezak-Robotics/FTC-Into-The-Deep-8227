package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonomousForwardOneFoot", group="Linear Opmode")
public class NewTestForward extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor leftDeadWheel = null;
    private DcMotor rightDeadWheel = null;

    static final double COUNTS_PER_INCH_REV_ENCODER = 1105.27; // Ticks per inch for REV Throughbore encoders
    static final double DRIVE_DISTANCE = 12; // Target distance in inches
    static final double DRIVE_SPEED = 0.1; // Speed

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        leftDeadWheel = hardwareMap.get(DcMotor.class, "leftDeadWheel");
        rightDeadWheel = hardwareMap.get(DcMotor.class, "rightDeadWheel");

        // Set the motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set Motor Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset dead wheel encoders (REV Throughbore Encoders)
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to RUN_USING_ENCODER for the dead wheels
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Set drive motors to power the robot forward
            frontLeft.setPower(DRIVE_SPEED);
            frontRight.setPower(DRIVE_SPEED);
            backLeft.setPower(DRIVE_SPEED);
            backRight.setPower(DRIVE_SPEED);

            // Calculate the target position in ticks for the dead wheels
            int targetPosition = (int)(DRIVE_DISTANCE * COUNTS_PER_INCH_REV_ENCODER);

            // Keep driving until the dead wheels have rotated enough
            while (opModeIsActive() &&
                    Math.abs(leftDeadWheel.getCurrentPosition()) < targetPosition &&
                    Math.abs(rightDeadWheel.getCurrentPosition()) < targetPosition) {
                // Display current position for debugging
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Dead Wheel Position", "Left: %d, Right: %d",
                        leftDeadWheel.getCurrentPosition(),
                        rightDeadWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motors once the target position is reached
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
}
