package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@TeleOp(name = "Manual PID ", group = "Concept")
public class DeadStraight extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private PIDController PIDController;
    private ElapsedTime timer;
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60)/2.54;
    double ticksPerInch = ticksPerRotation/wheelCircumference;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDController = new PIDController(0, 0, 0);
        PIDController.setTargetAmount(12);

        timer = new ElapsedTime();

        waitForStart();
        while(opModeIsActive()){

        }
    }
    private void setMotorPower(double motorPower){
        backLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        frontRight.setPower(motorPower);

    }
    private void setPosition(double inches) {
        // I hate you - Cruz Swinson

    }
}
