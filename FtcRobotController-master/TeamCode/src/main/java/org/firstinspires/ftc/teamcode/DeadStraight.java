package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@TeleOp(name = "Manual PID OLD", group = "Concept")
public class DeadStraight extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor rightDeadWheel;
    private DcMotor leftDeadWheel;
    private double motorTicks;
    private PIDController PIDController;
    private ElapsedTime timer;
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60);
    double inchesPerTick = (wheelCircumference/ticksPerRotation)/2.54;
    double ticksPerInch = (ticksPerRotation/wheelCircumference)*25.4;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontRight");
        rightDeadWheel = hardwareMap.get(DcMotor.class, "rightDeadWheel");
        leftDeadWheel = hardwareMap.get(DcMotor.class,"leftDeadWheel");


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

        PIDController = new PIDController(0.00002, 0, 0, 0.1);
        //kP= 0.00003, kI= 0.000003, kD= 0.000005

        timer = new ElapsedTime();
        waitForStart();
        while(opModeIsActive()){
            setPosition(12);
            double currentPosition = ((leftDeadWheel.getCurrentPosition() + rightDeadWheel.getCurrentPosition())/2);
            double deltaTime = timer.seconds();
            timer.reset();

            double output = PIDController.update(currentPosition, deltaTime);
            setMotorPower(output);
            telemetry.addData("Left Wheel", leftDeadWheel.getCurrentPosition());
            telemetry.addData("Right Wheel", rightDeadWheel.getCurrentPosition());
            telemetry.addData("Integral", PIDController.integral*PIDController.kI);
            telemetry.addData("Derivative", PIDController.derivative*PIDController.kD);
            telemetry.addData("Target Ticks", motorTicks);
            telemetry.addData("Motor Power", output);
            telemetry.addData("ElapsedTime", deltaTime);
            telemetry.addData("ERROR", PIDController.error);
            telemetry.update();
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
        motorTicks = inches * ticksPerInch;
        PIDController.setTargetAmount(motorTicks);
    }
}
