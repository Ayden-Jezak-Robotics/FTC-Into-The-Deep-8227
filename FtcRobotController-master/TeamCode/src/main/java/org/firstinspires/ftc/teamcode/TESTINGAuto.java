package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TESTINGAuto", group = "Test")
public class TESTINGAuto extends LinearOpMode {

    RobotState initialState = new RobotState(24, -63, 0, 0, 0, 0, 0, 0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();
        robot.justArm(0.5,0.7,2,0.5,false);
        robot.moveToPositionAndHeading(new RobotState(5,-40,0,0,0.6,0.7,0.5,0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(5,-32,0,3,0.6,0.7,0.5,0.5));
        //robot.justArm(0.5,0.7,1.5,0.5,false);
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(5,-32,0,0,0.6,0.7,0.5,0.5));
        robot.manualPower(-0.5,-0.5,-0.5,-0.5);
        robot.openGrabber();

        robot.setAllCurrentPositions(5,-44,0,0,0.6,0.7);
        robot.moveToPositionAndHeading(new RobotState(20,-55,0,0,0,0.7,0.2,0.2));

        robot.moveToPositionAndHeading(new RobotState(34, -16, 0, 0, 0, 0.7, 0.5, 0.5));
        sleep(200);
        robot.moveToPositionAndHeading(new RobotState(50, -16, 0, 0, 0, 0.7, 0.5, 0.5));
        sleep(200);
        robot.moveToPositionAndHeading(new RobotState(50, -56, 0, 0, 0, 0.7, 0.5, 0.5));
        sleep(200);
        robot.moveToPositionAndHeading(new RobotState(48, -16, 0, 0, 0, 0.7, 0.5, 0.5));
        sleep(200);
        robot.moveToPositionAndHeading(new RobotState(58, -16, 0, 0, 0, 0.7, 0.5, 0.5));
        sleep(200);
        robot.moveToPositionAndHeading(new RobotState(58, -56, 0, 0, 0, 0.7, 0.5, 0.5));
        sleep(200);
        robot.moveToPositionAndHeading(new RobotState(58, -16, 0, 0, 0, 0.7, 0.5, 0.5));
        sleep(200);
        robot.manualPower(0.4,-0.4,-0.4,0.4);
        robot.setAllCurrentPositions(61, -16, 0, 0, 0, 0.7);
        robot.moveToPositionAndHeading(new RobotState(61, -56, 0, 0, 0, 0.7, 0.5, 0.5));
    }
}