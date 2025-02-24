package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpRedRight", group = "Test")
public class AutoOpRedRight extends LinearOpMode {

    RobotState initialState = new RobotState(24, -63, 0, 0, 0, 0, 0, 0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();
        robot.moveToPositionAndHeading(new RobotState(34, -16, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(48, -16, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(48, -60, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(48, -16, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(56, -8, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(56, -60, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(56, -16, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(60, -16, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(60, -60, 0, 0, 0, 0, 0.5, 0.5));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(28, -18, 0, 0, 0.6, 0.7, 1, 1));
        sleep(1000);
        robot.justArm(0,0.7,0.5,0.5,true);
    }
}