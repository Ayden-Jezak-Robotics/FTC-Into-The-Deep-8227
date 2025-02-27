package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "LEFTAuto", group = "Test")
public class AutoOpFinalBlueLeft extends LinearOpMode {

    RobotState initialState = new RobotState(-24, -63, 0, 0, 0, 0, 0, 0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(-34, -54, 0, 0, 0.3, 0.7, 0.5, 0.5));
        robot.moveToPositionAndHeading(new RobotState(-44, -44, -45, 0, 0.6, 0.7, 0.5, 0.5));
        robot.wristUp();
        robot.moveToPositionAndHeading(new RobotState(-55, -55, -45, 26, 0.6, 0.7, 0.5, 0.5));
        robot.justArm(0.8,0.7,0.2,0.2,true);
        robot.wristDown();
        robot.moveToPositionAndHeading(new RobotState(-45, -43, 0, 0, 0.6, 0.7, 0.5, 0.5));
        robot.justArm(0,0.7,1,1,false);
        robot.wristUp();
        robot.moveToPositionAndHeading(new RobotState(-55, -55, -45, 26, 0.6, 0.7, 0.5, 0.5));
        robot.justArm(0.8,0.7,0.5,0.5,true);
        robot.wristDown();
        robot.moveToPositionAndHeading(new RobotState(-56, -42, 0, 0, 0.6, 0.7, 0.5, 0.5));
        robot.justArm(0,0.7,1,1,false);
        robot.wristUp();
        robot.moveToPositionAndHeading(new RobotState(-55, -55, -45, 26, 0.6, 0.7, 0.5, 0.5));
        robot.justArm(0.8,0.7,0.5,0.5,true);
        robot.wristDown();
        robot.moveToPositionAndHeading(new RobotState(-28, -18, 0, 0, 0.6, 0.7, 1, 1));
        robot.justArm(0,0.7,0.5,0.5,true);
    }
}