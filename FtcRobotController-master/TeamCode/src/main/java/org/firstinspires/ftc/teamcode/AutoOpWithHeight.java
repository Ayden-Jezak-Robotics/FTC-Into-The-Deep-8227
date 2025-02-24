package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpWithHeight", group = "Main")
public class AutoOpWithHeight extends LinearOpMode {

    RobotState initialState = new RobotState(-24, -63, 0, 0, 0, 0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(-44, -44, -45, 0, 0, 0, 1, 1));
        robot.justArm(0.8,0.7,1,1,false);
        robot.wristUp();
        robot.moveToPositionAndHeading(new RobotState(-55, -55, -45, 26, 0, 0, 1, 1));
        robot.justArm(0.8,0.7,0.2,0.1,true);
        robot.wristDown();
        robot.moveToPositionAndHeading(new RobotState(-46, -43, 0, 0, 0, 0, 1, 1));
        robot.justArm(0,0.7,2,1,false);
        robot.justArm(0.8,0.7,1,1,false);
        robot.wristUp();
        robot.moveToPositionAndHeading(new RobotState(-55, -55, -45, 26, 0, 0, 1, 1));
        robot.justArm(0.8,0.7,0.2,0.1,true);
        robot.wristDown();
        robot.moveToPositionAndHeading(new RobotState(-56, -42, 0, 0, 0, 0, 1, 1));
    }
}