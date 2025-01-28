package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpBlueLeft", group = "Draft")
public class AutoOpBlueLeft extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.turnOn();
        //robot.moveToPositionAndHeading(new RobotState(0, 24, 0));
        //robot.moveToPositionAndHeading(new RobotState(24, 24, 45));
        //robot.moveToPositionAndHeading(new RobotState(0, 0, 45));
        //robot.moveToPositionAndHeading(new RobotState(0, 0, 0));

    }


}
