package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpTurnRight", group = "Test")
public class AutoOpTurnRight extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0, 0,false,false);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();

        robot.moveToNewRobotState(new RobotState(0, 0, -53, 0, 0,false,false));

    }
}