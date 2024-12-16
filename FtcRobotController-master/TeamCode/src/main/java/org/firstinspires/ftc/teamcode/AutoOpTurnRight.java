package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpTurnRight", group = "Draft")
public class AutoOpTurnRight extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(0, 0, -53));

    }
}