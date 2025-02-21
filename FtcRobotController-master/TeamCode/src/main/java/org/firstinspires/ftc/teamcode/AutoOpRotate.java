package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoOpRotate", group = "Test")
public class AutoOpRotate extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0, 0, 0, 0, 0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(12, 0, 0, 12, 0, 0, 2, 1));

    }
}