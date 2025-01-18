package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpSpin", group = "Draft")
public class AutoOpSpin extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        int numberOfSpins = 20;

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(0, 0, (360 * numberOfSpins)));

    }
}
