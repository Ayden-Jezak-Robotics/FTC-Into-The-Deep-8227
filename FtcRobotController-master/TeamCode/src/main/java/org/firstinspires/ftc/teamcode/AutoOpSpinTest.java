package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpSpinTest", group = "Test")
public class AutoOpSpinTest extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0,0,0,0,0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        int numberOfSpins = 10;

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(0, 0, (360 * numberOfSpins), 0, 0, 0, 0, 0));

    }
}
