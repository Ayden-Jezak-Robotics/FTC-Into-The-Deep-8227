package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoOpRotate", group = "Test")
public class AutoOpRotate extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0, 0, false, false);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() <= 25 ) {

            robot.spin(.5);

        }
        robot.stop();

        sleep(5000);


    }
}