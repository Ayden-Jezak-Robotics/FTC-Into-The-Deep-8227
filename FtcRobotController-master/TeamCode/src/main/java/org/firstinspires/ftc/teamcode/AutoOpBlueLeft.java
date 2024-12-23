package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name = "AutoOpBlueLeft", group = "Draft")
public class AutoOpBlueLeft extends LinearOpMode {

    Position initialPosition = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
    double initialHeading = 0;

    private final Robot robot = new Robot(this, hardwareMap, telemetry, initialPosition, initialHeading);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 36, 12, 0, 0), 0);

        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 24, -12, 0, 0), 90);

    }


}
