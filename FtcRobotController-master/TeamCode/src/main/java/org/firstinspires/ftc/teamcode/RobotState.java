package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotState {

    final Position position;
    final double heading; // In Degrees


    // Constructor to initialize the state
    public RobotState(int x, int y, int heading) {
        this.position = new Position(DistanceUnit.INCH, x, y, 0, System.nanoTime());
        this.heading = heading; // in degrees
    }

}
