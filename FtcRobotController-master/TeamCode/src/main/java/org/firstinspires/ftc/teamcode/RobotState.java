package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotState {

    final Position position = new Position(DistanceUnit.INCH, 0,0,0,System.nanoTime());
    double heading; // In Degrees
    double armHeight;
    double armAngle; // What units here?
    boolean wristIsUp;
    boolean grabberIsOpen;

    // Constructor to initialize the state
    public RobotState(int x, int y, int heading, double armHeight, double armAngle, boolean wristIsUp, boolean grabberIsOpen) {
        this.position.x = x;
        this.position.y = y;
        this.heading = heading; // in degrees
        this.armHeight = armHeight;
        this.armAngle = armAngle;

        this.wristIsUp = wristIsUp;
        this.grabberIsOpen = grabberIsOpen;
    }

    public RobotState(RobotState state) {
        this.position.x = state.position.x;
        this.position.y = state.position.y;
        this.heading = state.heading;
        this.armHeight = state.armHeight;
        this.armAngle = state.armAngle;
        this.wristIsUp = state.wristIsUp;
        this.grabberIsOpen = state.grabberIsOpen;
    }
}
