package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotState {

    final Position position;
    double heading; // In Degrees
    double armHeight;
    double armAngle; // What units here?
    boolean elbowIsExtended;
    boolean wristIsUp;
    boolean grabberIsOpen;
    // final double armAngleTime;
    // final double extendTime;


    // Constructor to initialize the state
    public RobotState(int x, int y, int heading, double armHeight, double armAngle, boolean elbowIsExtended, boolean wristIsUp, boolean grabberIsOpen) {
        this.position = new Position(DistanceUnit.INCH, x, y, 0, System.nanoTime()); // z is always 0 
        this.heading = heading; // in degrees
        this.armHeight = armHeight;
        this.armAngle = armAngle;
        this.elbowIsExtended = elbowIsExtended;
        this.wristIsUp = wristIsUp;
        this.grabberIsOpen = grabberIsOpen;
    }
}
