package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class IMUUtility {

    private final BHI260IMU imu;
    private double previousHeading;

    IMUUtility(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        IMU.Parameters imuParameters = new IMU.Parameters(orientationOnRobot);

        imu.initialize(imuParameters);

        previousHeading = 0;
    }

    double getCurrentHeading() {
        double currentHeading;

        currentHeading = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY, // TODO - Is this the right order to get firstAngle?
                LMMHS.getAngleUnit()
        ).firstAngle;

        return currentHeading;
    }

    public double getPreviousHeading() {
        return this.previousHeading;
    }

    public void setPreviousHeading(double newValue) {
        this.previousHeading = newValue;
    }

    void resetIMU() {
        imu.resetYaw();
    }
}
