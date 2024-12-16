
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import android.util.Size;

/*
import java.util.List;

@TeleOp(name = "AprilTag Second Look", group = "Concept")
public class AprilTagSecondLook extends LinearOpMode
{
    double xTotal;
    double yTotal;
    double zTotal;
    double yawTotal;
    double cycleCount;
    @Override


    public void runOpMode() {
        VisionPortal myVisionPortal = null; // Declare outside the try block
        try {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"),
                    null // No processor, just raw feed
            );
            telemetry.addData("Camera", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Camera Error", e.getMessage());
        }
        telemetry.update();*/
        /*Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        AprilTagProcessor myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();*/

        /*VisionPortal myVisionPortal = new VisionPortal.Builder()
                .addProcessor(myAprilTagProcessor) // Add the AprilTag processor
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Use your webcam
                .setCameraResolution(new Size(640, 480)) // Set resolution (optional)
                .setCameraMonitorViewId(hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())) // Set the monitor view
                .build();
                */
        /*
        VisionPortal myVisionPortal;
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);
         */
/*
        waitForStart();
        while(opModeIsActive()){
            List<AprilTagDetection> myAprilTagDetections; // list of all detections
            AprilTagDetection myAprilTagDetection; // current detection in for() loop
            int myAprilTagIdCode; // ID code of current detection, in for() loop
            myAprilTagDetections = myAprilTagProcessor.getDetections();
            for (AprilTagDetection tagID: myAprilTagDetections) {
                if (tagID.metadata != null) {
                    myAprilTagIdCode = tagID.id;
                    double myX = tagID.robotPose.getPosition().x;
                    double myY = tagID.robotPose.getPosition().y;
                    double myZ = tagID.robotPose.getPosition().z;
                    double myYaw = tagID.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    xTotal += myX;
                    yTotal += myY;
                    zTotal += myZ;
                    yawTotal += myYaw;
                    cycleCount += 1;

                    if (cycleCount == 100) {
                        double xAverage = xTotal/100;
                        double yAverage = yTotal/100;
                        double zAverage = zTotal/100;
                        double yawAverage = yawTotal/100;
                        telemetry.addData("ID", myAprilTagIdCode);
                        telemetry.addData("X", xAverage);
                        telemetry.addData("Y", yAverage);
                        telemetry.addData("Z", zAverage);
                        telemetry.addData("Yaw", yawAverage);
                        telemetry.update();
                        xTotal = 0;
                        yTotal = 0;
                        zTotal = 0;
                        yawTotal = 0;
                        cycleCount = 0;
                    }
                }
            }
            telemetry.update();
        }
    }
}
*/