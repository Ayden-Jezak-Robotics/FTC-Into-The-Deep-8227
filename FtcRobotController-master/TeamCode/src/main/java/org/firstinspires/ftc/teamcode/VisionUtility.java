package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


public class VisionUtility {

    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;
    private HardwareMap hardwareMap;

    VisionUtility(HardwareMap hardwareMap) {

        Position cameraPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        this.hardwareMap = hardwareMap;

        this.myAprilTagProcessor = new AprilTagProcessor.Builder()
               .setCameraPose(cameraPosition, cameraOrientation)
               .setDrawAxes(true)
               .build();
        this.myVisionPortal = new VisionPortal.Builder()
               .setShowStatsOverlay(true)
               .setCamera(BlocksOpModeCompanion.hardwareMap.get(WebcamName.class, "Webcam 1"))
               .addProcessor(myAprilTagProcessor)
               .setCameraResolution(new Size(640, 480))
               .setStreamFormat(VisionPortal.StreamFormat.YUY2)
               .enableLiveView(true)
               .build();
    }

    private List<AprilTagDetection> getDetections() {
        long currentTime = System.nanoTime();

        List<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();

        if (myAprilTagDetections == null || myAprilTagDetections.isEmpty()) {
            // No detections found; return null or handle appropriately
            return null;
        }

        for (AprilTagDetection detection : myAprilTagDetections) {
            if (detection.metadata != null && detection.robotPose != null) {

                long detectionAge = currentTime - detection.frameAcquisitionNanoTime;
                if (detectionAge >= Constants.MAX_AGE_NANOSECONDS) {
                    myAprilTagDetections.remove(detection);

                }
            } else {
                myAprilTagDetections.remove(detection);
            }
        }
        if (myAprilTagDetections.isEmpty()) {
            return null;
        } else {
            return myAprilTagDetections;
        }
    }

    public Pose3D getPose() {
        Pose3D robotPosition = null; // Start with a null position to handle cases where no detection is valid

        List<AprilTagDetection> myAprilTagDetections = getDetections();

        for (AprilTagDetection detection : myAprilTagDetections) {
            robotPosition = detection.robotPose;
            break; // Assuming you want the position from the first valid detection
        }

        return robotPosition; // Could still be null if no valid detection found
    }

}
