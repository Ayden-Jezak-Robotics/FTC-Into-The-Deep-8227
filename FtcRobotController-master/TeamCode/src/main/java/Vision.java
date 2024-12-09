package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.AprilTagProcessor;

public class Vision extends LinearOpMode
{

    public void runOpMode() throws InterruptedException
    {
        int[] portalsList = VisionPortal.makeMultiPortalView(1 /*or however many april tags you have */, VisionPortal.MultiPortalLayout.HORIZONTAL);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480))
            .enableCameraMonitoring(true)
            .setCameraMonitorViewId(portalsList[0])
            .build();

        waitForStart();

        while (!isStopRequested() && opModeisActive())
        {

            if (tagProcessor.getDetections().size() > 0)
            {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);

                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }
            telemetry.update();
        }

    }
}