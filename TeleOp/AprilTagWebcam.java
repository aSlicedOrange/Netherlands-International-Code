package org.firstinspires.ftc.teamcode.mechanisms;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.*;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    public Telemetry telemetry;
    public void init(HardwareMap hwMap, Telemetry telemetry, String CameraName) {
        this.telemetry = telemetry;

        AprilTagLibrary myLibrary = new AprilTagLibrary.Builder() // New AprilTags are made.
                .addTag(0, "Tag 0", 0.0508, DistanceUnit.CM)
                .addTag(1, "Tag 1", 0.0508, DistanceUnit.CM)
                .addTag(2, "Tag 2", 0.0508, DistanceUnit.CM)
                .addTag(20, "Tag 20", 0.0508, DistanceUnit.CM)
                .addTag(24, "Tag 24", 0.0508, DistanceUnit.CM)
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true) // In competition time Jack and Hubert, we need to turn these to false. Code will ship with false though for debuggig, keep them true to ensure we can see the tracking.
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(myLibrary)
                .setLensIntrinsics(578,578,320,240)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDecimation(3) // Decimation ensures less pixels are being repeatedly tracked which saves memory and increases speed of tracking. Based of 1-4, 4 being best for optimisation. Problem will be potato video quality
                .build(); // Test with 3 for now, then if you want, do 4. Choose what works best for you.

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, CameraName));
        builder.setCameraResolution(new Size(240, 240)); // Major change 1 - Reduced resolution to ensure that quality parses through tracking more easily. Currently at 280p though can drop to 240 if necessary. Just best of both worlds.
        builder.addProcessor(aprilTagProcessor);
        builder.setLiveViewContainerId(0);
        builder.enableLiveView(true); // Ensures video feed isn't shown. For debugging, keep it ON. For actual games, keeping it off improves tracking accuracy - lowers resources, up on CPU.
        // Concern: there is a slight chance that if we are using an inadequate SDK that this change may prevent tracking. Check first, if no track, then immediately cut this code.
        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId ==null) {return;}
        if (detectedId.metadata != null)
        {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        }
        else
        {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }



    } //


    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

// Improvements done here are quality of life improvements to assist in final competition. Main work right now is auto-tracking.
}