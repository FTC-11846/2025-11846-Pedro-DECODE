package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Vision subsystem for AprilTag detection and goal tracking
 */
public class VisionSubsystem {
    // AprilTag IDs
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;
    public static final int MOTIF_GPP_TAG_ID = 21;
    public static final int MOTIF_PGP_TAG_ID = 22;
    public static final int MOTIF_PPG_TAG_ID = 23;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    public VisionSubsystem(HardwareMap hardwareMap) {
        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Initialize vision portal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Get all currently detected AprilTags
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Find the goal AprilTag (Blue or Red)
     * Returns null if no goal tag is visible
     */
    public AprilTagDetection findGoalTag() {
        List<AprilTagDetection> detections = getDetections();
        
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (detection.id == BLUE_GOAL_TAG_ID || detection.id == RED_GOAL_TAG_ID) {
                    return detection;
                }
            }
        }
        
        return null;
    }

    /**
     * Find a specific motif pattern tag
     */
    public AprilTagDetection findMotifTag(int targetId) {
        List<AprilTagDetection> detections = getDetections();
        
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == targetId) {
                return detection;
            }
        }
        
        return null;
    }

    /**
     * Check if any AprilTags are currently visible
     */
    public boolean hasDetections() {
        return !getDetections().isEmpty();
    }

    /**
     * Get the camera state
     */
    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    /**
     * Get friendly name for AprilTag ID
     */
    public static String getTagFriendlyName(int tagId) {
        switch (tagId) {
            case BLUE_GOAL_TAG_ID: return "Blue Goal";
            case RED_GOAL_TAG_ID: return "Red Goal";
            case MOTIF_GPP_TAG_ID: return "Motif GPP";
            case MOTIF_PGP_TAG_ID: return "Motif PGP";
            case MOTIF_PPG_TAG_ID: return "Motif PPG";
            default: return "Unknown Tag";
        }
    }

    /**
     * Result class for auto-aim calculations
     */
    public static class AutoAimResult {
        public final boolean success;
        public final double distanceInches;
        public final double bearingDegrees;
        public final int tagId;
        public final String message;

        public AutoAimResult(boolean success, double distance, double bearing, 
                           int tagId, String message) {
            this.success = success;
            this.distanceInches = distance;
            this.bearingDegrees = bearing;
            this.tagId = tagId;
            this.message = message;
        }

        public static AutoAimResult failure(String message) {
            return new AutoAimResult(false, 0, 0, -1, message);
        }

        public static AutoAimResult success(AprilTagDetection detection) {
            String message = String.format("SUCCESS - %s @ %.1f in, %.1f deg",
                VisionSubsystem.getTagFriendlyName(detection.id),
                detection.ftcPose.range,
                detection.ftcPose.bearing);
            
            return new AutoAimResult(
                true,
                detection.ftcPose.range,
                detection.ftcPose.bearing,
                detection.id,
                message
            );
        }
    }

    /**
     * Get auto-aim data for the goal
     * Returns result object with distance, bearing, and success status
     */
    public AutoAimResult getAutoAimData() {
        AprilTagDetection goalTag = findGoalTag();
        
        if (goalTag == null) {
            int visibleCount = getDetections().size();
            return AutoAimResult.failure(
                String.format("FAILED - No goal tag detected (%d tags visible)", visibleCount)
            );
        }
        
        return AutoAimResult.success(goalTag);
    }

    /**
     * Close the vision portal when done
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
