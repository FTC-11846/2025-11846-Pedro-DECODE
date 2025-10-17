package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Vision subsystem for AprilTag detection and goal tracking
 */
public class Vision {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    // ==================== TUNABLE CONSTANTS ====================

    @Configurable
    public static class Constants {
        // AprilTag IDs for DECODE season
        public static int BLUE_GOAL_TAG_ID = 20;
        public static int RED_GOAL_TAG_ID = 24;
        public static int MOTIF_GPP_TAG_ID = 21;
        public static int MOTIF_PGP_TAG_ID = 22;
        public static int MOTIF_PPG_TAG_ID = 23;

        // Vision processing
        public static int DECIMATION = 2;  // Higher = faster but less accurate
        public static String WEBCAM_NAME = "Webcam 1";
    }

    // ==================== CONSTRUCTOR ====================

    public Vision(HardwareMap hardwareMap) {
        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(Constants.DECIMATION);

        // Initialize vision portal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();
    }

    // ==================== PUBLIC QUERY METHODS ====================

    /**
     * Get all currently detected AprilTags
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Find the goal AprilTag (Blue or Red)
     * Returns null if no goal tag is visible OR pose estimation failed
     */
    public AprilTagDetection findGoalTag() {
        List<AprilTagDetection> detections = getDetections();

        for (AprilTagDetection detection : detections) {
            // Check metadata AND ftcPose are not null
            if (detection.metadata != null && detection.ftcPose != null) {
                if (detection.id == Constants.BLUE_GOAL_TAG_ID ||
                        detection.id == Constants.RED_GOAL_TAG_ID) {
                    return detection;
                }
            }
        }

        return null;
    }

    /**
     * Find a specific motif pattern tag
     * Returns null if tag not visible OR pose estimation failed
     */
    public AprilTagDetection findMotifTag(int targetId) {
        List<AprilTagDetection> detections = getDetections();

        for (AprilTagDetection detection : detections) {
            // Check both metadata AND ftcPose are not null
            if (detection.metadata != null && detection.ftcPose != null && detection.id == targetId) {
                return detection;
            }
        }

        return null;
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
        if (tagId == Constants.BLUE_GOAL_TAG_ID) return "Blue Goal";
        if (tagId == Constants.RED_GOAL_TAG_ID) return "Red Goal";
        if (tagId == Constants.MOTIF_GPP_TAG_ID) return "Motif GPP";
        if (tagId == Constants.MOTIF_PGP_TAG_ID) return "Motif PGP";
        if (tagId == Constants.MOTIF_PPG_TAG_ID) return "Motif PPG";
        return "Unknown Tag";
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

    /**
     * Check if an AprilTag detection is valid (has pose data)
     */
    public boolean isValidDetection(AprilTagDetection detection) {
        return detection != null && detection.metadata != null && detection.ftcPose != null;
    }

    // ==================== RESULT CLASS ====================

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
                    Vision.getTagFriendlyName(detection.id),
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
}