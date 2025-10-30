package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Vision subsystem with universal configuration constants
 */
@Configurable
public class Vision {
    
    // ==================== CONFIGURATION OBJECTS ====================
    
    public static TagIdentification tagIds = new TagIdentification();
    public static CameraSettings camera = new CameraSettings();
    public static GoalPositions goalPositions = new GoalPositions();
    
    // ==================== NESTED CONFIG CLASSES ====================
    
    public static class TagIdentification {
        public int blueGoalTagId = 20;
        public int redGoalTagId = 24;
        public int motifGppTagId = 21;
        public int motifPgpTagId = 22;
        public int motifPpgTagId = 23;
    }
    
    public static class CameraSettings {
        public int decimation = 2;  // Higher = faster but less accurate
        public String webcamName = "Webcam 1";
    }

    public static class GoalPositions {
        public double blueGoalX = 15.0;   // Center of BlueGoal AprilTag
        public double blueGoalY = 129.0;
        public double redGoalX = 129.0;   // Center of RedGoal AprilTag
        public double redGoalY = 129.0;

        // Shooting offsets - aim past the tag toward goal center
        public double shootingOffsetX = 15.0;  // Horizontal offset
        public double shootingOffsetY = 15.0;  // Forward toward wall

        /**
         * Get shooting target pose (offset from tag for better accuracy)
         * Red: +15,+15 from tag | Blue: -15,+15 from tag
         */
        public Pose getShootingTarget(int tagId) {
            if (tagId == tagIds.blueGoalTagId) {
                return new Pose(blueGoalX - shootingOffsetX, blueGoalY + shootingOffsetY, 0);
            } else if (tagId == tagIds.redGoalTagId) {
                return new Pose(redGoalX + shootingOffsetX, redGoalY + shootingOffsetY, 0);
            }
            return null;
        }
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final VisionPortal visionPortal;
    
    @IgnoreConfigurable
    private final AprilTagProcessor aprilTag;
    
    // ==================== CONSTRUCTOR ====================
    
    public Vision(HardwareMap hardwareMap) {
        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(camera.decimation);
        
        // Initialize vision portal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, camera.webcamName))
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
                if (detection.id == tagIds.blueGoalTagId ||
                    detection.id == tagIds.redGoalTagId) {
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
        if (tagId == tagIds.blueGoalTagId) return "Blue Goal";
        if (tagId == tagIds.redGoalTagId) return "Red Goal";
        if (tagId == tagIds.motifGppTagId) return "Motif GPP";
        if (tagId == tagIds.motifPgpTagId) return "Motif PGP";
        if (tagId == tagIds.motifPpgTagId) return "Motif PPG";
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
