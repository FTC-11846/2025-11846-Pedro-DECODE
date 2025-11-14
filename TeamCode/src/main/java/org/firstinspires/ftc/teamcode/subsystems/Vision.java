package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Vision subsystem using FTC SDK's built-in pose estimation
 *
 * KEY FEATURES:
 * - Uses AprilTagProcessor.setCameraPose() for accurate robot pose calculation
 * - Uses detection.robotPose (includes camera offset automatically)
 * - Uses PedroPathing's official FTC→Pedro coordinate conversion
 * - Single consolidated conversion point for all coordinate transforms
 * - No manual trigonometry or homebrew geometry!
 */
@Configurable
public class Vision {

    // ==================== CONFIGURATION CLASS ====================

    public static Config config = new Config();

    public static class Config {
        // Tag IDs (from current game)
        public int blueGoalTagId = 20;
        public int redGoalTagId = 24;
        public int motifGppTagId = 21;
        public int motifPgpTagId = 22;
        public int motifPpgTagId = 23;

        // Camera settings
        public int decimation = 2;
        public String webcamName = "Webcam 1";

        // Camera physical mounting (inches and degrees)
        // Robot-specific values set in [RobotName]Abilities.applyConfiguration()
        // FTC SDK format: X=right, Y=forward, Z=up (from robot center)
        public double cameraPosX = 0.0;      // Right offset (inches)
        public double cameraPosY = 0.0;      // Forward offset (inches)
        public double cameraPosZ = 0.0;      // Up offset (inches)
        public double cameraYaw = 0.0;       // Heading offset (degrees)
        public double cameraPitch = 0.0;     // Tilt angle (degrees)
        public double cameraRoll = 0.0;      // Rotation angle (degrees)

        // Shooting target offsets (Pedro coordinates, inches)
        // Aim past the tag toward goal center
        public double shootingOffsetX = 15.0;  // Horizontal offset
        public double shootingOffsetY = 15.0;  // Forward toward wall

        /**
         * Get shooting target pose (offset from goal tag)
         * Fetches tag position from library, converts to Pedro, applies offset
         */
        public Pose getShootingTarget(int tagId, AprilTagLibrary library) {
            AprilTagMetadata tagMetadata = library.lookupTag(tagId);
            if (tagMetadata == null || tagMetadata.fieldPosition == null) {
                return null;  // Tag not in library
            }

            // Get tag position from library (FTC coordinates)
            double ftcX = tagMetadata.fieldPosition.get(0);  // X in inches
            double ftcY = tagMetadata.fieldPosition.get(1);  // Y in inches

            // Convert FTC tag position to Pedro
            Pose tagPedroPose = convertFTCToPedro(ftcX, ftcY, 0);
            double pedroTagX = tagPedroPose.getX();
            double pedroTagY = tagPedroPose.getY();

            // Apply shooting offset based on tag
            if (tagId == blueGoalTagId) {
                return new Pose(pedroTagX - shootingOffsetX, pedroTagY + shootingOffsetY, 0);
            } else if (tagId == redGoalTagId) {
                return new Pose(pedroTagX + shootingOffsetX, pedroTagY + shootingOffsetY, 0);
            }

            return new Pose(pedroTagX, pedroTagY, 0);  // No offset for other tags
        }
    }

    // ==================== SUBSYSTEM FIELDS ====================

    @IgnoreConfigurable
    private final VisionPortal visionPortal;

    @IgnoreConfigurable
    private final AprilTagProcessor aprilTag;

    @IgnoreConfigurable
    private final AprilTagLibrary tagLibrary;

    // ==================== CONSTRUCTOR ====================

    /**
     * Create Vision subsystem with FTC SDK pose estimation
     * Uses current season's tag library and robot-specific camera pose
     */
    public Vision(HardwareMap hardwareMap) {
        // Get current season's tag library
        tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

        // Configure camera pose from Vision.config (set by robot's applyConfiguration)
        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                config.cameraPosX,
                config.cameraPosY,
                config.cameraPosZ,
                0  // acquisitionTime
        );

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                config.cameraYaw,
                config.cameraPitch,
                config.cameraRoll,
                0  // acquisitionTime
        );

        // Initialize AprilTag processor with library
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(config.decimation);

        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, config.webcamName))
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
            if (detection.metadata != null && detection.robotPose != null) {
                if (detection.id == config.blueGoalTagId ||
                        detection.id == config.redGoalTagId) {
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
            if (detection.metadata != null && detection.robotPose != null &&
                    detection.id == targetId) {
                return detection;
            }
        }

        return null;
    }

    /**
     * Extract robot pose from AprilTag detection
     * Uses FTC SDK's robotPose (already accounts for camera offset!)
     * Converts to Pedro coordinates using official pipeline
     *
     * @return Pedro Pose or null if pose unavailable
     */
    public Pose getRobotPoseFromTag(AprilTagDetection detection) {
        if (detection == null || detection.robotPose == null) {
            return null;
        }

        // Extract FTC SDK pose components
        Position robotPosition = detection.robotPose.getPosition();
        YawPitchRollAngles orientation = detection.robotPose.getOrientation();

        // Convert to Pedro coordinates
        return convertFTCToPedro(
                robotPosition.x,
                robotPosition.y,
                orientation.getYaw(AngleUnit.RADIANS)
        );
    }

    /**
     * Get goal tag position in Pedro coordinates
     * Fetches from library and transforms to Pedro system
     */
    public Pose getGoalTagPose(int tagId) {
        AprilTagMetadata metadata = tagLibrary.lookupTag(tagId);
        if (metadata == null || metadata.fieldPosition == null) {
            return null;
        }

        // Get position from library (FTC coordinates)
        double ftcX = metadata.fieldPosition.get(0);
        double ftcY = metadata.fieldPosition.get(1);

        // Convert to Pedro (heading not critical for tag positions)
        return convertFTCToPedro(ftcX, ftcY, 0);
    }

    /**
     * Get shooting target for current goal tag
     * Uses library position + shooting offsets
     */
    public Pose getShootingTarget(int tagId) {
        return config.getShootingTarget(tagId, tagLibrary);
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
        if (tagId == config.blueGoalTagId) return "Blue Goal";
        if (tagId == config.redGoalTagId) return "Red Goal";
        if (tagId == config.motifGppTagId) return "Motif GPP";
        if (tagId == config.motifPgpTagId) return "Motif PGP";
        if (tagId == config.motifPpgTagId) return "Motif PPG";
        return "Unknown Tag";
    }

    /**
     * Get auto-aim data for the goal
     * Returns result object with distance, bearing, and success status
     *
     * NOTE: This uses ftcPose for display/telemetry only
     * Actual auto-aim calculations should use localization pose → shooting target
     */
    public AutoAimResult getAutoAimData() {
        AprilTagDetection goalTag = findGoalTag();

        if (goalTag == null) {
            int visibleCount = getDetections().size();
            return AutoAimResult.failure(
                    String.format("FAILED - No goal tag detected (%d tags visible)", visibleCount)
            );
        }

        if (goalTag.ftcPose == null) {
            return AutoAimResult.failure("FAILED - Pose estimation unavailable");
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
        return detection != null && detection.metadata != null &&
                detection.robotPose != null && detection.ftcPose != null;
    }

    // ==================== COORDINATE CONVERSION (CONSOLIDATED) ====================

    /**
     * Convert FTC coordinate system to Pedro coordinate system
     * Manual conversion since PoseConverter appears to have issues with tag metadata
     *
     * FTC: Origin at field center (72" from corner)
     *      X+ toward red, Y+ toward blue (perpendicular left), Z+ up
     * Pedro: Origin at bottom-left corner
     *        X+ toward red, Y+ toward far side, Z+ up
     *
     * Conversion: Rotate -90° around Z, then shift by (72, 72)
     */
    private static Pose convertFTCToPedro(double ftcX, double ftcY, double ftcYawRad) {
        // Step 1: Rotate -90° around Z axis
        // x' = y, y' = -x
        double rotatedX = ftcY;
        double rotatedY = -ftcX;
        double rotatedHeading = ftcYawRad - Math.PI / 2;  // Heading also rotates

        // Step 2: Translate origin from center to corner
        double pedroX = rotatedX + 72.0;
        double pedroY = rotatedY + 72.0;

        // Normalize heading to -π to π
        while (rotatedHeading > Math.PI) rotatedHeading -= 2 * Math.PI;
        while (rotatedHeading < -Math.PI) rotatedHeading += 2 * Math.PI;

        return new Pose(pedroX, pedroY, rotatedHeading);
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