package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
 * KEY FEATURES (Session 2 refactor):
 * - Uses AprilTagProcessor.setCameraPose() for accurate robot pose calculation
 * - Extracts global robot pose from detection.robotPose field
 * - Uses PedroPathing's official FTC->Pedro coordinate conversion
 * - Uses current season's tag library from AprilTagGameDatabase
 * - No manual trigonometry or homebrew geometry needed!
 * - New Config/Constants pattern updated with Vision as first system
 * |__ All constants w/ defaults in Subsystems, CharacterStats maps the same object instance
 * |__ and applyConfiguration() overwrites robot-specific values. (Update to iterative Reflection function)
 */
@Configurable
public class Vision {

    // ==================== CONFIGURATION CLASS ====================

    public static Config config = new Config();

    public static class Config {
        // Tag IDs (from current game)
        @Deprecated  ///  The FTC AprilTag code directly returns metadata, we don't need our own lookup tables and matching!
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
         * Fetches actual tag position from library, then applies offset
         */
        public Pose getShootingTarget(int tagId, AprilTagLibrary library) {
            AprilTagMetadata tagMetadata = library.lookupTag(tagId);
            if (tagMetadata == null) {
                return null;  // Tag not in library
            }

            // Get tag position from library (FTC coordinates)
            double tagX = tagMetadata.fieldPosition.get(0);  // X in inches
            double tagY = tagMetadata.fieldPosition.get(1);  // Y in inches

            // Convert FTC tag position to Pedro using official converter
            Pose pedroPose = convertFTCPoseToPedro(tagX, tagY, 0);
            double pedroTagX = pedroPose.getX();
            double pedroTagY = pedroPose.getY();

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
        // Get current season's tag library (has all positions already!)
        tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

        // Configure camera pose from Vision.config (set by robot's applyConfiguration)
        // FTC SDK uses: X=right, Y=forward, Z=up (INCHES)
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

        // Initialize AprilTag processor with library, SDK now handles all transformations!
        // AprilTagProcessor.Builder() defaults to inches & degrees, so no Units not needed!
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
            // Check metadata AND robotPose are not null
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
     * Extract robot pose from AprilTag detection using SDK's pose estimation
     * Uses PedroPathing's official FTC->Pedro coordinate conversion
     *
     * FTC SDK robotPose: X toward red, Y toward blue (left), Z up, origin at field center (inches)
     * Pedro: X toward red, Y toward far side, origin at bottom-left
     *
     * @return Pedro Pose or null if pose unavailable
     */
    public Pose getRobotPoseFromTag(AprilTagDetection detection) {
        if (detection == null || detection.robotPose == null) {
            return null;
        }

        // Extract FTC SDK pose (already accounts for camera offset!)
        Position robotPosition = detection.robotPose.getPosition();
        YawPitchRollAngles orientation = detection.robotPose.getOrientation();

        // Use PedroPathing's official FTC->Pedro converter
        return convertFTCPoseToPedro(
                robotPosition.x,  // inches
                robotPosition.y,  // inches
                orientation.getYaw(AngleUnit.RADIANS)
        );
    }

    /**
     * Get goal tag position in Pedro coordinates
     * Fetches from library and transforms to Pedro system
     */
    public Pose getGoalTagPose(int tagId) {
        AprilTagMetadata metadata = tagLibrary.lookupTag(tagId);
        if (metadata == null) {
            return null;
        }

        // Get position from library (FTC coordinates)
        double ftcX = metadata.fieldPosition.get(0);  // inches
        double ftcY = metadata.fieldPosition.get(1);  // inches

        // Get heading from tag orientation (optional - may not be needed)
        double heading = 0;  // Can extract from metadata.fieldOrientation if needed

        // Use official converter
        return convertFTCPoseToPedro(ftcX, ftcY, heading);
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
    @Deprecated  ///  The FTC AprilTag code directly returns metadata, we don't need our own lookup tables and matching
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
     */
    public AutoAimResult getAutoAimData() {
        AprilTagDetection goalTag = findGoalTag();

        if (goalTag == null) {
            int visibleCount = getDetections().size();
            return AutoAimResult.failure(
                    String.format("FAILED - No goal tag detected (%d tags visible)", visibleCount)
            );
        }

        // Use ftcPose for distance/bearing (still useful for display)
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

    // ==================== COORDINATE CONVERSION HELPER ====================

    /**
     * Convert FTC coordinate system pose to Pedro coordinate system pose
     * Uses PedroPathing's official coordinate transformation
     *
     * @param x FTC X coordinate (inches, toward red alliance)
     * @param y FTC Y coordinate (inches, toward blue alliance/left)
     * @param heading Robot heading in radians
     * @return Pose in Pedro coordinate system
     */
    private static Pose convertFTCPoseToPedro(double x, double y, double heading) {
        return new Pose(x, y, heading, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
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