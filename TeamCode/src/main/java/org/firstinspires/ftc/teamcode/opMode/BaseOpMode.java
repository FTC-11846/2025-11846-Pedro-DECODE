package org.firstinspires.ftc.teamcode.opMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BallFeed;
import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.MainCharacter;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldMirror;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.StartingPosition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * BaseOpMode - Abstract base class for all competition OpModes
 *
 * Provides:
 * - Progressive selection UI (Robot → Alliance → Position → Ready)
 * - State restoration from previous OpMode (Auto → TeleOp)
 * - Common subsystem initialization
 * - Relocalization utilities using AprilTag goals (Tags 20, 24)
 * - Emergency stop coordination
 *
 * Hardware Init Timing:
 * - IMU initialized in init() (required early)
 * - Subsystems initialized AFTER position selected (prevents wrong robot crash)
 * - Follower created in start() (after all selections confirmed)
 *
 * Child classes (TeleOp, Autonomous) extend this and override hooks
 */
@Configurable
public abstract class BaseOpMode extends OpMode {

    // ==================== RELOCALIZATION CONFIGURATION ====================

    public static class RelocalizationConfig {
        public boolean enabled = true;
        public double minConfidence = 0.8;        // Min detection confidence
        public double maxPoseError = 24.0;        // Inches - reject if too far off
        public double blendRatio = 0.3;           // 30% vision, 70% odometry
        public double rateLimitSeconds = 0.5;     // Time between relocalization attempts
    }

    public static RelocalizationConfig relocalization = new RelocalizationConfig();
    private ElapsedTime relocalizationTimer = new ElapsedTime();

    // ==================== SELECTION STATE ====================

    protected enum InitStage {
        SELECT_ROBOT,      // Stage 0: Choose robot (TestBot, 22154, 11846)
        SELECT_ALLIANCE,   // Stage 1: Choose alliance (Red, Blue)
        SELECT_POSITION,   // Stage 2: Choose position (Near, Far)
        READY              // Stage 3: Ready to start
    }

    private InitStage currentStage = InitStage.SELECT_ROBOT;

    // Selection indices
    private int selectedRobotIndex = 0;
    private int selectedAllianceIndex = 0;
    private int selectedPositionIndex = 0;

    // Flag to track if hardware initialized
    private boolean hardwareInitialized = false;

    // ==================== CONFIRMED SELECTIONS ====================

    protected MainCharacter character;
    protected Alliance alliance;
    protected StartingPosition position;

    // ==================== COMMON SUBSYSTEMS ====================

    protected Shooter shooter;
    protected BallFeed ballFeed;
    protected Vision vision;
    protected LED led;  // null if robot doesn't have LEDs
    protected Intake intake;  // null if robot doesn't have intake
    protected ColorSensors colorSensors;  // null if robot doesn't have color sensors

    protected Follower follower;
    protected IMU imu;
    protected TelemetryManager telemetryM;

    // ==================== BUTTON STATE TRACKING ====================

    private boolean aButtonLast = false;
    private boolean bButtonLast = false;
    private ElapsedTime buttonRateLimit = new ElapsedTime();
    private static final double BUTTON_RATE_MS = 300;

    // ==================== INIT LIFECYCLE ====================

    @Override
    public final void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize IMU early (needed for follower later)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Check if we have saved state from previous OpMode (Auto → TeleOp)
        if (RobotState.hasState()) {
            // Restore from saved state
            character = RobotState.activeRobot;
            alliance = RobotState.activeAlliance;
            MainCharacter.ACTIVE_ROBOT = character;

            // Initialize hardware with restored robot
            initializeSubsystems();

            // Skip to position selection (alliance already known)
            currentStage = InitStage.SELECT_POSITION;
            selectedAllianceIndex = alliance == Alliance.RED ? 0 : 1;

            telemetryM.debug("=== STATE RESTORED ===");
            telemetryM.debug(RobotState.getStateString());
        } else {
            // Normal init - start from robot selection
            telemetryM.debug("=== INITIALIZATION ===");
            telemetryM.debug("Waiting for robot selection...");
        }

        // Child class can override for additional init
        onInitialize();

        telemetryM.update(telemetry);
    }

    @Override
    public final void init_loop() {
        switch (currentStage) {
            case SELECT_ROBOT:
                handleRobotSelection();
                break;
            case SELECT_ALLIANCE:
                handleAllianceSelection();
                break;
            case SELECT_POSITION:
                handlePositionSelection();
                break;
            case READY:
                handleReadyState();
                break;
        }

        telemetryM.update(telemetry);
    }

    @Override
    public final void start() {
        // Create follower with selected starting pose
        follower = Constants.createFollower(hardwareMap, character);

        // Set starting pose (restore from state or use selection)
        Pose startPose;
        if (RobotState.hasState() && RobotState.lastKnownPose != null) {
            startPose = RobotState.lastKnownPose;
            RobotState.clearState();  // Clear after use
        } else {
            startPose = getStartingPose();
        }
        follower.setPose(startPose);

        // Set coordinate system offsets for Panels field view
        com.bylazar.field.PanelsField.INSTANCE.getField()
                .setOffsets(com.bylazar.field.PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        // Child class handles start behavior
        onStart();
    }

    @Override
    public final void stop() {
        // Emergency stop all subsystems
        if (shooter != null) shooter.emergencyStop();
        if (ballFeed != null) ballFeed.stopFeed();
        if (intake != null) intake.stopAll();
        if (vision != null) vision.close();

        // Child class cleanup
        onStop();
    }

    // ==================== SELECTION HANDLERS ====================

    private void handleRobotSelection() {
        // DPAD up/down to change selection
        if (gamepad1.dpad_up && buttonRateLimit.milliseconds() > BUTTON_RATE_MS) {
            selectedRobotIndex = (selectedRobotIndex - 1 + MainCharacter.values().length)
                    % MainCharacter.values().length;
            buttonRateLimit.reset();
        } else if (gamepad1.dpad_down && buttonRateLimit.milliseconds() > BUTTON_RATE_MS) {
            selectedRobotIndex = (selectedRobotIndex + 1) % MainCharacter.values().length;
            buttonRateLimit.reset();
        }

        // A to confirm
        if (gamepad1.a && !aButtonLast) {
            character = MainCharacter.values()[selectedRobotIndex];
            character.getAbilities().applyConfiguration();
            MainCharacter.ACTIVE_ROBOT = character;
            currentStage = InitStage.SELECT_ALLIANCE;

            // NOTE: DO NOT initialize subsystems yet!
            // Wait until position selected (user has 2 more chances to go back)
        }
        aButtonLast = gamepad1.a;

        // Display
        displayRobotSelection();
    }

    private void handleAllianceSelection() {
        // DPAD up/down
        if (gamepad1.dpad_up && buttonRateLimit.milliseconds() > BUTTON_RATE_MS) {
            selectedAllianceIndex = (selectedAllianceIndex - 1 + Alliance.values().length)
                    % Alliance.values().length;
            buttonRateLimit.reset();
        } else if (gamepad1.dpad_down && buttonRateLimit.milliseconds() > BUTTON_RATE_MS) {
            selectedAllianceIndex = (selectedAllianceIndex + 1) % Alliance.values().length;
            buttonRateLimit.reset();
        }

        // A to confirm
        if (gamepad1.a && !aButtonLast) {
            alliance = Alliance.values()[selectedAllianceIndex];
            currentStage = InitStage.SELECT_POSITION;
        }

        // B to go back
        if (gamepad1.b && !bButtonLast) {
            currentStage = InitStage.SELECT_ROBOT;
        }

        aButtonLast = gamepad1.a;
        bButtonLast = gamepad1.b;

        // Display
        displayAllianceSelection();
    }

    private void handlePositionSelection() {
        // DPAD up/down
        if (gamepad1.dpad_up && buttonRateLimit.milliseconds() > BUTTON_RATE_MS) {
            selectedPositionIndex = (selectedPositionIndex - 1 + StartingPosition.values().length)
                    % StartingPosition.values().length;
            buttonRateLimit.reset();
        } else if (gamepad1.dpad_down && buttonRateLimit.milliseconds() > BUTTON_RATE_MS) {
            selectedPositionIndex = (selectedPositionIndex + 1) % StartingPosition.values().length;
            buttonRateLimit.reset();
        }

        // A to confirm
        if (gamepad1.a && !aButtonLast) {
            position = StartingPosition.values()[selectedPositionIndex];
            currentStage = InitStage.READY;

            // NOW initialize hardware (user has confirmed all choices)
            if (!hardwareInitialized) {
                initializeSubsystems();
                hardwareInitialized = true;
            }
        }

        // B to go back
        if (gamepad1.b && !bButtonLast) {
            currentStage = InitStage.SELECT_ALLIANCE;
        }

        aButtonLast = gamepad1.a;
        bButtonLast = gamepad1.b;

        // Display
        displayPositionSelection();
    }

    private void handleReadyState() {
        // B to restart selection
        if (gamepad1.b && !bButtonLast) {
            currentStage = InitStage.SELECT_ROBOT;
            selectedRobotIndex = 0;
            selectedAllianceIndex = 0;
            selectedPositionIndex = 0;
        }
        bButtonLast = gamepad1.b;

        // Display summary
        displayReadySummary();
    }

    // ==================== SUBSYSTEM INITIALIZATION ====================

    private void initializeSubsystems() {
        shooter = new Shooter(hardwareMap, character);
        ballFeed = new BallFeed(hardwareMap, character);
        vision = new Vision(hardwareMap);

        // Only initialize LED if robot has the hardware
        if (character.getAbilities().hasLEDSystem()) {
            led = new LED(hardwareMap, character);
        } else {
            led = null;
        }

        // Only initialize intake if robot has it
        if (character.getAbilities().getIntakeMode() != CharacterStats.IntakeMode.NONE) {
            intake = new Intake(hardwareMap, character);
        } else {
            intake = null;
        }

        // Only initialize color sensors if robot has them
        if (character.getAbilities().hasColorSensors()) {
            colorSensors = new ColorSensors(hardwareMap, character);
        } else {
            colorSensors = null;
        }

        // Child class can add more subsystems
        onSubsystemsInitialized();
    }

    // ==================== RELOCALIZATION METHODS ====================

    /**
     * Perform periodic relocalization using AprilTag detection
     * Call this in loop() for continuous drift correction
     *
     * Uses ONLY goal tags (20 Blue, 24 Red) - NOT motif tags
     * Auto-rate-limited and applies blended pose correction
     */
    protected void relocalize() {
        if (!relocalization.enabled) return;
        if (relocalizationTimer.seconds() < relocalization.rateLimitSeconds) return;
        if (vision == null) return;

        // Find best goal tag (prefer closer for accuracy)
        AprilTagDetection bestTag = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection detection : vision.getDetections()) {
            if (!vision.isValidDetection(detection)) continue;

            // ONLY use goal tags (20 Blue, 24 Red)
            if (detection.id != Vision.tagIds.blueGoalTagId &&
                    detection.id != Vision.tagIds.redGoalTagId) {
                continue;
            }

            // Prefer closer tags for better accuracy
            if (detection.ftcPose.range < bestRange) {
                bestTag = detection;
                bestRange = detection.ftcPose.range;
            }
        }

        if (bestTag == null) return;

        // Calculate pose from tag
        Pose visionPose = calculatePoseFromTag(bestTag);
        if (visionPose == null) return;

        Pose currentPose = follower.getPose();

        // Check if error is reasonable
        double error = Math.hypot(
                visionPose.getX() - currentPose.getX(),
                visionPose.getY() - currentPose.getY()
        );

        if (error > relocalization.maxPoseError) {
            return; // Error too large, likely bad detection
        }

        // Blend poses
        double visionWeight = relocalization.blendRatio;
        double odoWeight = 1.0 - visionWeight;

        Pose blendedPose = new Pose(
                currentPose.getX() * odoWeight + visionPose.getX() * visionWeight,
                currentPose.getY() * odoWeight + visionPose.getY() * visionWeight,
                currentPose.getHeading() // Trust odometry heading
        );

        // Apply correction
        follower.setPose(blendedPose);
        relocalizationTimer.reset();
    }

    /**
     * Calculate robot pose from AprilTag detection
     * Uses Vision's goal tag positions (20 Blue, 24 Red)
     */
    private Pose calculatePoseFromTag(AprilTagDetection tag) {
        // Get tag field position from Vision
        Pose tagPose = getGoalTagPose(tag.id);
        if (tagPose == null) {
            return null; // Unknown tag
        }

        // Get detection data
        if (tag.ftcPose == null) {
            return null; // Invalid detection
        }

        double range = tag.ftcPose.range;      // Distance to tag (inches)
        double bearing = tag.ftcPose.bearing;  // Horizontal angle to tag (degrees)

        // Convert bearing to radians
        double bearingRad = Math.toRadians(bearing);

        // Robot's heading (from current odometry - we trust this more than vision)
        double robotHeading = follower.getPose().getHeading();

        // Calculate absolute bearing to tag
        double absoluteBearing = robotHeading + bearingRad;

        // Calculate robot position
        // Robot is "range" distance away from tag, at angle "absoluteBearing" FROM the tag
        // So: robot position = tag position - (range * direction vector)
        double robotX = tagPose.getX() - range * Math.cos(absoluteBearing);
        double robotY = tagPose.getY() - range * Math.sin(absoluteBearing);

        // TODO: Apply camera offset transform if camera not at robot center
        // This would require camera offset constants (forward, right offsets)

        return new Pose(robotX, robotY, robotHeading);
    }

    /**
     * Get the field pose for a goal tag using Vision's configuration
     * Returns Pose in Pedro coordinates with proper heading
     */
    private Pose getGoalTagPose(int tagId) {
        if (tagId == Vision.tagIds.blueGoalTagId) {
            // Blue goal on right side of field, facing left (180°)
            return new Pose(
                    Vision.goalPositions.blueGoalX,
                    Vision.goalPositions.blueGoalY,
                    Math.toRadians(180)
            );
        } else if (tagId == Vision.tagIds.redGoalTagId) {
            // Red goal on left side of field, facing right (0°)
            return new Pose(
                    Vision.goalPositions.redGoalX,
                    Vision.goalPositions.redGoalY,
                    Math.toRadians(0)
            );
        }
        return null;
    }

    // ==================== UTILITIES ====================

    /**
     * Get the starting pose based on selected alliance and position
     * Uses FieldMirror to handle alliance-specific mirroring
     * Returns PEDRO COORDINATES
     */
    protected Pose getStartingPose() {
        Pose bluePose = position.getBluePose();
        return FieldMirror.mirrorPose(bluePose, alliance);
    }

    // ==================== DISPLAY HELPERS ====================

    private void displayRobotSelection() {
        telemetryM.debug("=== SELECT ROBOT ===");
        telemetryM.debug("Use DPAD UP/DOWN, Press A to confirm");
        telemetryM.debug("");
        for (int i = 0; i < MainCharacter.values().length; i++) {
            String marker = (i == selectedRobotIndex) ? " >>> " : "     ";
            telemetryM.debug(marker + MainCharacter.values()[i].toString());
        }
    }

    private void displayAllianceSelection() {
        telemetryM.debug("=== SELECT ALLIANCE ===");
        telemetryM.debug("Robot: " + character.toString() + " ✓");
        telemetryM.debug("");
        telemetryM.debug("Use DPAD UP/DOWN");
        telemetryM.debug("Press A to confirm, B to go back");
        telemetryM.debug("");
        for (int i = 0; i < Alliance.values().length; i++) {
            String marker = (i == selectedAllianceIndex) ? " >>> " : "     ";
            telemetryM.debug(marker + Alliance.values()[i].toString());
        }
    }

    private void displayPositionSelection() {
        telemetryM.debug("=== SELECT STARTING POSITION ===");
        telemetryM.debug("Robot: " + character.toString() + " ✓");
        telemetryM.debug("Alliance: " + alliance.toString() + " ✓");
        telemetryM.debug("");
        telemetryM.debug("Use DPAD UP/DOWN");
        telemetryM.debug("Press A to confirm, B to go back");
        telemetryM.debug("");
        for (int i = 0; i < StartingPosition.values().length; i++) {
            String marker = (i == selectedPositionIndex) ? " >>> " : "     ";
            // Show alliance-specific name (e.g., "Red Near")
            String displayName = alliance.toString() + " " + StartingPosition.values()[i].toString();
            telemetryM.debug(marker + displayName);
        }
    }

    private void displayReadySummary() {
        telemetryM.debug("=== READY TO START ===");
        telemetryM.debug("Robot: " + character.toString() + " ✓");
        telemetryM.debug("Alliance: " + alliance.toString() + " ✓");
        telemetryM.debug("Position: " + alliance + " " + position + " ✓");
        telemetryM.debug("");

        Pose startPose = getStartingPose();
        telemetryM.debug(String.format("Start Pose (Pedro): X=%.1f Y=%.1f H=%.1f°",
                startPose.getX(),
                startPose.getY(),
                Math.toDegrees(startPose.getHeading())));
        telemetryM.debug("");
        telemetryM.debug("Press START to begin");
        telemetryM.debug("Press B to change selections");
    }

    // ==================== CHILD CLASS HOOKS ====================

    /**
     * Override to add custom initialization logic
     * Called during init() after IMU setup
     */
    protected void onInitialize() {
        // Override in child class if needed
    }

    /**
     * Override to add custom subsystems after base subsystems initialized
     * Called after Shooter, BallFeed, Vision, LED, Intake, ColorSensors are created
     */
    protected void onSubsystemsInitialized() {
        // Override in child class if needed
    }

    /**
     * Override to add custom start behavior
     * Called in start() after follower is created and pose is set
     */
    protected void onStart() {
        // Override in child class if needed
    }

    /**
     * Override to add custom stop behavior
     * Called in stop() after subsystems are emergency stopped
     */
    protected void onStop() {
        // Override in child class if needed
    }
}