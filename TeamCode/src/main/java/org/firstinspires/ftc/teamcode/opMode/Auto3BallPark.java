package org.firstinspires.ftc.teamcode.opMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.FieldMirror;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Autonomous - 3 Ball Scoring with Park
 *
 * Strategy:
 * 1. Move to shooting position
 * 2. Auto-aim at goal (uses Shooter.setAutoAimVelocity!)
 * 3. Shoot 3 balls with timed feeding
 * 4. Park in observation zone
 *
 * Fallback Modes:
 * - If auto-aim times out (no goal tag detected) → Park only
 * - If robot gets stuck → Park only
 *
 * IMPORTANT: All positions use PEDRO COORDINATES!
 * Reference: https://pedropathing.com/docs/pathing/reference/coordinates
 *
 * State Transfer:
 * - Saves robot/alliance/pose to RobotState at end
 * - TeleOp can read this to skip selection
 *
 * Path Design:
 * - Use Pedro Visualizer for complex paths: https://visualizer.pedropathing.com/
 * - This example uses simple BezierLine paths
 */
@Autonomous(name = "Auto - 3 Ball + Park", group = "Competition", preselectTeleOp = "TeleOp - Competition")
@Configurable
public class Auto3BallPark extends BaseOpMode {

    // ==================== CONFIGURABLE CONSTANTS ====================

    /**
     * Autonomous constants - NOW CONFIGURABLE VIA PANELS!
     * Instance-based pattern matches rest of codebase (Shooter, BallFeed, etc.)
     */
    public static AutoConfig autoConfig = new AutoConfig();

    public static class AutoConfig {
        // Shooting position (BLUE coordinates - auto-mirrored for RED)
        public double shootX = 56.8;  // Mid-field (field center)
        public double shootYNear = 15.4;  // Near audience side
        public double shootYFar = 109.0;  // Far side
        public double shootHeadingNear = 110.0;  // Face blue wall (toward goal)
        public double shootHeadingFar = 147.6;  // Face blue wall (toward goal)

        // Park position (BLUE coordinates - observation zone)
        public double parkX = 44.0;  // Closer to red wall
        public double parkYNear = 40.0;
        public double parkYFar = 70.0;
        public double parkHeadingDeg = 270.0;  // Face audience

        // Timeouts (seconds)
        public double maxAutoAimTime = 8.0;
        public double shootDuration = 12.0;
        public double parkTimeout = 3.0;
        public double velocityToleranceRPM = 50.0;

        // Shot timing (seconds)
        public double stopFeedDelay = 0.6;  // Stop feeding between shots
        public double stopFeedDuration = 0.2;  // How long to stop
        public double minTimeBetweenShots = 1.0;  // Minimum time between shots

        // Auto-aim distance thresholds (inches)
        public double minShootDistance = 24.0;

        // Stuck detection
        public double minProgressDistance = 3.0;  // inches per check interval
        public double stuckCheckInterval = 1.0;  // seconds between checks
    }

    // ==================== STATE MACHINE ====================

    private enum AutoState {
        MOVE_TO_SHOOT,     // Drive to shooting position
        HOLD_POS,          // Brief pause before auto-aim
        AUTO_AIM,          // Aim at goal using vision
        SHOOT,             // Fire balls with timed feeding
        MOVE_TO_PARK,      // Drive to park position
        PARK,              // Stay parked
        DONE               // Autonomous complete
    }

    // ==================== INSTANCE FIELDS ====================

    private AutoState currentState = AutoState.MOVE_TO_SHOOT;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();

    // Fallback tracking
    private boolean fallbackMode = false;
    private String fallbackReason = "";

    // Paths
    private PathChain pathToShoot;
    private PathChain pathToPark;

    // Stuck detection
    private Pose lastStuckCheckPose;
    private ElapsedTime stuckCheckTimer = new ElapsedTime();

    // Shot tracking
    private int shotsFired = 0;
    private boolean feedingStopped = false;

    // ==================== START ====================

    @Override
    protected void onStart() {
        // Apply character-specific shooter velocities
        shooter.setProperShooterVelocities();

        // Build paths based on selected position (uses Pedro coordinates!)
        buildPaths();

        // Follow first path
        follower.followPath(pathToShoot, false);
        shooter.setHighVelocity();

        // Start state machine
        currentState = AutoState.MOVE_TO_SHOOT;
        stateTimer.reset();
        autoTimer.reset();
        shotTimer.reset();

        // Initialize stuck detection
        lastStuckCheckPose = follower.getPose();
        stuckCheckTimer.reset();
    }

    // ==================== MAIN LOOP ====================

    @Override
    public void loop() {
        // Update follower (NO debug() call - it conflicts with telemetry!)
        follower.update();

        // Relocalize automatically every 0.5s (rate-limited in method)
        relocalize();

        // Update subsystems
        shooter.periodic();
        ballFeed.periodic();

        // Check if stuck (only during movement states)
        if (currentState == AutoState.MOVE_TO_SHOOT || currentState == AutoState.MOVE_TO_PARK) {
            checkIfStuck();
        }

        // State machine
        switch (currentState) {
            case MOVE_TO_SHOOT:
                handleMoveToShoot();
                break;
            case HOLD_POS:
                handleHoldPos();
                break;
            case AUTO_AIM:
                handleAutoAim();
                break;
            case SHOOT:
                handleShoot();
                break;
            case MOVE_TO_PARK:
                handleMoveToPark();
                break;
            case PARK:
                handlePark();
                break;
            case DONE:
                // Do nothing, autonomous complete
                break;
        }

        // Display telemetry ONCE at end of loop
        displayAutoTelemetry();
    }

    // ==================== PATH BUILDING (PEDRO COORDINATES) ====================

    private void buildPaths() {
        // Get starting pose (already alliance-aware from BaseOpMode)
        Pose startPose = getStartingPose();

        // Determine positions based on NEAR vs FAR
        boolean isNear = startPoseInSelectMenu.toString().contains("Near");
        double shootY = isNear ? autoConfig.shootYNear : autoConfig.shootYFar;
        double shootHeading = Math.toRadians(isNear ? autoConfig.shootHeadingNear : autoConfig.shootHeadingFar);
        double parkY = isNear ? autoConfig.parkYNear : autoConfig.parkYFar;

        // Build path to shooting position (PEDRO COORDINATES), then mirror
        Pose shootPose = new Pose(autoConfig.shootX, shootY, shootHeading);
        shootPose = FieldMirror.mirrorPose(shootPose, alliance);

        // Build path using the mirrored pose
        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Build path to park position (PEDRO COORDINATES)
        Pose parkPose = new Pose(
                autoConfig.parkX,
                parkY,
                Math.toRadians(autoConfig.parkHeadingDeg)
        );
        parkPose = FieldMirror.mirrorPose(parkPose, alliance);

        pathToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    // ==================== STATE HANDLERS ====================

    private void handleMoveToShoot() {
        if (!follower.isBusy()) {
            // Reached shooting position
            currentState = AutoState.HOLD_POS;
            stateTimer.reset();
        }
    }

    private void handleHoldPos() {
        // Brief pause to settle before auto-aim
        if (stateTimer.seconds() > 0.1) {
            follower.followPath(pathToShoot, false);
            currentState = AutoState.AUTO_AIM;
            stateTimer.reset();
        }
    }

    private void handleAutoAim() {
        Vision.AutoAimResult result = vision.getAutoAimData();

        // Check timeout
        if (stateTimer.seconds() > autoConfig.maxAutoAimTime) {
            // FALLBACK: Auto-aim timed out - no goal tag detected
            fallbackMode = true;
            fallbackReason = "Auto-aim timeout (no goal tag)";
            follower.followPath(pathToPark, true);
            currentState = AutoState.MOVE_TO_PARK;
            stateTimer.reset();
            return;
        }

        // Check if we have valid target
        if (!result.success) {
            return; // Keep waiting for goal tag
        }

        // Validate minimum shooting distance
        if (result.distanceInches < autoConfig.minShootDistance) {
            // FALLBACK: Too close to shoot safely
            fallbackMode = true;
            fallbackReason = String.format("Too close to goal (%.1f in)", result.distanceInches);
            follower.followPath(pathToPark, true);
            currentState = AutoState.MOVE_TO_PARK;
            stateTimer.reset();
            return;
        }

        // Set auto-aim velocity and start shooting
        if (startPoseInSelectMenu.toString().contains("Far")) {
            shooter.setLowVelocity();
        } else {
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
        }

        // Start intake (helps feeding)
        if (intake != null) {
            intake.startIntakeOne();
            intake.startIntakeTwo();
        }

        currentState = AutoState.SHOOT;
        stateTimer.reset();
        shotTimer.reset();
        shotsFired = 0;
        feedingStopped = false;
    }

    private void handleShoot() {
        // Wait for shooter to reach velocity
        double velocityError = Math.abs(
                shooter.getTargetVelocityRPM() - shooter.getActualVelocityRPM()
        );

        // Check if shooter is ready (or timeout after 3 seconds)
        if (velocityError < autoConfig.velocityToleranceRPM || stateTimer.seconds() > 3.0) {

            // SHOT TIMING LOGIC:
            // Stop feeding briefly between shots to prevent double-feeds
            double timeSinceLastShot = shotTimer.seconds();

            if (timeSinceLastShot > autoConfig.stopFeedDelay &&
                    timeSinceLastShot < (autoConfig.stopFeedDelay + autoConfig.stopFeedDuration)) {
                // Stop window: briefly stop feeding
                if (!feedingStopped) {
                    ballFeed.feedLeftAuto(true);   // Stop left
                    ballFeed.feedRightAuto(true);  // Stop right
                    feedingStopped = true;
                }
            } else if (timeSinceLastShot >= autoConfig.minTimeBetweenShots) {
                // Ready for next shot
                if (shotsFired < 3) {
                    // Fire shot (alternate left/right for 22154)
                    if (shotsFired == 0 ) {
                        ballFeed.feedLeftAuto(false);   // Start left feed
                    } else if( shotsFired == 1 ) {
                        ballFeed.feedRightAuto(false);  // Start right feed
                    } else {
                        ballFeed.feedLeftAuto(false);   // Start left feed
                        ballFeed.feedRightAuto(false);   // Start left feed
                    }

                    shotsFired++;
                    shotTimer.reset();
                    feedingStopped = false;
                }
            }
        }

        // Done shooting? Move to park
        if (shotsFired >= 3 && stateTimer.seconds() > autoConfig.shootDuration) {
            shooter.stop();
            ballFeed.stopFeed();

            // Follow park path
            follower.followPath(pathToPark, true);
            currentState = AutoState.MOVE_TO_PARK;
            stateTimer.reset();

            // Reset stuck detection for park path
            lastStuckCheckPose = follower.getPose();
            stuckCheckTimer.reset();
        }
    }

    private void handleMoveToPark() {
        if (!follower.isBusy()) {
            // Reached park position
            currentState = AutoState.PARK;
            stateTimer.reset();
        }

        // Safety timeout
        if (stateTimer.seconds() > autoConfig.parkTimeout) {
            // FAILSAFE: Park timeout - stop trying to prevent field damage
            currentState = AutoState.PARK;
            stateTimer.reset();
        }
    }

    private void handlePark() {
        // Hold position for a moment
        if (stateTimer.seconds() > 0.5) {
            currentState = AutoState.DONE;

            // Save state for TeleOp transition
            RobotState.saveState(character, alliance, follower.getPose());
        }
    }

    // ==================== STUCK DETECTION ====================

    /**
     * Check if robot is stuck (not making progress toward goal)
     * Prevents damaging field by spinning in place
     */
    private void checkIfStuck() {
        if (stuckCheckTimer.seconds() < autoConfig.stuckCheckInterval) {
            return; // Not time to check yet
        }

        // Calculate distance traveled since last check
        Pose currentPose = follower.getPose();
        double dx = currentPose.getX() - lastStuckCheckPose.getX();
        double dy = currentPose.getY() - lastStuckCheckPose.getY();
        double distanceTraveled = Math.sqrt(dx * dx + dy * dy);

        if (distanceTraveled < autoConfig.minProgressDistance && follower.isBusy()) {
            // FAILSAFE: Robot is stuck - abort to park state
            follower.breakFollowing();  // Stop current path
            currentState = AutoState.PARK;
            stateTimer.reset();

            if (!fallbackMode) {
                fallbackMode = true;
                fallbackReason = "Robot stuck (not making progress)";
            }
        }

        // Update for next check
        lastStuckCheckPose = currentPose;
        stuckCheckTimer.reset();
    }

    // ==================== TELEMETRY ====================

    private void displayAutoTelemetry() {
        // Clear telemetry (fresh start each loop)
        telemetryM.debug("=== AUTONOMOUS ===");
        telemetryM.debug(String.format("Time: %.1fs | State: %s", autoTimer.seconds(), currentState));
        if (fallbackMode) {
            telemetryM.debug("⚠ FALLBACK: " + fallbackReason);
        }
        telemetryM.debug("");

        // Robot state
        telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.0f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug(String.format("Busy: %s", follower.isBusy() ? "YES" : "NO"));
        telemetryM.debug("");

        // Shooter state
        double actualRPM = shooter.getActualVelocityRPM();
        double targetRPM = shooter.getTargetVelocityRPM();
        double error = Math.abs(targetRPM - actualRPM);
        telemetryM.debug(String.format("Shooter: %.0f/%.0f RPM (err: %.0f)",
                actualRPM, targetRPM, error));
        telemetryM.debug("");

        // BallFeed state (DETAILED for debugging!)
        telemetryM.debug("=== BALL FEED ===");
        telemetryM.debug(String.format("Shots: %d/3 | Timer: %.2fs", shotsFired, shotTimer.seconds()));
        telemetryM.debug(String.format("L: %s | R: %s",
                ballFeed.getLeftStateString(),
                ballFeed.getRightStateString()));
        telemetryM.debug(String.format("Feeding: L=%s R=%s",
                ballFeed.isLeftFeeding() ? "YES" : "NO",
                ballFeed.isRightFeeding() ? "YES" : "NO"));
        telemetryM.debug("");

        // Vision state
        List<AprilTagDetection> detections = vision.getDetections();
        telemetryM.debug(String.format("Vision: %d tags", detections.size()));
        if (currentState == AutoState.AUTO_AIM && detections.size() > 0) {
            Vision.AutoAimResult result = vision.getAutoAimData();
            if (result.success) {
                telemetryM.debug(String.format("  Goal: Tag %d @ %.1f in",
                        result.tagId, result.distanceInches));
            }
        }

        // Single update at end
        telemetryM.update(telemetry);
    }

    // ==================== STOP ====================

    @Override
    protected void onStop() {
        // Ensure state is saved even if we didn't reach DONE
        if (currentState != AutoState.DONE) {
            RobotState.saveState(character, alliance, follower.getPose());
        }
    }
}