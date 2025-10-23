package org.firstinspires.ftc.teamcode.opMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Autonomous - 1 Ball Scoring with Park Fallback
 * 
 * Strategy:
 * 1. Move to shooting position
 * 2. Auto-aim at goal (uses Shooter.setAutoAimVelocity!)
 * 3. Shoot ball
 * 4. Park in observation zone
 * 
 * Fallback Modes:
 * - If auto-aim times out (no goal tag detected) → Park only
 * - If shooter doesn't reach velocity → Park only
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
@Autonomous(name = "Auto - 1 Ball + Park", group = "Competition")
public class Auto1BallPark extends BaseOpMode {
    
    // ==================== AUTONOMOUS CONSTANTS (PEDRO COORDINATES) ====================
    
    @Configurable
    public static class AutoConstants {
        // Shooting position (BLUE coordinates - auto-mirrored for RED)
        public static double SHOOT_X = 72.0;  // Mid-field (field center)
        public static double SHOOT_Y_NEAR = 30.0;  // Near audience side
        public static double SHOOT_Y_FAR = 114.0;  // Far side
        public static double SHOOT_HEADING_DEG = 180.0;  // Face blue wall (toward goal)
        
        // Park position (BLUE coordinates - observation zone)
        public static double PARK_X = 100.0;  // Closer to red wall
        public static double PARK_Y_NEAR = 40.0;
        public static double PARK_Y_FAR = 104.0;
        public static double PARK_HEADING_DEG = 270.0;  // Face audience
        
        // Timeouts (seconds)
        public static double MAX_AUTO_AIM_TIME = 5.0;  // Was 3.0
        public static double SHOOT_DURATION = 1.5;  // Time to feed ball
        public static double PARK_TIMEOUT = 8.0;  // Was 5.0
        public static double STUCK_TIMEOUT = 3.0;  // NEW: Detect if not moving
        
        // Auto-aim distance thresholds (inches)
        public static double MIN_SHOOT_DISTANCE = 24.0;
        // NOTE: MAX_DISTANCE removed - shooter scales power automatically
        // Future: Could add logic to move closer if too far
        
        // Shooter velocity error tolerance (RPM)
        public static double VELOCITY_TOLERANCE = 150.0;  // Increased from 100
        
        // Stuck detection
        public static double MIN_PROGRESS_DISTANCE = 3.0;  // inches per check interval
        public static double STUCK_CHECK_INTERVAL = 1.0;  // seconds between checks
    }
    
    // ==================== STATE MACHINE ====================
    
    private enum AutoState {
        MOVE_TO_SHOOT,      // Drive to shooting position
        AUTO_AIM,           // Aim at goal using vision
        SHOOT,              // Fire ball
        MOVE_TO_PARK,       // Drive to park position
        PARK,               // Stay parked
        DONE                // Autonomous complete
    }
    
    private AutoState currentState = AutoState.MOVE_TO_SHOOT;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();
    
    // Fallback tracking
    private boolean fallbackMode = false;
    private String fallbackReason = "";
    
    // Paths
    private PathChain pathToShoot;
    private PathChain pathToPark;
    
    // Stuck detection
    private Pose lastStuckCheckPose;
    private ElapsedTime stuckCheckTimer = new ElapsedTime();
    
    // ==================== START ====================
    
    @Override
    protected void onStart() {
        // Build paths based on selected position (uses Pedro coordinates!)
        buildPaths();
        
        // Follow first path
        follower.followPath(pathToShoot, true);
        
        // Start state machine
        currentState = AutoState.MOVE_TO_SHOOT;
        stateTimer.reset();
        autoTimer.reset();
        
        // Initialize stuck detection
        lastStuckCheckPose = follower.getPose();
        stuckCheckTimer.reset();
    }
    
    // ==================== MAIN LOOP ====================
    
    @Override
    public void loop() {
        // Update subsystems
        shooter.periodic();
        ballFeed.periodic();
        follower.update();
        
        // Periodic relocalization (if enabled)
        // performRelocalization();  // Uncomment when calculatePoseFromTag implemented
        
        // Check if stuck (only during movement states)
        if (currentState == AutoState.MOVE_TO_SHOOT || currentState == AutoState.MOVE_TO_PARK) {
            checkIfStuck();
        }
        
        // State machine
        switch (currentState) {
            case MOVE_TO_SHOOT:
                handleMoveToShoot();
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
        
        // Display telemetry
        displayAutoTelemetry();
    }
    
    // ==================== PATH BUILDING (PEDRO COORDINATES) ====================
    
    private void buildPaths() {
        // Get starting pose (already alliance-aware from BaseOpMode)
        Pose startPose = getStartingPose();
        
        // Determine positions based on NEAR vs FAR
        double shootY = position.toString().contains("Near") ? 
            AutoConstants.SHOOT_Y_NEAR : AutoConstants.SHOOT_Y_FAR;
        double parkY = position.toString().contains("Near") ?
            AutoConstants.PARK_Y_NEAR : AutoConstants.PARK_Y_FAR;
        
        // Build path to shooting position (PEDRO COORDINATES)
        Pose shootPose = new Pose(
            AutoConstants.SHOOT_X, 
            shootY,
            Math.toRadians(AutoConstants.SHOOT_HEADING_DEG)
        );
        
        pathToShoot = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(
                startPose.getHeading(),
                Math.toRadians(AutoConstants.SHOOT_HEADING_DEG)
            )
            .build();
        
        // Build path to park position (PEDRO COORDINATES)
        Pose parkPose = new Pose(
            AutoConstants.PARK_X,
            parkY,
            Math.toRadians(AutoConstants.PARK_HEADING_DEG)
        );
        
        pathToPark = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, parkPose))
            .setLinearHeadingInterpolation(
                Math.toRadians(AutoConstants.SHOOT_HEADING_DEG),
                Math.toRadians(AutoConstants.PARK_HEADING_DEG)
            )
            .build();
    }
    
    // ==================== STATE HANDLERS ====================
    
    private void handleMoveToShoot() {
        if (!follower.isBusy()) {
            // Reached shooting position
            currentState = AutoState.AUTO_AIM;
            stateTimer.reset();
        }
    }
    
    private void handleAutoAim() {
        Vision.AutoAimResult result = vision.getAutoAimData();
        
        // Check timeout
        if (stateTimer.seconds() > AutoConstants.MAX_AUTO_AIM_TIME) {
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
        if (result.distanceInches < AutoConstants.MIN_SHOOT_DISTANCE) {
            // FALLBACK: Too close to shoot safely
            fallbackMode = true;
            fallbackReason = "Too close to goal (" + result.distanceInches + " in)";
            follower.followPath(pathToPark, true);
            currentState = AutoState.MOVE_TO_PARK;
            stateTimer.reset();
            return;
        }
        
        // NOTE: No maximum distance check - shooter auto-aims for any distance
        // Future enhancement: Could add logic to move closer if confidence low
        
        // Set auto-aim velocity and start shooting
        shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
        currentState = AutoState.SHOOT;
        stateTimer.reset();
    }
    
    private void handleShoot() {
        // Wait for shooter to reach velocity
        double velocityError = Math.abs(
            shooter.getTargetVelocityRPM() - shooter.getActualVelocityRPM()
        );
        
        if (velocityError < AutoConstants.VELOCITY_TOLERANCE || stateTimer.seconds() > 1.5) {
            // Shooter ready (or timeout) - feed ball
            if (!ballFeed.isFeeding()) {
                ballFeed.startFeed();
            }
        }
        
        // Wait for shoot duration
        if (stateTimer.seconds() > AutoConstants.SHOOT_DURATION) {
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
        if (stateTimer.seconds() > AutoConstants.PARK_TIMEOUT) {
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
        if (stuckCheckTimer.seconds() < AutoConstants.STUCK_CHECK_INTERVAL) {
            return; // Not time to check yet
        }
        
        // Calculate distance traveled since last check
        Pose currentPose = follower.getPose();
        double dx = currentPose.getX() - lastStuckCheckPose.getX();
        double dy = currentPose.getY() - lastStuckCheckPose.getY();
        double distanceTraveled = Math.sqrt(dx * dx + dy * dy);
        
        if (distanceTraveled < AutoConstants.MIN_PROGRESS_DISTANCE && follower.isBusy()) {
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
        telemetryM.debug("=== AUTONOMOUS ===");
        telemetryM.debug("Robot: " + character.toString());
        telemetryM.debug("Alliance: " + alliance.toString());
        telemetryM.debug("Position: " + position.toString());
        telemetryM.debug("");
        
        telemetryM.debug("State: " + currentState);
        telemetryM.debug("Time: " + String.format("%.1fs", autoTimer.seconds()));
        if (fallbackMode) {
            telemetryM.debug("FALLBACK: " + fallbackReason);
        }
        telemetryM.debug("");
        
        telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.1f°",
            follower.getPose().getX(),
            follower.getPose().getY(),
            Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");
        
        telemetryM.debug("Shooter: " + String.format("%.0f/%.0f RPM", 
            shooter.getActualVelocityRPM(),
            shooter.getTargetVelocityRPM()));
        telemetryM.debug("Ball Feed: " + (ballFeed.isFeeding() ? "FEEDING" : "IDLE"));
        telemetryM.debug("Follower: " + (follower.isBusy() ? "BUSY" : "IDLE"));
        
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
