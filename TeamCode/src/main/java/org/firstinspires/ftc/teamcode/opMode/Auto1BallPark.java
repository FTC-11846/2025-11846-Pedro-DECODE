package org.firstinspires.ftc.teamcode.opMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Autonomous - 1 Ball Scoring with Park Fallback
 *
 * Strategy:
 * 1. Move to shooting position
 * 2. Auto-aim at goal (using existing Shooter auto-aim!)
 * 3. Shoot ball
 * 4. Park in observation zone
 *
 * Fallback: If auto-aim fails, just park (guaranteed points)
 *
 * Alliance Awareness: Paths defined in BLUE coordinates, mirrored via BaseOpMode
 *
 * NOTE: This uses loop-based OpMode (not LinearOpMode) because:
 * - Pedro Pathing designed for follower.update() in loop()
 * - Enables continuous relocalization
 * - Consistent with BaseOpMode architecture
 *
 * Path Design:
 * - Use Pedro Visualizer to design complex paths: https://visualizer.pedropathing.com/
 * - Export code and adapt to your OpMode
 * - This example uses simple BezierLine paths
 */
@Autonomous(name = "Auto - 1 Ball + Park", group = "Competition")
public class Auto1BallPark extends BaseOpMode {

    // ==================== AUTONOMOUS CONSTANTS ====================

    @Configurable
    public static class AutoConstants {
        // Shooting position (BLUE coordinates - mirrored for RED automatically)
        public static double SHOOT_X = 72.0;  // Mid-field
        public static double SHOOT_Y_NEAR = 24.0;  // Near audience side
        public static double SHOOT_Y_FAR = 120.0;  // Far side
        public static double SHOOT_HEADING_DEG = 180.0;  // Face goal

        // Park position (BLUE coordinates - observation zone)
        public static double PARK_X = 96.0;
        public static double PARK_Y_NEAR = 36.0;
        public static double PARK_Y_FAR = 108.0;
        public static double PARK_HEADING_DEG = 180.0;

        // Timeouts (seconds)
        public static double MAX_AUTO_AIM_TIME = 3.0;
        public static double SHOOT_DURATION = 1.0;
        public static double PARK_TIMEOUT = 5.0;

        // Auto-aim distance thresholds (inches)
        public static double MIN_SHOOT_DISTANCE = 24.0;
        public static double MAX_SHOOT_DISTANCE = 96.0;

        // Shooter velocity error tolerance (RPM)
        public static double VELOCITY_TOLERANCE = 100.0;
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

    // Fallback mode (if auto-aim fails)
    private boolean fallbackMode = false;

    // Paths
    private PathChain pathToShoot;
    private PathChain pathToPark;

    // ==================== START ====================

    @Override
    protected void onStart() {
        // Build paths based on selected position
        buildPaths();

        // Follow first path
        follower.followPath(pathToShoot, true);

        // Start state machine
        currentState = AutoState.MOVE_TO_SHOOT;
        stateTimer.reset();
        autoTimer.reset();
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

    // ==================== PATH BUILDING ====================

    private void buildPaths() {
        // Get starting pose (already alliance-aware from BaseOpMode)
        Pose startPose = getStartingPose();

        // Determine positions based on NEAR vs FAR
        double shootY = position.toString().contains("Near") ?
                AutoConstants.SHOOT_Y_NEAR : AutoConstants.SHOOT_Y_FAR;
        double parkY = position.toString().contains("Near") ?
                AutoConstants.PARK_Y_NEAR : AutoConstants.PARK_Y_FAR;

        // Build path to shooting position
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

        // Build path to park position
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
            // Auto-aim failed - enter fallback mode (just park)
            fallbackMode = true;
            follower.followPath(pathToPark, true);
            currentState = AutoState.MOVE_TO_PARK;
            stateTimer.reset();
            return;
        }

        // Check if we have valid target
        if (!result.success) {
            return; // Keep waiting for goal tag
        }

        // Validate shooting distance
        if (result.distanceInches < AutoConstants.MIN_SHOOT_DISTANCE ||
                result.distanceInches > AutoConstants.MAX_SHOOT_DISTANCE) {
            fallbackMode = true;
            follower.followPath(pathToPark, true);
            currentState = AutoState.MOVE_TO_PARK;
            stateTimer.reset();
            return;
        }

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

        if (velocityError < AutoConstants.VELOCITY_TOLERANCE || stateTimer.seconds() > 1.0) {
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
            currentState = AutoState.PARK;
            stateTimer.reset();
        }
    }

    private void handlePark() {
        // Just stay here until autonomous ends
        if (stateTimer.seconds() > 1.0) {
            currentState = AutoState.DONE;
        }
    }

    // ==================== TELEMETRY ====================

    private void displayAutoTelemetry() {
        telemetryM.debug("=== AUTONOMOUS ===");
        telemetryM.debug("Robot: " + character.toString());
        telemetryM.debug("Alliance: " + alliance.toString());
        telemetryM.debug("Position: " + position.toString());
        telemetryM.debug("");

        telemetryM.debug("State: " + currentState);
        telemetryM.debug("Time: " + String.format("%.1f", autoTimer.seconds()));
        if (fallbackMode) {
            telemetryM.debug("FALLBACK MODE - Park Only");
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
}
