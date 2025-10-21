package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BallFeed;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.MainCharacter;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * TeleOp with Commit 2 features:
 * - Runtime robot selection
 * - Single-shot auto-aim (Y button)
 * - Toggle continuous tracking (Left Bumper GP2)
 * - AprilTag loss tolerance with timeout
 *
 * Future: Localization pose fallback when tag lost (post-competition)
 */
@TeleOp(name = "TeleOp - Subsystems", group = "Competition")
public class TeleopDriveSubsystems extends OpMode {

    // ==================== SUBSYSTEMS ====================

    private Shooter shooter;
    private BallFeed ballFeed;
    private Vision vision;
    private LED led; // Only initialized if robot has LED hardware

    // ==================== CONFIGURATION ====================

    private MainCharacter character;
    private IMU imu;
    private TelemetryManager telemetryM;

    // ==================== STARTING POSITION ====================

    @Configurable
    public static class StartPoseConstants {
        public static Pose RED_NEAR = new Pose(56, 8, Math.toRadians(0));
        public static Pose RED_FAR = new Pose(56, 136, Math.toRadians(0));
        public static Pose BLUE_NEAR = new Pose(86, 8, Math.toRadians(180));
        public static Pose BLUE_FAR = new Pose(86, 136, Math.toRadians(180));

        private static final String[] POSITION_NAMES = {"Red Near", "Red Far", "Blue Near", "Blue Far"};
        private static final Pose[] POSITIONS = {RED_NEAR, RED_FAR, BLUE_NEAR, BLUE_FAR};
    }

    // ==================== SELECTION STATE ====================

    private int selectedRobotIndex = 0;
    private boolean robotSelected = false;

    private int selectedPosition = 0;
    private boolean positionSelected = false;

    // ==================== TRACKING STATE ====================

    private boolean trackingEnabled = false;
    private ElapsedTime trackingTimer = new ElapsedTime();
    private boolean singleShotMode = false;

    // AprilTag loss tolerance
    private ElapsedTime tagLossTimer = new ElapsedTime();
    private Vision.AutoAimResult lastValidResult = null;

    // ==================== BUTTON STATE ====================

    private boolean leftBumperLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean yButtonLast = false;
    private boolean bButtonLast = false;

    private ElapsedTime gamepadRateLimit = new ElapsedTime();
    private static final double RATE_LIMIT_MS = 300;

    private String lastAutoAimMessage = "";

    // ==================== INITIALIZATION ====================

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetryM.debug("=== INITIALIZATION ===");
        telemetryM.debug("Waiting for robot selection...");
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Stage 1: Robot Selection
        if (!robotSelected) {
            if (gamepad1.dpad_up && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                selectedRobotIndex = (selectedRobotIndex - 1 + MainCharacter.values().length)
                        % MainCharacter.values().length;
                gamepadRateLimit.reset();
            } else if (gamepad1.dpad_down && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                selectedRobotIndex = (selectedRobotIndex + 1) % MainCharacter.values().length;
                gamepadRateLimit.reset();
            }

            if (gamepad1.a) {
                character = MainCharacter.values()[selectedRobotIndex];
                MainCharacter.ACTIVE_ROBOT = character;
                robotSelected = true;

                // Initialize subsystems after robot selection
                initializeSubsystems();
            }

            telemetryM.debug("=== SELECT ROBOT ===");
            telemetryM.debug("Use DPAD UP/DOWN, Press A to confirm");
            telemetryM.debug("");
            for (int i = 0; i < MainCharacter.values().length; i++) {
                String marker = (i == selectedRobotIndex) ? " >>> " : "     ";
                telemetryM.debug(marker + MainCharacter.values()[i].toString());
            }
            telemetryM.update(telemetry);
            return;
        }

        // Stage 2: Position Selection
        if (!positionSelected) {
            if (gamepad1.dpad_up && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                selectedPosition = (selectedPosition - 1 + StartPoseConstants.POSITIONS.length)
                        % StartPoseConstants.POSITIONS.length;
                gamepadRateLimit.reset();
            } else if (gamepad1.dpad_down && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                selectedPosition = (selectedPosition + 1) % StartPoseConstants.POSITIONS.length;
                gamepadRateLimit.reset();
            }

            if (gamepad1.a) {
                Pose startPose = StartPoseConstants.POSITIONS[selectedPosition];
                positionSelected = true;
                follower.setPose(startPose);
            }

            telemetryM.debug("=== SELECT STARTING POSITION ===");
            telemetryM.debug("Robot: " + character.toString());
            telemetryM.debug("Use DPAD UP/DOWN, Press A to confirm");
            telemetryM.debug("");
            for (int i = 0; i < StartPoseConstants.POSITION_NAMES.length; i++) {
                String marker = (i == selectedPosition) ? " >>> " : "     ";
                telemetryM.debug(marker + StartPoseConstants.POSITION_NAMES[i]);
            }
            telemetryM.update(telemetry);
            return;
        }

        // Stage 3: Ready to start
        displayReadyMessage();
    }

    private void displayReadyMessage() {
        telemetryM.debug("=== READY TO START ===");
        telemetryM.debug("Robot: " + character.toString());
        telemetryM.debug("Position: " + StartPoseConstants.POSITION_NAMES[selectedPosition]);
        telemetryM.debug("");
        telemetryM.debug("Press START to begin TeleOp");
        telemetryM.update(telemetry);
    }

    private void initializeSubsystems() {
        shooter = new Shooter(hardwareMap, character);
        ballFeed = new BallFeed(hardwareMap, character);
        vision = new Vision(hardwareMap);

        // Only initialize LED if robot has the hardware
        if (character.hasLEDSystem()) {
            led = new LED(hardwareMap, character);
        } else {
            led = null;
        }
    }

    // ==================== START ====================

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();

        // Set coordinate system offsets
        com.bylazar.field.PanelsField.INSTANCE.getField()
                .setOffsets(com.bylazar.field.PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    // ==================== MAIN LOOP ====================

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // === UPDATE SUBSYSTEMS ===
        shooter.periodic();
        ballFeed.periodic();

        // === GAMEPAD 1: DRIVING ===
        handleDriving();

        // === GAMEPAD 2: SHOOTER AND BALL FEED ===
        handleShooterControls();
        handleBallFeedControls();

        // === TRACKING AUTO-DISABLE ===
        handleTrackingTimeout();

        // === UPDATE FOLLOWER ===
        follower.update();

        // === TELEMETRY ===
        displayTelemetry();
        draw();
    }

    // ==================== DRIVING ====================

    private void handleDriving() {
        // Reset IMU if both triggers + A pressed
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5 && gamepad1.a) {
            imu.resetYaw();
        }

        // Check for driver override on rotation
        if (trackingEnabled && Math.abs(gamepad1.right_stick_x) > Shooter.AutoAimConstants.OVERRIDE_THRESHOLD) {
            trackingEnabled = false;
            singleShotMode = false;
            lastValidResult = null;
            lastAutoAimMessage = "Tracking override - driver control";
        }

        // Calculate rotation input
        double rotationInput;
        if (trackingEnabled) {
            rotationInput = calculateTrackingRotation();
        } else {
            rotationInput = -gamepad1.right_stick_x;
        }

        // Field-relative vs robot-relative
        boolean fieldRelative = !gamepad1.left_bumper;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                rotationInput,
                fieldRelative
        );
    }

    // ==================== SHOOTER CONTROLS ====================

    private void handleShooterControls() {
        // DPAD Right: High velocity
        if (gamepad2.dpad_right && !dpadRightLast) {
            shooter.setHighVelocity();
        }
        dpadRightLast = gamepad2.dpad_right;

        // DPAD Left: Low velocity
        if (gamepad2.dpad_left && !dpadLeftLast) {
            shooter.setLowVelocity();
        }
        dpadLeftLast = gamepad2.dpad_left;

        // B button: Stop shooter AND tracking
        if (gamepad2.b && !bButtonLast) {
            shooter.stop();
            trackingEnabled = false;
            singleShotMode = false;
            lastValidResult = null;
        }
        bButtonLast = gamepad2.b;

        // Y button: Single-shot auto-aim (brief tracking burst)
        if (gamepad2.y && !yButtonLast) {
            performSingleShotAutoAim();
        }
        yButtonLast = gamepad2.y;

        // Left Bumper: Toggle continuous tracking
        if (gamepad2.left_bumper && !leftBumperLast) {
            toggleTracking();
        }
        leftBumperLast = gamepad2.left_bumper;
    }

    // ==================== BALL FEED CONTROLS ====================

    private void handleBallFeedControls() {
        // Check if robot has dual-independent ball feed
        if (character == MainCharacter.ROBOT_22154 || character == MainCharacter.ROBOT_11846) {
            // Dual-independent control
            double leftPower = gamepad2.left_trigger;
            double rightPower = gamepad2.right_trigger;

            if (leftPower > 0.1 || rightPower > 0.1) {
                // For 22154: CRServo - continuous feed while held
                // For 11846: Sweep servo - timed feed triggered by threshold
                ballFeed.setIndependentPowers(leftPower, rightPower);
            } else {
                ballFeed.stopFeed();
            }
        } else {
            // Single control for TestBot - Right trigger only, timed feed
            if (gamepad2.right_trigger > 0.5 && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                ballFeed.startFeed();
                gamepadRateLimit.reset();
            }
        }
    }
    // ==================== AUTO-AIM TRACKING ====================

    /**
     * Calculate rotation power to track goal using P controller
     * Features AprilTag loss tolerance - continues tracking with last known bearing
     * for TAG_LOSS_TIMEOUT seconds before giving up
     *
     * Future enhancement: Use localization pose to calculate bearing when tag lost
     */
    private double calculateTrackingRotation() {
        if (!trackingEnabled) return 0;

        Vision.AutoAimResult result = vision.getAutoAimData();

        if (result.success) {
            // Tag visible - reset timer and cache result
            tagLossTimer.reset();
            lastValidResult = result;

            // Update shooter velocity while tracking
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);

            // Calculate heading error (bearing is in degrees)
            double headingError = result.bearingDegrees;

            // Check if within deadband
            if (Math.abs(headingError) < Shooter.AutoAimConstants.HEADING_DEADBAND_DEG) {
                return 0; // Close enough, no correction needed
            }

            // P controller
            double correction = headingError * Shooter.AutoAimConstants.HEADING_P_GAIN;

            // Clamp to max rotation speed
            correction = Math.max(-Shooter.AutoAimConstants.MAX_TRACKING_ROTATION,
                    Math.min(Shooter.AutoAimConstants.MAX_TRACKING_ROTATION, correction));

            return correction;
        } else {
            // Tag lost - check timeout
            if (tagLossTimer.seconds() > Shooter.AutoAimConstants.TAG_LOSS_TIMEOUT) {
                // Lost for too long - disable tracking
                trackingEnabled = false;
                singleShotMode = false;
                lastAutoAimMessage = "Tracking lost - tag timeout";
                lastValidResult = null;
                return 0;
            } else {
                // Brief loss - continue with last known bearing
                if (lastValidResult != null) {
                    lastAutoAimMessage = String.format("Tag lost - using cached (%.1fs)",
                            tagLossTimer.seconds());

                    // TODO (post-competition): Calculate bearing from localization pose
                    // instead of using cached bearing. This will handle robot movement
                    // during tag loss more accurately.

                    // Use cached bearing for heading correction
                    double headingError = lastValidResult.bearingDegrees;

                    if (Math.abs(headingError) < Shooter.AutoAimConstants.HEADING_DEADBAND_DEG) {
                        return 0;
                    }

                    double correction = headingError * Shooter.AutoAimConstants.HEADING_P_GAIN;
                    correction = Math.max(-Shooter.AutoAimConstants.MAX_TRACKING_ROTATION,
                            Math.min(Shooter.AutoAimConstants.MAX_TRACKING_ROTATION, correction));

                    return correction;
                } else {
                    // No cached data - disable
                    trackingEnabled = false;
                    singleShotMode = false;
                    lastAutoAimMessage = "Tracking lost - no cached data";
                    return 0;
                }
            }
        }
    }

    /**
     * Single-shot auto-aim: Enable tracking for brief duration
     */
    private void performSingleShotAutoAim() {
        Vision.AutoAimResult result = vision.getAutoAimData();

        if (result.success) {
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
            trackingEnabled = true;
            singleShotMode = true;
            trackingTimer.reset();
            tagLossTimer.reset();
            lastValidResult = result;
            lastAutoAimMessage = result.message + " - Single-shot tracking";
        } else {
            lastAutoAimMessage = result.message;
        }
    }

    /**
     * Toggle continuous tracking mode
     */
    private void toggleTracking() {
        if (!trackingEnabled) {
            // Enable tracking
            Vision.AutoAimResult result = vision.getAutoAimData();
            if (result.success) {
                shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
                trackingEnabled = true;
                singleShotMode = false;
                tagLossTimer.reset();
                lastValidResult = result;
                lastAutoAimMessage = "TRACKING ENABLED - " + result.message;
            } else {
                lastAutoAimMessage = "Cannot track - " + result.message;
            }
        } else {
            // Disable tracking
            trackingEnabled = false;
            singleShotMode = false;
            lastValidResult = null;
            lastAutoAimMessage = "Tracking disabled";
        }
    }

    /**
     * Handle single-shot tracking timeout
     */
    private void handleTrackingTimeout() {
        // Single-shot mode auto-disable after duration
        if (singleShotMode && trackingTimer.seconds() >= Shooter.AutoAimConstants.SINGLE_SHOT_DURATION) {
            trackingEnabled = false;
            singleShotMode = false;
            lastValidResult = null;
            lastAutoAimMessage = "Single-shot complete";
        }

        // Auto-aim spindown deleted, driver will manually stop Shooter
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry() {
        List<AprilTagDetection> detections = vision.getDetections();

        // === APRILTAG VISION ===
        telemetryM.debug("=== APRILTAG VISION ===");
        telemetryM.debug("Camera: " + vision.getCameraState());
        telemetryM.debug("Tags: " + detections.size());

        if (!detections.isEmpty()) {
            for (AprilTagDetection d : detections) {
                if (vision.isValidDetection(d)) {
                    telemetryM.debug(String.format("  %s: %.1fin, %.1fdeg",
                            Vision.getTagFriendlyName(d.id),
                            d.ftcPose.range,
                            d.ftcPose.bearing));
                }
            }
        }
        telemetryM.debug("");

        // === ROBOT STATUS ===
        telemetryM.debug("=== ROBOT STATUS ===");
        telemetryM.debug("Robot: " + character.toString());
        telemetryM.debug("Position: " + StartPoseConstants.POSITION_NAMES[selectedPosition]);
        telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.1f",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");

        // === SHOOTER STATUS ===
        telemetryM.debug("=== SHOOTER ===");

        if (trackingEnabled) {
            telemetryM.debug("Mode: " + (singleShotMode ? "SINGLE-SHOT TRACKING" : "CONTINUOUS TRACKING"));
            if (singleShotMode) {
                double remaining = Shooter.AutoAimConstants.SINGLE_SHOT_DURATION - trackingTimer.seconds();
                telemetryM.debug(String.format("Time Left: %.2fs", Math.max(0, remaining)));
            }
        } else {
            telemetryM.debug("Mode: Manual");
        }

        telemetryM.debug(String.format("Target: %.0f RPM", shooter.getTargetVelocityRPM()));
        telemetryM.debug(String.format("Actual: %.0f RPM", shooter.getActualVelocityRPM()));

        if (shooter.isAutoAimActive()) {
            telemetryM.debug(String.format("Distance: %.1fin to Tag %d",
                    shooter.getLastDetectedDistance(),
                    shooter.getLastDetectedTagId()));
        }

        if (!lastAutoAimMessage.isEmpty()) {
            telemetryM.debug(lastAutoAimMessage);
        }
        telemetryM.debug("");

        // === BALL FEED STATUS ===
        telemetryM.debug("=== BALL FEED ===");
        telemetryM.debug("Feeding: " + (ballFeed.isFeeding() ? "YES" : "NO"));
        if (ballFeed.isFeeding()) {
            telemetryM.debug(String.format("Time Left: %.2fs", ballFeed.getRemainingFeedTime()));
        }
        telemetryM.debug("");

// === CONTROLS REMINDER ===
        telemetryM.debug("=== CONTROLS (GP2) ===");
        telemetryM.debug("Y: Single-shot auto-aim");
        telemetryM.debug("LBump: Toggle tracking");
        telemetryM.debug("DPad L/R: Low/High velocity");

// Ball feed controls vary by robot
        if (character == MainCharacter.ROBOT_22154 || character == MainCharacter.ROBOT_11846) {
            telemetryM.debug("LT/RT: Feed left/right");
        } else {
            telemetryM.debug("RTrig: Feed ball");
        }

        telemetryM.debug("B: Stop all");
        telemetryM.update(telemetry);
    }

    // ==================== SHUTDOWN ====================

    @Override
    public void stop() {
        shooter.emergencyStop();
        ballFeed.stopFeed();
        vision.close();
        trackingEnabled = false;
        singleShotMode = false;
        lastValidResult = null;
    }
}