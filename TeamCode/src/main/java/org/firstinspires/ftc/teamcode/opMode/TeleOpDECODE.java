package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * TeleOp - Competition driver control
 * 
 * Features:
 * - Runtime robot/alliance/position selection (via BaseCompetitionOpMode)
 * - Field-relative and robot-relative drive modes
 * - Single-shot auto-aim (Y button)
 * - Toggle continuous tracking (Left Bumper GP2)
 * - Periodic relocalization using AprilTags
 * - LED feedback (if robot has LEDs)
 */
@TeleOp(name = "TeleOp - Competition", group = "Competition")
public class TeleOpDECODE extends BaseOpMode {
    
    // Auto-aim constants are now in Shooter class
    // Access via Shooter.AutoAimConstants.*
    
    // ==================== TRACKING STATE ====================
    
    private boolean trackingEnabled = false;
    private ElapsedTime trackingTimer = new ElapsedTime();
    private boolean singleShotMode = false;
    private ElapsedTime autoAimSpinDownTimer = new ElapsedTime();
    
    // ==================== BUTTON STATE ====================
    
    private boolean leftBumperLast = false;
    private boolean rightBumperLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean yButtonLast = false;
    private boolean bButtonLast = false;
    
    private ElapsedTime gamepadRateLimit = new ElapsedTime();
    private static final double RATE_LIMIT_MS = 300;
    
    private String lastAutoAimMessage = "";
    
    // ==================== START ====================
    
    @Override
    protected void onStart() {
        // Start TeleOp drive mode
        follower.startTeleopDrive();
        follower.update();
    }
    
    // ==================== MAIN LOOP ====================
    
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // === UPDATE SUBSYSTEMS ===
        shooter.periodic();
        ballFeed.periodic();
        
        // === PERIODIC RELOCALIZATION ===
        performRelocalization();
        
        // === GAMEPAD 1: DRIVING ===
        handleDriving();
        
        // === GAMEPAD 1: LED CONTROL (if available) ===
        if (led != null) {
            handleLEDControl();
        }
        
        // === GAMEPAD 2: SHOOTER AND BALL FEED ===
        handleShooterControls();
        handleBallFeedControls();
        
        // === TRACKING AUTO-DISABLE ===
        handleTrackingTimeout();

        // === UPDATE FOLLOWER ===
        follower.update();

        // === TELEMETRY ===
        displayTelemetry();
        if (follower != null) { draw(); }   // Guard draw() call - follower initialized in start(), not init()
    }
    
    // ==================== DRIVING CONTROL ====================
    
    private void handleDriving() {
        // Reset IMU if both triggers + A pressed
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5 && gamepad1.a) {
            imu.resetYaw();
        }
        
        // Check for driver override on rotation
        if (trackingEnabled && Math.abs(gamepad1.right_stick_x) > Shooter.AutoAimConstants.OVERRIDE_THRESHOLD) {
            trackingEnabled = false;
            singleShotMode = false;
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
        // Left bumper held = robot-relative mode
        boolean fieldRelative = !gamepad1.left_bumper;
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            rotationInput,
            fieldRelative
        );
    }
    
    /**
     * Calculate rotation power to track goal using P controller
     */
    private double calculateTrackingRotation() {
        if (!trackingEnabled) return 0;
        
        Vision.AutoAimResult result = vision.getAutoAimData();
        if (!result.success) {
            // Lost target - disable tracking
            trackingEnabled = false;
            singleShotMode = false;
            lastAutoAimMessage = "Tracking lost - no goal visible";
            return 0;
        }
        
        // Update shooter velocity while tracking
        shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
        autoAimSpinDownTimer.reset();
        
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
    }
    
    // ==================== LED CONTROL ====================
    
    private void handleLEDControl() {
        // Left Bumper: Green
        if (gamepad1.left_bumper && !leftBumperLast) {
            led.setGreen();
        }
        leftBumperLast = gamepad1.left_bumper;
        
        // Right Bumper: Purple
        if (gamepad1.right_bumper && !rightBumperLast) {
            led.setPurple();
        }
        rightBumperLast = gamepad1.right_bumper;
    }
    
    // ==================== SHOOTER CONTROL ====================
    
    private void handleShooterControls() {
        // DPAD Right: High velocity
        if (gamepad2.dpad_right && !dpadRightLast) {
            shooter.setHighVelocity();
            autoAimSpinDownTimer.reset();
        }
        dpadRightLast = gamepad2.dpad_right;
        
        // DPAD Left: Low velocity
        if (gamepad2.dpad_left && !dpadLeftLast) {
            shooter.setLowVelocity();
            autoAimSpinDownTimer.reset();
        }
        dpadLeftLast = gamepad2.dpad_left;
        
        // B button: Stop shooter AND tracking
        if (gamepad2.b && !bButtonLast) {
            shooter.stop();
            trackingEnabled = false;
            singleShotMode = false;
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
            autoAimSpinDownTimer.reset();
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
                autoAimSpinDownTimer.reset();
                lastAutoAimMessage = "TRACKING ENABLED - " + result.message;
            } else {
                lastAutoAimMessage = "Cannot track - " + result.message;
            }
        } else {
            // Disable tracking
            trackingEnabled = false;
            singleShotMode = false;
            lastAutoAimMessage = "Tracking disabled";
        }
    }
    
    /**
     * Handle single-shot tracking timeout and auto-aim spin-down
     */
    private void handleTrackingTimeout() {
        // Single-shot mode auto-disable after duration
        if (singleShotMode && trackingTimer.seconds() >= Shooter.AutoAimConstants.SINGLE_SHOT_DURATION) {
            trackingEnabled = false;
            singleShotMode = false;
            lastAutoAimMessage = "Single-shot complete";
        }
        
        // Auto-aim velocity spin-down after timeout
        if (shooter.isAutoAimActive() &&
            !trackingEnabled &&
            autoAimSpinDownTimer.seconds() >= Shooter.AutoAimConstants.AUTO_AIM_SPIN_DOWN_TIME) {
            shooter.stop();
        }
    }
    
    // ==================== BALL FEED CONTROL ====================
    
    private void handleBallFeedControls() {
        // Right Trigger: Feed ball
        if (gamepad2.right_trigger > 0.5 && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            ballFeed.startFeed();
            gamepadRateLimit.reset();
        }
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
        telemetryM.debug("Alliance: " + alliance.toString());
        telemetryM.debug("Position: " + position.toString());
        telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.1f°",
            follower.getPose().getX(),
            follower.getPose().getY(),
            Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");
        
        // === SHOOTER STATUS ===
        telemetryM.debug("=== SHOOTER ===");
        
        if (trackingEnabled) {
            telemetryM.debug("Mode: " + (singleShotMode ? "SINGLE-SHOT" : "CONTINUOUS") + " TRACKING");
            if (singleShotMode) {
                double remaining = Shooter.AutoAimConstants.SINGLE_SHOT_DURATION - trackingTimer.seconds();
                telemetryM.debug(String.format("Time Left: %.2fs", Math.max(0, remaining)));
            }
        } else if (shooter.isAutoAimActive()) {
            telemetryM.debug("Mode: AUTO-AIM (spinning down)");
            double remaining = Shooter.AutoAimConstants.AUTO_AIM_SPIN_DOWN_TIME - autoAimSpinDownTimer.seconds();
            telemetryM.debug(String.format("Spin-down: %.2fs", Math.max(0, remaining)));
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
        telemetryM.debug("=== CONTROLS ===");
        telemetryM.debug("GP1: Left Stick=Drive, Right Stick=Rotate");
        telemetryM.debug("GP1: LBump=Robot-Relative Mode");
        if (led != null) {
            telemetryM.debug("GP1: LBump=Green LED, RBump=Purple LED");
        }
        telemetryM.debug("GP2: Y=Single Auto-Aim, LBump=Toggle Track");
        telemetryM.debug("GP2: DPad L/R=Low/High RPM, RTrig=Feed");
        telemetryM.debug("GP2: B=Stop All");
        
        telemetryM.update(telemetry);
    }
    
    // ==================== STOP ====================
    
    @Override
    protected void onStop() {
        trackingEnabled = false;
        singleShotMode = false;
    }
}
