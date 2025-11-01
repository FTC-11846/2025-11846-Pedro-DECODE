package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import android.annotation.SuppressLint;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

/**
 * TeleOp - Competition driver control
 *
 * NEW CONTROL SCHEME (Bundle 4):
 * - GP1: Drive + Shooter controls (auto-aim on LB, velocity on DPad)
 * - GP2: Intake (DPad Up/Down) + Independent Feed (L/R Triggers)
 *
 * Features:
 * - Runtime robot/alliance/position selection (via BaseOpMode)
 * - Field-relative drive (hold LB for robot-relative)
 * - Single-shot auto-aim (GP1 Left Bumper)
 * - Manual velocity presets (GP1 DPad L/R)
 * - Independent intake control (GP2 DPad - toggle on release)
 * - Independent L/R feed lanes (GP2 Triggers)
 * - Automatic color detection + LED feedback
 * - Periodic relocalization using AprilTags
 */
@TeleOp(name = "TeleOp - Competition", group = "Competition")
public class TeleOpDECODE extends BaseOpMode {

    // ==================== STATE TRACKING ====================

    private boolean trackingEnabled = false;
    private ElapsedTime trackingTimer = new ElapsedTime();
    private boolean singleShotMode = false;
    private ElapsedTime autoAimSpinDownTimer = new ElapsedTime();

    private String lastAutoAimMessage = "";

    // Color sensor state for telemetry
    private ColorSensors.BallColor frontLeftLaneColor = ColorSensors.BallColor.NONE;
    private ColorSensors.BallColor frontRightLaneColor = ColorSensors.BallColor.NONE;
    private ColorSensors.BallColor backLeftLaneColor = ColorSensors.BallColor.NONE;
    private ColorSensors.BallColor backRightLaneColor = ColorSensors.BallColor.NONE;


    // ==================== BUTTON STATE TRACKING ====================

    // GP1 buttons
    private boolean gp1_leftBumperLast = false;
    private boolean gp1_rightBumperLast = false;
    private boolean gp1_dpadLeftLast = false;
    private boolean gp1_dpadRightLast = false;
    private boolean gp1_bButtonLast = false;

    // GP2 buttons
    private boolean gp2_dpadUpLast = false;
    private boolean gp2_dpadDownLast = false;
    private boolean gp2_bButtonLast = false;

    private ElapsedTime gamepadRateLimit = new ElapsedTime();
    private static final double RATE_LIMIT_MS = 300;

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
        // Update follower first
        follower.update();

        // Call relocalization automatically (rate-limited internally)
        relocalize();

        // Update subsystems
        shooter.periodic();
        ballFeed.periodic();

        // Handle all controls
        handleDriveControls();          // GP1 drive + rotation
        handleShooterControls();        // GP1 shooter (MOVED FROM GP2!)
        handleIntakeControls();         // GP2 intake toggle
        handleBallFeedControls();       // GP2 independent L/R feed
        handleColorSensorDisplay();     // Automatic color detection
        handleEmergencyStop();          // GP1+GP2 B button

//        // Auto-aim timeout handling
//        handleTrackingTimeout();  @Deprecated, should be deleted when the method is

        // Display telemetry
        displayTelemetry();

        // Draw robot on dashboard (with safety check for pose validity)
        if (follower != null && follower.getPose() != null) {
            try {
                draw();
            } catch (Exception e) {
                // Silently catch drawing errors - non-critical for robot function
            }
        }

        /**
         * BaseOpMode in the Dashboard now has a True/False field to trigger a Panels Refresh
         */
        if (panelsControl.triggerRefresh) {
            panelsControl.triggerRefresh = false;  // Reset BEFORE refresh
            PanelsConfigurables.INSTANCE.refreshClass(this);
        }
    }

    // ==================== GP1: DRIVE CONTROLS ====================

    private void handleDriveControls() {

        // Check for driver override on rotation during tracking
        if (trackingEnabled && Math.abs(gamepad1.right_stick_x) > Shooter.autoAim.overrideThreshold) {
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


        /**
         * Field-relative (traditional) drive control!!!  (all robot-centric stripped)
         * NOTE: the really stupid inlay hint for the 4th param is Opposite! We must
         * set it "true" for field-relative driving!
         */
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, rotationInput, true);
    }

    /**
     * Calculate rotation power to track goal using P controller
     */
    /**
     * Calculate rotation power to track goal using P controller
     * Uses AprilTag when visible, falls back to localization pose
     */
    private double calculateTrackingRotation() {
        if (!trackingEnabled) return 0;

        Vision.AutoAimResult result = vision.getAutoAimData();

        double headingError;

        if (result.success) {
            // ====== VISION-BASED AUTO-AIM ======
            // Tag is visible - calculate bearing to SHOOTING TARGET (not tag)

            Pose robotPose = follower.getPose();
            Pose shootingTarget = Vision.goalPositions.getShootingTarget(result.tagId);

            // Calculate bearing to shooting target
            double dx = shootingTarget.getX() - robotPose.getX();
            double dy = shootingTarget.getY() - robotPose.getY();
            double bearingToTarget = Math.atan2(dy, dx);
            double currentHeading = robotPose.getHeading();

            // Calculate heading error
            double errorRad = bearingToTarget - currentHeading;
            while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
            while (errorRad < -Math.PI) errorRad += 2 * Math.PI;
            headingError = Math.toDegrees(errorRad);

            // Use vision-measured distance to TAG for shooter velocity
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
            autoAimSpinDownTimer.reset();

            lastAutoAimMessage = String.format("Vision track (%.1f in, %.1f°)",
                    result.distanceInches, headingError);

        } else {
            // ====== LOCALIZATION-BASED AUTO-AIM ======
            // Tag lost - calculate from pose to SHOOTING TARGET

            Pose robotPose = follower.getPose();
            Pose shootingTarget = Vision.goalPositions.getShootingTarget(
                    alliance.isBlue() ? Vision.tagIds.blueGoalTagId : Vision.tagIds.redGoalTagId
            );

            double dx = shootingTarget.getX() - robotPose.getX();
            double dy = shootingTarget.getY() - robotPose.getY();

            double bearingToTarget = Math.atan2(dy, dx);
            double currentHeading = robotPose.getHeading();

            // Calculate heading error
            double errorRad = bearingToTarget - currentHeading;
            while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
            while (errorRad < -Math.PI) errorRad += 2 * Math.PI;
            headingError = Math.toDegrees(errorRad);

            // Calculate distance for shooter velocity
            double distance = Math.hypot(dx, dy);
            shooter.setAutoAimVelocity(distance,
                    alliance.isBlue() ? Vision.tagIds.blueGoalTagId : Vision.tagIds.redGoalTagId);

            lastAutoAimMessage = String.format("Pose track (%.1f in, %.1f°)",
                    distance, headingError);
        }

        // Deadband
        if (Math.abs(headingError) < Shooter.autoAim.headingDeadbandDeg) {
            return 0;
        }

        // P controller
        return headingError * Shooter.autoAim.headingPGain;
    }

    /**
     * Get goal pose based on current alliance
     */
    private Pose getGoalPose() {
        if (alliance.isBlue()) {
            return new Pose(Vision.goalPositions.blueGoalX,
                    Vision.goalPositions.blueGoalY,
                    Math.toRadians(180));
        } else {
            return new Pose(Vision.goalPositions.redGoalX,
                    Vision.goalPositions.redGoalY,
                    0);
        }
    }

    // ==================== GP1: SHOOTER Motor CONTROLS (MOVED FROM GP2!) ====================

    private void handleShooterControls() {
        // Auto-aim (GP1 Left Bumper) - single-shot mode
        if (gamepad1.left_bumper && !gp1_leftBumperLast) {
            performSingleShotAutoAim();
        }
        gp1_leftBumperLast = gamepad1.left_bumper;

        // Low velocity preset (GP1 DPad Left)
        if (gamepad1.dpad_left && !gp1_dpadLeftLast) {
            shooter.setLowVelocity();
            autoAimSpinDownTimer.reset();
        }
        gp1_dpadLeftLast = gamepad1.dpad_left;

        // High velocity preset (GP1 DPad Right)
        if (gamepad1.dpad_right && !gp1_dpadRightLast) {
            shooter.setHighVelocity();
            autoAimSpinDownTimer.reset();
        }
        gp1_dpadRightLast = gamepad1.dpad_right;
    }

    /**
     *  This function is the entry point to AutoAim w/ AutoTracking
     *  Recently enhanced (Oct-29) to fully rely on localization for activation
     *  or whenever goalTag is lost
     */
    private void performSingleShotAutoAim() {
        Vision.AutoAimResult result = vision.getAutoAimData();

        if (result.success) {
            // Tag visible - use direct measurement
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
            lastAutoAimMessage = result.message + " - Tracking";
        } else {
            // No tag - calculate from pose
            Pose robotPose = follower.getPose();
            Pose goalPose = getGoalPose();

            double dx = goalPose.getX() - robotPose.getX();
            double dy = goalPose.getY() - robotPose.getY();
            double distance = Math.hypot(dx, dy);

            shooter.setAutoAimVelocity(distance,
                    alliance.isBlue() ? Vision.tagIds.blueGoalTagId : Vision.tagIds.redGoalTagId);

            lastAutoAimMessage = String.format("Pose-based (%.1f in)", distance);
        }

        // Always enable (whether tag visible or not)
        trackingEnabled = true;
        singleShotMode = true;
        trackingTimer.reset();
        autoAimSpinDownTimer.reset();
    }

    /**
     * Drive/Design teams don't want Auto-aim/AutoTracking to timeout, so its deprecated for now
     * Delete this method in the next couple days, once this choice is validated on robot! Oct-29
     */
    @Deprecated
    private void handleTrackingTimeout() {
        // Disable single-shot tracking after duration
        if (singleShotMode && trackingTimer.seconds() >= Shooter.autoAim.singleShotDuration) {
            trackingEnabled = false;
            singleShotMode = false;
            lastAutoAimMessage = "Single-shot complete";
        }

        // Spin down shooter after auto-aim timeout
        if (shooter.isAutoAimActive() && !trackingEnabled &&
                autoAimSpinDownTimer.seconds() >= Shooter.autoAim.autoAimSpinDownTime) {
            shooter.stop();
        }
    }

    // ==================== GP2: INTAKE CONTROLS ====================

    private void handleIntakeControls() {
        if (intake == null) return;  // Robot doesn't have intake

        // Intake One toggle (GP2 DPad Up)
        // Use falling edge detection (button release) to prevent double-toggle
        if (!gamepad2.dpad_up && gp2_dpadUpLast) {
            intake.toggleIntakeOne();
        }
        gp2_dpadUpLast = gamepad2.dpad_up;

        // Intake Two toggle (GP2 DPad Down) - only for dual-independent mode
        if (!gamepad2.dpad_down && gp2_dpadDownLast) {
            if (intake.getMode() == CharacterStats.IntakeMode.DUAL_INDEPENDENT_TOGGLE) {
                intake.toggleIntakeTwo();
            }
        }
        gp2_dpadDownLast = gamepad2.dpad_down;
    }

    // ==================== GP2: INDEPENDENT BALL FEED CONTROLS ====================

    private void handleBallFeedControls() {
        if(ballFeed.getMode() == CharacterStats.BallFeedMode.SINGLE_CRSERVO){
            // Feed single lane on any trigger
            if (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
                if (gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                    ballFeed.feedLeft();  // Single motor, single lane!
                    gamepadRateLimit.reset();
                }
            }
        } else {
            // Independent L/R control for dual motor robots
            if (gamepad2.left_trigger > 0.5 && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                ballFeed.feedLeft();
                gamepadRateLimit.reset();
            }
            if (gamepad2.right_trigger > 0.5 && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                ballFeed.feedRight();
                gamepadRateLimit.reset();
            }
        }
    }

    // ==================== COLOR SENSOR + LED INTEGRATION ====================

    private void handleColorSensorDisplay() {
        if (colorSensors == null) return;  // Robot doesn't have color sensors

        // Detect colors continuously
        frontLeftLaneColor = colorSensors.detectFrontLeftLane();
        frontRightLaneColor = colorSensors.detectFrontRightLane();
        backLeftLaneColor = colorSensors.detectBackLeftLane();
        backRightLaneColor = colorSensors.detectBackRightLane();

        // Update LEDs automatically if robot has them
        if (led != null) {
            //TODO Change function to take into account both color sensors on each side
            led.showLeftLaneColor(backLeftLaneColor);
            led.showRightLaneColor(backRightLaneColor);
        }
    }

    // ==================== EMERGENCY STOP ====================

    private void handleEmergencyStop() {
        // B button on either gamepad stops everything
        if (gamepad1.b || gamepad2.b) {
            shooter.emergencyStop();
            ballFeed.stopFeed();
            if (intake != null) {
                intake.stopAll();
            }
            trackingEnabled = false;
            singleShotMode = false;
            lastAutoAimMessage = "EMERGENCY STOP";
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
        telemetryM.debug("Position: " + (startPoseInSelectMenu != null ? startPoseInSelectMenu.toString() : "From Auto"));
        telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");

        // === SHOOTER ===
        telemetryM.debug("=== SHOOTER ===");

        if (trackingEnabled) {
            telemetryM.debug("Mode: " + (singleShotMode ? "SINGLE-SHOT" : "CONTINUOUS") + " TRACKING");
            if (singleShotMode) {
                double remaining = Shooter.autoAim.singleShotDuration - trackingTimer.seconds();
                telemetryM.debug(String.format("Time Left: %.2fs", Math.max(0, remaining)));
            }
        } else if (shooter.isAutoAimActive()) {
            telemetryM.debug("Mode: AUTO-AIM (spinning down)");
            double remaining = Shooter.autoAim.autoAimSpinDownTime - autoAimSpinDownTimer.seconds();
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

///        === RELOCALIZATION STATS ===
        displayRelocalizationStats();  /// <-- the FUTURE of Tele! modular blocks included by a single function call!

        // === INTAKE STATUS ===
        if (intake != null) {
            telemetryM.debug("=== INTAKE ===");
            telemetryM.debug("Front/One: " + (intake.isIntakeOneRunning() ? "ON" : "OFF"));
            if (intake.getMode() == CharacterStats.IntakeMode.DUAL_INDEPENDENT_TOGGLE) {
                telemetryM.debug("Back/Two: " + (intake.isIntakeTwoRunning() ? "ON" : "OFF"));
            }
            telemetryM.debug("");
        }

        // === BALL FEED ===
        telemetryM.debug("=== BALL FEED ===");
        telemetryM.debug("Left: " + ballFeed.getLeftStateString());
        telemetryM.debug("Right: " + ballFeed.getRightStateString());
        telemetryM.debug("");

        // === BALL DETECTION ===
        if (colorSensors != null) {
            telemetryM.debug("=== BALL DETECTION ===");
            telemetryM.debug("Front Left Lane: " + frontLeftLaneColor);
            telemetryM.debug("Front Right Lane: " + frontRightLaneColor);
            telemetryM.debug("Back Left Lane: " + backLeftLaneColor);
            telemetryM.debug("Back Right Lane: " + backRightLaneColor);
            telemetryM.debug("");
        }

        // === CONTROLS GUIDE ===
        telemetryM.debug("=== CONTROLS ===");
        telemetryM.debug("GP1: Left Stick=Drive, Right Stick=Rotate");
        telemetryM.debug("GP1: LBump=Auto-Aim, DPad L/R=Low/High RPM");
        telemetryM.debug("GP2: DPad Up=Intake1, DPad Down=Intake2");
        telemetryM.debug("GP2: LT=Feed Left, RT=Feed Right");
        telemetryM.debug("GP1/GP2: B=Stop All");

        telemetryM.update(telemetry);
    }

    @Override
    protected void onStop() {
        trackingEnabled = false;
        singleShotMode = false;

        if (intake != null) {
            intake.stopAll();
        }
    }
}