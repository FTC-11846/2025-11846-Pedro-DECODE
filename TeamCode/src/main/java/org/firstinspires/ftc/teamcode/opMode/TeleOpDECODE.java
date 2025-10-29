package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import android.annotation.SuppressLint;

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

        // Auto-aim timeout handling
        handleTrackingTimeout();

        // Display telemetry
        displayTelemetry();

        // Draw robot on dashboard (with safety check for pose validity)
        if (follower != null && follower.getPose() != null) {
            try {
                draw();
            } catch (Exception e) {
                // Silently catch drawing errors - non-critical for robot function
            }
        }    }

    // ==================== GP1: DRIVE CONTROLS ====================

    private void handleDriveControls() {
        // Reset IMU if both triggers + A pressed
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5 && gamepad1.a) {
            imu.resetYaw();
        }

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

        // Field-relative by default, robot-relative if holding left bumper
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
            trackingEnabled = false;
            singleShotMode = false;
            lastAutoAimMessage = "Tracking lost - no goal visible";
            return 0;
        }

        shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
        autoAimSpinDownTimer.reset();

        double headingError = result.bearingDegrees;

        if (Math.abs(headingError) < Shooter.autoAim.headingDeadbandDeg) {
            return 0;
        }

        double correction = headingError * Shooter.autoAim.headingPGain;
        correction = Math.max(-Shooter.autoAim.maxTrackingRotation,
                Math.min(Shooter.autoAim.maxTrackingRotation, correction));

        return correction;
    }

    // ==================== GP1: SHOOTER CONTROLS (MOVED FROM GP2!) ====================

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
        telemetryM.debug("Position: " + position.toString());
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