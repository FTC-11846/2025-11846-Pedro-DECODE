package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import android.annotation.SuppressLint;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

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
    private BallColor frontLeftLaneColor = BallColor.NONE;
    private BallColor frontRightLaneColor = BallColor.NONE;
    private BallColor backLeftLaneColor = BallColor.NONE;
    private BallColor backRightLaneColor = BallColor.NONE;

    // ==================== POSES FOR TELEOP ====================

    private Pose redIntakePose = new Pose(18,9,180);
    private Pose blueIntakePose = new Pose(126,9,0);
    private Pose nearLoadingZoneShootPose = new Pose(72, 24, 62);
    private Pose nearGoalShootPose = new Pose(72, 72, 51);

    public PathChain pathToRedIntake;
    public PathChain pathToBlueIntake;
    public PathChain pathToNearLoadingZoneShoot;
    public PathChain pathToNearGoalShoot;

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
        if (colorSensors != null){ colorSensors.periodic(); }

        // Handle all controls
        handleDriveControls();          // GP1 drive + rotation
        handleShooterControls();        // GP1 shooter (MOVED FROM GP2!)
        handleIntakeControls();         // GP2 intake toggle
        handleBallFeedControls();       // GP2 independent L/R feed
        handleEndgameControls();
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

// ==================== CONFIGURABLE PRECISION MODE ====================
// Add this to BaseOpMode or create a new DriveConfig class

    public static DriveConfig driveConfig = new DriveConfig();

    public static class DriveConfig {
        public double precisionPowerMultiplier = 0.2;  // Reduce to 20% when held
    }

// ==================== UPDATED handleDriveControls() ====================

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

        // ✨ PRECISION MODE: Reduce power when GP1 Right Bumper held
        double powerMultiplier = 1.0;
        if (gamepad1.right_bumper) {
            powerMultiplier = driveConfig.precisionPowerMultiplier;
        }

        // Apply precision scaling and send to follower
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * powerMultiplier,  // Forward/back scaled
                -gamepad1.left_stick_x * powerMultiplier,  // Strafe scaled
                rotationInput * powerMultiplier,            // Rotation scaled
                true  // Field-relative
        );
    }

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
            Pose shootingTarget = vision.getShootingTarget(result.tagId);

            if (shootingTarget == null) {
                return 0;  // Safety check
            }

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
            int goalTagId = alliance.isBlue() ? Vision.config.blueGoalTagId : Vision.config.redGoalTagId;
            Pose shootingTarget = vision.getShootingTarget(goalTagId);

            if (shootingTarget == null) {
                return 0;  // Safety check
            }

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
            shooter.setAutoAimVelocity(distance, goalTagId);

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
     * This function is the entry point to AutoAim w/ AutoTracking
     * Recently enhanced (Oct-29) to fully rely on localization for activation
     * or whenever goalTag is lost
     */
    private void performSingleShotAutoAim() {
        Vision.AutoAimResult result = vision.getAutoAimData();

        if (result.success) {
            // Tag visible - use direct measurement
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
            lastAutoAimMessage = result.message + " - Tracking";
        } else {
            // No tag - calculate from pose to shooting target
            Pose robotPose = follower.getPose();
            int goalTagId = alliance.isBlue() ? Vision.config.blueGoalTagId : Vision.config.redGoalTagId;
            Pose shootingTarget = vision.getShootingTarget(goalTagId);

            if (shootingTarget != null) {
                double dx = shootingTarget.getX() - robotPose.getX();
                double dy = shootingTarget.getY() - robotPose.getY();
                double distance = Math.hypot(dx, dy);

                shooter.setAutoAimVelocity(distance, goalTagId);
                lastAutoAimMessage = String.format("Pose-based (%.1f in)", distance);
            } else {
                lastAutoAimMessage = "FAILED - No shooting target available";
                return;  // Don't enable tracking if we can't aim
            }
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
            /// Currently 11846 wants new bottom intake control fused with One
            if (intake.getMode() == CharacterStats.IntakeMode.DUAL_STAGE_SERVO_BOTTOM) {
                intake.toggleIntakeTwo();
            }
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
            led.showLeftLaneColor(frontLeftLaneColor, backLeftLaneColor);
            led.showRightLaneColor(frontRightLaneColor, backRightLaneColor);
        }
    }

    // ==================== GP1: ENDGAME SYSTEM ====================

    private void handleEndgameControls(){
        if (endgame.getRobotName() == "11846") {
            if(gamepad1.left_trigger >= 0.75){
                endgame.runEndgameFunction();
            } else if(gamepad1.right_trigger >= 0.75){
                endgame.reverseEndgameFunction();
            } else {
                endgame.hold();
            }
        } else {
            if(gamepad1.right_trigger > 0.75 && gamepad1.left_trigger > 0.75){
                // Run both functions; the correct one will set the motor speeds inside the endgame system
                endgame.runEndgameFunction();
                //} else if(gamepad1.right_bumper){
                //    endgame.reverseEndgameFunction();
            }
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

    /** Temporary DEBUG Telemetry for Field Positions
     *
     */
    /**
     * DEBUG: Display all field element poses in both coordinate systems
     * This will help diagnose if tag positions are being converted correctly
     */
    private void displayFieldElementPosesDebug() {
        telemetryM.debug("=== COORDINATE DEBUG ===");

        AprilTagLibrary library = AprilTagGameDatabase.getCurrentGameTagLibrary();

// ===== BLUE GOAL TAG (20) =====
        AprilTagMetadata blueGoalMeta = library.lookupTag(Vision.config.blueGoalTagId);
        if (blueGoalMeta != null) {
            double ftcX = blueGoalMeta.fieldPosition.get(0);
            double ftcY = blueGoalMeta.fieldPosition.get(1);
            double ftcZ = blueGoalMeta.fieldPosition.get(2);

            // Convert Quaternion to Orientation to extract angles
            Orientation orientation = blueGoalMeta.fieldOrientation.toOrientation(
                    AxesReference.EXTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            );
            double ftcRoll = orientation.firstAngle;   // X rotation
            double ftcPitch = orientation.secondAngle; // Y rotation
            double ftcYaw = orientation.thirdAngle;    // Z rotation

            telemetryM.debug(String.format("BlueGoal(FTC): X=%.1f Y=%.1f Z=%.1f", ftcX, ftcY, ftcZ));
            telemetryM.debug(String.format("               Yaw=%.1f Pitch=%.1f Roll=%.1f",
                    ftcYaw, ftcPitch, ftcRoll));

            Pose tagPedro = vision.getGoalTagPose(Vision.config.blueGoalTagId);
            if (tagPedro != null) {
                telemetryM.debug(String.format("BlueGoal(PP):  X=%.1f Y=%.1f H=%.1f",
                        tagPedro.getX(), tagPedro.getY(), Math.toDegrees(tagPedro.getHeading())));
            }

            Pose shootPedro = vision.getShootingTarget(Vision.config.blueGoalTagId);
            if (shootPedro != null) {
                telemetryM.debug(String.format("BlueShoot(PP): X=%.1f Y=%.1f H=%.1f",
                        shootPedro.getX(), shootPedro.getY(), Math.toDegrees(shootPedro.getHeading())));
            }
        }

        telemetryM.debug("");

// ===== RED GOAL TAG (24) =====
        AprilTagMetadata redGoalMeta = library.lookupTag(Vision.config.redGoalTagId);
        if (redGoalMeta != null) {
            double ftcX = redGoalMeta.fieldPosition.get(0);
            double ftcY = redGoalMeta.fieldPosition.get(1);
            double ftcZ = redGoalMeta.fieldPosition.get(2);

            // Convert Quaternion to Orientation to extract angles
            Orientation orientation = redGoalMeta.fieldOrientation.toOrientation(
                    AxesReference.EXTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            );
            double ftcRoll = orientation.firstAngle;
            double ftcPitch = orientation.secondAngle;
            double ftcYaw = orientation.thirdAngle;

            telemetryM.debug(String.format("RedGoal(FTC):  X=%.1f Y=%.1f Z=%.1f", ftcX, ftcY, ftcZ));
            telemetryM.debug(String.format("               Yaw=%.1f Pitch=%.1f Roll=%.1f",
                    ftcYaw, ftcPitch, ftcRoll));

            Pose tagPedro = vision.getGoalTagPose(Vision.config.redGoalTagId);
            if (tagPedro != null) {
                telemetryM.debug(String.format("RedGoal(PP):   X=%.1f Y=%.1f H=%.1f",
                        tagPedro.getX(), tagPedro.getY(), Math.toDegrees(tagPedro.getHeading())));
            }

            Pose shootPedro = vision.getShootingTarget(Vision.config.redGoalTagId);
            if (shootPedro != null) {
                telemetryM.debug(String.format("RedShoot(PP):  X=%.1f Y=%.1f H=%.1f",
                        shootPedro.getX(), shootPedro.getY(), Math.toDegrees(shootPedro.getHeading())));
            }
        }
        telemetryM.debug("");
        telemetryM.debug("=== ROBOT STATE ===");

        Pose robotPose = follower.getPose();
        telemetryM.debug(String.format("Robot(PP):     X=%.1f Y=%.1f H=%.1f",
                robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading())));

        // Show starting pose for reference
        telemetryM.debug(String.format("StartPose(PP): %s = X=%.1f Y=%.1f H=%.1f",
                alliance + " " + startPoseInSelectMenu,
                actualStartingPose.getX(),
                actualStartingPose.getY(),
                Math.toDegrees(actualStartingPose.getHeading())));

        // Vision comparison
        List<AprilTagDetection> detections = vision.getDetections();
        for (AprilTagDetection d : detections) {
            if (vision.isValidDetection(d)) {
                Pose visionPose = vision.getRobotPoseFromTag(d);
                if (visionPose != null) {
                    double error = Math.hypot(
                            visionPose.getX() - robotPose.getX(),
                            visionPose.getY() - robotPose.getY()
                    );
                    telemetryM.debug(String.format("Vision(PP):    X=%.1f Y=%.1f H=%.1f (err: %.1f)",
                            visionPose.getX(), visionPose.getY(),
                            Math.toDegrees(visionPose.getHeading()), error));
                    telemetryM.debug(String.format("  Tag %d: range=%.1f bearing=%.1f",
                            d.id, d.ftcPose.range, d.ftcPose.bearing));

                    // Show FTC robot pose from detection
                    telemetryM.debug(String.format("  RobotFTC: X=%.1f Y=%.1f Z=%.1f Yaw=%.1f",
                            d.robotPose.getPosition().x,
                            d.robotPose.getPosition().y,
                            d.robotPose.getPosition().z,
                            d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            }
        }

        telemetryM.debug("");
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry() {
     //   displayFieldElementPosesDebug();  /// Field element Pose DEBUG, mute or delete!

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
            } else if (intake.getMode() == CharacterStats.IntakeMode.DUAL_STAGE_SERVO_BOTTOM) {
                telemetryM.debug("Bottom/Two: " + (intake.isIntakeTwoRunning() ? "ON" : "OFF"));
            }
            telemetryM.debug("");
        }

        // Add precision mode indicator
        if (gamepad1.right_bumper) {
            telemetryM.debug(String.format("🎯 PRECISION MODE (%.0f%% power)",
                    driveConfig.precisionPowerMultiplier * 100));
        }

        // === BALL FEED ===
        telemetryM.debug("=== BALL FEED ===");
        telemetryM.debug("Left: " + ballFeed.getLeftStateString());
        telemetryM.debug("Right: " + ballFeed.getRightStateString());
        telemetryM.debug("");

        // === BALL DETECTION ===
        if (colorSensors != null) {
            telemetryM.debug("=== BALL DETECTION ===");
            telemetryM.debug("Front Left Lane Color: " + ColorSensors.flBallColor);
            telemetryM.debug("Front Right Lane Color: " + frontRightLaneColor);
            telemetryM.debug("Back Left Lane Color: " + backLeftLaneColor);
            telemetryM.debug("Back Right Lane Color: " + backRightLaneColor);
            telemetryM.debug("");
            telemetryM.debug("Front Left Raw R Color: " + ColorSensors.flRGBValueR);
            telemetryM.debug("Front Left Raw G Color: " + ColorSensors.flRGBValueG);
            telemetryM.debug("Front Left Raw B Color: " + ColorSensors.flRGBValueB);
            telemetryM.debug("Front Right Raw Color: " + ColorSensors.frRGBValue);
            telemetryM.debug("Back Left Raw Color: " + ColorSensors.blRGBValue);
            telemetryM.debug("Back Right Raw Color: " + ColorSensors.brRGBValue);
            telemetryM.debug("");
            telemetryM.debug("Ball Count: " + ColorSensors.ballCount);
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