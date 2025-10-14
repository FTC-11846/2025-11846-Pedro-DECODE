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
 * Main Character Energy TeleOp - Universal OpMode for all robots! 🌟
 * Uses subsystem architecture with runtime robot selection
 */
@TeleOp(name = "Main Character TeleOp", group = "Competition")
public class TeleopDriveSubsystems extends OpMode {

    // ==================== SUBSYSTEMS ====================

    private Shooter shooter;
    private BallFeed ballFeed;
    private Vision vision;
    private LED led; // null if robot doesn't have LEDs

    // Robot configuration - selected during init_loop
    private MainCharacter character;

    // Drive system
    private IMU imu;
    private TelemetryManager telemetryM;

    // ==================== STARTING POSITION SELECTION ====================

    @Configurable
    public static class StartPoseConstants {
        public static Pose RED_NEAR = new Pose(56, 8, Math.toRadians(0));
        public static Pose RED_FAR = new Pose(56, 136, Math.toRadians(0));
        public static Pose BLUE_NEAR = new Pose(86, 8, Math.toRadians(180));
        public static Pose BLUE_FAR = new Pose(86, 136, Math.toRadians(180));

        private static final String[] POSITION_NAMES = {
                "Red Near", "Red Far", "Blue Near", "Blue Far"
        };
        private static final Pose[] POSITIONS = {
                RED_NEAR, RED_FAR, BLUE_NEAR, BLUE_FAR
        };
    }

    // Selection state
    private enum InitState {
        SELECT_ROBOT,
        SELECT_POSITION,
        READY
    }

    private InitState initState = InitState.SELECT_ROBOT;
    private int selectedRobotIndex = 0;
    private int selectedPositionIndex = 0;
    private final MainCharacter[] robots = MainCharacter.values();

    // Button state tracking
    private boolean rightBumperLast = false;
    private boolean leftBumperLast = false;
    private boolean bButtonLast = false;
    private boolean yButtonLast = false;
    private boolean xButtonLast = false;
    private boolean aButtonLast = false;

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

        telemetryM.debug("=== MAIN CHARACTER ENERGY ===");
        telemetryM.debug("Ready for configuration!");
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        switch (initState) {
            case SELECT_ROBOT:
                handleRobotSelection();
                break;
            case SELECT_POSITION:
                handlePositionSelection();
                break;
            case READY:
                displayReadyMessage();
                break;
        }
    }

    private void handleRobotSelection() {
        // DPAD navigation
        if (gamepad1.dpad_up && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            selectedRobotIndex = (selectedRobotIndex - 1 + robots.length) % robots.length;
            gamepadRateLimit.reset();
        } else if (gamepad1.dpad_down && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            selectedRobotIndex = (selectedRobotIndex + 1) % robots.length;
            gamepadRateLimit.reset();
        }

        // A to confirm robot selection
        if (gamepad1.a && !aButtonLast) {
            character = robots[selectedRobotIndex];
            MainCharacter.ACTIVE_ROBOT = character; // Set global active robot

            // Initialize subsystems now that we know which robot
            initializeSubsystems();

            // Move to position selection
            initState = InitState.SELECT_POSITION;
            gamepadRateLimit.reset();
        }
        aButtonLast = gamepad1.a;

        // Display
        telemetryM.debug("=== SELECT YOUR ROBOT ===");
        telemetryM.debug("Use DPAD UP/DOWN, Press A to confirm");
        telemetryM.debug("");
        for (int i = 0; i < robots.length; i++) {
            String marker = (i == selectedRobotIndex) ? " >>> " : "     ";
            telemetryM.debug(marker + robots[i].toString());
        }
        telemetryM.update(telemetry);
    }

    private void handlePositionSelection() {
        // DPAD navigation
        if (gamepad1.dpad_up && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            selectedPositionIndex = (selectedPositionIndex - 1 + StartPoseConstants.POSITIONS.length)
                    % StartPoseConstants.POSITIONS.length;
            gamepadRateLimit.reset();
        } else if (gamepad1.dpad_down && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            selectedPositionIndex = (selectedPositionIndex + 1) % StartPoseConstants.POSITIONS.length;
            gamepadRateLimit.reset();
        }

        // A to confirm position
        if (gamepad1.a && !aButtonLast) {
            Pose startPose = StartPoseConstants.POSITIONS[selectedPositionIndex];
            follower.setPose(startPose);
            initState = InitState.READY;
            gamepadRateLimit.reset();
        }
        aButtonLast = gamepad1.a;

        // Display
        telemetryM.debug("=== SELECT STARTING POSITION ===");
        telemetryM.debug("Robot: " + character.toString());
        telemetryM.debug("Use DPAD UP/DOWN, Press A to confirm");
        telemetryM.debug("");
        for (int i = 0; i < StartPoseConstants.POSITION_NAMES.length; i++) {
            String marker = (i == selectedPositionIndex) ? " >>> " : "     ";
            telemetryM.debug(marker + StartPoseConstants.POSITION_NAMES[i]);
        }
        telemetryM.update(telemetry);
    }

    private void displayReadyMessage() {
        telemetryM.debug("=== READY TO START ===");
        telemetryM.debug("Robot: " + character.toString());
        telemetryM.debug("Position: " + StartPoseConstants.POSITION_NAMES[selectedPositionIndex]);
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

        // Set coordinate system offsets for field visualization
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

        // === GAMEPAD 1: DRIVING & LED ===
        handleDriving();
        handleLED();

        // === GAMEPAD 2: SHOOTER AND BALL FEED ===
        handleShooterControls();
        handleBallFeedControls();

        // === UPDATE FOLLOWER ===
        follower.update();

        // === TELEMETRY ===
        displayTelemetry();
        draw();
    }

    // ==================== CONTROL HANDLERS ====================

    private void handleDriving() {
        // Reset IMU: both triggers + A
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5 && gamepad1.a) {
            imu.resetYaw();
        }

        // Robot-relative when A is held, field-relative otherwise
        boolean fieldRelative = !gamepad1.a;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                fieldRelative
        );
    }

    private void handleLED() {
        if (led == null) return; // Skip if robot doesn't have LEDs

        // Left bumper: Green
        if (gamepad1.left_bumper && !leftBumperLast) {
            led.setGreen();
        }
        leftBumperLast = gamepad1.left_bumper;

        // Right bumper: Purple
        if (gamepad1.right_bumper && !rightBumperLast) {
            led.setPurple();
        }
        rightBumperLast = gamepad1.right_bumper;
    }

    private void handleShooterControls() {
        // Right bumper: High velocity
        if (gamepad2.right_bumper) {
            shooter.setHighVelocity();
        }

        // Left bumper: Low velocity
        if (gamepad2.left_bumper) {
            shooter.setLowVelocity();
        }

        // B button: Stop shooter
        if (gamepad2.b && !bButtonLast) {
            shooter.stop();
        }
        bButtonLast = gamepad2.b;

        // Y button: Auto-aim
        if (gamepad2.y && !yButtonLast) {
            performAutoAim();
        }
        yButtonLast = gamepad2.y;
    }

    private void handleBallFeedControls() {
        // X button: Feed ball
        if (gamepad2.x && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            ballFeed.startFeed();
            gamepadRateLimit.reset();
        }

        // A button: Reverse (unjam)
        if (gamepad2.a && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            ballFeed.startReverse();
            gamepadRateLimit.reset();
        }
    }

    private void performAutoAim() {
        Vision.AutoAimResult result = vision.getAutoAimData();

        if (result.success) {
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
            lastAutoAimMessage = result.message;
        } else {
            lastAutoAimMessage = result.message;
        }
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry() {
        List<AprilTagDetection> detections = vision.getDetections();

        // === ROBOT INFO ===
        telemetryM.debug("=== MAIN CHARACTER: " + character.toString() + " ===");
        telemetryM.debug("Position: " + StartPoseConstants.POSITION_NAMES[selectedPositionIndex]);
        telemetryM.debug("Drive Mode: " + (gamepad1.a ? "Robot-Relative" : "Field-Relative"));
        telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");

        // === APRILTAG VISION ===
        telemetryM.debug("=== VISION ===");
        telemetryM.debug("Camera: " + vision.getCameraState());
        telemetryM.debug("Tags: " + detections.size());

        if (!detections.isEmpty()) {
            for (AprilTagDetection d : detections) {
                telemetryM.debug(String.format("  %s: %.1f in, %.1f°",
                        Vision.getTagFriendlyName(d.id),
                        d.ftcPose.range,
                        d.ftcPose.bearing));
            }
        }
        telemetryM.debug("");

        // === SHOOTER STATUS ===
        telemetryM.debug("=== SHOOTER ===");
        telemetryM.debug("Mode: " + (shooter.isAutoAimActive() ? "AUTO-AIM" : "Manual"));
        telemetryM.debug(String.format("Target: %.0f RPM", shooter.getTargetVelocityRPM()));
        telemetryM.debug(String.format("Actual: %.0f RPM", shooter.getActualVelocityRPM()));

        if (shooter.isAutoAimActive()) {
            telemetryM.debug(String.format("Auto-Aim: %.1f in to Tag %d",
                    shooter.getLastDetectedDistance(),
                    shooter.getLastDetectedTagId()));
        }

        if (!lastAutoAimMessage.isEmpty()) {
            telemetryM.debug(lastAutoAimMessage);
        }
        telemetryM.debug("");

        // === BALL FEED STATUS ===
        telemetryM.debug("=== BALL FEED ===");
        telemetryM.debug("Mode: " + ballFeed.getMode());
        telemetryM.debug("Active: " + (ballFeed.isFeeding() ? "YES" : "NO"));
        if (ballFeed.isFeeding()) {
            telemetryM.debug(String.format("Time Left: %.2fs", ballFeed.getRemainingFeedTime()));
        }
        telemetryM.debug("");

        // === LED STATUS (if equipped) ===
        if (led != null) {
            telemetryM.debug("=== LED ===");
            telemetryM.debug("Left Bumper: Green");
            telemetryM.debug("Right Bumper: Purple");
        }

        telemetryM.update(telemetry);
    }

    // ==================== STOP ====================

    @Override
    public void stop() {
        shooter.emergencyStop();
        ballFeed.stopFeed();
        vision.close();
        if (led != null) {
            led.turnOff();
        }
    }
}