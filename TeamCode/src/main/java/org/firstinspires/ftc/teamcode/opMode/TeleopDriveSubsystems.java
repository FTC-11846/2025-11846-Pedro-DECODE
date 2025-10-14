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
import org.firstinspires.ftc.teamcode.subsystems.BallFeedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MainCharacter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Refactored TeleOp using subsystem architecture
 * Works with all robots via RobotConfig
 */
@TeleOp(name = "TeleOp - Subsystems", group = "Competition")
public class TeleopDriveSubsystems extends OpMode {

    // Subsystems
    private ShooterSubsystem shooter;
    private BallFeedSubsystem indexer;
    private VisionSubsystem vision;
    
    // Robot configuration
    private MainCharacter config;
    
    // Drive system
    private IMU imu;
    private TelemetryManager telemetryM;

    // Starting position selection
    @Configurable
    public static class StartPoseConstants {
        public static Pose RED_NEAR = new Pose(56, 8, Math.toRadians(0));
        public static Pose RED_FAR = new Pose(56, 136, Math.toRadians(0));
        public static Pose BLUE_NEAR = new Pose(86, 8, Math.toRadians(180));
        public static Pose BLUE_FAR = new Pose(86, 136, Math.toRadians(180));

        private static final String[] POSITION_NAMES = {"Red Near", "Red Far", "Blue Near", "Blue Far"};
        private static final Pose[] POSITIONS = {RED_NEAR, RED_FAR, BLUE_NEAR, BLUE_FAR};
    }

    private int selectedPosition = 0;
    private boolean positionSelected = false;
    
    // Button state tracking
    private boolean rightBumperLast = false;
    private boolean leftBumperLast = false;
    private boolean bButtonLast = false;
    private boolean yButtonLast = false;
    private boolean xButtonLast = false;

    private ElapsedTime gamepadRateLimit = new ElapsedTime();
    private static final double RATE_LIMIT_MS = 300;

    private String lastAutoAimMessage = "";

    @Override
    public void init() {
        // Set which robot we're using - CHANGE THIS FOR EACH ROBOT
        config = MainCharacter.ACTIVE_ROBOT; // or hardcode: RobotConfig.ROBOT_22154
        
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Initialize subsystems
        shooter = new ShooterSubsystem(hardwareMap, config);
        indexer = new BallFeedSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        telemetryM.debug("Robot: " + config.name());
        telemetryM.debug("Dual Shooters: " + config.hasDualShooters());
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (!positionSelected) {
            // Position selection logic
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
            telemetryM.debug("Use DPAD UP/DOWN, Press A to confirm");
            telemetryM.debug("");
            for (int i = 0; i < StartPoseConstants.POSITION_NAMES.length; i++) {
                String marker = (i == selectedPosition) ? " >>> " : "     ";
                telemetryM.debug(marker + StartPoseConstants.POSITION_NAMES[i]);
            }
            telemetryM.update(telemetry);
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
        
        // Set coordinate system offsets
        com.bylazar.field.PanelsField.INSTANCE.getField()
            .setOffsets(com.bylazar.field.PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // === UPDATE SUBSYSTEMS ===
        shooter.periodic();
        indexer.periodic();
        
        // === GAMEPAD 1: DRIVING ===
        handleDriving();
        
        // === GAMEPAD 2: SHOOTER AND INDEXER ===
        handleShooterControls();
        handleIndexerControls();
        
        // === UPDATE FOLLOWER ===
        follower.update();
        
        // === TELEMETRY ===
        displayTelemetry();
        draw();
    }

    private void handleDriving() {
        // Reset IMU if both triggers + A pressed
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5 && gamepad1.a) {
            imu.resetYaw();
        }
        
        // Field-relative vs robot-relative
        boolean fieldRelative = !gamepad1.left_bumper;
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x,
            fieldRelative
        );
    }

    private void handleShooterControls() {
        // Right bumper: High velocity
        if (gamepad2.right_bumper && !rightBumperLast) {
            shooter.setHighVelocity();
        }
        rightBumperLast = gamepad2.right_bumper;

        // Left bumper: Low velocity
        if (gamepad2.left_bumper && !leftBumperLast) {
            shooter.setLowVelocity();
        }
        leftBumperLast = gamepad2.left_bumper;

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

    private void handleIndexerControls() {
        // X button: Feed ball
        if (gamepad2.x && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            indexer.startFeed();
            gamepadRateLimit.reset();
        }
    }

    private void performAutoAim() {
        VisionSubsystem.AutoAimResult result = vision.getAutoAimData();
        
        if (result.success) {
            shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
            lastAutoAimMessage = result.message;
        } else {
            lastAutoAimMessage = result.message;
        }
    }

    private void displayTelemetry() {
        List<AprilTagDetection> detections = vision.getDetections();
        
        // === APRILTAG VISION ===
        telemetryM.debug("=== APRILTAG VISION ===");
        telemetryM.debug("Camera: " + vision.getCameraState());
        telemetryM.debug("Tags: " + detections.size());

        if (!detections.isEmpty()) {
            for (AprilTagDetection d : detections) {
                telemetryM.debug(String.format("  %s: %.1fin, %.1fdeg",
                    VisionSubsystem.getTagFriendlyName(d.id),
                    d.ftcPose.range,
                    d.ftcPose.bearing));
            }
        }
        telemetryM.debug("");

        // === ROBOT STATUS ===
        telemetryM.debug("=== ROBOT STATUS ===");
        telemetryM.debug("Robot: " + config.name());
        telemetryM.debug("Position: " + StartPoseConstants.POSITION_NAMES[selectedPosition]);
        telemetryM.debug(String.format("Pose: X=%.1f Y=%.1f H=%.1f",
            follower.getPose().getX(),
            follower.getPose().getY(),
            Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");

        // === SHOOTER STATUS ===
        telemetryM.debug("=== SHOOTER ===");
        telemetryM.debug("Mode: " + (shooter.isAutoAimActive() ? "AUTO-AIM" : "Manual"));
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

        // === INDEXER STATUS ===
        telemetryM.debug("=== INDEXER ===");
        telemetryM.debug("Feeding: " + (indexer.isFeeding() ? "YES" : "NO"));
        if (indexer.isFeeding()) {
            telemetryM.debug(String.format("Time Left: %.2fs", indexer.getRemainingFeedTime()));
        }

        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.emergencyStop();
        indexer.stopFeed();
        vision.close();
    }
}
