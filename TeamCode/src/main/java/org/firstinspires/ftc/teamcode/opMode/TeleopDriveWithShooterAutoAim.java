/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode adds AprilTag-based auto-aim functionality to the robot's shooter.
 * Press Y (gamepad2) to automatically detect the goal AprilTag and set shooter power
 * based on distance using a tunable ballistic equation.
 *
 * Ballistic Equation: power = baseline + (distance * linearFactor)
 * - Clamps output between minPower and maxPower
 * - All constants are tunable via dashboard
 */
@TeleOp(name = "Robot Relative P-Follower With Auto-Aim", group = "Robot")
public class TeleopDriveWithShooterAutoAim extends OpMode {

    // Shooter constants inner class
    @Configurable
    public static class ShooterConstants {
        public static double SHOOTER_LOW_VOLTAGE = 0.5;
        public static double SHOOTER_HIGH_VOLTAGE = 1.0;
        public static double INDEXER_RUNTIME_SECONDS = 0.25;

        // Auto-aim ballistic equation parameters
        // Formula: power = BASELINE_POWER + (distance_in_inches * LINEAR_CORRECTION_FACTOR)
        // Based on observed range: 0.5 @ 42in (3.5ft), 0.85 @ 180in (15ft)
        public static double BASELINE_POWER = 0.395;           // Extrapolated starting power
        public static double LINEAR_CORRECTION_FACTOR = 0.0025; // Power increase per inch
        public static double MIN_POWER = 0.5;                  // Minimum shooter power
        public static double MAX_POWER = 0.85;                 // Maximum shooter power
    }

    // Starting position constants inner class
    @Configurable
    public static class StartPoseConstants {
        public static Pose RED_NEAR = new Pose(56, 8, Math.toRadians(0));
        public static Pose RED_FAR = new Pose(56, 136, Math.toRadians(0));
        public static Pose BLUE_NEAR = new Pose(86, 8, Math.toRadians(180));
        public static Pose BLUE_FAR = new Pose(86, 136, Math.toRadians(180));

        private static final String[] POSITION_NAMES = {"Red Near", "Red Far", "Blue Near", "Blue Far"};
        private static final Pose[] POSITIONS = {
                StartPoseConstants.RED_NEAR,
                StartPoseConstants.RED_FAR,
                StartPoseConstants.BLUE_NEAR,
                StartPoseConstants.BLUE_FAR
        };
    }

    // AprilTag IDs for goals
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final int MOTIF_GPP_TAG_ID = 21;
    private static final int MOTIF_PGP_TAG_ID = 22;
    private static final int MOTIF_PPG_TAG_ID = 23;
    private static final boolean USE_WEBCAM = true;

    // Hardware declarations
    DcMotorEx shooterMotor;
    CRServo leftIndexServo;
    CRServo rightIndexServo;
    IMU imu;

    // AprilTag vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Position selection
    private int selectedPosition = 0;
    private boolean positionSelected = false;
    private Pose startingPosition = StartPoseConstants.POSITIONS[0];

    private TelemetryManager telemetryM;

    // Timers and state
    ElapsedTime indexTimer = new ElapsedTime();
    ElapsedTime gamepadRateLimit = new ElapsedTime();
    private static final double RATE_LIMIT_MS = 300;

    boolean indexOn, rightBumperPressedLast, leftBumperPressedLast, bPressedLast, yPressedLast, checkTimer = false;
    double shooterVoltage = 0;

    // Auto-aim state
    private boolean autoAimActive = false;
    private double lastDetectedDistance = 0;
    private int lastDetectedTagId = -1;
    private String lastAutoAimMessage = "";

    @Override
    public void init() {
        // Initialize Panels Telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Initialize shooter hardware
        shooterMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        leftIndexServo = hardwareMap.get(CRServo.class, "feedServoL");
        rightIndexServo = hardwareMap.get(CRServo.class, "feedServoR");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIndexServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Initialize AprilTag detection
        initAprilTag();
    }

    @Override
    public void init_loop() {
        if (!positionSelected) {
            if (gamepad1.dpad_up && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                selectedPosition = (selectedPosition + 1) % StartPoseConstants.POSITIONS.length;
                gamepadRateLimit.reset();
            } else if (gamepad1.dpad_down && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
                selectedPosition = (selectedPosition - 1 + StartPoseConstants.POSITIONS.length) % StartPoseConstants.POSITIONS.length;
                gamepadRateLimit.reset();
            }

            if (gamepad1.a) {
                startingPosition = StartPoseConstants.POSITIONS[selectedPosition];
                positionSelected = true;
                follower.setPose(startingPosition);
            }

            telemetryM.debug("=== SELECT STARTING POSITION ===");
            telemetryM.debug("Use DPAD UP/DOWN to select, Press A to confirm");
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
    }

    @Override
    public void loop() {
        // Run shooter logic first to apply power from last loop
        runShooterVoltage();

        // Handle Gamepad 1 (Driving)
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5 && gamepad1.a) {
            imu.resetYaw();
        }
        if (gamepad1.left_bumper) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        // Handle Gamepad 2 (Shooter and Indexer)

        // Manual high power
        if (gamepad2.right_bumper && !rightBumperPressedLast) {
            shooterVoltage = ShooterConstants.SHOOTER_HIGH_VOLTAGE;
            autoAimActive = false;
        }
        rightBumperPressedLast = gamepad2.right_bumper;

        // Manual low power
        if (gamepad2.left_bumper && !leftBumperPressedLast) {
            shooterVoltage = ShooterConstants.SHOOTER_LOW_VOLTAGE;
            autoAimActive = false;
        }
        leftBumperPressedLast = gamepad2.left_bumper;

        // Stop shooter
        if (gamepad2.b && !bPressedLast) {
            shooterVoltage = 0;
            autoAimActive = false;
        }
        bPressedLast = gamepad2.b;

        // AUTO-AIM: Y button triggers AprilTag-based power calculation
        if (gamepad2.y && !yPressedLast) {
            detectGoalAndSetPower();
        }
        yPressedLast = gamepad2.y;

        // Indexer control
        if (gamepad2.x && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            if (!indexOn) {
                index(1);
                indexOn = true;
                indexTimer.reset();
                checkTimer = true;
                gamepadRateLimit.reset();
            }
        }

        // Timer cleanup
        if (checkTimer && indexTimer.seconds() >= ShooterConstants.INDEXER_RUNTIME_SECONDS) {
            index(0);
            indexOn = false;
            checkTimer = false;
        }

        // Update follower
        follower.update();

        // === APRILTAG STATUS (runs every loop) ===
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetryM.debug("=== APRILTAG VISION ===");
        telemetryM.debug("Camera State: " + visionPortal.getCameraState());
        telemetryM.debug("Tags Detected: " + currentDetections.size());

        if (currentDetections.size() > 0) {
            for (AprilTagDetection detection : currentDetections) {
                String tagName = getTagFriendlyName(detection.id);
                telemetryM.debug("  " + tagName + " (ID " + detection.id + ")");
                telemetryM.debug("    Distance: " + String.format("%.1f", detection.ftcPose.range) + " in");
                telemetryM.debug("    Bearing: " + String.format("%.1f", detection.ftcPose.bearing) + " deg");
            }
        } else {
            telemetryM.debug("  No tags visible");
        }
        telemetryM.debug("");

        // === ROBOT STATUS ===
        telemetryM.debug("=== ROBOT STATUS ===");
        telemetryM.debug("Start Position: " + StartPoseConstants.POSITION_NAMES[selectedPosition]);
        telemetryM.debug("Drive Mode: " + (gamepad1.left_bumper ? "Robot Relative" : "Field Relative"));
        telemetryM.debug("Robot X: " + String.format("%.1f", follower.getPose().getX()));
        telemetryM.debug("Robot Y: " + String.format("%.1f", follower.getPose().getY()));
        telemetryM.debug("Robot Heading: " + String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");

        // === SHOOTER STATUS ===
        telemetryM.debug("=== SHOOTER STATUS ===");
        telemetryM.debug("Mode: " + (autoAimActive ? "AUTO-AIM" : "Manual"));
        telemetryM.debug("Target Power: " + String.format("%.2f", shooterVoltage));
        telemetryM.debug("Actual RPM: " + String.format("%.0f", shooterMotor.getVelocity(AngleUnit.DEGREES)/6));

        if (autoAimActive) {
            telemetryM.debug("Auto-Aim Target: " + getTagFriendlyName(lastDetectedTagId));
            telemetryM.debug("Distance: " + String.format("%.1f", lastDetectedDistance) + " in");
        }

        if (!lastAutoAimMessage.isEmpty()) {
            telemetryM.debug("Last Auto-Aim: " + lastAutoAimMessage);
        }

        telemetryM.update(telemetry);
        draw();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
        index(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Get friendly name for AprilTag ID
     */
    private String getTagFriendlyName(int tagId) {
        switch (tagId) {
            case BLUE_GOAL_TAG_ID:
                return "Blue Goal";
            case RED_GOAL_TAG_ID:
                return "Red Goal";
            case MOTIF_GPP_TAG_ID:
                return "Motif GPP";
            case MOTIF_PGP_TAG_ID:
                return "Motif PGP";
            case MOTIF_PPG_TAG_ID:
                return "Motif PPG";
            default:
                return "Unknown Tag";
        }
    }

    /**
     * Initialize AprilTag detection system
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /**
     * Detect goal AprilTag and calculate shooter power based on distance
     * Uses tunable ballistic equation: power = baseline + (distance * linearFactor)
     */
    private void detectGoalAndSetPower() {
        lastAutoAimMessage = ""; // Clear previous message

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection goalTag = null;

        telemetryM.debug("Y PRESSED - Searching for goal tags...");

        // Search for goal AprilTags (ID 20 for Blue, ID 24 for Red)
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetryM.debug("  Found tag ID: " + detection.id);
                if (detection.id == BLUE_GOAL_TAG_ID || detection.id == RED_GOAL_TAG_ID) {
                    goalTag = detection;
                    telemetryM.debug("  -> This is a GOAL tag!");
                    break;
                }
            }
        }

        if (goalTag != null) {
            // Get distance to goal in inches
            double distanceInches = goalTag.ftcPose.range;
            lastDetectedDistance = distanceInches;
            lastDetectedTagId = goalTag.id;

            // Calculate power using ballistic equation
            double calculatedPower = ShooterConstants.BASELINE_POWER +
                    (distanceInches * ShooterConstants.LINEAR_CORRECTION_FACTOR);

            // Clamp to safe min/max values
            calculatedPower = Math.max(ShooterConstants.MIN_POWER,
                    Math.min(ShooterConstants.MAX_POWER, calculatedPower));

            shooterVoltage = calculatedPower;
            autoAimActive = true;

            lastAutoAimMessage = "SUCCESS - " + getTagFriendlyName(goalTag.id) +
                    " @ " + String.format("%.1f", distanceInches) + "in" +
                    " -> Power " + String.format("%.2f", calculatedPower);

            telemetryM.debug("AUTO-AIM SUCCESS!");
            telemetryM.debug("  Target: " + getTagFriendlyName(goalTag.id));
            telemetryM.debug("  Distance: " + String.format("%.1f", distanceInches) + " inches");
            telemetryM.debug("  Calculated Power: " + String.format("%.2f", calculatedPower));
        } else {
            lastAutoAimMessage = "FAILED - No goal tag detected (" + currentDetections.size() + " tags visible)";
            autoAimActive = false;

            telemetryM.debug("AUTO-AIM FAILED - No goal AprilTag found");
            telemetryM.debug("  Total tags visible: " + currentDetections.size());
        }
    }

    /**
     * Apply shooter voltage
     */
    public void runShooterVoltage() {
        shooterMotor.setPower(shooterVoltage);
    }

    /**
     * Control indexer servos
     */
    public void index(double power) {
        leftIndexServo.setPower(power);
        rightIndexServo.setPower(power);
    }
}