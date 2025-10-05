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
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode combines field-relative mecanum drive with AprilTag detection for positioning.
 *
 * CONTROLS:
 * - Left joystick: Drive direction (field-relative)
 * - Right joystick X: Rotate robot
 * - A button: Reset robot heading (yaw)
 * - Left bumper: Switch to robot-relative driving
 * - DPad Up: Resume camera streaming
 * - DPad Down: Stop camera streaming (save CPU)
 *
 * HARDWARE REQUIREMENTS:
 * - 4 mecanum wheels on motors named: fl, fr, bl, br
 * - IMU named: imu
 * - Webcam named: Webcam 1 (or phone camera if USE_WEBCAM = false)
 *
 * This was created by combining TeleopMecanumFieldRelativeDrive and AprilTagTeleOutOnly
 * for Wayland M.S. FTC Team 2025 DECODE season.
 */
@TeleOp(name = "Robot: Mecanum + AprilTag Field Relative", group = "Robot")
@Disabled  // Add this line to each OpMode
public class TeleOpMecanumRelativeAndAprilTags extends OpMode {

    // Drive system motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // IMU for field-relative driving
    IMU imu;

    // AprilTag vision system
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        // Set left motors in reverse for mecanum drive
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Use encoders for more accurate driving (remove if no encoder wires)
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU for field-relative driving
        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: Change these to match your Control Hub orientation on the robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Initialize AprilTag vision system
        initAprilTag();

        // Initialization complete
        telemetry.addData("Status", "Initialized - Ready to start!");
        telemetry.addData("Camera", USE_WEBCAM ? "Webcam" : "Phone Camera");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === DRIVE CONTROLS ===

        // Reset robot heading if A button pressed
        if (gamepad1.a) {
            imu.resetYaw();
            telemetry.addLine("*** YAW RESET ***");
        }

        // Drive field-relative or robot-relative based on left bumper
        if (gamepad1.left_bumper) {
            // Robot-relative driving (like RC car)
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Drive Mode", "ROBOT RELATIVE");
        } else {
            // Field-relative driving (joystick direction relative to field)
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Drive Mode", "FIELD RELATIVE");
        }

        // === CAMERA CONTROLS ===

        // Control camera streaming to save CPU resources
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
            telemetry.addData("Camera", "STREAMING STOPPED");
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
            telemetry.addData("Camera", "STREAMING RESUMED");
        }

        // === TELEMETRY OUTPUT ===

        // Drive system status
        telemetry.addData("Robot Heading", "%.1f degrees",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // AprilTag detection data
        telemetryAprilTag();

        // Control instructions
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left stick: Drive direction");
        telemetry.addLine("Right stick X: Rotate robot");
        telemetry.addLine("A: Reset heading");
        telemetry.addLine("Left bumper: Robot relative mode");
        telemetry.addLine("DPad Up/Down: Camera on/off");

        // Push all telemetry to Driver Station
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up camera resources when OpMode stops
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // === DRIVE METHODS ===

    /**
     * Drive the robot field-relative (joystick direction relative to field, not robot)
     */
    private void driveFieldRelative(double forward, double right, double rotate) {
        // Convert direction to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate angle by the robot's current heading
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Drive with the adjusted values
        drive(newForward, newRight, rotate);
    }

    /**
     * Drive the robot with mecanum wheel calculations
     * Thanks to FTC16072 for sharing this algorithm!
     */
    public void drive(double forward, double right, double rotate) {
        // Calculate power for each wheel based on movement components
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // Reduce this for slower driving (good for outreach/demos)

        // Normalize powers so none exceed 1.0 while maintaining proportions
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // Set motor powers
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    // === APRILTAG VISION METHODS ===

    /**
     * Initialize the AprilTag processor and vision portal
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                // Default settings work well for most cases
                // Uncomment and modify these if needed:
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set camera source
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Optional camera settings:
        //builder.setCameraResolution(new Size(640, 480));
        //builder.enableLiveView(true);
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        //builder.setAutoStopLiveView(false);

        // Add the AprilTag processor and build
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    /**
     * Add telemetry data about detected AprilTags
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addLine();
        telemetry.addData("=== APRILTAGS ===", "");
        telemetry.addData("# Tags Detected", currentDetections.size());

        // Display info for each detected tag
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Known tag with position data
                telemetry.addLine(String.format("\n==== TAG ID %d: %s ====",
                        detection.id, detection.metadata.name));
                telemetry.addLine(String.format("Position XYZ: %.1f, %.1f, %.1f (inches)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("Rotation PRY: %.1f, %.1f, %.1f (degrees)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("Range/Bearing: %.1f inches, %.1f degrees",
                        detection.ftcPose.range, detection.ftcPose.bearing));
            } else {
                // Unknown tag - just show basic info
                telemetry.addLine(String.format("\n==== TAG ID %d: Unknown ====", detection.id));
                telemetry.addLine(String.format("Center: %.0f, %.0f (pixels)",
                        detection.center.x, detection.center.y));
            }
        }

        // Add key for understanding the data
        if (currentDetections.size() > 0) {
            telemetry.addLine();
            telemetry.addLine("Key: X=Right, Y=Forward, Z=Up");
            telemetry.addLine("     P=Pitch, R=Roll, Y=Yaw rotation");
        }
    }
}