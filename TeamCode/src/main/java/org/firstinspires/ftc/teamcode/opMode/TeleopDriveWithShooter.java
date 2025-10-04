/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met: *
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 *  This was copied from external samples to the 2025-11846 repo's teamcode folder 2025-09-14
 */
@TeleOp(name = "Robot Relative P-Follower With Shooter", group = "Robot")
public class TeleopDriveWithShooter extends OpMode {
    // Shooter constants inner class
    @Configurable
    public static class ShooterConstants {
        public static double SHOOTER_LOW_VOLTAGE = 0.5;
        public static double SHOOTER_HIGH_VOLTAGE = 1.0;
        public static double INDEXER_RUNTIME_SECONDS = 0.25;
    }

    // Starting position constants inner class
    @Configurable
    public static class StartPoseConstants {
        // These constants define the starting positions for different alliance/field positions
        // Public so they're editable in dashboard
        public static Pose RED_NEAR = new Pose(56, 8, Math.toRadians(0));
        public static Pose RED_FAR = new Pose(56, 136, Math.toRadians(0));
        public static Pose BLUE_NEAR = new Pose(86, 8, Math.toRadians(180));
        public static Pose BLUE_FAR = new Pose(86, 136, Math.toRadians(180));

        // Private helpers for the selection menu - not editable
        private static final String[] POSITION_NAMES = {"Red Near", "Red Far", "Blue Near", "Blue Far"};
        private static final Pose[] POSITIONS = {
                StartPoseConstants.RED_NEAR,
                StartPoseConstants.RED_FAR,
                StartPoseConstants.BLUE_NEAR,
                StartPoseConstants.BLUE_FAR
        };
    }

    // This declares the motor to be used for the shooter
    DcMotorEx shooterMotor;

    // This declares the two servos used for indexing
    CRServo leftIndexServo;
    CRServo rightIndexServo;

    // Add these fields to the main class:
    private int selectedPosition = 0;
    private boolean positionSelected = false;
    private Pose startingPosition = StartPoseConstants.POSITIONS[0];

    private TelemetryManager telemetryM;

    ElapsedTime indexTimer = new ElapsedTime();
    ElapsedTime gamepadRateLimit = new ElapsedTime();
    private static final double RATE_LIMIT_MS = 300; // Prevent multiple presses within 300ms

    boolean indexOn, rightBumperPressedLast, leftBumperPressedLast, bPressedLast, checkTimer = false;
    double shooterVoltage = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        // Initialize Panels Telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        leftIndexServo = hardwareMap.get(CRServo.class, "feedServoL");
        rightIndexServo = hardwareMap.get(CRServo.class, "feedServoR");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIndexServo.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
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
        telemetryM.debug("Start Position", StartPoseConstants.POSITION_NAMES[selectedPosition]);
        // Run shooter logic first to apply power from last loop
        runShooterVoltage();

        // Handle Gamepad 1 (Driving)

        // Holdover for Field-Relative driving, LT+RT+a to rotate field 90-degrees,
        // could potentially mess up other things using localization, we should eventually delete this
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5 && gamepad1.a) {
            imu.resetYaw();
        }
        if (gamepad1.left_bumper) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        // Handle Gamepad 2 (Shooter and Indexer)
        if (gamepad2.right_bumper && !rightBumperPressedLast) {
            shooterVoltage = ShooterConstants.SHOOTER_HIGH_VOLTAGE;
        }
        rightBumperPressedLast = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !leftBumperPressedLast) {
            shooterVoltage = ShooterConstants.SHOOTER_LOW_VOLTAGE;
        }
        leftBumperPressedLast = gamepad2.left_bumper;

        if (gamepad2.b && !bPressedLast) {
            shooterVoltage = 0;
        }
        bPressedLast = gamepad2.b;


        if (gamepad2.x && gamepadRateLimit.milliseconds() > RATE_LIMIT_MS) {
            if (!indexOn) {
                index(1);
                indexOn = true;
                indexTimer.reset();
                checkTimer = true;
                gamepadRateLimit.reset(); // Reset rate limit timer
            }
        }

        // Timer cleanup (same as current)
        if (checkTimer && indexTimer.seconds() >= 0.25) {
            index(0);
            indexOn = false;
            checkTimer = false;
        }


        // Update follower and telemetry
        follower.update();

        telemetryM.debug("Mode", gamepad1.left_bumper ? "Robot Relative" : "Field Relative");
        telemetryM.debug("Shooter Target Voltage", shooterVoltage);
        telemetryM.debug("Shooter Actual Velocity (RPM)", shooterMotor.getVelocity(AngleUnit.DEGREES)/6);
        telemetryM.debug("Robot X", follower.getPose().getX());
        telemetryM.debug("Robot Y", follower.getPose().getY());
        telemetryM.debug("Robot Heading", follower.getPose().getHeading());
        telemetryM.update(telemetry);

        draw();

//        // In your loop() method:
//        FieldManager panelsField = PanelsField.INSTANCE.getField();
//        Style robotStyle = new Style("", "#3F51B5", 0.75);
//        panelsField.drawRobot(follower.getPose(), robotStyle);
//        panelsField.update();
    }

    @Override
    public void stop() {
        // Ensure motors are stopped when OpMode ends
        shooterMotor.setPower(0);
        index(0);
    }

//    public void runShooterVelocity(){
//        shooterMotor.setVelocity(shooterVelocity,AngleUnit.DEGREES);
//    }

    public void runShooterVoltage(){
        shooterMotor.setPower(shooterVoltage);
    }

    public void index(double power){
        leftIndexServo.setPower(power);
        rightIndexServo.setPower(power);
    }
}