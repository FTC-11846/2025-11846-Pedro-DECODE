package org.firstinspires.ftc.teamcode.OpModes; // Or your preferred package

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory; // If using Panels drawing
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;         // If using PanelsTelemetry

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Assuming you have this from Pedro Pathing setup
import com.bylazar.telemetry.PanelsTelemetry; // If using PanelsTelemetry


@TeleOp(name = "Pedro: Field Relative Mecanum With Shooter", group = "PedroPathing")
public class TeleopPedroFieldRelativeWithShooter extends OpMode {

    // Pedro Pathing Follower
    public static Follower follower; // Made static if you access it from Tuning or a shared context
    // If not, it can be non-static.

    // Hardware from TeleopMecanumFieldRelativeDriveWithShooter
    DcMotorEx shooterMotor;
    CRServo leftIndexServo;
    CRServo rightIndexServo;

    ElapsedTime indexTimer = new ElapsedTime();
    boolean indexOn = false;
    boolean checkTimer = false; // Renamed from TeleopMecanumFieldRelativeDriveWithShooter for clarity
    double shooterVelocitySetting = 0; // Renamed for clarity, was shooterVelocity

    // IMU - Pedro Pathing's follower might manage its own IMU instance or expect one to be passed.
    // For now, we'll manage it here for field-relative calculations directly.
    // If your `Constants.createFollower` handles IMU initialization for the follower,
    // you might get it from follower.getIMU() or similar.
    IMU imu;

    // Variables for button presses (to avoid repeated actions)
    boolean yPressedLastFrame_gp2 = false;
    boolean bPressedLastFrame_gp2 = false;
    boolean rightBumperPressedLastFrame_gp2 = false;
    boolean leftBumperPressedLastFrame_gp2 = false;


    @Override
    public void init() {
        // Initialize Follower (assuming Constants.createFollower exists and is configured)
        // If follower is meant to be shared via a static context like in Tuning.java,
        // you might initialize it there or ensure it's not null here.
        // For a standalone OpMode, initializing here is common.
        if (follower == null) { // Basic null check
            follower = Constants.createFollower(hardwareMap);
        }
        follower.setStartingPose(new Pose(0, 0, 0)); // Set your desired starting pose

        // Initialize Telemetry Manager (if using Panels)
        if (telemetryM == null) { // Basic null check
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        }

        // Hardware Initialization
        shooterMotor = hardwareMap.get(DcMotorEx.class, "sm");
        leftIndexServo = hardwareMap.get(CRServo.class, "lis");
        rightIndexServo = hardwareMap.get(CRServo.class, "ris");

        // Motor and Servo Directions
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIndexServo.setDirection(DcMotorSimple.Direction.REVERSE);
        // Assuming drive motor directions are handled within Constants.createFollower()
        // or the Follower's initialization. If not, set them here.
        // e.g., follower.setMotorDirections(...);

        // IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT; // CHANGE AS NEEDED
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;    // CHANGE AS NEEDED
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Note: Pedro Pathing's Follower might also initialize/use its own IMU.
        // Ensure this doesn't conflict or that one source of heading is consistently used.
        // If `follower` uses its own IMU internally, you might not need this `imu` instance here
        // and would get heading from `follower.getPose().getHeading()`.
        // However, for direct field-relative calculations before passing to follower,
        // using this `imu` instance is fine.

        telemetryM.debug("OpMode Initialized. Ready to Start.");
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {// if (PanelsTelemetry.INSTANCE.getField() != null) { // Check if field drawing is available
        drawCurrentAndHistory(); // Or just drawCurrent()
        // }
        follower.update(); // Update pose during init_loop
        telemetryM.debug("Robot Pose: " + follower.getPose().toString());
        telemetryM.debug("Press Start to begin TeleOp.");
        telemetryM.update(telemetry);
        drawCurrentAndHistory(); // Or just drawCurrent()
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true); // Enable teleop drive mode in the follower
        follower.update();
        indexTimer.reset();
        imu.resetYaw(); // Reset IMU heading at the start of teleop
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // --- Drive Control ---
        double yInput = -gamepad1.left_stick_y;  // Forward/Backward
        double xInput = -gamepad1.left_stick_x;  // Strafe Left/Right (Note: Pedro might expect positive X as right)
        double rxInput = -gamepad1.right_stick_x; // Rotation

        // IMU Reset
        if (gamepad1.a) {
            imu.resetYaw();
            // Optionally, if follower maintains its own heading state separate from direct IMU reads:
            // follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        double driveYPower;
        double driveXPower;

        if (gamepad1.left_bumper) { // Robot Relative Drive
            driveYPower = yInput;
            driveXPower = xInput;
        } else { // Field Relative Drive
            // Get current robot heading from IMU in radians
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the joystick inputs by the negative of the robot's heading
            double rotatedX = xInput * Math.cos(-robotHeading) - yInput * Math.sin(-robotHeading);
            double rotatedY = xInput * Math.sin(-robotHeading) + yInput * Math.cos(-robotHeading);

            driveXPower = rotatedX;
            driveYPower = rotatedY;
        }

        // Send desired velocities/powers to the follower
        // The follower converts these into individual motor commands.
        // The 'true' for holonomic usually means it's a mecanum/swerve drive.
        follower.setTeleOpDrive(driveYPower, driveXPower, rxInput, true);

        // --- Shooter and Indexer Logic (from TeleopMecanumFieldRelativeDriveWithShooter) ---
        runShooter(); // Combined shooter power and indexing logic

        // --- Update Follower ---
        follower.update(); // THIS IS CRUCIAL: updates odometry and robot pose

        // --- Telemetry ---
        telemetryM.addData("Mode", gamepad1.left_bumper ? "Robot Relative" : "Field Relative");
        telemetryM.addData("X Pose", String.format("%.2f", follower.getPose().getX()));
        telemetryM.addData("Y Pose", String.format("%.2f", follower.getPose().getY()));
        telemetryM.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.addData("IMU Yaw (deg)", String.format("%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));

        telemetryM.addData("Shooter Set Velocity", shooterVelocitySetting);
        telemetryM.addData("Shooter Actual Velocity (RPM)", String.format("%.2f", shooterMotor.getVelocity(AngleUnit.DEGREES) / 6.0));
        telemetryM.addData("Indexer On", indexOn);
        telemetryM.addData("Indexer Timer", String.format("%.2f",indexTimer.seconds()));

        // Add regular FTC Telemetry if not using Panels exclusively
        telemetry.addData("X Pose", "%.2f", follower.getPose().getX());
        telemetry.addData("Y Pose", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        drawCurrentAndHistory();
    }

    // Combined and adapted shooter/indexer methods
    private void runShooter() {
        // Shooter velocity adjustment
        if (gamepad2.right_bumper && !rightBumperPressedLastFrame_gp2) {
            shooterVelocitySetting = 2000; // Example: Far shot RPM (adjust as needed)
            // Note: DcMotorEx.setVelocity() takes ticks per second.
            // You'll need to convert RPM to ticks/sec.
            // ticksPerSec = RPM * TicksPerRevolution / 60
        }
        rightBumperPressedLastFrame_gp2 = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !leftBumperPressedLastFrame_gp2) {
            shooterVelocitySetting = 1700; // Example: Close shot RPM (adjust as needed)
        }
        leftBumperPressedLastFrame_gp2 = gamepad2.left_bumper;

        // Example: Toggle shooter on/off with gamepad2.y (or set power directly)
        // This is a simplified version; you might want more sophisticated state management.
        if (gamepad2.y && !yPressedLastFrame_gp2) {
            if (shooterMotor.getPower() > 0) {
                shooterMotor.setVelocity(0); // Or setPower(0)
            } else {
                // Convert shooterVelocitySetting (RPM) to Ticks Per Second for setVelocity
                // This depends on your motor's TPR (Ticks Per Revolution)
                double motorTPR = shooterMotor.getMotorType().getTicksPerRev();
                double targetTicksPerSecond = shooterVelocitySetting * motorTPR / 60.0;
                shooterMotor.setVelocity(targetTicksPerSecond);
            }
        }
        yPressedLastFrame_gp2 = gamepad2.y;

        if(gamepad2.b && !bPressedLastFrame_gp2){ // Emergency stop for shooter
            shooterMotor.setVelocity(0); // or shooterMotor.setPower(0);
            shooterVelocitySetting = 0;
        }
        bPressedLastFrame_gp2 = gamepad2.b;


        // Indexer logic
        if (gamepad2.x) { // Using 'x' directly, assuming it's a "hold to index" or a toggle needed
            if (!indexOn) { // If not already on, turn it on and reset timer
                index(1.0); // Run indexer forward
                indexOn = true;
                indexTimer.reset();
                checkTimer = true;
            }
        } else { // If x is released
            //noinspection StatementWithEmptyBody
            if (indexOn && indexTimer.seconds() < 0.25) {
                // Still running based on timer, do nothing until timer finishes or x is pressed again
            } else if (indexOn) { // x released and timer either done or wasn't critical
                index(0.0); // Stop indexer
                indexOn = false;
                checkTimer = false;
            }
        }

        // Auto-stop indexer after a short duration if it was started
        if (checkTimer && indexOn && indexTimer.seconds() >= 0.25) {
            index(0.0);
            indexOn = false;
            checkTimer = false;
        }
    }

    // Helper method for indexer servos
    private void index(double power) {
        leftIndexServo.setPower(power);
        rightIndexServo.setPower(power);
    }

    @Override
    public void stop() {
        // Ensure motors and servos are stopped when OpMode ends
        if (follower != null) {
            follower.setTeleOpDrive(0,0,0, true); // Stop robot movement
            // follower.stop(); // If follower has a specific stop method
        }
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
        }
        if (leftIndexServo != null) {
            leftIndexServo.setPower(0);
        }
        if (rightIndexServo != null) {
            rightIndexServo.setPower(0);
        }
    }
}
