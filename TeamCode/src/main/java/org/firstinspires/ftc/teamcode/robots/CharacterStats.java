package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * CharacterStats - Abstract base class with robot-specific configuration
 * 
 * The nested config classes below hold tuning values for the currently active robot.
 * Values are populated by calling applyConfiguration() after robot selection.
 * 
 * "Main Character Energy" - Every robot is unique! 🌟
 */
@Configurable
public abstract class CharacterStats {
    
    // ==================== CONFIGURATION OBJECTS ====================
    
    public static RobotIdentity _00_robotIdentity = new RobotIdentity();
    public static ShooterConfig shooterConfig = new ShooterConfig();
    public static BallFeedConfig ballFeedConfig = new BallFeedConfig();
    public static IntakeConfig intakeConfig = new IntakeConfig();
    public static VisionConfig visionConfig = new VisionConfig();
    public static IMUConfig imuConfig = new IMUConfig();  // ← ADD THIS


    // ==================== NESTED CONFIG CLASSES ====================
    
    public static class RobotIdentity {
        public String REMINDER = "⚠️ Robot-specific values - set in [RobotName]Abilities class ⚠️";
        public String activeRobot = "NOT_SET";
    }
    
    public static class ShooterConfig {
        public double highVelocityRPM = 0;
        public double lowVelocityRPM = 0;
        public double baselinePower = 0;
        public double pidfP = 2;
    }

    public static class BallFeedConfig {
        public double feedDuration = 0.25;
        public double reverseDuration = 0.0;  // 22154: prevents double-feed
        public double holdDuration = 0.0;      // 11846: gate hold time
        public double idlePower = 0.0;  // NEW: Power when idle (negative = slow reverse)

        // NEW: Servo position control (for 11846)
        public double ballFeedIdlePos = 0.5;      // Center/neutral position
        public double ballFeedLeftSweep = -0.4;   // Left gate offset from idle
        public double ballFeedRightSweep = 0.4;   // Right gate offset from idle
    }
    
    public static class IntakeConfig {
        public String intakeModeName = "NONE";
    }

    public static class VisionConfig {
        public double cameraForwardOffset = 0.0;  // Inches forward from robot center (+ = forward)
        public double cameraRightOffset = 0.0;    // Inches right from robot center (+ = right)
        public double cameraHeadingOffset = 0.0;  // Radians CCW from robot heading
    }

    public static class IMUConfig {
        public String logoFacingDirection = "RIGHT";  // RIGHT, LEFT, UP, DOWN, FORWARD, BACKWARD
        public String usbFacingDirection = "UP";      // RIGHT, LEFT, UP, DOWN, FORWARD, BACKWARD
    }


    // ==================== IDENTITY ====================
    
    /**
     * Get the full display name of this robot
     * Example: "Robot 22154", "TestBot"
     */
    public abstract String getDisplayName();
    
    /**
     * Get the short name for compact displays
     * Example: "22154", "TB", "11846"
     */
    public abstract String getShortName();
    
    // ==================== SHOOTER CONFIGURATION ====================
    
    /**
     * Get the hardware name of the left/primary shooter motor
     */
    public abstract String getShooterMotorLName();
    
    /**
     * Get the hardware name of the right shooter motor (null if single motor)
     */
    public abstract String getShooterMotorRName();
    
    /**
     * Does this robot have dual shooter motors?
     */
    public boolean hasDualShooters() {
        return getShooterMotorRName() != null;
    }
    
    /**
     * Get high velocity RPM for this robot's shooter
     * Reads from shared config populated by applyConfiguration()
     */
    public double getHighVelocityRPM() {
        return shooterConfig.highVelocityRPM;
    }
    
    /**
     * Get low velocity RPM for this robot's shooter
     * Reads from shared config populated by applyConfiguration()
     */
    public double getLowVelocityRPM() {
        return shooterConfig.lowVelocityRPM;
    }
    
    /**
     * Get auto-aim baseline power for this robot
     * Reads from shared config populated by applyConfiguration()
     */
    public double getBaselinePower() {
        return shooterConfig.baselinePower;
    }
    
    /**
     * Get PIDF P coefficient for shooter velocity control
     * Reads from shared config populated by applyConfiguration()
     */
    public double getShooterPIDFP() {
        return shooterConfig.pidfP;
    }
    
    // ==================== BALL FEED CONFIGURATION ====================
    
    /**
     * Get the hardware name of the left ball feed motor/servo
     */
    public abstract String getBallFeedMotorLName();
    
    /**
     * Get the hardware name of the right ball feed motor/servo
     */
    public abstract String getBallFeedMotorRName();
    
    /**
     * Get the ball feed mode for this robot
     */
    public abstract BallFeedMode getBallFeedMode();
    
    /**
     * Get default feed duration in seconds
     * Reads from shared config populated by applyConfiguration()
     */
    public double getDefaultFeedDuration() {
        return ballFeedConfig.feedDuration;
    }
    
    /**
     * Get reverse duration in seconds (0 = no reverse)
     * 22154: Reverses to prevent double-feed
     * 11846: No reverse
     */
    public double getFeedReverseDuration() {
        return ballFeedConfig.reverseDuration;
    }
    
    /**
     * Get hold duration in seconds (gate open time for ball to pass)
     * 11846: Needs time for ball to pass gate
     * 22154: N/A
     */
    public double getFeedHoldDuration() {
        return ballFeedConfig.holdDuration;
    }

    /**
     * Get idle power for CRServo feed motors (0 = stopped, negative = slow reverse)
     * 22154: Maintains back-pressure to prevent ball creep
     * TestBot/11846: No idle power
     */
    public double getIdlePower() {
        return ballFeedConfig.idlePower;
    }

    public double getFeedIdlePos() {
        return ballFeedConfig.ballFeedIdlePos;
    }

    public double getFeedLeftSweep() {
        return ballFeedConfig.ballFeedLeftSweep;
    }

    public double getFeedRightSweep() {
        return ballFeedConfig.ballFeedRightSweep;
    }
    // ==================== INTAKE CONFIGURATION ====================
    
    /**
     * Get the intake mode for this robot
     */
    public abstract IntakeMode getIntakeMode();
    
    /**
     * Get the hardware name of intake one (primary/front)
     */
    public String getIntakeOneMotorName() {
        return null;
    }
    
    /**
     * Get the hardware name of intake two (secondary/back)
     */
    public String getIntakeTwoMotorName() {
        return null;
    }
    
    // ==================== COLOR SENSOR CONFIGURATION ====================
    
    /**
     * Does this robot have color sensors for ball detection?
     */
    public boolean hasColorSensors() {
        return false;
    }
    
    /**
     * Get the hardware name of the front left lane color sensor
     */
    public String getFrontLeftLaneColorSensorName() {
        return null;
    }
    
    /**
     * Get the hardware name of the front right lane color sensor
     */
    public String getFrontRightLaneColorSensorName() {
        return null;
    }

    /**
     * Get the hardware name of the back left lane color sensor
     */
    public String getBackLeftLaneColorSensorName() {
        return null;
    }

    /**
     * Get the hardware name of the right lane color sensor
     */
    public String getBackRightLaneColorSensorName() {
        return null;
    }
    
    // ==================== LED CONFIGURATION ====================
    
    /**
     * Does this robot have an LED system?
     */
    public boolean hasLEDSystem() {
        return false; // Most robots don't have LEDs
    }
    
    /**
     * Get the hardware name of the left LED servo (null if no LEDs)
     */
    public String getLEDServoLName() {
        return null;
    }
    
    /**
     * Get the hardware name of the right LED servo (null if no LEDs)
     */
    public String getLEDServoRName() {
        return null;
    }
    
    // ==================== MECHANISM CONFIGURATION ====================
    
    /**
     * Does this robot have a folding mechanism?
     */
    public boolean hasFoldingMechanism() {
        return false;
    }
    
    /**
     * Does this robot have lifters?
     */
    public boolean hasLifters() {
        return false;
    }
    
    /**
     * Get the hardware name of the left lifter motor (null if no lifters)
     */
    public String getLifterMotorLName() {
        return null;
    }
    
    /**
     * Get the hardware name of the right lifter motor (null if no lifters)
     */
    public String getLifterMotorRName() {
        return null;
    }
    
    /**
     * Does this robot have color sensors?
     * @deprecated Use hasColorSensors() instead
     */
    @Deprecated
    public boolean hasColorSensor() {
        return hasColorSensors();
    }
    
    /**
     * Get the hardware name of the color sensor (null if no sensor)
     * @deprecated Use getLeftLaneColorSensorName() or getRightLaneColorSensorName() instead
     */
    @Deprecated
    public String getColorSensorName() {
        return null;
    }
    
    // ==================== VISION/CAMERA CONFIGURATION ====================

    /**
     * Get camera forward offset from robot center in inches
     * Positive = camera is forward of robot center
     */
    public double getCameraForwardOffset() {
        return visionConfig.cameraForwardOffset;
    }

    /**
     * Get camera right offset from robot center in inches
     * Positive = camera is right of robot center
     */
    public double getCameraRightOffset() {
        return visionConfig.cameraRightOffset;
    }

    /**
     * Get camera heading offset from robot heading in radians
     * Positive = camera rotated CCW from robot heading
     */
    public double getCameraHeadingOffset() {
        return visionConfig.cameraHeadingOffset;
    }

    // ==================== IMU CONFIGURATION ====================

    /**
     * Get IMU logo facing direction for this robot
     */
    public RevHubOrientationOnRobot.LogoFacingDirection getIMULogoDirection() {
        return RevHubOrientationOnRobot.LogoFacingDirection.valueOf(imuConfig.logoFacingDirection);
    }

    /**
     * Get IMU USB facing direction for this robot
     */
    public RevHubOrientationOnRobot.UsbFacingDirection getIMUUsbDirection() {
        return RevHubOrientationOnRobot.UsbFacingDirection.valueOf(imuConfig.usbFacingDirection);
    }

    // ==================== CONFIGURATION APPLICATION ====================
    
    /**
     * Apply this robot's configuration values to the shared config objects.
     * Called after robot selection in init_loop.
     * 
     * Child classes implement this to copy their specific values into
     * the static config objects above.
     */
    public abstract void applyConfiguration();
    
    // ==================== ENUMS ====================

    public enum BallFeedMode {
        SINGLE_CRSERVO,           // TestBot: 1 CRServo
        DUAL_CRSERVO_INDEPENDENT, // 22154: 2 CRServos, independent control
        DUAL_SERVO_GATES          // 11846: 2 Position Servos (gates)
    }
    
    public enum IntakeMode {
        NONE,                       // No intake hardware
        SINGLE_TOGGLE,              // Single motor, toggle on/off
        DUAL_INDEPENDENT_TOGGLE,    // Two motors, each toggles independently
        SINGLE_CONTINUOUS           // Single motor, runs while button held
    }
}
