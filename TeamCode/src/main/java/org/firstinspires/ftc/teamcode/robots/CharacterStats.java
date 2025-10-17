package org.firstinspires.ftc.teamcode.robots;

import com.pedropathing.geometry.Pose;

/**
 * CharacterStats - Abstract base class defining all robot configuration methods
 * Each robot extends this class and overrides methods for their specific hardware
 * 
 * "Main Character Energy" - Every robot is unique! 🌟
 */
public abstract class CharacterStats {
    
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
     */
    public abstract double getHighVelocityRPM();
    
    /**
     * Get low velocity RPM for this robot's shooter
     */
    public abstract double getLowVelocityRPM();
    
    /**
     * Get auto-aim baseline power for this robot
     */
    public abstract double getBaselinePower();
    
    /**
     * Get PIDF P coefficient for shooter velocity control
     */
    public abstract double getShooterPIDFP();
    
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
     */
    public double getDefaultFeedDuration() {
        return 0.25; // Most robots use this default
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
     */
    public boolean hasColorSensor() {
        return false;
    }
    
    /**
     * Get the hardware name of the color sensor (null if no sensor)
     */
    public String getColorSensorName() {
        return null;
    }
    
    // ==================== STARTING POSES ====================
    
    /**
     * Get the default starting pose for this robot
     * Override if robot has a different preferred heading
     */
    public Pose getDefaultStartPose() {
        return new Pose(56, 8, Math.toRadians(0));
    }
    
    /**
     * Get Red Near starting pose for this robot
     */
    public Pose getRedNearPose() {
        return new Pose(56, 8, getDefaultHeading());
    }
    
    /**
     * Get Red Far starting pose for this robot
     */
    public Pose getRedFarPose() {
        return new Pose(56, 136, getDefaultHeading());
    }
    
    /**
     * Get Blue Near starting pose for this robot
     */
    public Pose getBlueNearPose() {
        return new Pose(86, 8, Math.toRadians(180));
    }
    
    /**
     * Get Blue Far starting pose for this robot
     */
    public Pose getBlueFarPose() {
        return new Pose(86, 136, Math.toRadians(180));
    }
    
    /**
     * Get default heading for this robot (in radians)
     */
    protected double getDefaultHeading() {
        return Math.toRadians(0);
    }
    
    // ==================== ENUMS ====================
    
    public enum BallFeedMode {
        SINGLE,              // One motor only
        DUAL_SYNCHRONIZED,   // Two motors, same power
        DUAL_INDEPENDENT     // Two motors, can control separately
    }
}
