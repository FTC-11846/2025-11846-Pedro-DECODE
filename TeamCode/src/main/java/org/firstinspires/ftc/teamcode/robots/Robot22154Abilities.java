package org.firstinspires.ftc.teamcode.robots;

import com.pedropathing.geometry.Pose;

/**
 * Robot22154Abilities - Configuration for competition robot 22154
 * Features: Dual shooters, LED system, lifters
 */
public class Robot22154Abilities extends CharacterStats {
    
    // ==================== PRIVATE TUNING CONSTANTS ====================
    
    // Shooter tuning values
    private static final double HIGH_VELOCITY_RPM = 4000;
    private static final double LOW_VELOCITY_RPM = 1500;
    private static final double BASELINE_POWER = 2000.0;
    private static final double PIDF_P = 20.0;
    
    // Ball feed tuning
    private static final double FEED_DURATION = 0.25;
    
    // Start pose tuning
    private static final double DEFAULT_HEADING_DEG = 270.0;
    
    // ==================== CONFIGURATION APPLICATION ====================
    
    @Override
    public void applyConfiguration() {
        // Update robot identity
        CharacterStats._00_robotIdentity.activeRobot = getDisplayName();
        
        // Copy shooter values into shared config
        CharacterStats.shooterConfig.highVelocityRPM = HIGH_VELOCITY_RPM;
        CharacterStats.shooterConfig.lowVelocityRPM = LOW_VELOCITY_RPM;
        CharacterStats.shooterConfig.baselinePower = BASELINE_POWER;
        CharacterStats.shooterConfig.pidfP = PIDF_P;
        
        // Copy ball feed values
        CharacterStats.ballFeedConfig.feedDuration = FEED_DURATION;
        
        // Copy start pose values
        CharacterStats.startPoseConfig.defaultHeadingDeg = DEFAULT_HEADING_DEG;
    }
    
    // ==================== IDENTITY ====================
    
    @Override
    public String getDisplayName() {
        return "Robot 22154";
    }
    
    @Override
    public String getShortName() {
        return "22154";
    }
    
    // ==================== SHOOTER CONFIGURATION ====================
    
    @Override
    public String getShooterMotorLName() {
        return "launchMotorL";
    }
    
    @Override
    public String getShooterMotorRName() {
        return "launchMotorR"; // This robot has dual shooters!
    }
    
    // ==================== BALL FEED CONFIGURATION ====================
    
    @Override
    public String getBallFeedMotorLName() {
        return "feedServoL";
    }
    
    @Override
    public String getBallFeedMotorRName() {
        return "feedServoR";
    }
    
    @Override
    public BallFeedMode getBallFeedMode() {
        return BallFeedMode.DUAL_INDEPENDENT;
    }
    
    // ==================== LED CONFIGURATION ====================
    
    @Override
    public boolean hasLEDSystem() {
        return true; // This robot has LEDs!
    }
    
    @Override
    public String getLEDServoLName() {
        return "ballColorLEDL";
    }
    
    @Override
    public String getLEDServoRName() {
        return "ballColorLEDR";
    }
    
    // ==================== MECHANISM CONFIGURATION ====================
    
    @Override
    public boolean hasLifters() {
        return true; // This robot has lifters!
    }
    
    @Override
    public String getLifterMotorLName() {
        return "liftMotorL";
    }
    
    @Override
    public String getLifterMotorRName() {
        return "liftMotorR";
    }
    
    // ==================== STARTING POSES ====================
    
    @Override
    protected double getDefaultHeading() {
        return Math.toRadians(startPoseConfig.defaultHeadingDeg);
    }
    
    @Override
    public Pose getDefaultStartPose() {
        return new Pose(56, 8, Math.toRadians(startPoseConfig.defaultHeadingDeg));
    }
    
    @Override
    public Pose getRedNearPose() {
        return new Pose(56, 8, Math.toRadians(startPoseConfig.defaultHeadingDeg));
    }
    
    @Override
    public Pose getRedFarPose() {
        return new Pose(56, 136, Math.toRadians(startPoseConfig.defaultHeadingDeg));
    }
}
