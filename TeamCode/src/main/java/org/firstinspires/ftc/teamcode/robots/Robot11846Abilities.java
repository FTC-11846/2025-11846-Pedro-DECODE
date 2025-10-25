package org.firstinspires.ftc.teamcode.robots;

/**
 * Robot11846Abilities - Configuration for competition robot 11846
 * Features: Single shooter, folding mechanism
 */
public class Robot11846Abilities extends CharacterStats {
    
    // ==================== PRIVATE TUNING CONSTANTS ====================
    
    // Shooter tuning values
    private static final double HIGH_VELOCITY_RPM = 5500;
    private static final double LOW_VELOCITY_RPM = 1500;
    private static final double BASELINE_POWER = 395.0;
    private static final double PIDF_P = 10.0;
    
    // Ball feed tuning
    private static final double FEED_DURATION = 0.25;
    
    // Start pose tuning (uses default 0°)
    private static final double DEFAULT_HEADING_DEG = 0.0;
    
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
        return "Robot 11846";
    }
    
    @Override
    public String getShortName() {
        return "11846";
    }
    
    // ==================== SHOOTER CONFIGURATION ====================
    
    @Override
    public String getShooterMotorLName() {
        return "launchMotorL";
    }
    
    @Override
    public String getShooterMotorRName() {
        return "launchMotorR";
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
    
    // ==================== MECHANISM CONFIGURATION ====================
    
    @Override
    public boolean hasFoldingMechanism() {
        return true; // This robot has a folding mechanism!
    }
}
