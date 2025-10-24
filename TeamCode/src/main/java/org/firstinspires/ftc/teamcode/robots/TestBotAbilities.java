package org.firstinspires.ftc.teamcode.robots;

/**
 * TestBotAbilities - Configuration for the development/testing robot
 * Used for alpha/beta testing of major features before competition
 */
public class TestBotAbilities extends CharacterStats {
    
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
        return "TestBot";
    }
    
    @Override
    public String getShortName() {
        return "TB";
    }
    
    // ==================== SHOOTER CONFIGURATION ====================
    
    @Override
    public String getShooterMotorLName() {
        return "launchMotor";
    }
    
    @Override
    public String getShooterMotorRName() {
        return null; // TestBot has single shooter
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
        return BallFeedMode.DUAL_SYNCHRONIZED;
    }
}
