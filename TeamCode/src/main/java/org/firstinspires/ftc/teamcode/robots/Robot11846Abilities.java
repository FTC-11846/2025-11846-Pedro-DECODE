package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Robot11846Abilities - Configuration for competition robot 11846
 * Features: Single shooter, dual-independent ball feed (gates), single-toggle intake, color sensors, LEDs
 */
public class Robot11846Abilities extends CharacterStats {
    
    // ==================== TUNABLE CONSTANTS ====================

    public static class ShooterConstants {
        public static double HIGH_VELOCITY_RPM = 5500;
        public static double LOW_VELOCITY_RPM = 1500;
        public static double BASELINE_POWER = 395.0;
        public static double PIDF_P = 10.0;
    }
    
    public static class BallFeedConstants {
        public static double FEED_DURATION = 0.5;     // Gate lowering time
        public static double REVERSE_DURATION = 0.5;  // Gate raising time (return)
        public static double HOLD_DURATION = 0.3;     // Hold gate down for ball to pass
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
        return "launchMotorR"; // Dual shooters
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
        return BallFeedMode.DUAL_INDEPENDENT;  // Independent L/R gate control
    }
    
    @Override
    public double getFeedReverseDuration() {
        return ballFeedConfig.reverseDuration;  // Gate return time
    }
    
    @Override
    public double getFeedHoldDuration() {
        return ballFeedConfig.holdDuration;  // Hold gate down for ball
    }
    
    // ==================== INTAKE CONFIGURATION ====================
    
    @Override
    public IntakeMode getIntakeMode() {
        return IntakeMode.SINGLE_TOGGLE;  // One motor, toggle on/off
    }
    
    @Override
    public String getIntakeOneMotorName() {
        return "intakeMotor";  // Single DCMotorEx
    }
    
    @Override
    public String getIntakeTwoMotorName() {
        return null;  // No second intake
    }
    
    // ==================== COLOR SENSOR CONFIGURATION ====================
    
    @Override
    public boolean hasColorSensors() {
        return false;  // Has sensors for ball detection
    }
    
//    @Override
//    public String getLeftLaneColorSensorName() {
//        return "leftColorSensor";
//    }
//
//    @Override
//    public String getRightLaneColorSensorName() {
//        return "rightColorSensor";
//    }

    // ==================== LED CONFIGURATION ====================
    
    @Override
    public boolean hasLEDSystem() {
        return false;  // Has LEDs for ball status display
    }
    
//    @Override
//    public String getLEDServoLName() {
//        return "ballColorLEDL";
//    }
//
//    @Override
//    public String getLEDServoRName() {
//        return "ballColorLEDR";
//    }
    
    // ==================== MECHANISM CONFIGURATION ====================
    
    @Override
    public boolean hasFoldingMechanism() {
        return true;  // Has folding mechanism
    }
    
    // ==================== STARTING POSES ====================
    // Robot 11846 uses default poses from base class (0° heading)
    
    // ==================== CONFIGURATION APPLICATION ====================
    
    @Override
    public void applyConfiguration() {
        _00_robotIdentity.activeRobot = getDisplayName();
        
        shooterConfig.highVelocityRPM = ShooterConstants.HIGH_VELOCITY_RPM;
        shooterConfig.lowVelocityRPM = ShooterConstants.LOW_VELOCITY_RPM;
        shooterConfig.baselinePower = ShooterConstants.BASELINE_POWER;
        shooterConfig.pidfP = ShooterConstants.PIDF_P;
        
        ballFeedConfig.feedDuration = BallFeedConstants.FEED_DURATION;
        ballFeedConfig.reverseDuration = BallFeedConstants.REVERSE_DURATION;
        ballFeedConfig.holdDuration = BallFeedConstants.HOLD_DURATION;
        
        intakeConfig.intakeModeName = getIntakeMode().name();
        
        startPoseConfig.defaultHeadingDeg = 0.0;
    }
}
