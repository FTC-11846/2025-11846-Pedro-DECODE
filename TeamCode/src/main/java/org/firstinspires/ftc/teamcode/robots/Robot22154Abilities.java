package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Robot22154Abilities - Configuration for competition robot 22154
 * Features: Dual shooters, dual-independent ball feed, dual-independent intake, color sensors, LEDs, lifters
 */
public class Robot22154Abilities extends CharacterStats {
    
    // ==================== TUNABLE CONSTANTS ====================
    
    @Configurable
    public static class ShooterConstants {
        public static double HIGH_VELOCITY_RPM = 4000;
        public static double LOW_VELOCITY_RPM = 1500;
        public static double BASELINE_POWER = 2000.0;
        public static double PIDF_P = 20.0;
    }
    
    @Configurable
    public static class BallFeedConstants {
        public static double FEED_DURATION = 0.25;
        public static double REVERSE_DURATION = 0.1;  // Prevent double-feed
        public static double HOLD_DURATION = 0.0;     // N/A for 22154
    }
    
    @Configurable
    public static class StartPoseConstants {
        public static double DEFAULT_HEADING_DEG = 270.0;
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
        return BallFeedMode.DUAL_INDEPENDENT;  // Independent L/R control
    }
    
    @Override
    public double getFeedReverseDuration() {
        return ballFeedConfig.reverseDuration;
    }
    
    @Override
    public double getFeedHoldDuration() {
        return ballFeedConfig.holdDuration;
    }
    
    // ==================== INTAKE CONFIGURATION ====================
    
    @Override
    public IntakeMode getIntakeMode() {
        return IntakeMode.DUAL_INDEPENDENT_TOGGLE;  // Two stages, each toggles
    }
    
    @Override
    public String getIntakeOneMotorName() {
        return "frontIntakeServo";  // Front stage
    }
    
    @Override
    public String getIntakeTwoMotorName() {
        return "backIntakeServo";   // Back stage
    }
    
    // ==================== COLOR SENSOR CONFIGURATION ====================
    
    @Override
    public boolean hasColorSensors() {
        return true;  // Has sensors for ball detection
    }
    
    @Override
    public String getFrontLeftLaneColorSensorName() {
        return "frontLeftColorSensor";
    }
    
    @Override
    public String getFrontRightLaneColorSensorName() {
        return "frontRightColorSensor";
    }

    @Override
    public String getBackLeftLaneColorSensorName() {
        return "backLeftColorSensor";
    }

    @Override
    public String getBackRightLaneColorSensorName() {
        return "backRightColorSensor";
    }
    
    // ==================== LED CONFIGURATION ====================
    
    @Override
    public boolean hasLEDSystem() {
        return true;  // Has LEDs for ball status display
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
        return true;
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
        return Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG);
    }
    
    @Override
    public Pose getDefaultStartPose() {
        return new Pose(56, 8, Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG));
    }
    
    @Override
    public Pose getRedNearPose() {
        return new Pose(56, 8, Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG));
    }
    
    @Override
    public Pose getRedFarPose() {
        return new Pose(56, 136, Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG));
    }
    
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
        
        startPoseConfig.defaultHeadingDeg = StartPoseConstants.DEFAULT_HEADING_DEG;
    }
}
