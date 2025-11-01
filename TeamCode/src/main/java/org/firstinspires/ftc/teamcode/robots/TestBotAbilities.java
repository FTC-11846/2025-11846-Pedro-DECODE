package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;

/**
 * TestBotAbilities - Configuration for basic test robot
 * Features: Single shooter, single ball feed only
 * NO intake, NO color sensors, NO LEDs, NO lifters
 */
public class TestBotAbilities extends CharacterStats {
    
    // ==================== TUNABLE CONSTANTS ====================
    public static class ShooterConstants {
        public static double HIGH_VELOCITY_RPM = 3000;
        public static double LOW_VELOCITY_RPM = 1000;
        public static double BASELINE_POWER = 1500.0;
        public static double PIDF_P = 15.0;
    }

    public static class BallFeedConstants {
        public static double FEED_DURATION = 0.25;
    }

    public static class VisionConstants {
        public static double CAMERA_FORWARD_OFFSET = 6.5;  // TODO: Measure actual offset!
        public static double CAMERA_RIGHT_OFFSET = 0.0;
        public static double CAMERA_HEADING_OFFSET = 0.0;  // = 0.0 If Robot & Camera face same direction
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
        ballFeedConfig.reverseDuration = 0.0;
        ballFeedConfig.holdDuration = 0.0;

        intakeConfig.intakeModeName = getIntakeMode().name();

        // Vision/Camera offset
        visionConfig.cameraForwardOffset = VisionConstants.CAMERA_FORWARD_OFFSET;
        visionConfig.cameraRightOffset = VisionConstants.CAMERA_RIGHT_OFFSET;
        visionConfig.cameraHeadingOffset = VisionConstants.CAMERA_HEADING_OFFSET;

        // IMU config
        imuConfig.logoFacingDirection = "RIGHT";
        imuConfig.usbFacingDirection = "UP";
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
        return null; // Single shooter
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
        return BallFeedMode.SINGLE_CRSERVO;  // Single CRServo
    }

    // ==================== INTAKE CONFIGURATION ====================
    
    @Override
    public IntakeMode getIntakeMode() {
        return IntakeMode.SINGLE_TOGGLE;  // No intake hardware
    }

    @Override
    public String getIntakeOneMotorName() {
        return "intakeMotor";  // Front stage
    }
    
    // ==================== COLOR SENSOR CONFIGURATION ====================
    
    @Override
    public boolean hasColorSensors() {
        return false;  // No color sensors
    }
    
    // ==================== LED CONFIGURATION ====================
    
    @Override
    public boolean hasLEDSystem() {
        return false;  // No LEDs
    }
    
    // ==================== STARTING POSES ====================
    // TestBot uses default poses from base class
    

}
