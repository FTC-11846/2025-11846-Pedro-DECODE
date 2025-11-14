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
        public static double CAMERA_FORWARD_OFFSET = -7;  /// FTC SDK uses X, Y, Z offsets, Confirm what X & Y means!
        public static double CAMERA_RIGHT_OFFSET = 0;
        public static double CAMERA_Z_OFFSET = 13.5; // TODO: Add these new constants to all robots.classes!
        public static double CAMERA_PITCH = 10.6;  // TODO: Switch from our own 2-param estimation to full FTC SDK pose estimation!
        public static double CAMERA_ROLL = 0;
        public static double CAMERA_YAW = -3.0;
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

        /**
        Vision/Camera offsets.  We write directly to vision_Config,
         which is just CharacterStats unique reference to the exact same
         static object instance defined, instantiated, and used by Vision class
        */
        vision_Config.cameraPosX = VisionConstants.CAMERA_RIGHT_OFFSET;
        vision_Config.cameraPosY = VisionConstants.CAMERA_FORWARD_OFFSET;
        vision_Config.cameraPosZ = VisionConstants.CAMERA_Z_OFFSET;
        vision_Config.cameraYaw = VisionConstants.CAMERA_YAW;
        vision_Config.cameraPitch = VisionConstants.CAMERA_PITCH;
        vision_Config.cameraRoll = VisionConstants.CAMERA_ROLL;

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
