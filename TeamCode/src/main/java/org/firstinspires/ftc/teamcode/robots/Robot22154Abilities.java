package org.firstinspires.ftc.teamcode.robots;

/**
 * Robot22154Abilities - Configuration for competition robot 22154
 * Features: Dual shooters, dual-independent ball feed, dual-independent intake, color sensors, LEDs, lifters
 */

 // @Configurable should NOT appear anywhere in this class!!!  Tune from CharacterStats in Panels
public class Robot22154Abilities extends CharacterStats {
    
    // ==================== TUNABLE CONSTANTS ====================
    
    public static class ShooterConstants {
        public static double HIGH_VELOCITY_RPM = 3000;
        public static double LOW_VELOCITY_RPM = 2350;
        public static double BASELINE_POWER = 0.0;
        public static double PIDF_P = 2.0;
    }
    
    public static class BallFeedConstants {
        public static double FEED_DURATION = 0.25;
        public static double REVERSE_DURATION = 0.0;  // Prevent double-feed
        public static double HOLD_DURATION = 0.0;     // N/A for 22154
        public static double IDLE_POWER = -0.2;  // NEW: Slow reverse when idle

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
        ballFeedConfig.reverseDuration = BallFeedConstants.REVERSE_DURATION;
        ballFeedConfig.holdDuration = BallFeedConstants.HOLD_DURATION;
        ballFeedConfig.idlePower = BallFeedConstants.IDLE_POWER;  // NEW


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
        imuConfig.logoFacingDirection = "UP";
        imuConfig.usbFacingDirection = "RIGHT";
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
        return BallFeedMode.DUAL_CRSERVO_INDEPENDENT;  // Independent L/R control
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
    public EndgameSystem endgameMechanism() {
        return EndgameSystem.LIFT;
    }
    
    @Override
    public String getLifterMotorLName() {
        return "liftMotorL";
    }
    
    @Override
    public String getLifterMotorRName() {
        return "liftMotorR";
    }

}
