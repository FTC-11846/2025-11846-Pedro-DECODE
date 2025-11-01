package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Robot11846Abilities - Configuration for competition robot 11846
 * Features: Single shooter, dual-independent ball feed (gates), single-toggle intake, color sensors, LEDs
 */
public class Robot11846Abilities extends CharacterStats {
    
    // ==================== TUNABLE CONSTANTS ====================

    public static class ShooterConstants {
        public static double HIGH_VELOCITY_RPM = 3000;
        public static double LOW_VELOCITY_RPM = 2600;
        public static double BASELINE_POWER = 395.0;
        public static double PIDF_P = 0.1;
    }

    public static class BallFeedConstants {
        public static double FEED_DURATION = 0.5;     // Gate lowering time
        public static double REVERSE_DURATION = 0.5;  // Gate raising time (return)
        public static double HOLD_DURATION = 0.3;     // Hold gate down for ball to pass
        public static double FEED_IDLE_POS = 0.5;
        public static double FEED_LEFT_SWEEP = -0.4;
        public static double FEED_RIGHT_SWEEP = 0.4;
    }

    public static class VisionConstants {
        public static double CAMERA_FORWARD_OFFSET = 6.0;  // TODO: Measure actual offset!
        public static double CAMERA_RIGHT_OFFSET = 2.5;
        public static double CAMERA_HEADING_OFFSET = 0.0;
    }

    // ==================== CONFIGURATION APPLICATION ====================

    @Override
    public void applyConfiguration() {
        // Robot identity
        _00_robotIdentity.activeRobot = getDisplayName();

        // Shooter config
        shooterConfig.highVelocityRPM = ShooterConstants.HIGH_VELOCITY_RPM;
        shooterConfig.lowVelocityRPM = ShooterConstants.LOW_VELOCITY_RPM;
        shooterConfig.baselinePower = ShooterConstants.BASELINE_POWER;
        shooterConfig.pidfP = ShooterConstants.PIDF_P;

        // BallFeed config - NOW INCLUDING SERVO POSITIONS
        ballFeedConfig.feedDuration = BallFeedConstants.FEED_DURATION;
        ballFeedConfig.reverseDuration = BallFeedConstants.REVERSE_DURATION;
        ballFeedConfig.holdDuration = BallFeedConstants.HOLD_DURATION;
        ballFeedConfig.ballFeedIdlePos = BallFeedConstants.FEED_IDLE_POS;
        ballFeedConfig.ballFeedLeftSweep = BallFeedConstants.FEED_LEFT_SWEEP;
        ballFeedConfig.ballFeedRightSweep = BallFeedConstants.FEED_RIGHT_SWEEP;

        // Intake config
        intakeConfig.intakeModeName = getIntakeMode().toString();

        // Vision/Camera offset
        visionConfig.cameraForwardOffset = VisionConstants.CAMERA_FORWARD_OFFSET;
        visionConfig.cameraRightOffset = VisionConstants.CAMERA_RIGHT_OFFSET;
        visionConfig.cameraHeadingOffset = VisionConstants.CAMERA_HEADING_OFFSET;

        // IMU config
        imuConfig.logoFacingDirection = "LEFT";
        imuConfig.usbFacingDirection = "DOWN";
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
        return BallFeedMode.DUAL_SERVO_GATES;
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


}
