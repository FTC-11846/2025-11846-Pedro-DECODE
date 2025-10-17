package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Robot22154Abilities - Configuration for competition robot 22154
 * Features: Dual shooters, LED system, lifters
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
    }

    @Configurable
    public static class StartPoseConstants {
        public static double DEFAULT_HEADING_DEG = 270.0;
    }

    // LED constants could go here if robot needs custom colors
    // @Configurable
    // public static class LEDConstants {
    //     public static double CUSTOM_COLOR = 0.555;
    // }

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

    @Override
    public double getHighVelocityRPM() {
        return ShooterConstants.HIGH_VELOCITY_RPM;
    }

    @Override
    public double getLowVelocityRPM() {
        return ShooterConstants.LOW_VELOCITY_RPM;
    }

    @Override
    public double getBaselinePower() {
        return ShooterConstants.BASELINE_POWER;
    }

    @Override
    public double getShooterPIDFP() {
        return ShooterConstants.PIDF_P;
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

    @Override
    public double getDefaultFeedDuration() {
        return BallFeedConstants.FEED_DURATION;
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

    // Future enhancement: Add color sensor capability
    // @Override
    // public boolean hasColorSensor() {
    //     return true;
    // }

    // @Override
    // public String getColorSensorName() {
    //     return "colorSensor";
    // }

    // ==================== STARTING POSES ====================

    @Override
    protected double getDefaultHeading() {
        return Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG);
    }

    @Override
    public Pose getDefaultStartPose() {
        return new Pose(56, 8, Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG));
    }

    // Red poses use the configured heading
    @Override
    public Pose getRedNearPose() {
        return new Pose(56, 8, Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG));
    }

    @Override
    public Pose getRedFarPose() {
        return new Pose(56, 136, Math.toRadians(StartPoseConstants.DEFAULT_HEADING_DEG));
    }
}