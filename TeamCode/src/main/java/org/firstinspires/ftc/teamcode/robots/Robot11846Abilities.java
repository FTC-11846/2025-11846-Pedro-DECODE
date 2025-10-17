package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Robot11846Abilities - Configuration for competition robot 11846
 * Features: Single shooter, folding mechanism
 */
public class Robot11846Abilities extends CharacterStats {

    // ==================== TUNABLE CONSTANTS ====================

    @Configurable
    public static class ShooterConstants {
        public static double HIGH_VELOCITY_RPM = 5500;
        public static double LOW_VELOCITY_RPM = 1500;
        public static double BASELINE_POWER = 395.0;
        public static double PIDF_P = 10.0;
    }

    @Configurable
    public static class BallFeedConstants {
        public static double FEED_DURATION = 0.25;
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
        return "launchMotor";
    }

    @Override
    public String getShooterMotorRName() {
        return null; // Single shooter
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

    // ==================== MECHANISM CONFIGURATION ====================

    @Override
    public boolean hasFoldingMechanism() {
        return true; // This robot has a folding mechanism!
    }

    // Note: Add hardware names for folding mechanism when implemented
    // public String getFoldingMotorName() {
    //     return "foldingMotor";
    // }

    // ==================== STARTING POSES ====================
    // Robot 11846 uses default poses from base class (0° heading)
}