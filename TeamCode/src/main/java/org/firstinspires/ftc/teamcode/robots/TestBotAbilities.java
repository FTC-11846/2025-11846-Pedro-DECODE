package org.firstinspires.ftc.teamcode.robots;

import com.bylazar.configurables.annotations.Configurable;

/**
 * TestBotAbilities - Configuration for the development/testing robot
 * Used for alpha/beta testing of major features before competition
 */
public class TestBotAbilities extends CharacterStats {

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
    // TestBot has no special mechanisms, all defaults from base class

    // ==================== STARTING POSES ====================
    // TestBot uses default poses from base class
}