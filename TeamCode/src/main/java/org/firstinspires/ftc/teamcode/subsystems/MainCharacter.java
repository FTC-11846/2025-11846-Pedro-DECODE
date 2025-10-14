package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Central configuration for all robots in the program
 * Eliminates need for separate branches - use one codebase for all robots
 */
public enum MainCharacter {
    TEST_BOT,
    ROBOT_22154,
    ROBOT_11846;

    // Set this to switch between robots - could also be set via dashboard
    public static MainCharacter ACTIVE_ROBOT = TEST_BOT;

    // Hardware names
    public String getShooterMotorLName() {
        switch (this) {
            case ROBOT_22154: return "launchMotorL";
            case TEST_BOT:
            case ROBOT_11846:
            default: return "launchMotor";
        }
    }

    public String getShooterMotorRName() {
        return (this == ROBOT_22154) ? "launchMotorR" : null;
    }

    // Hardware capabilities
    public boolean hasDualShooters() {
        return this == ROBOT_22154;
    }

    public boolean hasLEDSystem() {
        return this == ROBOT_22154;
    }

    public boolean hasFoldingMechanism() {
        return this == ROBOT_11846;
    }

    public boolean hasLifters() {
        return this == ROBOT_22154;
    }

    // Shooter tuning constants
    public double getShooterLowVelocity() {
        return 1500.0; // Common for now, can differentiate later
    }

    public double getShooterHighVelocity() {
        switch (this) {
            case ROBOT_22154: return 4000.0;
            case TEST_BOT:
            case ROBOT_11846:
            default: return 5500.0;
        }
    }

    // Auto-aim ballistics (can be tuned per robot)
    public double getBaselinePower() {
        switch (this) {
            case ROBOT_22154: return 2000.0;
            case TEST_BOT:
            case ROBOT_11846:
            default: return 395.0;
        }
    }

    public double getLinearCorrectionFactor() {
        return 18.0; // Common for now
    }

    // PIDF tuning (22154 has different tuning due to dual motors)
    public double getShooterP() {
        return (this == ROBOT_22154) ? 20.0 : 10.0;
    }

    public double getShooterI() {
        return 0.0;
    }

    public double getShooterD() {
        return 0.0;
    }

    public double getShooterF(double voltage) {
        double nominalVoltage = 13.0;
        double maxTicksPerSec = 3000.0;
        double baseF = 32767.0 / maxTicksPerSec;
        return baseF * (nominalVoltage / voltage);
    }

    // Starting poses (example - adjust as needed)
    public com.pedropathing.geometry.Pose getDefaultStartPose() {
        switch (this) {
            case ROBOT_22154:
                return new com.pedropathing.geometry.Pose(56, 8, Math.toRadians(270));
            case ROBOT_11846:
                return new com.pedropathing.geometry.Pose(56, 8, Math.toRadians(0));
            case TEST_BOT:
            default:
                return new com.pedropathing.geometry.Pose(56, 8, Math.toRadians(0));
        }
    }
}
