package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Main Character configuration - defines hardware and behavior for each robot
 * "Main Character Energy" - because every robot deserves to be the star! 🌟
 */
public enum MainCharacter {
    TEST_BOT,
    ROBOT_22154,
    ROBOT_11846;

    // This will be set during init_loop via driver station selection
    public static MainCharacter ACTIVE_ROBOT = TEST_BOT;

    // ==================== HARDWARE CONFIGURATION ====================

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

    public boolean hasDualShooters() {
        return this == ROBOT_22154;
    }

    // Ball Feed configuration
    public String getBallFeedMotorLName() {
        return "feedServoL";
    }

    public String getBallFeedMotorRName() {
        return "feedServoR";
    }

    public BallFeedMode getBallFeedMode() {
        // All robots currently use dual synchronized
        // Can be changed per robot as needed
        return BallFeedMode.DUAL_SYNCHRONIZED;
    }

    // LED configuration
    public boolean hasLEDSystem() {
        return this == ROBOT_22154;
    }

    public String getLEDServoLName() {
        return (this == ROBOT_22154) ? "ballColorLEDL" : null;
    }

    public String getLEDServoRName() {
        return (this == ROBOT_22154) ? "ballColorLEDR" : null;
    }

    // Other mechanisms
    public boolean hasFoldingMechanism() {
        return this == ROBOT_11846;
    }

    public boolean hasLifters() {
        return this == ROBOT_22154;
    }

    public String getLifterMotorLName() {
        return (this == ROBOT_22154) ? "liftMotorL" : null;
    }

    public String getLifterMotorRName() {
        return (this == ROBOT_22154) ? "liftMotorR" : null;
    }

    // ==================== STARTING POSES ====================

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

    // ==================== ENUMS ====================

    public enum BallFeedMode {
        SINGLE,              // One motor only
        DUAL_SYNCHRONIZED,   // Two motors, same power
        DUAL_INDEPENDENT     // Two motors, can control separately (future)
    }

    // ==================== UTILITY METHODS ====================

    @Override
    public String toString() {
        switch (this) {
            case TEST_BOT: return "TestBot";
            case ROBOT_22154: return "Robot 22154";
            case ROBOT_11846: return "Robot 11846";
            default: return "Unknown";
        }
    }

    public String getShortName() {
        switch (this) {
            case TEST_BOT: return "TB";
            case ROBOT_22154: return "22154";
            case ROBOT_11846: return "11846";
            default: return "???";
        }
    }
}