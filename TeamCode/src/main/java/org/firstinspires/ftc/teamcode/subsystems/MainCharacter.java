package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.robots.Robot11846Abilities;
import org.firstinspires.ftc.teamcode.robots.Robot22154Abilities;
import org.firstinspires.ftc.teamcode.robots.TestBotAbilities;

/**
 * MainCharacter - Enum wrapper for robot selection
 * Each robot returns its CharacterStats implementation
 *
 * "Main Character Energy" - Every robot deserves to be the star! 🌟
 */
public enum MainCharacter {
    TEST_BOT(new TestBotAbilities()),
    ROBOT_22154(new Robot22154Abilities()),
    ROBOT_11846(new Robot11846Abilities());

    // This will be set during init_loop via driver station selection
    public static MainCharacter ACTIVE_ROBOT = TEST_BOT;

    // Each robot has its own CharacterStats implementation
    private final CharacterStats abilities;

    MainCharacter(CharacterStats abilities) {
        this.abilities = abilities;
    }

    /**
     * Get the CharacterStats for this robot
     * This is the main way to access robot configuration!
     */
    public CharacterStats getAbilities() {
        return abilities;
    }

    // ==================== CONVENIENCE METHODS ====================
    // These delegate to the CharacterStats implementation

    @Override
    public String toString() {
        return abilities.getDisplayName();
    }

    public String getShortName() {
        return abilities.getShortName();
    }

    // ==================== DEPRECATED METHODS ====================
    // These are kept for backward compatibility during migration
    // They simply delegate to CharacterStats
    // TODO: Remove these after updating all subsystems

    @Deprecated
    public String getShooterMotorLName() {
        return abilities.getShooterMotorLName();
    }

    @Deprecated
    public String getShooterMotorRName() {
        return abilities.getShooterMotorRName();
    }

    @Deprecated
    public boolean hasDualShooters() {
        return abilities.hasDualShooters();
    }

    @Deprecated
    public String getBallFeedMotorLName() {
        return abilities.getBallFeedMotorLName();
    }

    @Deprecated
    public String getBallFeedMotorRName() {
        return abilities.getBallFeedMotorRName();
    }

    @Deprecated
    public CharacterStats.BallFeedMode getBallFeedMode() {
        return abilities.getBallFeedMode();
    }

    @Deprecated
    public boolean hasLEDSystem() {
        return abilities.hasLEDSystem();
    }

    @Deprecated
    public String getLEDServoLName() {
        return abilities.getLEDServoLName();
    }

    @Deprecated
    public String getLEDServoRName() {
        return abilities.getLEDServoRName();
    }

    @Deprecated
    public boolean hasFoldingMechanism() {
        return abilities.hasFoldingMechanism();
    }

    @Deprecated
    public boolean hasLifters() {
        return abilities.hasLifters();
    }

    @Deprecated
    public String getLifterMotorLName() {
        return abilities.getLifterMotorLName();
    }

    @Deprecated
    public String getLifterMotorRName() {
        return abilities.getLifterMotorRName();
    }

    @Deprecated
    public com.pedropathing.geometry.Pose getDefaultStartPose() {
        return abilities.getDefaultStartPose();
    }

    // ==================== ENUMS ====================
    // Keep the enum here for compatibility, but use CharacterStats.BallFeedMode

    @Deprecated
    public enum BallFeedMode {
        SINGLE,
        DUAL_SYNCHRONIZED,
        DUAL_INDEPENDENT
    }
}