package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;

/**
 * LED subsystem for RGB indicator lights (GoBilda PWM-controlled LEDs)
 * Now uses CharacterStats for configuration
 * Only available on robots with LED hardware
 */
public class LED {
    private final CharacterStats stats;
    private final Servo ledL;
    private final Servo ledR;

    // ==================== TUNABLE CONSTANTS ====================

    @Configurable
    public static class Constants {
        // GoBilda RGB LED PWM positions
        // See: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
        public static double GREEN = 0.444;
        public static double PURPLE = 0.722;
        public static double RED = 0.167;
        public static double BLUE = 0.611;
        public static double YELLOW = 0.333;
        public static double WHITE = 0.889;
        public static double OFF = 0.056;
    }

    // ==================== CONSTRUCTOR ====================

    /**
     * Create LED using MainCharacter enum (backward compatible)
     */
    public LED(HardwareMap hardwareMap, MainCharacter character) {
        this(hardwareMap, character.getAbilities());
    }

    /**
     * Create LED using CharacterStats directly (preferred)
     */
    public LED(HardwareMap hardwareMap, CharacterStats stats) {
        this.stats = stats;

        if (!stats.hasLEDSystem()) {
            throw new IllegalStateException(stats.getDisplayName() + " does not have LED system!");
        }

        ledL = hardwareMap.get(Servo.class, stats.getLEDServoLName());
        ledR = hardwareMap.get(Servo.class, stats.getLEDServoRName());
    }

    // ==================== PUBLIC CONTROL METHODS ====================

    /**
     * Set both LEDs to the same color
     */
    public void setColor(double colorPosition) {
        ledL.setPosition(colorPosition);
        ledR.setPosition(colorPosition);
    }

    /**
     * Convenience methods for common colors
     */
    public void setGreen() {
        setColor(Constants.GREEN);
    }

    public void setPurple() {
        setColor(Constants.PURPLE);
    }

    public void setRed() {
        setColor(Constants.RED);
    }

    public void setBlue() {
        setColor(Constants.BLUE);
    }

    public void setYellow() {
        setColor(Constants.YELLOW);
    }

    public void setWhite() {
        setColor(Constants.WHITE);
    }

    public void turnOff() {
        setColor(Constants.OFF);
    }

    /**
     * Set LEDs to alliance color (Green or Purple for DECODE)
     */
    public void setAllianceColor(boolean isGreen) {
        setColor(isGreen ? Constants.GREEN : Constants.PURPLE);
    }

    /**
     * Set left and right LEDs independently
     */
    public void setIndependent(double leftColor, double rightColor) {
        ledL.setPosition(leftColor);
        ledR.setPosition(rightColor);
    }

    // ==================== GETTERS ====================

    public double getCurrentLeftPosition() {
        return ledL.getPosition();
    }

    public double getCurrentRightPosition() {
        return ledR.getPosition();
    }

    public String getRobotName() {
        return stats.getDisplayName();
    }
}