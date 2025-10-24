package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;

/**
 * LED subsystem with universal configuration constants
 */
@Configurable
public class LED {
    
    // ==================== CONFIGURATION OBJECT ====================
    
    public static ColorPositions colors = new ColorPositions();
    
    // ==================== NESTED CONFIG CLASS ====================
    
    public static class ColorPositions {
        // GoBilda RGB LED PWM positions
        // See: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
        public double green = 0.444;
        public double purple = 0.722;
        public double red = 0.167;
        public double blue = 0.611;
        public double yellow = 0.333;
        public double white = 0.889;
        public double off = 0.056;
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final CharacterStats stats;
    
    @IgnoreConfigurable
    private final Servo ledL;
    
    @IgnoreConfigurable
    private final Servo ledR;
    
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
        setColor(colors.green);
    }
    
    public void setPurple() {
        setColor(colors.purple);
    }
    
    public void setRed() {
        setColor(colors.red);
    }
    
    public void setBlue() {
        setColor(colors.blue);
    }
    
    public void setYellow() {
        setColor(colors.yellow);
    }
    
    public void setWhite() {
        setColor(colors.white);
    }
    
    public void turnOff() {
        setColor(colors.off);
    }
    
    /**
     * Set LEDs to alliance color (Green or Purple for DECODE)
     */
    public void setAllianceColor(boolean isGreen) {
        setColor(isGreen ? colors.green : colors.purple);
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
