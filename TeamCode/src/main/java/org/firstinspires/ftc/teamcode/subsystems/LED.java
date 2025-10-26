package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;

/**
 * LED subsystem for RGB indicator lights (GoBilda PWM-controlled LEDs)
 * Displays ball lane colors and game state feedback
 */
@Configurable
public class LED {
    
    // ==================== CONFIGURATION OBJECT ====================
    
    public static LEDColors ledColors = new LEDColors();
    
    // ==================== NESTED CONFIG CLASS ====================
    
    public static class LEDColors {
        // GoBilda RGB LED PWM positions
        // See: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
        public double GREEN = 0.444;
        public double PURPLE = 0.722;
        public double RED = 0.167;
        public double BLUE = 0.611;
        public double YELLOW = 0.333;
        public double WHITE = 0.889;
        public double OFF = 0.056;
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final CharacterStats stats;
    
    @IgnoreConfigurable
    private final Servo servoL;
    
    @IgnoreConfigurable
    private final Servo servoR;
    
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
            throw new IllegalStateException(stats.getDisplayName() + " does not have LED hardware!");
        }
        
        servoL = hardwareMap.get(Servo.class, stats.getLEDServoLName());
        servoR = hardwareMap.get(Servo.class, stats.getLEDServoRName());
    }
    
    // ==================== BASIC COLOR METHODS ====================
    
    /**
     * Set both LEDs to green (success/ready)
     */
    public void setGreen() {
        servoL.setPosition(ledColors.GREEN);
        servoR.setPosition(ledColors.GREEN);
    }
    
    /**
     * Set both LEDs to purple (error/warning)
     */
    public void setPurple() {
        servoL.setPosition(ledColors.PURPLE);
        servoR.setPosition(ledColors.PURPLE);
    }
    
    /**
     * Set both LEDs to red
     */
    public void setRed() {
        servoL.setPosition(ledColors.RED);
        servoR.setPosition(ledColors.RED);
    }
    
    /**
     * Set both LEDs to blue
     */
    public void setBlue() {
        servoL.setPosition(ledColors.BLUE);
        servoR.setPosition(ledColors.BLUE);
    }
    
    /**
     * Set both LEDs to yellow
     */
    public void setYellow() {
        servoL.setPosition(ledColors.YELLOW);
        servoR.setPosition(ledColors.YELLOW);
    }
    
    /**
     * Turn off both LEDs
     */
    public void off() {
        servoL.setPosition(ledColors.OFF);
        servoR.setPosition(ledColors.OFF);
    }
    
    // ==================== LANE-SPECIFIC METHODS ====================
    
    /**
     * Show detected ball color in left lane
     */
    public void showLeftLaneColor(ColorSensors.BallColor color) {
        switch (color) {
            case RED:
                servoL.setPosition(ledColors.RED);
                break;
            case BLUE:
                servoL.setPosition(ledColors.BLUE);
                break;
            case YELLOW:
                servoL.setPosition(ledColors.YELLOW);
                break;
            case NONE:
                servoL.setPosition(ledColors.OFF);
                break;
        }
    }
    
    /**
     * Show detected ball color in right lane
     */
    public void showRightLaneColor(ColorSensors.BallColor color) {
        switch (color) {
            case RED:
                servoR.setPosition(ledColors.RED);
                break;
            case BLUE:
                servoR.setPosition(ledColors.BLUE);
                break;
            case YELLOW:
                servoR.setPosition(ledColors.YELLOW);
                break;
            case NONE:
                servoR.setPosition(ledColors.OFF);
                break;
        }
    }
    
    /**
     * Show that both lanes have correct colors (ready to shoot)
     */
    public void showBothLanesReady() {
        servoL.setPosition(ledColors.GREEN);
        servoR.setPosition(ledColors.GREEN);
    }
    
    /**
     * Show error state (wrong colors detected)
     */
    public void showError() {
        servoL.setPosition(ledColors.PURPLE);
        servoR.setPosition(ledColors.PURPLE);
    }
    
    // ==================== INDEPENDENT CONTROL ====================
    
    /**
     * Set left LED to specific color
     */
    public void setLeftColor(double position) {
        servoL.setPosition(position);
    }
    
    /**
     * Set right LED to specific color
     */
    public void setRightColor(double position) {
        servoR.setPosition(position);
    }
    
    // ==================== GETTERS ====================
    
    public String getRobotName() {
        return stats.getDisplayName();
    }
}
