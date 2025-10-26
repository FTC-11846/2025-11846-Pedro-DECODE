package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;

/**
 * ColorSensors subsystem - Detects ball colors in left/right lanes
 * Uses REV Color Sensor V3 to identify RED, BLUE, or YELLOW balls
 */
@Configurable
public class ColorSensors {
    
    // ==================== CONFIGURATION OBJECT ====================
    
    public static ColorThresholds colorThresholds = new ColorThresholds();
    
    // ==================== NESTED CONFIG CLASS ====================
    
    public static class ColorThresholds {
        // HSV hue thresholds (0-360 degrees)
        public double redHueMin = 0.0;
        public double redHueMax = 30.0;
        public double blueHueMin = 200.0;
        public double blueHueMax = 260.0;
        public double yellowHueMin = 45.0;
        public double yellowHueMax = 75.0;
        
        // Minimum saturation to consider color valid (0-1)
        public double minSaturation = 0.3;
        
        // Minimum value/brightness to consider color valid (0-1)
        public double minValue = 0.2;
    }
    
    // ==================== ENUMS ====================
    
    public enum BallColor {
        RED,
        BLUE,
        YELLOW,
        NONE
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final CharacterStats stats;
    
    @IgnoreConfigurable
    private final NormalizedColorSensor leftSensor;
    
    @IgnoreConfigurable
    private final NormalizedColorSensor rightSensor;
    
    // ==================== CONSTRUCTOR ====================
    
    /**
     * Create ColorSensors using MainCharacter enum (backward compatible)
     */
    public ColorSensors(HardwareMap hardwareMap, MainCharacter character) {
        this(hardwareMap, character.getAbilities());
    }
    
    /**
     * Create ColorSensors using CharacterStats directly (preferred)
     */
    public ColorSensors(HardwareMap hardwareMap, CharacterStats stats) {
        this.stats = stats;
        
        if (!stats.hasColorSensors()) {
            throw new IllegalStateException(stats.getDisplayName() + " does not have color sensors!");
        }
        
        leftSensor = hardwareMap.get(NormalizedColorSensor.class, stats.getLeftLaneColorSensorName());
        rightSensor = hardwareMap.get(NormalizedColorSensor.class, stats.getRightLaneColorSensorName());
    }
    
    // ==================== PUBLIC DETECTION METHODS ====================
    
    /**
     * Detect ball color in left lane
     */
    public BallColor detectLeftLane() {
        return detectColor(leftSensor);
    }
    
    /**
     * Detect ball color in right lane
     */
    public BallColor detectRightLane() {
        return detectColor(rightSensor);
    }
    
    /**
     * Get raw RGB values from left sensor (for tuning)
     */
    public NormalizedRGBA getLeftRGBA() {
        return leftSensor.getNormalizedColors();
    }
    
    /**
     * Get raw RGB values from right sensor (for tuning)
     */
    public NormalizedRGBA getRightRGBA() {
        return rightSensor.getNormalizedColors();
    }
    
    /**
     * Get HSV values from left sensor (for tuning)
     */
    public float[] getLeftHSV() {
        return rgbaToHSV(leftSensor.getNormalizedColors());
    }
    
    /**
     * Get HSV values from right sensor (for tuning)
     */
    public float[] getRightHSV() {
        return rgbaToHSV(rightSensor.getNormalizedColors());
    }
    
    // ==================== GETTERS ====================
    
    public String getRobotName() {
        return stats.getDisplayName();
    }
    
    // ==================== PRIVATE DETECTION LOGIC ====================
    
    /**
     * Detect ball color from a sensor
     * Uses HSV color space for more reliable detection
     */
    private BallColor detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] hsv = rgbaToHSV(colors);
        
        float hue = hsv[0];         // 0-360
        float saturation = hsv[1];  // 0-1
        float value = hsv[2];       // 0-1
        
        // Check if color is valid (not too dark or too washed out)
        if (saturation < colorThresholds.minSaturation || value < colorThresholds.minValue) {
            return BallColor.NONE;
        }
        
        // Check hue ranges
        if (isInRange(hue, colorThresholds.redHueMin, colorThresholds.redHueMax)) {
            return BallColor.RED;
        } else if (isInRange(hue, colorThresholds.blueHueMin, colorThresholds.blueHueMax)) {
            return BallColor.BLUE;
        } else if (isInRange(hue, colorThresholds.yellowHueMin, colorThresholds.yellowHueMax)) {
            return BallColor.YELLOW;
        }
        
        return BallColor.NONE;
    }
    
    /**
     * Convert RGBA to HSV color space
     */
    private float[] rgbaToHSV(NormalizedRGBA rgba) {
        float[] hsv = new float[3];
        
        // Convert to 0-255 range for Android Color API
        int red = (int) (rgba.red * 255);
        int green = (int) (rgba.green * 255);
        int blue = (int) (rgba.blue * 255);
        
        Color.RGBToHSV(red, green, blue, hsv);
        
        return hsv;
    }
    
    /**
     * Check if value is in range (handles wrap-around for red hue)
     */
    private boolean isInRange(float value, double min, double max) {
        // Handle red wrap-around (350-360 and 0-10)
        if (min > max) {
            return value >= min || value <= max;
        }
        return value >= min && value <= max;
    }
}
