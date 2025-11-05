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
        //
//        public double redHueMin = 0.0;
//        public double redHueMax = 360.0;
//        public double blueHueMin = 0.0;
//        public double blueHueMax = 360.0;
//        public double yellowHueMin = 0.0;
//        public double yellowHueMax = 360.0;
        
//        // Minimum saturation to consider color valid (0-1)
//        public double minSaturation = 0.3;
//
//        // Minimum value/brightness to consider color valid (0-1)
//        public double minValue = 0.2;
    }
    
    // ==================== ENUMS ====================
    
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final CharacterStats stats;
    
    @IgnoreConfigurable
    private final NormalizedColorSensor frontLeftSensor;
    
    @IgnoreConfigurable
    private final NormalizedColorSensor frontRightSensor;

    @IgnoreConfigurable
    private final NormalizedColorSensor backLeftSensor;

    @IgnoreConfigurable
    private final NormalizedColorSensor backRightSensor;

    @IgnoreConfigurable
    public static float flRGBValueR;

    @IgnoreConfigurable
    public static float flRGBValueG;

    @IgnoreConfigurable
    public static float flRGBValueB;

    @IgnoreConfigurable
    public static BallColor flBallColor;

    @IgnoreConfigurable
    public static BallColor frBallColor;

    @IgnoreConfigurable
    public static BallColor blBallColor;

    @IgnoreConfigurable
    public static BallColor brBallColor;

    @IgnoreConfigurable
    public static int ballCount;

    @IgnoreConfigurable
    public static int frRGBValue;

    @IgnoreConfigurable
    public static int blRGBValue;

    @IgnoreConfigurable
    public static int brRGBValue;


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
        
        frontLeftSensor = hardwareMap.get(NormalizedColorSensor.class, stats.getFrontLeftLaneColorSensorName());
        frontRightSensor = hardwareMap.get(NormalizedColorSensor.class, stats.getFrontRightLaneColorSensorName());
        backLeftSensor = hardwareMap.get(NormalizedColorSensor.class, stats.getBackLeftLaneColorSensorName());
        backRightSensor = hardwareMap.get(NormalizedColorSensor.class, stats.getBackRightLaneColorSensorName());

        flRGBValueR = 0;
        flRGBValueG = 0;
        flRGBValueB = 0;
        flBallColor = BallColor.NONE;
        frRGBValue = 0;
        frBallColor = BallColor.NONE;
        blRGBValue = 0;
        blBallColor = BallColor.NONE;
        brRGBValue = 0;
        brBallColor = BallColor.NONE;
        ballCount = 0;
    }
    
    // ==================== PUBLIC DETECTION METHODS ====================
    
    /**
     * Detect ball color in the front of the left lane
     */
    public BallColor detectFrontLeftLane() {
        return detectColor(frontLeftSensor);
    }
    
    /**
     * Detect ball color in the front of right lane
     */
    public BallColor detectFrontRightLane() {
        return detectColor(frontRightSensor);
    }

    /**
     * Detect ball color in the back of the left lane
     */
    public BallColor detectBackLeftLane() {
        return detectColor(backLeftSensor);
    }

    /**
     * Detect ball color in the back of right lane
     */
    public BallColor detectBackRightLane() {
        return detectColor(backRightSensor);
    }

    /**
     * Get raw RGB values from front left sensor (for tuning)
     */
    public NormalizedRGBA getFrontLeftRGBA() {
        return frontLeftSensor.getNormalizedColors();
    }
    
    /**
     * Get raw RGB values from front right sensor (for tuning)
     */
    public NormalizedRGBA getFrontRightRGBA() {
        return frontRightSensor.getNormalizedColors();
    }

    /**
     * Get raw RGB values from back left sensor (for tuning)
     */
    public NormalizedRGBA getBackLeftRGBA() {
        return backLeftSensor.getNormalizedColors();
    }

    /**
     * Get raw RGB values from back right sensor (for tuning)
     */
    public NormalizedRGBA getBackRightRGBA() {
        return backRightSensor.getNormalizedColors();
    }

    /**
     * Get HSV values from front left sensor (for tuning)
     */
    public float[] getFrontLeftHSV() {
        return rgbaToHSV(frontLeftSensor.getNormalizedColors());
    }
    
    /**
     * Get HSV values from front right sensor (for tuning)
     */
    public float[] getFrontRightHSV() {
        return rgbaToHSV(frontRightSensor.getNormalizedColors());
    }

    /**
     * Get HSV values from back left sensor (for tuning)
     */
    public float[] getBackLeftHSV() {
        return rgbaToHSV(backLeftSensor.getNormalizedColors());
    }

    /**
     * Get HSV values from back right sensor (for tuning)
     */
    public float[] getBackRightHSV() {
        return rgbaToHSV(backRightSensor.getNormalizedColors());
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
        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;
        
        // Check hue ranges
        //TODO FIX THIS USING ACTUAL COLOR THRESHOLDS FOR THE BALLS
        if ((blue > (1.5 * red) && green < (red * 1.5))){
            return BallColor.PURPLE;
        } else if ((blue > (red * 1.75) && green > (red * 1.8))) {
            return BallColor.GREEN;
        } else {
            return BallColor.NONE;
        }
    }

    private int getBallCount(){
        int flBall = flBallColor == BallColor.NONE ? 0 : 1;
        int frBall = frBallColor == BallColor.NONE ? 0 : 1;
        int blBall = blBallColor == BallColor.NONE ? 0 : 1;
        int brBall = brBallColor == BallColor.NONE ? 0 : 1;
        return flBall + frBall + blBall + brBall;
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

    public void periodic(){
        flRGBValueR = frontLeftSensor.getNormalizedColors().red * 255;
        flRGBValueG = frontLeftSensor.getNormalizedColors().green * 255;
        flRGBValueB = frontLeftSensor.getNormalizedColors().blue * 255;
        flBallColor = detectColor(frontLeftSensor);
        frRGBValue = frontRightSensor.getNormalizedColors().toColor();
        frBallColor = detectColor(frontRightSensor);
        blRGBValue = backLeftSensor.getNormalizedColors().toColor();
        blBallColor = detectColor(backLeftSensor);
        brRGBValue = backRightSensor.getNormalizedColors().toColor();
        brBallColor = detectColor(backRightSensor);
        ballCount = getBallCount();
    }
}
