package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.robots.CharacterStats.BallFeedMode;

/**
 * Ball feeding subsystem with universal configuration constants
 * Robot-specific tuning values come from CharacterStats (populated via applyConfiguration)
 */
@Configurable
public class BallFeed {
    
    // ==================== CONFIGURATION OBJECT ====================
    
    public static FeedingControl feedingControl = new FeedingControl();
    
    // ==================== NESTED CONFIG CLASS ====================
    
    public static class FeedingControl {
        public double feedPower = 1.0;
        public double reversePower = -1.0;
        public double leftPowerMultiplier = 1.0;
        public double rightPowerMultiplier = 1.0;
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final CharacterStats stats;
    
    @IgnoreConfigurable
    private final BallFeedMode mode;
    
    @IgnoreConfigurable
    private final CRServo motorL;
    
    @IgnoreConfigurable
    private final CRServo motorR; // null for single motor mode
    
    @IgnoreConfigurable
    private final ElapsedTime feedTimer = new ElapsedTime();
    
    @IgnoreConfigurable
    private boolean isFeeding = false;
    
    @IgnoreConfigurable
    private double feedDurationSeconds;
    
    // ==================== CONSTRUCTOR ====================
    
    /**
     * Create BallFeed using MainCharacter enum (backward compatible)
     */
    public BallFeed(HardwareMap hardwareMap, MainCharacter character) {
        this(hardwareMap, character.getAbilities());
    }
    
    /**
     * Create BallFeed using CharacterStats directly (preferred)
     */
    public BallFeed(HardwareMap hardwareMap, CharacterStats stats) {
        this.stats = stats;
        this.mode = stats.getBallFeedMode();
        this.feedDurationSeconds = stats.getDefaultFeedDuration();
        
        // Initialize motors based on mode
        switch (mode) {
            case SINGLE:
                // Only left motor
                motorL = hardwareMap.get(CRServo.class, stats.getBallFeedMotorLName());
                motorL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorR = null;
                break;
            
            case DUAL_SYNCHRONIZED:
            case DUAL_INDEPENDENT:
                // Both motors
                motorL = hardwareMap.get(CRServo.class, stats.getBallFeedMotorLName());
                motorR = hardwareMap.get(CRServo.class, stats.getBallFeedMotorRName());
                motorL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorR.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            
            default:
                throw new IllegalStateException("Unknown BallFeedMode: " + mode);
        }
    }
    
    // ==================== PUBLIC CONTROL METHODS ====================
    
    /**
     * Start feeding a ball (runs for configured duration)
     */
    public void startFeed() {
        if (!isFeeding) {
            setMotorPowers(feedingControl.feedPower, feedingControl.feedPower);
            feedTimer.reset();
            isFeeding = true;
        }
    }
    
    /**
     * Start feeding with custom duration
     */
    public void startFeed(double durationSeconds) {
        this.feedDurationSeconds = durationSeconds;
        startFeed();
    }
    
    /**
     * Reverse feed (e.g., to unjam)
     */
    public void startReverse() {
        startReverse(feedDurationSeconds);
    }
    
    /**
     * Reverse feed with custom duration
     */
    public void startReverse(double durationSeconds) {
        if (!isFeeding) {
            setMotorPowers(feedingControl.reversePower, feedingControl.reversePower);
            this.feedDurationSeconds = durationSeconds;
            feedTimer.reset();
            isFeeding = true;
        }
    }
    
    /**
     * Manually stop feeding (for emergency or manual control)
     */
    public void stopFeed() {
        setMotorPowers(0, 0);
        isFeeding = false;
    }
    
    /**
     * Set custom feed power (-1.0 to 1.0)
     * Use this for manual control or custom feed speeds
     */
    public void setFeedPower(double power) {
        setMotorPowers(power, power);
        isFeeding = (power != 0);
    }
    
    /**
     * Set independent powers for left and right motors
     * Works in all modes (synchronized mode just uses same power for both)
     */
    public void setIndependentPowers(double leftPower, double rightPower) {
        if (mode != BallFeedMode.DUAL_INDEPENDENT) {
            // Just apply synchronized power for non-independent modes
            setMotorPowers(leftPower, leftPower);
        } else {
            // Apply independent powers for DUAL_INDEPENDENT
            setMotorPowers(leftPower, rightPower);
        }
        isFeeding = (leftPower != 0 || rightPower != 0);
    }
    
    /**
     * Must be called in loop() to handle timed feeding
     * Automatically stops feeding after duration expires
     */
    public void periodic() {
        if (isFeeding && feedTimer.seconds() >= feedDurationSeconds) {
            stopFeed();
        }
    }
    
    // ==================== GETTERS ====================
    
    public boolean isFeeding() {
        return isFeeding;
    }
    
    public double getRemainingFeedTime() {
        if (!isFeeding) return 0;
        double remaining = feedDurationSeconds - feedTimer.seconds();
        return Math.max(0, remaining);
    }
    
    public void setFeedDuration(double seconds) {
        this.feedDurationSeconds = seconds;
    }
    
    public double getFeedDuration() {
        return feedDurationSeconds;
    }
    
    public BallFeedMode getMode() {
        return mode;
    }
    
    public String getRobotName() {
        return stats.getDisplayName();
    }
    
    // ==================== PRIVATE HELPER METHODS ====================
    
    private void setMotorPowers(double leftPower, double rightPower) {
        // Apply multipliers if in independent mode
        if (mode == BallFeedMode.DUAL_INDEPENDENT) {
            leftPower *= feedingControl.leftPowerMultiplier;
            rightPower *= feedingControl.rightPowerMultiplier;
        }
        
        motorL.setPower(leftPower);
        if (motorR != null) {
            motorR.setPower(rightPower);
        }
    }
}
