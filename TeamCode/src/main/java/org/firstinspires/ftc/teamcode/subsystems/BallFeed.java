package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Ball feeding subsystem - handles single, dual synchronized, or dual independent motors
 * Automatically adapts to robot configuration via MainCharacter
 */
public class BallFeed {
    private final MainCharacter character;
    private final MainCharacter.BallFeedMode mode;
    private final CRServo motorL;
    private final CRServo motorR; // null for single motor mode

    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean isFeeding = false;
    private double feedDurationSeconds = Constants.DEFAULT_FEED_DURATION;

    // ==================== TUNABLE CONSTANTS ====================

    @Configurable
    public static class Constants {
        public static double DEFAULT_FEED_DURATION = 0.25;  // seconds
        public static double FEED_POWER = 1.0;
        public static double REVERSE_POWER = -1.0;

        // For dual independent mode (future)
        public static double LEFT_POWER_MULTIPLIER = 1.0;
        public static double RIGHT_POWER_MULTIPLIER = 1.0;
    }

    // ==================== CONSTRUCTOR ====================

    public BallFeed(HardwareMap hardwareMap, MainCharacter character) {
        this.character = character;
        this.mode = character.getBallFeedMode();

        // Initialize motors based on mode
        switch (mode) {
            case SINGLE:
                // Only left motor
                motorL = hardwareMap.get(CRServo.class, character.getBallFeedMotorLName());
                motorL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorR = null;
                break;

            case DUAL_SYNCHRONIZED:
            case DUAL_INDEPENDENT:
                // Both motors
                motorL = hardwareMap.get(CRServo.class, character.getBallFeedMotorLName());
                motorR = hardwareMap.get(CRServo.class, character.getBallFeedMotorRName());
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
            setMotorPowers(Constants.FEED_POWER, Constants.FEED_POWER);
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
            setMotorPowers(Constants.REVERSE_POWER, Constants.REVERSE_POWER);
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
     * Only works in DUAL_INDEPENDENT mode
     */
    public void setIndependentPowers(double leftPower, double rightPower) {
        if (mode != MainCharacter.BallFeedMode.DUAL_INDEPENDENT) {
            throw new IllegalStateException("Independent power control only available in DUAL_INDEPENDENT mode");
        }
        setMotorPowers(leftPower, rightPower);
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

    public MainCharacter.BallFeedMode getMode() {
        return mode;
    }

    // ==================== PRIVATE HELPER METHODS ====================

    private void setMotorPowers(double leftPower, double rightPower) {
        // Apply multipliers if in independent mode
        if (mode == MainCharacter.BallFeedMode.DUAL_INDEPENDENT) {
            leftPower *= Constants.LEFT_POWER_MULTIPLIER;
            rightPower *= Constants.RIGHT_POWER_MULTIPLIER;
        }

        motorL.setPower(leftPower);
        if (motorR != null) {
            motorR.setPower(rightPower);
        }
    }
}