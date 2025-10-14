package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Indexer subsystem for feeding balls into the shooter
 * Handles timed feeding operations
 */
public class BallFeedSubsystem {
    private final CRServo leftServo;
    private final CRServo rightServo;
    
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean isFeeding = false;
    private double feedDurationSeconds = 0.25;
    
    public BallFeedSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "feedServoL");
        rightServo = hardwareMap.get(CRServo.class, "feedServoR");
        
        // Set directions - left servo is reversed
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Start feeding a ball (runs for configured duration)
     */
    public void startFeed() {
        if (!isFeeding) {
            leftServo.setPower(1.0);
            rightServo.setPower(1.0);
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
     * Manually stop feeding (for emergency or manual control)
     */
    public void stopFeed() {
        leftServo.setPower(0);
        rightServo.setPower(0);
        isFeeding = false;
    }

    /**
     * Set custom feed power (-1.0 to 1.0)
     * Use this for manual control or reverse feeding
     */
    public void setFeedPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
        isFeeding = (power != 0);
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

    /**
     * Check if currently feeding
     */
    public boolean isFeeding() {
        return isFeeding;
    }

    /**
     * Get remaining feed time in seconds
     */
    public double getRemainingFeedTime() {
        if (!isFeeding) return 0;
        double remaining = feedDurationSeconds - feedTimer.seconds();
        return Math.max(0, remaining);
    }

    /**
     * Set the default feed duration
     */
    public void setFeedDuration(double seconds) {
        this.feedDurationSeconds = seconds;
    }

    /**
     * Get the configured feed duration
     */
    public double getFeedDuration() {
        return feedDurationSeconds;
    }
}
