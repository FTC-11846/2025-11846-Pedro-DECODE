package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.robots.CharacterStats.BallFeedMode;

/**
 * Ball feeding subsystem with feed-and-reverse state machine
 * - 22154: Feed wheels forward → reverse to prevent double-feed
 * - 11846: Servos lower gates → hold for ball to pass → return to raised position
 * 
 * Independent left/right lane control via triggers
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

    // ==================== STATE TRACKING ====================

    private enum FeedState {
        IDLE,       // Not feeding
        FEEDING,    // Moving forward (both robots) or lowering gate (11846)
        HOLDING,    // 11846 only: gate down, waiting for ball to pass
        REVERSING,  // 22154 only: reversing to prevent double-feed / 11846: raising gate
        RETURNING   // Alias for REVERSING, used by 11846
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
    private final ElapsedTime leftTimer = new ElapsedTime();

    @IgnoreConfigurable
    private final ElapsedTime rightTimer = new ElapsedTime();

    @IgnoreConfigurable
    private FeedState leftState = FeedState.IDLE;

    @IgnoreConfigurable
    private FeedState rightState = FeedState.IDLE;

    @IgnoreConfigurable
    private final double feedDuration;

    @IgnoreConfigurable
    private final double reverseDuration;

    @IgnoreConfigurable
    private final double holdDuration;

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
        this.feedDuration = stats.getDefaultFeedDuration();
        this.reverseDuration = stats.getFeedReverseDuration();
        this.holdDuration = stats.getFeedHoldDuration();

        // Initialize motors based on mode
        switch (mode) {
            case SINGLE:
                // Only left motor
                motorL = hardwareMap.get(CRServo.class, stats.getBallFeedMotorLName());
//                motorL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorR = null;
                break;

            case DUAL_SYNCHRONIZED:
            case DUAL_INDEPENDENT:
                // Both motors
                motorL = hardwareMap.get(CRServo.class, stats.getBallFeedMotorLName());
                motorR = hardwareMap.get(CRServo.class, stats.getBallFeedMotorRName());
                motorL.setDirection(CRServo.Direction.REVERSE);
                motorR.setDirection(CRServo.Direction.FORWARD);
                break;

            default:
                throw new IllegalStateException("Unknown BallFeedMode: " + mode);
        }
    }

    // ==================== PUBLIC CONTROL METHODS ====================

    /**
     * Feed left lane (trigger by GP2 LT)
     * Starts state machine: FEEDING → (HOLDING) → REVERSING/RETURNING → IDLE
     */
    public void feedLeft() {
        if (leftState != FeedState.IDLE) return; // Prevent double-trigger

        leftState = FeedState.FEEDING;
        leftTimer.reset();
        updateLeftMotor();
    }

    /**
     * Feed right lane (trigger by GP2 RT)
     * Starts state machine: FEEDING → (HOLDING) → REVERSING/RETURNING → IDLE
     */
    public void feedRight() {
        if (motorR == null) return; // Single motor mode
        if (rightState != FeedState.IDLE) return; // Prevent double-trigger

        rightState = FeedState.FEEDING;
        rightTimer.reset();
        updateRightMotor();
    }

    /**
     * Legacy synchronized feed (backward compatibility)
     */
    public void startFeed() {
        feedLeft();
        if (motorR != null) {
            feedRight();
        }
    }

    /**
     * Manually stop feeding (emergency or manual control)
     */
    public void stopFeed() {
        leftState = FeedState.IDLE;
        rightState = FeedState.IDLE;
//        motorL.setPower(0);
//        if (motorR != null) {
//            motorR.setPower(0);
//        }
    }

    /**
     * Set custom feed power (-1.0 to 1.0) for manual control
     */
//    public void setFeedPower(double power) {
//        setMotorPosition(power, power);
//    }

    /**
     * Set independent powers for left and right motors
     */
//    public void setIndependentPowers(double leftPower, double rightPower) {
//        if (mode != BallFeedMode.DUAL_INDEPENDENT) {
//            setMotorPosition(leftPower, leftPower);
//        } else {
//            setMotorPosition(leftPower, rightPower);
//        }
//    }

    /**
     * Must be called in loop() to handle state machine
     */
    public void periodic() {
        updateLeftStateMachine();
        updateRightStateMachine();
    }

    // ==================== GETTERS ====================

    public boolean isFeeding() {
        return leftState != FeedState.IDLE || rightState != FeedState.IDLE;
    }

    public boolean isLeftFeeding() {
        return leftState != FeedState.IDLE;
    }

    public boolean isRightFeeding() {
        return rightState != FeedState.IDLE;
    }

    public String getLeftStateString() {
        return leftState.toString();
    }

    public String getRightStateString() {
        return rightState.toString();
    }

    public BallFeedMode getMode() {
        return mode;
    }

    public String getRobotName() {
        return stats.getDisplayName();
    }

    // ==================== PRIVATE STATE MACHINE ====================

    private void updateLeftStateMachine() {
        switch (leftState) {
            case IDLE:
                // Waiting for trigger
                break;

            case FEEDING:
                if (leftTimer.seconds() >= feedDuration) {
                    // Check if we need HOLDING or go straight to REVERSING
                    if (holdDuration > 0) {
                        leftState = FeedState.HOLDING;
                        leftTimer.reset();
                        updateLeftMotor(); // Stop motor during hold
                    } else if (reverseDuration > 0) {
                        leftState = FeedState.REVERSING;
                        leftTimer.reset();
                        updateLeftMotor(); // Start reverse
                    } else {
                        leftState = FeedState.IDLE;
                        updateLeftMotor(); // Stop
                    }
                }
                break;

            case HOLDING:
                // 11846: Gate is down, waiting for ball to pass
                if (leftTimer.seconds() >= holdDuration) {
                    leftState = FeedState.RETURNING;
                    leftTimer.reset();
                    updateLeftMotor(); // Start return
                }
                break;

            case REVERSING:
            case RETURNING:
                if (leftTimer.seconds() >= (reverseDuration > 0 ? reverseDuration : feedDuration)) {
                    leftState = FeedState.IDLE;
                    updateLeftMotor(); // Stop
                }
                break;
        }
    }

    private void updateRightStateMachine() {
        if (motorR == null) return;

        switch (rightState) {
            case IDLE:
                // Waiting for trigger
                break;

            case FEEDING:
                if (rightTimer.seconds() >= feedDuration) {
                    if (holdDuration > 0) {
                        rightState = FeedState.HOLDING;
                        rightTimer.reset();
                        updateRightMotor();
                    } else if (reverseDuration > 0) {
                        rightState = FeedState.REVERSING;
                        rightTimer.reset();
                        updateRightMotor();
                    } else {
                        rightState = FeedState.IDLE;
                        updateRightMotor();
                    }
                }
                break;

            case HOLDING:
                if (rightTimer.seconds() >= holdDuration) {
                    rightState = FeedState.RETURNING;
                    rightTimer.reset();
                    updateRightMotor();
                }
                break;

            case REVERSING:
            case RETURNING:
                if (rightTimer.seconds() >= (reverseDuration > 0 ? reverseDuration : feedDuration)) {
                    rightState = FeedState.IDLE;
                    updateRightMotor();
                }
                break;
        }
    }

    private void updateLeftMotor() {
        double power = 0.0;

        switch (leftState) {
            case FEEDING:
                power = 1.0;
//                power = feedingControl.feedPower * feedingControl.leftPowerMultiplier;
                break;
            case HOLDING:
                power = 0.0; // Stop during hold
                break;
            case REVERSING:
            case RETURNING:
                power = -1.0;
//                position = feedingControl.reversePower * feedingControl.leftPowerMultiplier;
                break;
            case IDLE:
                power = 0.0;
                break;
        }
        if(stats.getShortName() == "TB"){
            motorL.setPower(-power);
        }else{
            motorL.setPower(power);
        }
    }

    private void updateRightMotor() {
        if (motorR == null) return;

        double power = 0.0;

        switch (rightState) {
            case FEEDING:
                power = 1.0;
//                position = feedingControl.feedPower * feedingControl.rightPowerMultiplier;
                break;
            case HOLDING:
                power = 0.0;
                break;
            case REVERSING:
            case RETURNING:
                power = -1.0;
//                position = feedingControl.reversePower * feedingControl.rightPowerMultiplier;
                break;
            case IDLE:
                power = 0.0;
                break;
        }
        if(stats.getShortName() == "TB"){
            motorR.setPower(-power);
        } else {
            motorR.setPower(power);
        }
    }

//    private void setMotorPowers(double leftPower, double rightPower) {
//        // Apply multipliers if in independent mode
//        if (mode == BallFeedMode.DUAL_INDEPENDENT) {
//            leftPower *= feedingControl.leftPowerMultiplier;
//            rightPower *= feedingControl.rightPowerMultiplier;
//        }
//        private void setMotorPosition(double leftPosition, double rightPosition) {
//            // Apply multipliers if in independent mode
//            if (mode == BallFeedMode.DUAL_INDEPENDENT) {
//                leftPosition *= feedingControl.leftPowerMultiplier;
//                rightPosition *= feedingControl.rightPowerMultiplier;
//            }
//
//            motorL.setPosition(leftPosition);
//        if (motorR != null) {
//            motorR.setPosition(rightPosition);
//        }
//    }
}
