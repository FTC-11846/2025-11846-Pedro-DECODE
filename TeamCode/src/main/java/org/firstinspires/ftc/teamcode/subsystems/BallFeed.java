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
 *
 * Supports 3 hardware configurations:
 * - TestBot: Single CRServo (continuous rotation)
 * - Robot 22154: Dual CRServos (independent feed wheels with reverse)
 * - Robot 11846: Dual Servos (position-controlled gates)
 *
 * State Machines:
 * - CRServo robots: FEEDING → REVERSING → IDLE
 * - Servo robots (gates): FEEDING (lower) → HOLDING (wait) → RETURNING (raise) → IDLE
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
        FEEDING,    // CRServo: forward motion / Servo: lowering gate
        HOLDING,    // Servo only: gate down, waiting for ball to pass
        REVERSING,  // CRServo: reversing to prevent double-feed
        RETURNING   // Servo only: raising gate back to idle position
    }

    // ==================== SUBSYSTEM FIELDS ====================

    @IgnoreConfigurable
    private final CharacterStats stats;

    @IgnoreConfigurable
    private final BallFeedMode mode;

    @IgnoreConfigurable
    private final Object motorL;  // CRServo or Servo

    @IgnoreConfigurable
    private final Object motorR;  // CRServo or Servo or null

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

    @IgnoreConfigurable
    private final double feedIdlePos;

    @IgnoreConfigurable
    private final double feedLeftSweep;

    @IgnoreConfigurable
    private final double feedRightSweep;

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
        this.feedIdlePos = stats.getFeedIdlePos();
        this.feedLeftSweep = stats.getFeedLeftSweep();
        this.feedRightSweep = stats.getFeedRightSweep();

        // Initialize hardware based on mode
        switch (mode) {
            case SINGLE_CRSERVO:
                // TestBot: Single CRServo only
                motorL = hardwareMap.get(CRServo.class, stats.getBallFeedMotorLName());
                motorR = null;
                break;

            case DUAL_CRSERVO_INDEPENDENT:
                // Robot 22154: Two CRServos for independent feed control
                motorL = hardwareMap.get(CRServo.class, stats.getBallFeedMotorLName());
                motorR = hardwareMap.get(CRServo.class, stats.getBallFeedMotorRName());
                ((CRServo) motorL).setDirection(CRServo.Direction.REVERSE);
                ((CRServo) motorR).setDirection(CRServo.Direction.FORWARD);
                break;

            case DUAL_SERVO_GATES:
                // Robot 11846: Two position servos as gates
                motorL = hardwareMap.get(Servo.class, stats.getBallFeedMotorLName());
                motorR = hardwareMap.get(Servo.class, stats.getBallFeedMotorRName());
                // Initialize to idle position
                ((Servo) motorL).setPosition(feedIdlePos);
                ((Servo) motorR).setPosition(feedIdlePos);
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
        updateLeftMotor();
        updateRightMotor();
    }

    /**
     * Must be called in loop() to handle state machine
     */
    public void periodic() {
        updateLeftStateMachine();
        updateRightStateMachine();
    }

    // ==================== GETTERS ====================

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

    // ==================== STATE MACHINE LOGIC ====================

    private void updateLeftStateMachine() {
        switch (leftState) {
            case FEEDING:
                // Check if feeding duration complete
                if (leftTimer.seconds() >= feedDuration) {
                    // Transition based on mode
                    if (mode == BallFeedMode.DUAL_SERVO_GATES && holdDuration > 0) {
                        // 11846: Move to HOLDING state
                        leftState = FeedState.HOLDING;
                        leftTimer.reset();
                    } else if (reverseDuration > 0) {
                        // 22154: Move to REVERSING state
                        leftState = FeedState.REVERSING;
                        leftTimer.reset();
                    } else {
                        // TestBot or no reverse: Go directly to IDLE
                        leftState = FeedState.IDLE;
                    }
                    updateLeftMotor();
                }
                break;

            case HOLDING:
                // Servo gate: Hold down for ball to pass through
                if (leftTimer.seconds() >= holdDuration) {
                    leftState = FeedState.RETURNING;
                    leftTimer.reset();
                    updateLeftMotor();
                }
                break;

            case REVERSING:
            case RETURNING:
                // Wait for reverse/return duration
                if (leftTimer.seconds() >= (mode == BallFeedMode.DUAL_SERVO_GATES ? reverseDuration : reverseDuration)) {
                    leftState = FeedState.IDLE;
                    updateLeftMotor();
                }
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    private void updateRightStateMachine() {
        if (motorR == null) return;

        switch (rightState) {
            case FEEDING:
                if (rightTimer.seconds() >= feedDuration) {
                    if (mode == BallFeedMode.DUAL_SERVO_GATES && holdDuration > 0) {
                        rightState = FeedState.HOLDING;
                        rightTimer.reset();
                    } else if (reverseDuration > 0) {
                        rightState = FeedState.REVERSING;
                        rightTimer.reset();
                    } else {
                        rightState = FeedState.IDLE;
                    }
                    updateRightMotor();
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
                if (rightTimer.seconds() >= (mode == BallFeedMode.DUAL_SERVO_GATES ? reverseDuration : reverseDuration)) {
                    rightState = FeedState.IDLE;
                    updateRightMotor();
                }
                break;

            case IDLE:
                break;
        }
    }

    // ==================== HARDWARE UPDATE METHODS ====================

    private void updateLeftMotor() {
        if (motorL == null) return;

        if (motorL instanceof CRServo) {
            // CRServo: Continuous rotation mode (power control)
            double power = calculateCRServoPower(leftState);
            // Apply TestBot direction fix
            if (stats.getShortName().equals("TB")) {
                power = -power;
            }
            ((CRServo) motorL).setPower(power);
        } else if (motorL instanceof Servo) {
            // Servo: Position mode (gate control)
            double position = calculateServoPosition(leftState, true); // true = left
            ((Servo) motorL).setPosition(position);
        }
    }

    private void updateRightMotor() {
        if (motorR == null) return;

        if (motorR instanceof CRServo) {
            // CRServo: Continuous rotation mode (power control)
            double power = calculateCRServoPower(rightState);
            // Apply TestBot direction fix
            if (stats.getShortName().equals("TB")) {
                power = -power;
            }
            ((CRServo) motorR).setPower(power);
        } else if (motorR instanceof Servo) {
            // Servo: Position mode (gate control)
            double position = calculateServoPosition(rightState, false); // false = right
            ((Servo) motorR).setPosition(position);
        }
    }

    /**
     * Calculate CRServo power based on state
     */
    private double calculateCRServoPower(FeedState state) {
        switch (state) {
            case FEEDING:
                return feedingControl.feedPower;
            case REVERSING:
                return feedingControl.reversePower;
            case HOLDING:
            case RETURNING:
            case IDLE:
            default:
                return 0.0;
        }
    }

    /**
     * Calculate Servo position based on state (for 11846 gates)
     * @param state Current feed state
     * @param isLeft True for left servo, false for right servo
     */
    private double calculateServoPosition(FeedState state, boolean isLeft) {
        double sweep = isLeft ? feedLeftSweep : feedRightSweep;

        switch (state) {
            case FEEDING:
            case HOLDING:
                // Gate down (feeding or holding for ball to pass)
                return feedIdlePos + sweep;
            case RETURNING:
            case IDLE:
            default:
                // Gate up (idle position)
                return feedIdlePos;
        }
    }
}