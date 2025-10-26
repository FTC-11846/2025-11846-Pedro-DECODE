package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.robots.CharacterStats.IntakeMode;

/**
 * Intake subsystem - Handles ball intake with mode-based configuration
 * Supports single/dual motors with toggle or continuous control
 */
@Configurable
public class Intake {
    
    // ==================== CONFIGURATION OBJECT ====================
    
    public static IntakeControl intakeControl = new IntakeControl();
    
    // ==================== NESTED CONFIG CLASS ====================
    
    public static class IntakeControl {
        public double intakeOnePower = 1.0;
        public double intakeTwoPower = 1.0;
        public double reverseMultiplier = -1.0;
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final CharacterStats stats;
    
    @IgnoreConfigurable
    private final IntakeMode mode;
    
    @IgnoreConfigurable
    private Object intakeOne;   // CRServo or DcMotorEx
    
    @IgnoreConfigurable
    private Object intakeTwo;   // CRServo or null
    
    @IgnoreConfigurable
    private boolean intakeOneState = false;
    
    @IgnoreConfigurable
    private boolean intakeTwoState = false;
    
    // ==================== CONSTRUCTOR ====================
    
    /**
     * Create Intake using MainCharacter enum (backward compatible)
     */
    public Intake(HardwareMap hardwareMap, MainCharacter character) {
        this(hardwareMap, character.getAbilities());
    }
    
    /**
     * Create Intake using CharacterStats directly (preferred)
     */
    public Intake(HardwareMap hardwareMap, CharacterStats stats) {
        this.stats = stats;
        this.mode = stats.getIntakeMode();
        
        // Initialize hardware based on mode
        switch (mode) {
            case NONE:
                intakeOne = null;
                intakeTwo = null;
                break;
            
            case SINGLE_TOGGLE:
            case SINGLE_CONTINUOUS:
                // Single DCMotorEx
                intakeOne = hardwareMap.get(DcMotorEx.class, stats.getIntakeOneMotorName());
                if(stats.getShortName() == "TB"){
                    ((DcMotorEx) intakeOne).setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    ((DcMotorEx) intakeOne).setDirection(DcMotorSimple.Direction.FORWARD);
                }
                ((DcMotorEx) intakeOne).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeTwo = null;
                break;
            
            case DUAL_INDEPENDENT_TOGGLE:
                // Two CRServos
                intakeOne = hardwareMap.get(CRServo.class, stats.getIntakeOneMotorName());
                intakeTwo = hardwareMap.get(CRServo.class, stats.getIntakeTwoMotorName());
                ((CRServo) intakeOne).setDirection(DcMotorSimple.Direction.FORWARD);
                ((CRServo) intakeTwo).setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            
            default:
                throw new IllegalStateException("Unknown IntakeMode: " + mode);
        }
    }
    
    // ==================== PUBLIC CONTROL METHODS ====================
    
    /**
     * Toggle intake one on/off (for button release detection)
     */
    public void toggleIntakeOne() {
        intakeOneState = !intakeOneState;
        applyIntakeOnePower();
    }
    
    /**
     * Toggle intake two on/off (for button release detection)
     * Only works in DUAL_INDEPENDENT_TOGGLE mode
     */
    public void toggleIntakeTwo() {
        if (mode != IntakeMode.DUAL_INDEPENDENT_TOGGLE) return;
        intakeTwoState = !intakeTwoState;
        applyIntakeTwoPower();
    }
    
    /**
     * Start intake one (for continuous mode)
     */
    public void startIntakeOne() {
        intakeOneState = true;
        applyIntakeOnePower();
    }
    
    /**
     * Stop intake one (for continuous mode)
     */
    public void stopIntakeOne() {
        intakeOneState = false;
        applyIntakeOnePower();
    }
    
    /**
     * Start intake two (for continuous mode)
     */
    public void startIntakeTwo() {
        if (mode != IntakeMode.DUAL_INDEPENDENT_TOGGLE) return;
        intakeTwoState = true;
        applyIntakeTwoPower();
    }
    
    /**
     * Stop intake two (for continuous mode)
     */
    public void stopIntakeTwo() {
        if (mode != IntakeMode.DUAL_INDEPENDENT_TOGGLE) return;
        intakeTwoState = false;
        applyIntakeTwoPower();
    }
    
    /**
     * Reverse intake one (for unjamming)
     */
    public void reverseIntakeOne() {
        if (intakeOne == null) return;
        
        double reversePower = intakeControl.intakeOnePower * intakeControl.reverseMultiplier;
        
        if (intakeOne instanceof DcMotorEx) {
            ((DcMotorEx) intakeOne).setPower(reversePower);
        } else if (intakeOne instanceof CRServo) {
            ((CRServo) intakeOne).setPower(reversePower);
        }
        
        intakeOneState = false; // Mark as off since we're reversing temporarily
    }
    
    /**
     * Reverse intake two (for unjamming)
     */
    public void reverseIntakeTwo() {
        if (intakeTwo == null) return;
        
        double reversePower = intakeControl.intakeTwoPower * intakeControl.reverseMultiplier;
        ((CRServo) intakeTwo).setPower(reversePower);
        
        intakeTwoState = false;
    }
    
    /**
     * Stop all intake motors immediately
     */
    public void stopAll() {
        intakeOneState = false;
        intakeTwoState = false;
        applyIntakeOnePower();
        applyIntakeTwoPower();
    }
    
    // ==================== GETTERS ====================
    
    public boolean isIntakeOneRunning() {
        return intakeOneState;
    }
    
    public boolean isIntakeTwoRunning() {
        return intakeTwoState;
    }
    
    public IntakeMode getMode() {
        return mode;
    }
    
    public String getRobotName() {
        return stats.getDisplayName();
    }
    
    // ==================== PRIVATE HELPER METHODS ====================
    
    private void applyIntakeOnePower() {
        if (intakeOne == null) return;
        
        double power = intakeOneState ? intakeControl.intakeOnePower : 0.0;
        
        if (intakeOne instanceof DcMotorEx) {
            ((DcMotorEx) intakeOne).setPower(power);
        } else if (intakeOne instanceof CRServo) {
            ((CRServo) intakeOne).setPower(power);
        }
    }
    
    private void applyIntakeTwoPower() {
        if (intakeTwo == null) return;
        
        double power = intakeTwoState ? intakeControl.intakeTwoPower : 0.0;
        ((CRServo) intakeTwo).setPower(power);
    }
}
