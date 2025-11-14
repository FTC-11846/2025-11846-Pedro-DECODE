package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;
import org.firstinspires.ftc.teamcode.robots.CharacterStats.EndgameSystem;

/**
 * Endgame subsystem to control either the lift system or the folding motor
 *
 * Supports 3 hardware configurations:
 * - TestBot: No Endgame
 * - Robot 22154: Dual Motors for lift
 * - Robot 11846: Single Motor for the folding bot
 */
@Configurable
public class Endgame {

    // ==================== CONFIGURATION OBJECT ====================

    public static EndgameSpeeds endgameSpeeds = new EndgameSpeeds();

    // ==================== NESTED CONFIG CLASS ====================

    public static class EndgameSpeeds {
        public double liftPower = 1.0;
        public double holdPower = 0.0;
        public double lowerPower = -1.0;
        public double foldInPower = 0.7;
        public double foldOutPower = -0.7;
    }

    // ==================== STATE TRACKING ====================

    // ==================== SUBSYSTEM FIELDS ====================

    @IgnoreConfigurable
    private final CharacterStats stats;

    @IgnoreConfigurable
    private final EndgameSystem endgameSystem;

    @IgnoreConfigurable
    private final DcMotor motorL;  // Motor or null

    @IgnoreConfigurable
    private final DcMotor motorR;  // Motor or null

    // ==================== CONSTRUCTOR ====================

    /**
     * Create BallFeed using MainCharacter enum (backward compatible)
     */
    public Endgame(HardwareMap hardwareMap, MainCharacter character) {
        this(hardwareMap, character.getAbilities());
    }

    /**
     * Create BallFeed using CharacterStats directly (preferred)
     */
    public Endgame(HardwareMap hardwareMap, CharacterStats stats) {
        this.stats = stats;
        this.endgameSystem = stats.endgameMechanism();

        //TODO Change the motors to run with encoders once actual positions are found
        switch (endgameSystem) {
            case FOLD:
                motorL = hardwareMap.get(DcMotor.class, stats.getFoldMotorName());
                motorR = null;
                break;
            case LIFT:
                motorL = hardwareMap.get(DcMotor.class, stats.getLifterMotorLName());
                motorL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorR = hardwareMap.get(DcMotor.class, stats.getLifterMotorRName());
                break;
            case NONE:
                motorL = null;
                motorR = null;
                break;
            default:
                throw new IllegalStateException("Unknown EndgameSystem: " + endgameSystem);
        }
    }

    // ==================== PUBLIC CONTROL METHODS ====================

    /**
     * Run endgame function (lift or fold)
     */
    public void runEndgameFunction() {
        if (motorR != null){ //Lift
            motorL.setPower(endgameSpeeds.liftPower);
            motorR.setPower(endgameSpeeds.liftPower);
        } else if (motorL != null && motorR == null){ //Fold
            motorL.setPower(endgameSpeeds.foldInPower);
        }
    }

    /**
     * Run hold function for lift
     */
    public void hold() {
        if (motorR != null){ //Lift
            motorL.setPower(endgameSpeeds.holdPower);
            motorR.setPower(endgameSpeeds.holdPower);
        }
    }

    /**
     * Do the inverse of the endgame function (lower lift or unfold robot)
     */
    public void reverseEndgameFunction() {
        if (motorR != null){
            motorL.setPower(endgameSpeeds.lowerPower);
            motorR.setPower(endgameSpeeds.lowerPower);
        } else if(motorL != null && motorR == null){
            motorL.setPower(endgameSpeeds.foldOutPower);
        }
    }

    public String getRobotName() {
        return stats.getShortName();
    }

}