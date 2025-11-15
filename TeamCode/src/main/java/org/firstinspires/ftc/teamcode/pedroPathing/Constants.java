package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MainCharacter;

/**
 * Constants for Pedro Pathing - Robot-Specific Configuration
 *
 * Each robot (TestBot, 22154, 11846) has its own set of:
 * - FollowerConstants (mass, accelerations, PIDF)
 * - MecanumConstants (motor names, directions, velocities)
 * - PinpointConstants (odometry configuration)
 */
public class Constants {


    // 🔧 TUNING: Change this to tune a different robot! 🔧
    //  (for Pedro Pathing Tuning OpMode, provides the 2nd parameter for createFollower()
    //////////////////////////////////  \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// =======================TODO: CHANGE THIS WHEN PUSHING CODE========================= //
    private static final MainCharacter TUNING_ROBOT = MainCharacter.ROBOT_11846;  // TEST_BOT, ROBOT_11846, ROBOT_22154
// =========================DID YOU CHANGE IT?!===============================
///  \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\  //////////////////////////////////////

    // Shared path constraints (can be made robot-specific if needed)
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

// ----------------------------------------------------------------------
// PUBLIC ALIASES FOR TUNING OPMODE
// These are assigned in the static initialization block below.
// ----------------------------------------------------------------------

    public static FollowerConstants followerConstants;
    public static MecanumConstants driveConstants;
    public static PinpointConstants localizerConstants;

// ----------------------------------------------------------------------
// ROBOT-SPECIFIC CONSTANTS DEFINITIONS (Keep these well-formatted for tuning)
// ----------------------------------------------------------------------

    // ==================== TEST BOT CONSTANTS ====================

    private static FollowerConstants testBotFollowerConstants = new FollowerConstants()
            .mass(9.53)   // kg - Must be updated after weighing (kg = Lbs / 2.2046)
            .forwardZeroPowerAcceleration(-45.86237723816607) // in/s^2
            .lateralZeroPowerAcceleration(-69.3200755294708) // in/s^2
//            .useSecondaryHeadingPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.375, 0, 0.025, 0));

    private static MecanumConstants testBotDriveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(42.68823290246679) // in/s
            .yVelocity(33.44927521953133); // in/s

    private static PinpointConstants testBotLocalizerConstants = new PinpointConstants()
            .forwardPodY(3) // inches
            .strafePodX(-6.4) // inches
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ==================== ROBOT 22154 CONSTANTS ====================

    private static FollowerConstants robot22154FollowerConstants = new FollowerConstants()
            .mass(15.88)
            .forwardZeroPowerAcceleration(-30.453093036274215)
            .lateralZeroPowerAcceleration(-75.86724421555218)
            //.useSecondaryHeadingPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.375, 0, 0.025, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0.01, 0, 0.01));

    private static MecanumConstants robot22154DriveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(53.64255938192052)
            .yVelocity(39.0105528343381);

    private static PinpointConstants robot22154LocalizerConstants = new PinpointConstants()
            .forwardPodY(3)
            .strafePodX(-7.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ==================== ROBOT 11846 CONSTANTS ====================

    private static FollowerConstants robot11846FollowerConstants = new FollowerConstants()
            .mass(12.70)
            .forwardZeroPowerAcceleration(-50.943)
            .lateralZeroPowerAcceleration(-63.789)
//            .useSecondaryHeadingPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.375, 0, 0.025, 0));

    private static MecanumConstants robot11846DriveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(41.356)
            .yVelocity(31.839);

    private static PinpointConstants robot11846LocalizerConstants = new PinpointConstants()
            .forwardPodY(2.875)
            .strafePodX(6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

// ----------------------------------------------------------------------
// INITIALIZATION AND HELPER METHODS
// ----------------------------------------------------------------------

    /**
     * Fixes the initial compile error by assigning public aliases
     * after all static private constants have been defined.
     */
    static {
        ConstantsSet defaultSet = getConstantsSet(TUNING_ROBOT);
        followerConstants = defaultSet.followerConsts;
        driveConstants = defaultSet.driveConsts;
        localizerConstants = defaultSet.localizerConsts;
    }

    /**
     * A simple container to group the three constants objects.
     */
    private static class ConstantsSet {
        final FollowerConstants followerConsts;
        final MecanumConstants driveConsts;
        final PinpointConstants localizerConsts;

        ConstantsSet(FollowerConstants f, MecanumConstants d, PinpointConstants l) {
            this.followerConsts = f;
            this.driveConsts = d;
            this.localizerConsts = l;
        }
    }

    /**
     * Internal method to select and return the correct constants set based on the robot.
     */
    private static ConstantsSet getConstantsSet(MainCharacter character) {
        switch (character) {
            case TEST_BOT:
                return new ConstantsSet(testBotFollowerConstants, testBotDriveConstants, testBotLocalizerConstants);
            case ROBOT_22154:
                return new ConstantsSet(robot22154FollowerConstants, robot22154DriveConstants, robot22154LocalizerConstants);
            case ROBOT_11846:
                return new ConstantsSet(robot11846FollowerConstants, robot11846DriveConstants, robot11846LocalizerConstants);
            default:
                throw new IllegalArgumentException("Unknown character: " + character);
        }
    }

// ----------------------------------------------------------------------
// FOLLOWER FACTORY - METHOD OVERLOADING
// ----------------------------------------------------------------------

    /**
     * **[VERSION 1 - For Tuning OpModes]**
     * Creates a Follower using the constants defined in the single, static TUNING_ROBOT switch.
     * This allows the Pedro Pathing Tuning OpMode (which expects one parameter) to work.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @return Configured Follower instance for the TUNING_ROBOT
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        // Calls the two-parameter version, forcing the use of the selected TUNING_ROBOT's constants.
        return createFollower(hardwareMap, TUNING_ROBOT);
    }

    /**
     * **[VERSION 2 - For Auto/TeleOp OpModes]**
     * Create a Follower instance with robot-specific constants selected at runtime.
     * This is the standard method for competitive OpModes.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param character Which robot we're running on
     * @return Configured Follower instance for the specified robot
     */
    public static Follower createFollower(HardwareMap hardwareMap, MainCharacter character) {
        ConstantsSet set = getConstantsSet(character);

        // Build and return follower with robot-specific constants
        return new FollowerBuilder(set.followerConsts, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(set.driveConsts)
                .pinpointLocalizer(set.localizerConsts)
                .build();
    }
}