package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Shooter subsystem - handles both single and dual motor configurations
 * Automatically adapts to different hardware via MainCharacter config
 */
public class Shooter {
    private final MainCharacter character;
    private final DcMotorEx shooterMotorL;
    private final DcMotorEx shooterMotorR; // null for single motor robots
    private final VoltageSensor batteryVoltageSensor;

    private double targetVelocityRPM = 0;
    private boolean autoAimActive = false;
    private double lastDetectedDistance = 0;
    private int lastDetectedTagId = -1;

    // ==================== TUNABLE CONSTANTS ====================

    @Configurable
    public static class Constants {
        // Velocity presets (RPM)
        public static double LOW_VELOCITY_RPM = 1500;
        public static double HIGH_VELOCITY_RPM_TEST = 5500;
        public static double HIGH_VELOCITY_RPM_22154 = 4000;
        public static double HIGH_VELOCITY_RPM_11846 = 5500;

        // Power limits
        public static double MIN_POWER_RPM = 1500;
        public static double MAX_POWER_RPM = 5500;

        // Auto-aim ballistics (distance-based velocity calculation)
        public static double BASELINE_POWER_TEST = 395.0;
        public static double BASELINE_POWER_22154 = 2000.0;
        public static double BASELINE_POWER_11846 = 395.0;
        public static double LINEAR_CORRECTION_FACTOR = 18.0; // RPM per inch

        // PIDF Tuning
        public static double PIDF_P_TEST = 10.0;
        public static double PIDF_P_22154 = 20.0;
        public static double PIDF_P_11846 = 10.0;
        public static double PIDF_I = 0.0;
        public static double PIDF_D = 0.0;

        // Voltage compensation
        public static double NOMINAL_VOLTAGE = 13.0;
        public static double MAX_TICKS_PER_SEC = 3000.0;

        // Debug mode
        public static boolean DEBUG_RUN_WITHOUT_ENCODER = false;
        public static double DEBUG_POWER = 1.0;
    }

    private static final double TICKS_TO_RPM = 28.0 / 60.0;

    // ==================== CONSTRUCTOR ====================

    public Shooter(HardwareMap hardwareMap, MainCharacter character) {
        this.character = character;
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize left/primary motor (all robots have this)
        shooterMotorL = hardwareMap.get(DcMotorEx.class, character.getShooterMotorLName());
        shooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        if (Constants.DEBUG_RUN_WITHOUT_ENCODER) {
            shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            shooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Initialize right motor only if robot has dual shooters
        if (character.hasDualShooters()) {
            shooterMotorR = hardwareMap.get(DcMotorEx.class, character.getShooterMotorRName());
            shooterMotorR.setDirection(DcMotorSimple.Direction.FORWARD);

            if (Constants.DEBUG_RUN_WITHOUT_ENCODER) {
                shooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                shooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else {
            shooterMotorR = null;
        }
    }

    // ==================== PUBLIC CONTROL METHODS ====================

    /**
     * Set shooter to low velocity (configured per robot)
     */
    public void setLowVelocity() {
        targetVelocityRPM = Constants.LOW_VELOCITY_RPM;
        autoAimActive = false;
    }

    /**
     * Set shooter to high velocity (configured per robot)
     */
    public void setHighVelocity() {
        switch (character) {
            case ROBOT_22154:
                targetVelocityRPM = Constants.HIGH_VELOCITY_RPM_22154;
                break;
            case ROBOT_11846:
                targetVelocityRPM = Constants.HIGH_VELOCITY_RPM_11846;
                break;
            case TEST_BOT:
            default:
                targetVelocityRPM = Constants.HIGH_VELOCITY_RPM_TEST;
                break;
        }
        autoAimActive = false;
    }

    /**
     * Set shooter to specific RPM
     */
    public void setVelocityRPM(double rpm) {
        targetVelocityRPM = clampVelocity(rpm);
        autoAimActive = false;
    }

    /**
     * Stop the shooter
     */
    public void stop() {
        targetVelocityRPM = 0;
        autoAimActive = false;
    }

    /**
     * Calculate and set velocity based on distance to target
     * Uses robot-specific ballistic constants
     */
    public void setAutoAimVelocity(double distanceInches, int tagId) {
        double baseline = getBaselinePower();
        double calculatedRPM = baseline + (distanceInches * Constants.LINEAR_CORRECTION_FACTOR);

        targetVelocityRPM = clampVelocity(calculatedRPM);
        autoAimActive = true;
        lastDetectedDistance = distanceInches;
        lastDetectedTagId = tagId;
    }

    /**
     * Must be called in loop() to apply velocity control with PIDF
     */
    public void periodic() {
        if (Constants.DEBUG_RUN_WITHOUT_ENCODER) {
            // Debug mode - run at fixed power
            shooterMotorL.setPower(Constants.DEBUG_POWER);
            if (shooterMotorR != null) {
                shooterMotorR.setPower(Constants.DEBUG_POWER);
            }
            return;
        }

        // Normal velocity control with voltage-scaled PIDF
        double voltage = batteryVoltageSensor.getVoltage();
        double scaledF = calculateScaledF(voltage);
        double p = getPIDFP();

        // Apply PIDF coefficients
        shooterMotorL.setVelocityPIDFCoefficients(p, Constants.PIDF_I, Constants.PIDF_D, scaledF);

        // Convert RPM to ticks per second and apply
        double targetTicksPerSec = targetVelocityRPM * TICKS_TO_RPM;
        shooterMotorL.setVelocity(targetTicksPerSec);

        // Apply to second motor if it exists
        if (shooterMotorR != null) {
            shooterMotorR.setVelocityPIDFCoefficients(p, Constants.PIDF_I, Constants.PIDF_D, scaledF);
            shooterMotorR.setVelocity(targetTicksPerSec);
        }
    }

    // ==================== GETTERS (FOR TELEMETRY) ====================

    /**
     * Get current actual velocity in RPM (average if dual motors)
     */
    public double getActualVelocityRPM() {
        double velocityL = shooterMotorL.getVelocity() / TICKS_TO_RPM;

        if (shooterMotorR != null) {
            double velocityR = shooterMotorR.getVelocity() / TICKS_TO_RPM;
            return (velocityL + velocityR) / 2.0;
        }

        return velocityL;
    }

    public double getTargetVelocityRPM() {
        return targetVelocityRPM;
    }

    public boolean isAutoAimActive() {
        return autoAimActive;
    }

    public double getLastDetectedDistance() {
        return lastDetectedDistance;
    }

    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }

    // ==================== EMERGENCY STOP ====================

    /**
     * Emergency stop - bypasses normal control
     */
    public void emergencyStop() {
        shooterMotorL.setPower(0);
        if (shooterMotorR != null) {
            shooterMotorR.setPower(0);
        }
        targetVelocityRPM = 0;
        autoAimActive = false;
    }

    // ==================== PRIVATE HELPER METHODS ====================

    private double clampVelocity(double rpm) {
        return Math.max(Constants.MIN_POWER_RPM, Math.min(Constants.MAX_POWER_RPM, rpm));
    }

    private double getBaselinePower() {
        switch (character) {
            case ROBOT_22154: return Constants.BASELINE_POWER_22154;
            case ROBOT_11846: return Constants.BASELINE_POWER_11846;
            case TEST_BOT:
            default: return Constants.BASELINE_POWER_TEST;
        }
    }

    private double getPIDFP() {
        switch (character) {
            case ROBOT_22154: return Constants.PIDF_P_22154;
            case ROBOT_11846: return Constants.PIDF_P_11846;
            case TEST_BOT:
            default: return Constants.PIDF_P_TEST;
        }
    }

    private double calculateScaledF(double voltage) {
        double baseF = 32767.0 / Constants.MAX_TICKS_PER_SEC;
        return baseF * (Constants.NOMINAL_VOLTAGE / voltage);
    }
}