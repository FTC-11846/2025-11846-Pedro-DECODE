package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;

/**
 * Shooter subsystem - now uses CharacterStats for configuration
 * Automatically adapts to different hardware via CharacterStats
 */
public class Shooter {
    private final CharacterStats stats;
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
        // Power limits (universal for all robots)
        public static double MIN_POWER_RPM = 1500;
        public static double MAX_POWER_RPM = 5500;

        // Auto-aim ballistics
        public static double LINEAR_CORRECTION_FACTOR = 18.0; // RPM per inch

        // PIDF Tuning (universal values, overridden by CharacterStats)
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

    /**
     * Create shooter using MainCharacter enum (backward compatible)
     */
    public Shooter(HardwareMap hardwareMap, MainCharacter character) {
        this(hardwareMap, character.getAbilities());
    }

    /**
     * Create shooter using CharacterStats directly (preferred)
     */
    public Shooter(HardwareMap hardwareMap, CharacterStats stats) {
        this.stats = stats;
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize left/primary motor (all robots have this)
        shooterMotorL = hardwareMap.get(DcMotorEx.class, stats.getShooterMotorLName());
        shooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        if (Constants.DEBUG_RUN_WITHOUT_ENCODER) {
            shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            shooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Initialize right motor only if robot has dual shooters
        if (stats.hasDualShooters()) {
            shooterMotorR = hardwareMap.get(DcMotorEx.class, stats.getShooterMotorRName());
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
     * Set shooter to low velocity (configured per robot in CharacterStats)
     */
    public void setLowVelocity() {
        targetVelocityRPM = stats.getLowVelocityRPM();
        autoAimActive = false;
    }

    /**
     * Set shooter to high velocity (configured per robot in CharacterStats)
     */
    public void setHighVelocity() {
        targetVelocityRPM = stats.getHighVelocityRPM();
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
     * Uses robot-specific ballistic constants from CharacterStats
     */
    public void setAutoAimVelocity(double distanceInches, int tagId) {
        double baseline = stats.getBaselinePower();
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
        double p = stats.getShooterPIDFP(); // Get P from CharacterStats

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

    public String getRobotName() {
        return stats.getDisplayName();
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

    private double calculateScaledF(double voltage) {
        double baseF = 32767.0 / Constants.MAX_TICKS_PER_SEC;
        return baseF * (Constants.NOMINAL_VOLTAGE / voltage);
    }
}