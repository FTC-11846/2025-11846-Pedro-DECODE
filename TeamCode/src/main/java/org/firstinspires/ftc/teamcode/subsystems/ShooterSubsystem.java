package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Shooter subsystem that handles both single and dual motor configurations
 * Uses RobotConfig to automatically adapt to different hardware
 */
public class ShooterSubsystem {
    private final MainCharacter config;
    private final DcMotorEx shooterMotorL;
    private final DcMotorEx shooterMotorR; // null for single motor robots
    private final VoltageSensor batteryVoltageSensor;

    private double targetVelocityRPM = 0;
    private boolean autoAimActive = false;
    private double lastDetectedDistance = 0;
    private int lastDetectedTagId = -1;

    // Constants
    private static final double TICKS_TO_RPM = 28.0 / 60.0;
    private static final double MIN_POWER = 1500;
    private static final double MAX_POWER = 5500;

    public ShooterSubsystem(HardwareMap hardwareMap, MainCharacter config) {
        this.config = config;
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize left/primary motor (all robots have this)
        shooterMotorL = hardwareMap.get(DcMotorEx.class, config.getShooterMotorLName());
        shooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize right motor only if robot has dual shooters
        if (config.hasDualShooters()) {
            shooterMotorR = hardwareMap.get(DcMotorEx.class, config.getShooterMotorRName());
            shooterMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            shooterMotorR = null;
        }
    }

    /**
     * Set shooter to low velocity (configured per robot)
     */
    public void setLowVelocity() {
        targetVelocityRPM = config.getShooterLowVelocity();
        autoAimActive = false;
    }

    /**
     * Set shooter to high velocity (configured per robot)
     */
    public void setHighVelocity() {
        targetVelocityRPM = config.getShooterHighVelocity();
        autoAimActive = false;
    }

    /**
     * Set shooter to specific RPM
     */
    public void setVelocityRPM(double rpm) {
        targetVelocityRPM = Math.max(MIN_POWER, Math.min(MAX_POWER, rpm));
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
        double calculatedRPM = config.getBaselinePower() + 
                               (distanceInches * config.getLinearCorrectionFactor());
        
        targetVelocityRPM = Math.max(MIN_POWER, Math.min(MAX_POWER, calculatedRPM));
        autoAimActive = true;
        lastDetectedDistance = distanceInches;
        lastDetectedTagId = tagId;
    }

    /**
     * Must be called in loop() to apply velocity control with PIDF
     */
    public void periodic() {
        double voltage = batteryVoltageSensor.getVoltage();
        double scaledF = config.getShooterF(voltage);

        // Apply PIDF coefficients
        shooterMotorL.setVelocityPIDFCoefficients(
            config.getShooterP(),
            config.getShooterI(),
            config.getShooterD(),
            scaledF
        );

        // Convert RPM to ticks per second
        double targetTicksPerSec = targetVelocityRPM * TICKS_TO_RPM;
        shooterMotorL.setVelocity(targetTicksPerSec);

        // Apply to second motor if it exists
        if (shooterMotorR != null) {
            shooterMotorR.setVelocityPIDFCoefficients(
                config.getShooterP(),
                config.getShooterI(),
                config.getShooterD(),
                scaledF
            );
            shooterMotorR.setVelocity(targetTicksPerSec);
        }
    }

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

    /**
     * Get target velocity
     */
    public double getTargetVelocityRPM() {
        return targetVelocityRPM;
    }

    /**
     * Check if auto-aim is currently active
     */
    public boolean isAutoAimActive() {
        return autoAimActive;
    }

    /**
     * Get last detected distance (for telemetry)
     */
    public double getLastDetectedDistance() {
        return lastDetectedDistance;
    }

    /**
     * Get last detected tag ID (for telemetry)
     */
    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }

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
}
