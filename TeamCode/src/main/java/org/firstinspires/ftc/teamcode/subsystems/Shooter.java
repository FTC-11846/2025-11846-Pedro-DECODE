package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.CharacterStats;

/**
 * Shooter subsystem with universal configuration constants
 * Robot-specific tuning values come from CharacterStats (populated via applyConfiguration)
 */
@Configurable
public class Shooter {
    
    // ==================== CONFIGURATION OBJECTS ====================
    
    public static VelocityControl velocityControl = new VelocityControl();
    public static AutoAim autoAim = new AutoAim();
    public static PIDF pidf = new PIDF();


    // Used to scale shooter RPM's
    public static class VelocityControl {
        //11846 Numbers
        public double minPowerRPM11846 = 2400;
        public double maxPowerRPM11846 = 3125;
        //22154 Numbers
        public double minPowerRPM22154 = 2350;
        public double maxPowerRPM22154 = 3120;

        //_TB Numbers
        public double minPowerRPM_TB = 2400;
        public double maxPowerRPM_TB = 3125;

        public double minPowerRPM;
        public double maxPowerRPM;
    }
    
    public static class AutoAim {
        public double linearCorrectionFactor = 34.0;
        public double headingPGain = 0.015;
        public double maxTrackingRotation = 0.3;
        public double headingDeadbandDeg = 2.0;
        public double singleShotDuration = 120.0;
        public double autoAimSpinDownTime = 120.0;
        public double overrideThreshold = 0.15;
//        public double tagLossTimeout = 1.0;   // Muted, currently no usages
    }
    
    public static class PIDF {
        public double pidfI = 0.2;
        public double pidfD = 0.001;
        public double nominalVoltage = 12.5;
        public double maxTicksPerSec = 3000.0;
        public boolean debugRunWithoutEncoder = false;
        public double debugPower = 1.0;
    }
    
    // ==================== SUBSYSTEM FIELDS ====================
    
    @IgnoreConfigurable
    private final CharacterStats stats;
    
    @IgnoreConfigurable
    private final DcMotorEx shooterMotorL;
    
    @IgnoreConfigurable
    private final DcMotorEx shooterMotorR; // null for single motor robots
    
    @IgnoreConfigurable
    private final VoltageSensor batteryVoltageSensor;
    
    @IgnoreConfigurable
    private double targetVelocityRPM = 0;
    
    @IgnoreConfigurable
    private boolean autoAimActive = false;
    
    @IgnoreConfigurable
    private double lastDetectedDistance = 0;
    
    @IgnoreConfigurable
    private int lastDetectedTagId = -1;
    
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
        if(stats.getShortName() != "22154"){
            shooterMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            shooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (pidf.debugRunWithoutEncoder) {
            shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            shooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        // Initialize right motor only if robot has dual shooters
        if (stats.hasDualShooters()) {
            shooterMotorR = hardwareMap.get(DcMotorEx.class, stats.getShooterMotorRName());
            if (stats.getShortName() != "22154"){
                shooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                shooterMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (pidf.debugRunWithoutEncoder) {
                shooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                shooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else {
            shooterMotorR = null;
        }
    }

    public void setProperShooterVelocities(){
        if (stats.getShortName() == "22154") {
            Shooter.velocityControl.maxPowerRPM = Shooter.velocityControl.minPowerRPM22154;
            Shooter.velocityControl.maxPowerRPM = Shooter.velocityControl.maxPowerRPM22154;
        } else if (stats.getShortName() == "11846"){
            Shooter.velocityControl.maxPowerRPM = Shooter.velocityControl.minPowerRPM11846;
            Shooter.velocityControl.maxPowerRPM = Shooter.velocityControl.maxPowerRPM11846;
        } else {
            Shooter.velocityControl.maxPowerRPM = Shooter.velocityControl.minPowerRPM_TB;
            Shooter.velocityControl.maxPowerRPM = Shooter.velocityControl.maxPowerRPM_TB;
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
        double calculatedRPM = baseline + (distanceInches * autoAim.linearCorrectionFactor);
        
        targetVelocityRPM = clampVelocity(calculatedRPM);
        autoAimActive = true;
        lastDetectedDistance = distanceInches;
        lastDetectedTagId = tagId;
    }
    
    /**
     * Must be called in loop() to apply velocity control with PIDF
     */
    public void periodic() {
        if (pidf.debugRunWithoutEncoder) {
            // Debug mode - run at fixed power
            shooterMotorL.setPower(pidf.debugPower);
            if (shooterMotorR != null) {
                shooterMotorR.setPower(pidf.debugPower);
            }
            return;
        }
        
        // Normal velocity control with voltage-scaled PIDF
        double voltage = batteryVoltageSensor.getVoltage();
        double scaledF = calculateScaledF(voltage);
        double p = stats.getShooterPIDFP(); // Get P from CharacterStats
        
        // Apply PIDF coefficients
        shooterMotorL.setVelocityPIDFCoefficients(p, pidf.pidfI, pidf.pidfD, scaledF);
        
        // Convert RPM to ticks per second and apply
        double targetTicksPerSec = targetVelocityRPM * TICKS_TO_RPM;
        shooterMotorL.setVelocity(targetTicksPerSec);
        
        // Apply to second motor if it exists
        if (shooterMotorR != null) {
            shooterMotorR.setVelocityPIDFCoefficients(p, pidf.pidfI, pidf.pidfD, scaledF);
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
    
    // ==================== DEBUG GETTERS (FOR TUNING) ====================

    /**
     * Get raw motor encoder position in ticks
     * Used for debugging encoder connections and motor behavior
     */
    public int getRawPositionTicks() {
        return shooterMotorL.getCurrentPosition();
    }

    /**
     * Get current commanded motor power (-1.0 to 1.0)
     * Used for verifying velocity control is applying correct power
     * Note: Only returns left motor power
     */
    public double getShooterLPower() {
        return shooterMotorL.getPower();
    }

    /**
     * Get current motor run mode (e.g., RUN_USING_ENCODER)
     * Used for verifying motor configuration
     */
    public DcMotor.RunMode getShooterLRunMode() {
        return shooterMotorL.getMode();
    }

    /**
     * Get raw velocity in ticks per second
     * Used for debugging velocity control and PIDF tuning
     */
    public double getRawVelocity() {
        return shooterMotorL.getVelocity();
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
        return Math.max(velocityControl.minPowerRPM, 
                       Math.min(velocityControl.maxPowerRPM, rpm));
    }
    
    private double calculateScaledF(double voltage) {
        double baseF = 32767.0 / pidf.maxTicksPerSec;
        return baseF * (pidf.nominalVoltage / voltage);
    }
}
