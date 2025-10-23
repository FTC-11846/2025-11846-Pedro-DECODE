package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * FieldMirror - Utility for mirroring poses between RED and BLUE alliance
 * 
 * Convention: Define all positions in BLUE coordinates, mirror for RED
 * 
 * Field coordinate system:
 * - X: 0 = blue wall, 144 = red wall
 * - Y: 0 = audience side, 144 = far side
 * - Heading: 0° = facing audience, 90° = facing far side
 */
public class FieldMirror {
    
    @Configurable
    public static class RelocalizationConstants {
        // How much to trust odometry vs vision when blending poses
        // 1.0 = trust odometry completely, 0.0 = trust vision completely
        public static double ODOMETRY_TRUST_RATIO = 0.7;
        
        // Minimum error (inches) before applying relocalization
        // Prevents jitter from small AprilTag detection variations
        public static double MIN_ERROR_THRESHOLD = 3.0;
        
        // Maximum error (inches) to accept from vision
        // Rejects obviously bad AprilTag detections
        public static double MAX_ERROR_THRESHOLD = 24.0;
    }
    
    // Field dimensions (inches)
    private static final double FIELD_WIDTH = 144.0;
    
    /**
     * Mirror a pose from BLUE alliance to RED alliance (or vice versa)
     * Returns the pose unchanged if alliance is BLUE
     * 
     * @param bluePose Pose in BLUE alliance coordinates
     * @param alliance Target alliance (RED or BLUE)
     * @return Pose in target alliance coordinates
     */
    public static Pose mirrorPose(Pose bluePose, Alliance alliance) {
        if (alliance.isBlue()) {
            return bluePose; // No mirroring needed
        }
        
        // Mirror for RED alliance
        double mirroredX = mirrorX(bluePose.getX());
        double mirroredY = bluePose.getY(); // Y stays the same
        double mirroredHeading = mirrorHeading(bluePose.getHeading());
        
        return new Pose(mirroredX, mirroredY, mirroredHeading);
    }
    
    /**
     * Mirror X coordinate across field centerline
     * X coordinate flips: 144 - x
     * 
     * Examples:
     * - Blue wall (X=0) → Red wall (X=144)
     * - Center (X=72) → Center (X=72)
     * - Red wall (X=144) → Blue wall (X=0)
     */
    public static double mirrorX(double blueX) {
        return FIELD_WIDTH - blueX;
    }
    
    /**
     * Mirror Y coordinate (no change - field is symmetric along Y axis)
     */
    public static double mirrorY(double blueY) {
        return blueY; // Y doesn't change in FTC field mirroring
    }
    
    /**
     * Mirror heading across field centerline
     * Heading formula: PI - heading (flips around vertical axis)
     * 
     * Examples (in degrees for clarity):
     * - 0° (facing audience) → 180° (facing audience, opposite side)
     * - 90° (facing far side) → 90° (facing far side)
     * - 180° (facing opponent) → 0° (facing opponent, opposite side)
     * - 270° (facing audience) → 270° (facing audience)
     */
    public static double mirrorHeading(double blueHeading) {
        return Math.PI - blueHeading;
    }
    
    /**
     * Blend two poses using weighted average
     * Used for relocalization: blend odometry pose with vision-based pose
     * 
     * @param poseA First pose (typically current odometry pose)
     * @param poseB Second pose (typically vision-corrected pose)
     * @param weightA Weight for first pose (0.0 to 1.0)
     * @return Blended pose
     * 
     * Example: blendPoses(odomPose, visionPose, 0.7)
     * Result: 70% odometry, 30% vision
     */
    public static Pose blendPoses(Pose poseA, Pose poseB, double weightA) {
        double weightB = 1.0 - weightA;
        
        double blendedX = (poseA.getX() * weightA) + (poseB.getX() * weightB);
        double blendedY = (poseA.getY() * weightA) + (poseB.getY() * weightB);
        
        // Blend heading using circular mean (handles wraparound correctly)
        double blendedHeading = circularMean(
            poseA.getHeading(), 
            poseB.getHeading(), 
            weightA
        );
        
        return new Pose(blendedX, blendedY, blendedHeading);
    }
    
    /**
     * Calculate circular mean of two angles (handles 0°/360° wraparound)
     * Uses vector averaging method for correct angular interpolation
     */
    private static double circularMean(double angle1, double angle2, double weight1) {
        double weight2 = 1.0 - weight1;
        
        // Convert to unit vectors
        double x = Math.cos(angle1) * weight1 + Math.cos(angle2) * weight2;
        double y = Math.sin(angle1) * weight1 + Math.sin(angle2) * weight2;
        
        // Convert back to angle
        return Math.atan2(y, x);
    }
    
    /**
     * Calculate Euclidean distance between two poses (ignoring heading)
     * Useful for determining if relocalization correction is needed
     */
    public static double distanceBetween(Pose pose1, Pose pose2) {
        double dx = pose2.getX() - pose1.getX();
        double dy = pose2.getY() - pose1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Check if relocalization should be applied based on error thresholds
     * 
     * @param currentPose Current odometry pose
     * @param visionPose Vision-corrected pose
     * @return true if relocalization should be applied
     */
    public static boolean shouldRelocalize(Pose currentPose, Pose visionPose) {
        double error = distanceBetween(currentPose, visionPose);
        
        // Too small - don't correct (avoids jitter)
        if (error < RelocalizationConstants.MIN_ERROR_THRESHOLD) {
            return false;
        }
        
        // Too large - reject as bad detection
        if (error > RelocalizationConstants.MAX_ERROR_THRESHOLD) {
            return false;
        }
        
        // Within acceptable range - apply correction
        return true;
    }
}
